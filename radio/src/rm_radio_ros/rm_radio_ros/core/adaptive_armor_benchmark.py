"""离线比较圆心候选的未来覆盖；不模拟机会消耗，不创建 ROS 或裁判请求。"""
from __future__ import annotations

import argparse
import gzip
import json
import math
from pathlib import Path
from statistics import median
import sys

import numpy as np

from .adaptive_armor import (
    ArmorConfig, Hit, best_center, center_bounds, coverages, integer, legal_center,
    number, weights,
)
from .adaptive_armor_replay import replay_steps


METHODS = ('fixed_center', 'coordinate_median', 'kmeans', 'maximum_coverage', 'kde_coverage')


def _project(center, config, radius):
    left, right, bottom, top = center_bounds(config, radius)
    left, right, bottom, top = math.ceil(left), math.floor(right), math.ceil(bottom), math.floor(top)
    if left > right or bottom > top:
        return None
    return (min(right, max(left, round(center[0]))), min(top, max(bottom, round(center[1]))))


def _kmeans(hits, config, radius, now, clusters):
    points = np.array([(h.x, h.y) for h in hits])
    mass = weights(hits, now, config.decay_sec)
    points, mass = points[mass > 0], mass[mass > 0]
    # Deterministic farthest-first initialization, followed by weighted Lloyd steps.
    centers = [points[int(np.argmax(mass))]]
    for _ in range(1, clusters):
        distance = ((points[:, None] - np.array(centers))**2).sum(axis=2).min(axis=1)
        if distance.max() == 0:
            break
        centers.append(points[int(np.argmax(distance * mass))])
    centers = np.array(centers)
    for _ in range(30):
        labels = ((points[:, None] - centers)**2).sum(axis=2).argmin(axis=1)
        updated = np.array([
            np.average(points[labels == i], axis=0, weights=mass[labels == i])
            if np.any(labels == i) else center for i, center in enumerate(centers)
        ])
        if np.allclose(updated, centers, rtol=0, atol=1e-6):
            break
        centers = updated
    candidates = sorted({_project(c, config, radius) for c in centers} - {None})
    if not candidates:
        return None
    scores = coverages(hits, candidates, radius, now, config.decay_sec)
    return candidates[int(np.argmax(scores))]


def _kde(hits, config, radius, now, bandwidth_mm):
    # Gaussian probability mass approximated on a 32x32 board grid. Normalize each
    # component inside the board so edge kernels do not lose their projectile mass.
    xs = (np.arange(32) + 0.5) * config.plate_width_mm / 32
    ys = (np.arange(32) + 0.5) * config.plate_height_mm / 32
    cells = np.array([(x, y) for x in xs for y in ys])
    density = np.zeros(len(cells))
    for hit, mass in zip(hits, weights(hits, now, config.decay_sec)):
        log_kernel = -((cells - (hit.x, hit.y))**2).sum(axis=1) / (2 * bandwidth_mm**2)
        kernel = np.exp(log_kernel - log_kernel.max())
        density += mass * kernel / kernel.sum()
    smoothed = [Hit(now, x, y, float(mass)) for (x, y), mass in zip(cells, density) if mass > 0]
    return best_center(smoothed, config, radius, now)


def candidate_centers(hits, config, radius, now, clusters=3, bandwidth_mm=5):
    """All methods see the same past hits, geometry and frozen radius."""
    if not hits:
        raise ValueError('选点至少需要一个历史命中')
    clusters = integer(clusters, 'clusters', 1, 8)
    bandwidth_mm = number(bandwidth_mm, 'bandwidth_mm', 0.1, 5000)
    # Common scaling leaves the weighted objective unchanged and avoids all-zero
    # weights when every historical hit is old relative to a short decay time.
    now = max(h.timestamp for h in hits)
    return {
        'fixed_center': _project((config.plate_width_mm / 2, config.plate_height_mm / 2), config, radius),
        'coordinate_median': _project((median(h.x for h in hits), median(h.y for h in hits)), config, radius),
        'kmeans': _kmeans(hits, config, radius, now, clusters),
        'maximum_coverage': best_center(hits, config, radius, now),
        'kde_coverage': _kde(hits, config, radius, now, bandwidth_mm),
    }


def benchmark(rows, config=None, *, horizon_sec=30, clusters=3, bandwidth_mm=5):
    """Compare non-overlapping future windows, keeping each source session separate.

    A forecast uses only accepted hits received before its cutoff. A delayed hit
    may score an earlier forecast if its source time belongs to that future window;
    it cannot change any saved center. Partial horizons and lost completeness are
    excluded. No simulated activation or availability is inferred from coverage.
    """
    horizon_sec = number(horizon_sec, 'horizon_sec', 1, 300)
    clusters = integer(clusters, 'clusters', 1, 8)
    bandwidth_mm = number(bandwidth_mm, 'bandwidth_mm', 0.1, 5000)
    sessions = []
    session = None
    statistics = None
    seen_hits = 0
    next_cutoff = 0
    for now, decision, row in replay_steps(rows, config):
        stats, cfg = decision.statistics, decision.config
        if stats is not statistics:
            if session is not None:
                session['observed_until'] = now
            statistics, session, seen_hits = stats, None, 0
            next_cutoff = now
        if not stats.match_id:
            continue
        if session is None:
            session = {
                'match_id': stats.match_id, 'source': stats.source, 'side': stats.side,
                'recorded_origin': row.get('data', {}).get('origin', 'unknown'),
                'evaluation_origin': 'replay', 'weight_basis': stats.weight_basis,
                'observed_until': now, 'windows': [], 'skipped_reasons': {},
            }
            sessions.append(session)
        session['observed_until'] = now
        new_hits = stats.hits['incoming'][seen_hits:]
        seen_hits = len(stats.hits['incoming'])
        for window in session['windows']:
            if not stats.complete:
                window['data_complete'] = False
            # Keep late future outcomes, but never admit an event already known
            # at cutoff (including the allowed source-clock lead of up to .5 s).
            if now > window['cutoff']:
                window['_future'].extend(
                    h for h in new_hits if window['cutoff'] < h.timestamp <= window['end']
                )
            if now <= window['end'] and cfg != window['_config']:
                window['configuration_changed'] = True
        if now < next_cutoff:
            continue
        proposal = decision.status['proposal']
        radius = proposal['radius_mm']
        training = [h for h in stats.hits['incoming'] if now-cfg.window_sec <= h.timestamp <= now]
        reason = ''
        if not stats.complete or not decision.status['statistics_fresh']:
            reason = '统计不完整或源心跳过期'
        elif not cfg.geometry_confirmed or cfg.boundary_policy == 'unconfirmed':
            reason = '检测面几何或边界尚未确认'
        elif any(h.x > cfg.plate_width_mm or h.y > cfg.plate_height_mm for h in stats.hits['incoming']):
            reason = '历史命中超出当前检测面'
        elif not radius or not math.isfinite(radius):
            reason = '缺少有效半径'
        elif len(training) < cfg.min_train_samples:
            reason = '历史训练样本不足'
        if reason:
            skipped = session['skipped_reasons']
            skipped[reason] = skipped.get(reason, 0) + 1
            continue
        next_cutoff = now + horizon_sec
        # Same deterministic cap as the online predictive window; never cap
        # cumulative rule statistics or future evaluation outcomes.
        training = sorted(training, key=lambda h: h.timestamp)[-2000:]
        centers = candidate_centers(training, cfg, radius, now, clusters, bandwidth_mm)
        session['windows'].append({
            'cutoff': now, 'end': now+horizon_sec, 'radius_mm': radius,
            'radius_source': proposal['radius_source'], 'train_count': len(training),
            'config': cfg.to_dict(), 'data_complete': True, 'configuration_changed': False,
            'methods': {name: {'center_mm': list(c) if c is not None else None,
                               'legal': c is not None and legal_center(cfg, radius, c)}
                        for name, c in centers.items()},
            '_future': [], '_config': cfg,
        })
    if not sessions:
        raise ValueError('日志缺少原始 session 事件，不能从显示状态重建比赛统计')
    for session in sessions:
        totals = {name: {'evaluated_windows': 0, 'covered_weight': 0.0,
                         'future_weight': 0.0, 'future_hit_count': 0,
                         'invalid_candidates': 0, 'proposal_changes': 0,
                         'opportunities_consumed': None, 'actual_center_changes': None,
                         'invalid_requests': None, 'adaptation_delay_sec': None}
                  for name in METHODS}
        previous = {}
        for window in session['windows']:
            future = window.pop('_future')
            window.pop('_config')
            window['future_hit_count'] = len(future)
            reason = ('未来时间段未完整记录' if session['observed_until'] < window['end'] else
                      '统计完整性丢失' if not window['data_complete'] else
                      '评估期间配置发生变化' if window['configuration_changed'] else
                      '未来时间段没有有效来袭命中' if not future else '')
            window['evaluation_reason'] = reason or '完整未来窗口'
            window['evaluated'] = not reason
            total_weight = sum(h.damage for h in future)
            for name, result in window['methods'].items():
                total = totals[name]
                center = result['center_mm']
                if not result['legal']:
                    total['invalid_candidates'] += 1
                elif name in previous and center != previous[name]:
                    total['proposal_changes'] += 1
                if result['legal']:
                    previous[name] = center
                if reason or not result['legal']:
                    result.update(future_coverage=None, expected_damage_multiplier=None)
                    continue
                covered = sum(h.damage for h in future
                              if (h.x-center[0])**2 + (h.y-center[1])**2 <= window['radius_mm']**2 + 1e-9)
                coverage = covered / total_weight
                result.update(future_coverage=coverage, expected_damage_multiplier=1.5-coverage)
                total['evaluated_windows'] += 1
                total['covered_weight'] += covered
                total['future_weight'] += total_weight
                total['future_hit_count'] += len(future)
        for total in totals.values():
            coverage = total['covered_weight']/total['future_weight'] if total['future_weight'] else None
            total.update(future_coverage=coverage,
                         expected_damage_multiplier=1.5-coverage if coverage is not None else None)
        session['summary'] = totals
    return {
        'schema': 'shark.radar.armor_benchmark.v1', 'sessions': sessions,
        'parameters': {'horizon_sec': horizon_sec, 'clusters': clusters,
                       'bandwidth_mm': bandwidth_mm, 'kde_grid': [32, 32]},
        'limitations': [
            '离线候选比较；不创建 ROS 节点或发令，不表示线上策略通过收益与稳定门控',
            '每个窗口固定选点时半径；不模拟未来半径变化、首次激活、机会消耗或裁判确认',
            '未来评分使用未衰减的逐发权重；count 模式是假设每发等伤害的估计',
            'K-means 使用确定性加权初始化；KDE 为板内归一化的 32×32 网格近似圆内积分',
            '按原始会话区分来源类型；合成结果不能作为实战表现或参数选择依据',
            'proposal_changes 仅为候选变化；适应延迟、实际改点与机会消耗尚未评估',
        ],
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--input', required=True, help='原始 JSONL / JSONL.gz，或 - 读取 stdin')
    parser.add_argument('--config', type=Path, help='配置；缺省读取日志目录的配置快照')
    parser.add_argument('--output', type=Path, help='JSON 对比报告；缺省 stdout')
    parser.add_argument('--horizon-sec', type=float, default=30)
    parser.add_argument('--clusters', type=int, default=3)
    parser.add_argument('--bandwidth-mm', type=float, default=5)
    args = parser.parse_args()
    config_file = args.config
    if config_file is None and args.input != '-':
        candidate = Path(args.input).parent / 'adaptive_armor_config_snapshot.json'
        config_file = candidate if candidate.is_file() else None
    cfg = ArmorConfig.from_dict(json.loads(config_file.read_text())) if config_file else None
    stream = (sys.stdin if args.input == '-' else gzip.open(args.input, 'rt', encoding='utf-8')
              if args.input.endswith('.gz') else open(args.input, encoding='utf-8'))
    try:
        report = benchmark((json.loads(line) for line in stream if line.strip()), cfg,
                           horizon_sec=args.horizon_sec, clusters=args.clusters,
                           bandwidth_mm=args.bandwidth_mm)
    finally:
        if stream is not sys.stdin:
            stream.close()
    content = json.dumps(report, ensure_ascii=False, allow_nan=False, sort_keys=True, indent=2) + '\n'
    if args.output:
        args.output.write_text(content, encoding='utf-8')
    else:
        sys.stdout.write(content)


if __name__ == '__main__':
    main()
