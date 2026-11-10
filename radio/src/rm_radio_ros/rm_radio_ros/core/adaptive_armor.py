"""装甲圆心策略；输入是队内适配器事件，不是新增的官方裁判协议。"""
from __future__ import annotations

from dataclasses import asdict, dataclass, fields
import math
from statistics import median

import numpy as np


EVENT_TOPIC = '/rm_radar_algorithm/armor_events'
CONFIG_TOPIC = '/rm_radar_algorithm/adaptive_armor_config'
STATUS_TOPIC = '/rm_radar_algorithm/adaptive_armor'
EVENT_SCHEMA = 'shark.radar.armor_event.v1'
CONFIG_SCHEMA = 'shark.radar.armor_config.v1'
FRAME = 'armor_surface_lower_left_mm'


def number(value, name, low=0.0, high=float('inf')) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ValueError(f'{name} 必须为数字')
    value = float(value)
    if not math.isfinite(value) or not low <= value <= high:
        raise ValueError(f'{name} 超出有效范围')
    return value


def integer(value, name, low=0, high=2**53 - 1) -> int:
    result = number(value, name, low, high)
    if result != int(result):
        raise ValueError(f'{name} 必须为整数')
    return int(result)


@dataclass(frozen=True)
class ArmorConfig:
    mode: str = 'fixed_center'
    plate_width_mm: float = 0.0
    plate_height_mm: float = 0.0
    plate_area_mm2: float = 0.0
    geometry_confirmed: bool = False
    boundary_policy: str = 'unconfirmed'
    automatic_requests: bool = False
    trusted_sources: tuple[str, ...] = ()
    window_sec: float = 120.0
    validation_sec: float = 30.0
    decay_sec: float = 60.0
    block_sec: float = 3.0
    min_train_samples: int = 20
    min_validation_samples: int = 12
    confidence: float = 0.95
    first_margin: float = 0.05
    change_margin: float = 0.10
    stable_sec: float = 5.0
    freshness_sec: float = 3.0
    confirm_timeout_sec: float = 5.0

    @classmethod
    def from_dict(cls, data):
        if not isinstance(data, dict) or set(data) - {f.name for f in fields(cls)}:
            raise ValueError('装甲策略配置包含未知字段')
        values = asdict(cls())
        values.update(data)
        if values['mode'] not in ('fixed_center', 'adaptive_coverage'):
            raise ValueError('选点模式无效')
        if values['boundary_policy'] not in ('unconfirmed', 'contained', 'clip'):
            raise ValueError('边界策略无效')
        for key in ('geometry_confirmed', 'automatic_requests'):
            if type(values[key]) is not bool:
                raise ValueError(f'{key} 必须为布尔值')
        sources = values['trusted_sources']
        if not isinstance(sources, (list, tuple)) or any(
            not isinstance(x, str) or not x.strip() or len(x) > 128 for x in sources
        ):
            raise ValueError('trusted_sources 必须为非空来源名称的列表')
        values['trusted_sources'] = tuple(sources)
        ranges = {
            'plate_width_mm': (0, 5000), 'plate_height_mm': (0, 5000),
            'plate_area_mm2': (0, 25_000_000), 'window_sec': (10, 900),
            'validation_sec': (1, 300), 'decay_sec': (1, 900),
            'block_sec': (1, 60), 'confidence': (0.8, 0.999),
            'first_margin': (0, 0.49), 'change_margin': (0, 0.99),
            'stable_sec': (1, 60), 'freshness_sec': (0.5, 10),
            'confirm_timeout_sec': (2, 30),
        }
        for key, bounds in ranges.items():
            values[key] = number(values[key], key, *bounds)
        for key in ('min_train_samples', 'min_validation_samples'):
            values[key] = integer(values[key], key, 3, 2000)
        if values['window_sec'] <= values['validation_sec']:
            raise ValueError('选点窗口必须大于验证窗口')
        if values['geometry_confirmed'] and min(
            values['plate_width_mm'], values['plate_height_mm']
        ) < 1:
            raise ValueError('确认几何前需提供检测面毫米宽高')
        return cls(**values)

    def to_dict(self):
        return asdict(self)


@dataclass(frozen=True)
class Hit:
    timestamp: float
    x: float
    y: float
    damage: float


class ArmorStatistics:
    def __init__(self):
        self.match_id = ''
        self.source = ''
        self.origin = ''
        self.side = ''
        self.started_at = 0.0
        self.last_received_at = 0.0
        self.last_source_time = 0.0
        self.complete = False
        self.weight_basis = 'count'
        self.shots = None
        self.hits = {'outgoing': [], 'incoming': []}
        self.seen = set()
        self.duplicates = 0

    def ingest(self, event, now, config):
        if not isinstance(event, dict) or event.get('schema') != EVENT_SCHEMA:
            raise ValueError('不支持的装甲事件 schema')
        if event.get('coordinate_frame') != FRAME:
            raise ValueError('命中坐标必须为检测面左下角原点的毫米坐标')
        timestamp = number(event.get('timestamp'), 'timestamp', 0, now + 0.5)
        kind = event.get('kind')
        if kind == 'session':
            match_id = event.get('match_id')
            source = event.get('source')
            if not all(isinstance(x, str) and 0 < len(x) <= 128 for x in (match_id, source)):
                raise ValueError('比赛与来源标识无效')
            if event.get('origin') not in ('live', 'replay', 'synthetic') or event.get('side') not in ('red', 'blue'):
                raise ValueError('事件来源类型或己方阵营无效')
            if type(event.get('complete_from_start')) is not bool:
                raise ValueError('必须声明是否具备当局完整统计')
            if event.get('weight_basis', 'count') not in ('count', 'baseline_damage'):
                raise ValueError('会话受击权重类型无效')
            if self.match_id:
                raise ValueError('当前比赛已绑定；等待裁判确认比赛结束后再开始新会话')
            self.match_id, self.source = match_id, source
            self.origin, self.side = event['origin'], event['side']
            self.started_at = timestamp
            self.complete = event['complete_from_start']
            self.weight_basis = event.get('weight_basis', 'count')
        else:
            if not self.match_id or any(event.get(k) != getattr(self, k) for k in ('match_id', 'source')):
                raise ValueError('事件与当前比赛/来源不匹配')
            if timestamp < self.started_at:
                raise ValueError('事件早于比赛会话')
            if kind == 'shots':
                shots = tuple(integer(event.get(k), k) for k in ('own_shots', 'opponent_shots'))
                if self.shots and any(n < old for n, old in zip(shots, self.shots)):
                    raise ValueError('累计发射数不能倒退')
                if timestamp < self.last_source_time:
                    raise ValueError('过期的累计统计快照')
                if type(event.get('data_complete')) is not bool:
                    raise ValueError('累计统计必须声明数据完整性')
                self.complete = self.complete and event['data_complete']
                self.shots = shots
            elif kind == 'hit':
                direction = event.get('direction')
                event_id = event.get('event_id')
                if direction not in self.hits or not isinstance(event_id, str) or not 0 < len(event_id) <= 256:
                    raise ValueError('命中方向或含源模块/展开序号的事件标识无效')
                if event.get('confirmed') is not True:
                    raise ValueError('命中尚未经适配器确认为有效弹丸事件')
                key = (direction, event_id)
                if key in self.seen:
                    self.duplicates += 1
                    return False
                x = integer(event.get('x_mm'), 'x_mm', 0, 65535)
                y = integer(event.get('y_mm'), 'y_mm', 0, 65535)
                if direction == 'incoming' and config.geometry_confirmed and (
                    x > config.plate_width_mm or y > config.plate_height_mm
                ):
                    raise ValueError('来袭命中超出已确认检测面；请核对模块坐标映射')
                if self.weight_basis == 'baseline_damage' and 'baseline_damage' not in event:
                    raise ValueError('伤害加权会话要求每发均提供倍率施加前的伤害')
                damage = number(event.get('baseline_damage', 1.0), 'baseline_damage', 0.001, 1_000_000)
                if self.weight_basis == 'count':
                    damage = 1.0
                self.hits[direction].append(Hit(timestamp, x, y, damage))
                self.seen.add(key)
            else:
                raise ValueError('未知装甲事件类型')
        self.last_received_at = now
        self.last_source_time = max(self.last_source_time, timestamp)
        return True

    def summary(self, config):
        own, enemy = self.hits['outgoing'], self.hits['incoming']
        result = {
            'own_hit_count': len(own) if self.match_id else None,
            'own_scored_hit_count': len(own) if self.match_id else None,
            'own_received_hit_count': len(enemy) if self.match_id else None,
            'own_sample_count': len(own), 'opponent_sample_count': len(enemy),
            'estimate_valid': False, 'duplicate_count': self.duplicates,
            'data_complete': self.complete,
            'weight_basis': self.weight_basis,
        }
        if not self.complete or not self.shots or not all(self.shots):
            return result
        if any(len(hits) < 10 or len(hits) > shots for hits, shots in zip((own, enemy), self.shots)):
            return result
        spreads = []
        for hits in (own, enemy):
            x, y = median(h.x for h in hits), median(h.y for h in hits)
            spreads.append(median(math.hypot(h.x - x, h.y - y) for h in hits))
        if not all(s > 0 for s in spreads):
            return result
        own_rate, enemy_rate = len(own) / self.shots[0], len(enemy) / self.shots[1]
        rho = (own_rate / spreads[0]) / (enemy_rate / spreads[1])
        # Stable even when rho is large; no artificial sigma epsilon.
        ratio = 1 / (1 + 2 / rho**2)
        area = config.plate_area_mm2 or config.plate_width_mm * config.plate_height_mm
        result.update(
            estimate_valid=True, own_hit_rate=own_rate, opponent_hit_rate=enemy_rate,
            own_spread_mm=spreads[0], opponent_spread_mm=spreads[1],
            relative_accuracy=rho, area_ratio=ratio,
            calculated_area_mm2=area * ratio if area > 0 else None,
            calculated_radius_mm=math.sqrt(area * ratio / math.pi) if area > 0 else None,
        )
        return result


def center_bounds(config, radius):
    inset = radius if config.boundary_policy == 'contained' else 0
    return (inset, config.plate_width_mm - inset, inset, config.plate_height_mm - inset)


def legal_center(config, radius, center):
    left, right, bottom, top = center_bounds(config, radius)
    x, y = center
    return 0 <= x <= 65535 and 0 <= y <= 65535 and left <= x <= right and bottom <= y <= top


def weights(hits, now, decay):
    return np.array([h.damage * math.exp(-(now - h.timestamp) / decay) for h in hits])


def coverages(hits, centers, radius, now, decay):
    if not hits:
        return np.zeros(len(centers))
    points = np.array([(h.x, h.y) for h in hits])
    weight = weights(hits, now, decay)
    result = []
    for start in range(0, len(centers), 128):
        distances = np.sum((np.array(centers[start:start + 128])[:, None, :] - points[None, :, :])**2, axis=2)
        result.extend((distances <= radius**2 + 1e-9) @ weight / weight.sum())
    return np.clip(np.array(result), 0, 1)


def best_center(hits, config, radius, now, current=None):
    left, right, bottom, top = center_bounds(config, radius)
    if left > right or bottom > top:
        return None
    # Bound the grid independently of physical board size. Refine several peaks.
    step = max(1, math.ceil(math.sqrt((right-left+1) * (top-bottom+1) / 4096)))
    xs = list(range(math.ceil(left), math.floor(right) + 1, step)) + [math.floor(right)]
    ys = list(range(math.ceil(bottom), math.floor(top) + 1, step)) + [math.floor(top)]
    candidates = {(x, y) for x in xs for y in ys}
    candidates.add((round(config.plate_width_mm / 2), round(config.plate_height_mm / 2)))
    candidates.update((round(h.x), round(h.y)) for h in hits)
    if current is not None:
        candidates.add(tuple(current))
    candidates = sorted(c for c in candidates if legal_center(config, radius, c))
    if not candidates:
        return None
    anchor = current or (config.plate_width_mm / 2, config.plate_height_mm / 2)

    def rank(i):
        c = candidates[i]
        return (-round(float(scores[i]), 12), (c[0]-anchor[0])**2 + (c[1]-anchor[1])**2, c)

    scores = coverages(hits, candidates, radius, now, config.decay_sec)
    peaks = sorted(range(len(candidates)), key=rank)[:4]
    while step > 1:
        finer = max(1, step // 4)
        refined = set(candidates[i] for i in peaks)
        for i in peaks:
            x, y = candidates[i]
            refined.update((xx, yy) for xx in range(x-step, x+step+1, finer)
                           for yy in range(y-step, y+step+1, finer)
                           if legal_center(config, radius, (xx, yy)))
        candidates = sorted(refined)
        scores = coverages(hits, candidates, radius, now, config.decay_sec)
        peaks = sorted(range(len(candidates)), key=rank)[:4]
        step = finer
    peak = float(scores.max())
    tied = [c for c, score in zip(candidates, scores) if abs(score - peak) < 1e-10]
    return min(tied, key=lambda c: ((c[0]-anchor[0])**2 + (c[1]-anchor[1])**2, c))


def validation_score(hits, center, current, radius, now, config):
    """Weighted Hoeffding bounds, conditional on independent time blocks.

    These are model bounds, not guarantees for a drifting opponent. Training
    samples are excluded by the caller; no per-projectile IID assumption.
    """
    if not hits:
        return {}
    groups = {}
    for hit in hits:
        groups.setdefault(math.floor(hit.timestamp / config.block_sec), []).append(hit)
    mass = np.array([weights(block, now, config.decay_sec).sum() for block in groups.values()])
    mass /= mass.sum()
    score = float(coverages(hits, [center], radius, now, config.decay_sec)[0])
    current_score = float(coverages(hits, [current], radius, now, config.decay_sec)[0]) if current else 0.0
    penalty = math.sqrt(float(mass @ mass) * math.log(1 / (1-config.confidence)) / 2)
    return {
        'coverage': score, 'coverage_lower': max(0, score-penalty),
        'current_coverage': current_score,
        'improvement_lower': score-current_score-2*penalty,
        'validation_blocks': len(groups), 'expected_damage_multiplier': 1.5-score,
    }


class AdaptiveArmorDecision:
    def __init__(self, config=None):
        self.config = config or ArmorConfig()
        self.statistics = ArmorStatistics()
        self.pending = None
        self.last_result = None
        self.last_error = ''
        self.choice = None
        self.choice_since = None
        self.running = False
        self.sequence = 0
        self.status = {}

    def configure(self, config):
        if config != self.config:
            # Match statistics and an in-flight request survive configuration changes.
            self.choice, self.choice_since = None, None
        self.config = config

    def ingest(self, event, now):
        if not isinstance(event, dict):
            raise ValueError('装甲事件必须为对象')
        if event.get('kind') == 'session' and self.pending:
            raise ValueError('存在未核对的请求，不能替换比赛会话')
        return self.statistics.ingest(event, now, self.config)

    def acknowledge(self, ack):
        if not self.pending or ack.get('request_id') != self.pending['request_id']:
            return
        if ack.get('written') is True:
            self.pending['written'] = True
        elif ack.get('dry_run') is True:
            self.pending['phase'] = 'dry_run'
        else:
            self.pending['phase'] = 'failed'

    def evaluate(self, now, referee, side, side_mismatch=False):
        config, stats = self.config, self.statistics
        game = referee.get('last_game_status') or {}
        sync = referee.get('last_radar_decision_sync') or {}

        def fresh(data):
            timestamp = data.get('timestamp')
            return isinstance(timestamp, (int, float)) and math.isfinite(timestamp) and 0 <= now-timestamp <= config.freshness_sec

        running = fresh(game) and game.get('match_running') is True
        if self.running and fresh(game) and game.get('match_running') is False:
            self.last_result = self.pending or self.last_result
            self.pending = None
            self.statistics = stats = ArmorStatistics()
            self.choice, self.choice_since = None, None
        if fresh(game):
            self.running = running
        current = None
        active = sync.get('armor_fortify_active') is True
        sync_valid = fresh(sync) and sync.get('valid_radar_info_length') is True
        radius = None
        if sync_valid and active:
            try:
                radius = number(sync.get('fortify_radius_mm'), '裁判半径', 0.001, 65535)
                current = (integer(sync.get('fortify_center_x_mm'), '裁判圆心 x', 0, 65535),
                           integer(sync.get('fortify_center_y_mm'), '裁判圆心 y', 0, 65535))
            except ValueError:
                sync_valid = False
        summary = stats.summary(config)
        statistics_fresh = (
            0 <= now-stats.last_received_at <= config.freshness_sec
            and 0 <= now-stats.last_source_time <= config.freshness_sec
        )
        summary['estimate_valid'] = summary['estimate_valid'] and statistics_fresh
        if radius is None and not active:
            radius = summary.get('calculated_radius_mm') if summary['estimate_valid'] else None
        opportunities = sync.get('remaining_fortify_opportunities')
        opportunities_valid = type(opportunities) is int and 0 <= opportunities <= 7
        if self.pending:
            p = self.pending
            if (p.get('written') and sync_valid and stats.side == side and not side_mismatch
                    and current == tuple(p['center_mm'])
                    and sync['timestamp'] > p['sent_at'] and opportunities_valid
                    and opportunities < p['opportunities_before']):
                p['phase'] = 'confirmed'
                self.last_result = dict(p)
                self.pending = None
            elif now-p['sent_at'] >= config.confirm_timeout_sec and p['phase'] == 'pending':
                p['phase'] = 'timeout'

        center = (round(config.plate_width_mm / 2), round(config.plate_height_mm / 2))
        training, validation = [], []
        incoming = stats.hits['incoming']
        for hit in sorted(incoming, key=lambda h: h.timestamp):
            if now-config.window_sec <= hit.timestamp <= now:
                (training if hit.timestamp <= now-config.validation_sec else validation).append(hit)
        # Only predictive windows are bounded; official cumulative statistics remain intact.
        training, validation = training[-2000:], validation[-2000:]
        geometry_known = min(config.plate_width_mm, config.plate_height_mm) >= 1
        enough_training = len(training) >= config.min_train_samples
        if geometry_known and radius and config.mode == 'adaptive_coverage' and enough_training:
            center = best_center(training, config, radius, now, current)
        legal = bool(geometry_known and radius and center and legal_center(config, radius, center))
        score = validation_score(validation, center, current, radius, now, config) if legal else {}
        choice = (center, radius, config.mode)
        if choice != self.choice:
            self.choice, self.choice_since = choice, now
        reason = ''
        checks = [
            (not stats.match_id, '等待真实命中事件源'),
            (not config.geometry_confirmed, '等待检测面物理尺寸确认'),
            (config.boundary_policy == 'unconfirmed', '等待核实加固圆边界约定'),
            (not running, '等待新鲜的比赛进行状态'),
            (side_mismatch or stats.side != side, '命中数据与裁判阵营不一致'),
            (not sync_valid or not opportunities_valid, '等待有效的 0x020E 裁判状态'),
            (not opportunities_valid or opportunities == 0, '当前无可用加固机会'),
            (stats.origin != 'live' or stats.source not in config.trusted_sources, '当前数据仅供预览，等待已确认的真实适配器'),
            (now-stats.last_received_at > config.freshness_sec or now-stats.last_source_time > config.freshness_sec, '命中统计心跳超时'),
            (not stats.complete or summary['own_hit_count'] is None or summary['own_hit_count'] < 10, '等待完整统计与己方命中 ≥10 发'),
            (geometry_known and any(h.x > config.plate_width_mm or h.y > config.plate_height_mm for h in incoming), '历史命中与当前检测面几何不一致'),
            (not summary['estimate_valid'], '双方命中率或散布无有效估计'),
            (not legal, '加固半径未知或没有可用的合法毫米圆心'),
            (not enough_training or len(validation) < config.min_validation_samples, '训练或独立验证样本不足'),
            (not active and score.get('coverage_lower', 0) <= 0.5+config.first_margin, '首次加固的覆盖收益下界不足'),
            (active and (center == current or score.get('improvement_lower', -1) <= config.change_margin), '更换圆心的预计改善不足'),
            (self.choice_since is None or now-self.choice_since < config.stable_sec, '等待建议圆心稳定'),
        ]
        for blocked, detail in checks:
            if blocked:
                reason = detail
                break
        command = None
        phase = 'waiting' if reason else 'ready'
        if self.pending:
            phase = self.pending['phase']
            reason = {'pending': '等待裁判回读确认', 'timeout': '请求超时，需核对裁判状态；停止自动重试',
                      'failed': '串口拒绝或写入失败，需核对后重启决策', 'dry_run': '仅完成串口演练，未实际生效'}[phase]
        elif not reason and not config.automatic_requests:
            phase, reason = 'preview', '建议已满足条件；自动请求未启用'
        elif not reason:
            self.sequence += 1
            request_id = f'armor-{stats.match_id[:64]}-{int(now*1_000_000)}-{self.sequence}'
            command = {
                'schema': 'shark.radar.command.v1', 'action': 'fortify_armor',
                'request_id': request_id, 'cumulative_hit_count': summary['own_hit_count'],
                'fortify_center_x_mm': center[0], 'fortify_center_y_mm': center[1],
                'reason': config.mode,
            }
            self.pending = dict(request_id=request_id, center_mm=list(center), sent_at=now,
                                opportunities_before=opportunities, phase='pending', written=False)
            phase, reason = 'pending', '已请求，等待串口与裁判确认'
        self.status = {
            'schema': 'shark.radar.adaptive_armor.v1', 'timestamp': now,
            'match_id': stats.match_id or None, 'source': stats.source or None,
            'origin': stats.origin or None, 'mode': config.mode,
            'config': config.to_dict(), **summary,
            'plate_width_mm': config.plate_width_mm or None,
            'plate_height_mm': config.plate_height_mm or None,
            'plate_area_mm2': config.plate_area_mm2 or None,
            'window_sec': config.window_sec, 'phase': phase, 'reason': reason,
            'statistics_fresh': statistics_fresh,
            'proposal': {
                'center_x_mm': center[0] if center and geometry_known else None,
                'center_y_mm': center[1] if center and geometry_known else None,
                'x_ratio': center[0]/config.plate_width_mm if center and geometry_known else 0.5,
                'y_ratio': center[1]/config.plate_height_mm if center and geometry_known else 0.5,
                'radius_mm': radius, 'radius_source': 'referee' if active and sync_valid else 'estimate',
                'fallback': config.mode == 'adaptive_coverage' and not enough_training,
                'legal': legal, 'train_count': len(training), 'validation_count': len(validation),
                **score, 'confidence_method': 'weighted_time_block_hoeffding',
                'confidence_assumption': '验证时间块近似独立；不保证未来分布不变',
            },
            'pending': dict(self.pending) if self.pending else None,
            'hit_points': [dict(x_mm=h.x, y_mm=h.y, timestamp=h.timestamp)
                           for h in sorted(incoming, key=lambda h: h.timestamp)[-160:]
                           if now-config.window_sec <= h.timestamp <= now],
            'last_result': self.last_result, 'last_error': self.last_error,
        }
        return command
