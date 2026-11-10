import json
import math
from dataclasses import replace

import pytest

from rm_radio_ros.core.adaptive_armor import (
    AdaptiveArmorDecision, ArmorConfig, ArmorStatistics, EVENT_SCHEMA, FRAME,
    Hit, best_center, coverages, validation_score,
)
from rm_radio_ros.core.adaptive_armor_replay import replay


def config(**changes):
    return ArmorConfig.from_dict({
        'plate_width_mm': 100, 'plate_height_mm': 100,
        'geometry_confirmed': True, 'boundary_policy': 'clip',
        'automatic_requests': True, 'trusted_sources': ['test-adapter'],
        'window_sec': 180, 'validation_sec': 60, 'stable_sec': 2, **changes,
    })


def event(kind, timestamp=100, **kwargs):
    return dict(schema=EVENT_SCHEMA, kind=kind, timestamp=timestamp,
                match_id='match-1', source='test-adapter', coordinate_frame=FRAME, **kwargs)


def session(stats, cfg, origin='live'):
    stats.ingest(event('session', origin=origin, side='red', complete_from_start=True), 100, cfg)


def referee(now, active=False, center=(50, 50), opportunities=2, radius=20):
    return {
        'last_game_status': {'timestamp': now, 'match_running': True},
        'last_radar_decision_sync': dict(
            timestamp=now, valid_radar_info_length=True, armor_fortify_active=active,
            fortify_center_x_mm=center[0], fortify_center_y_mm=center[1],
            fortify_radius_mm=radius, remaining_fortify_opportunities=opportunities),
    }


def populated(mode='adaptive_coverage', origin='live', center=(10, 85)):
    decision = AdaptiveArmorDecision(config(mode=mode))
    session(decision.statistics, decision.config, origin)
    for i in range(40):
        decision.ingest(event('hit', 110+i, direction='outgoing', event_id=f'B1:0:{i}',
                              confirmed=True, x_mm=40+i%3, y_mm=40+(i//3)%3), 300)
    for i in range(120):
        timestamp = 160+i if i < 70 else 245+(i-70)
        decision.ingest(event('hit', timestamp, direction='incoming', event_id=f'R1:0:{i}',
                              confirmed=True, x_mm=center[0]+i%3,
                              y_mm=center[1]+(i//3)%3), 300)
    heartbeat(decision, 300)
    return decision


def heartbeat(decision, now, **overrides):
    decision.ingest(event('shots', now, own_shots=1000, opponent_shots=1000,
                          data_complete=True, **overrides), now)


def test_empty_defaults_have_relative_center_and_never_send():
    d = AdaptiveArmorDecision()
    assert d.evaluate(100, referee(100), 'red') is None
    assert d.status['own_hit_count'] is None
    assert d.status['proposal']['center_x_mm'] is None
    assert d.status['proposal']['x_ratio'] == 0.5
    assert '真实命中' in d.status['reason']


def test_stats_count_outgoing_gate_separately_and_use_cumulative_median_radius():
    cfg, stats = config(), ArmorStatistics()
    session(stats, cfg)
    for direction in ('outgoing', 'incoming'):
        for i in range(12):
            stats.ingest(event('hit', 110+i, direction=direction, event_id=f'robot:module:{i}',
                               confirmed=True, x_mm=40+10*(i%2), y_mm=50), 150, cfg)
    stats.ingest(event('shots', 150, own_shots=24, opponent_shots=48, data_complete=True), 150, cfg)
    summary = stats.summary(cfg)
    assert summary['own_scored_hit_count'] == summary['own_received_hit_count'] == 12
    assert summary['own_hit_rate'] == 0.5
    assert summary['opponent_hit_rate'] == 0.25
    assert summary['own_spread_mm'] == 5
    assert summary['relative_accuracy'] == 2
    assert summary['area_ratio'] == pytest.approx(2/3)


def test_duplicate_reordered_hits_do_not_double_count_or_refresh_heartbeat():
    cfg, stats = config(), ArmorStatistics()
    session(stats, cfg)
    hit = event('hit', 120, direction='incoming', event_id='R1:module0:65536',
                confirmed=True, x_mm=10, y_mm=10)
    assert stats.ingest(hit, 120, cfg)
    assert not stats.ingest(hit, 300, cfg)
    assert len(stats.hits['incoming']) == 1
    assert stats.last_received_at == 120
    other = dict(hit, timestamp=110, event_id='R1:module1:65536')
    assert stats.ingest(other, 121, cfg)
    assert len(stats.hits['incoming']) == 2


@pytest.mark.parametrize('change', [
    {'x_mm': -1}, {'y_mm': 101}, {'x_mm': float('nan')}, {'x_mm': True},
    {'x_mm': 1.5}, {'confirmed': False}, {'coordinate_frame': 'map_cm'},
    {'timestamp': 301}, {'baseline_damage': 0}, {'match_id': 'wrong'},
])
def test_invalid_hit_does_not_mutate_statistics(change):
    cfg, stats = config(), ArmorStatistics()
    session(stats, cfg)
    hit = event('hit', 120, direction='incoming', event_id='R1:0:1',
                confirmed=True, x_mm=10, y_mm=10)
    with pytest.raises(ValueError):
        stats.ingest(dict(hit, **change), 200, cfg)
    assert not stats.hits['incoming']


def test_missing_and_zero_spread_are_not_replaced_with_epsilon():
    d = populated()
    d.statistics.hits['outgoing'] = [Hit(120, 50, 50, 1)] * 40
    assert d.statistics.summary(d.config)['estimate_valid'] is False
    assert d.evaluate(300, referee(300), 'red') is None


def test_counter_rollback_and_missing_data_cannot_claim_complete_statistics():
    d = populated()
    with pytest.raises(ValueError):
        d.ingest(event('shots', 301, own_shots=999, opponent_shots=1000, data_complete=True), 301)
    d.ingest(event('shots', 301, own_shots=1000, opponent_shots=1000, data_complete=False), 301)
    heartbeat(d, 302)
    assert d.statistics.summary(d.config)['estimate_valid'] is False


def test_maximum_coverage_finds_off_center_hotspot_and_ties_keep_current():
    hits = [Hit(100, 10+i%3, 85+(i//3)%3, 1) for i in range(90)]
    cfg = config(mode='adaptive_coverage')
    c = best_center(hits, cfg, 12, 200)
    scores = coverages(hits, [(50, 50), c], 12, 200, cfg.decay_sec)
    assert scores[0] == 0
    assert scores[1] == pytest.approx(1)
    assert best_center(hits, cfg, 12, 200, (11, 86)) == (11, 86)


def test_containment_can_have_no_solution_without_shrinking_radius():
    cfg = config(boundary_policy='contained')
    assert best_center([Hit(100, 50, 50, 1)], cfg, 60, 100) is None


def test_correlated_burst_does_not_count_as_many_independent_blocks():
    cfg = config()
    burst = [Hit(100, 50, 50, 1)] * 100
    result = validation_score(burst, (50, 50), None, 10, 101, cfg)
    assert result['coverage'] == pytest.approx(1)
    assert result['validation_blocks'] == 1
    assert result['coverage_lower'] < 0.5


def test_holdout_and_outgoing_gate_fixed_center_and_algorithm_differ():
    fixed = populated('fixed_center')
    adaptive = populated()
    assert fixed.evaluate(300, referee(300), 'red') is None
    assert fixed.status['proposal']['coverage'] == 0
    assert adaptive.evaluate(300, referee(300), 'red') is None  # stability gate
    assert adaptive.status['proposal']['coverage'] == pytest.approx(1)
    assert adaptive.status['proposal']['train_count'] == 70
    assert adaptive.status['proposal']['validation_count'] == 50
    heartbeat(adaptive, 303)
    command = adaptive.evaluate(303, referee(303), 'red')
    assert command['action'] == 'fortify_armor'
    assert command['cumulative_hit_count'] == 40  # NOT 120 incoming hits
    assert command['fortify_center_x_mm'] < 40
    assert json.loads(json.dumps(adaptive.status, allow_nan=False))


@pytest.mark.parametrize('origin', ['synthetic', 'replay'])
def test_replay_and_synthetic_origins_never_issue_commands(origin):
    d = populated(origin=origin)
    d.evaluate(300, referee(300), 'red')
    heartbeat(d, 303)
    assert d.evaluate(303, referee(303), 'red') is None
    assert d.pending is None
    assert '预览' in d.status['reason']


@pytest.mark.parametrize('change', [
    {'trusted_sources': ()}, {'geometry_confirmed': False},
    {'boundary_policy': 'unconfirmed'}, {'automatic_requests': False},
])
def test_live_requests_require_verified_configuration(change):
    d = populated()
    d.configure(replace(d.config, **change))
    d.evaluate(300, referee(300), 'red')
    heartbeat(d, 303)
    assert d.evaluate(303, referee(303), 'red') is None


@pytest.mark.parametrize('case', ['stale', 'future', 'wrong_side', 'zero_opportunities', 'nine_hits'])
def test_request_gates(case):
    d = populated()
    d.evaluate(300, referee(300), 'red')
    heartbeat(d, 303)
    r = referee(303)
    side = 'red'
    if case == 'stale':
        r = referee(299)
    elif case == 'future':
        r = referee(304)
    elif case == 'wrong_side':
        side = 'blue'
    elif case == 'zero_opportunities':
        r = referee(303, opportunities=0)
    else:
        d.statistics.hits['outgoing'] = d.statistics.hits['outgoing'][:9]
    assert d.evaluate(303, r, side) is None


def test_pending_only_confirms_after_written_and_fresh_matching_referee_consumption():
    d = populated()
    d.evaluate(300, referee(300), 'red')
    heartbeat(d, 303)
    command = d.evaluate(303, referee(303), 'red')
    center = (command['fortify_center_x_mm'], command['fortify_center_y_mm'])
    r = referee(304, active=True, center=center, opportunities=1)
    d.evaluate(304, r, 'red')
    assert d.pending is not None  # a matching old/unrelated circle is insufficient
    d.acknowledge(dict(request_id=command['request_id'], written=True))
    d.evaluate(304, r, 'red')
    assert d.pending is None
    assert d.last_result['phase'] == 'confirmed'


def test_timeout_mode_change_and_stale_game_never_retry_pending_request():
    d = populated()
    d.evaluate(300, referee(300), 'red')
    heartbeat(d, 303)
    command = d.evaluate(303, referee(303), 'red')
    d.configure(replace(d.config, mode='fixed_center'))
    assert d.evaluate(310, {}, 'red') is None
    assert d.pending['phase'] == 'timeout'
    d.configure(replace(d.config, mode='adaptive_coverage'))
    heartbeat(d, 313)
    assert d.evaluate(313, referee(313), 'red') is None
    assert d.pending['request_id'] == command['request_id']


def test_match_end_resets_cumulative_stats_and_pending():
    d = populated()
    d.evaluate(300, referee(300), 'red')
    r = referee(301)
    r['last_game_status']['match_running'] = False
    d.evaluate(301, r, 'red')
    assert d.statistics.match_id == ''
    assert d.status['own_hit_count'] is None


def test_existing_armor_uses_actual_referee_radius_and_compares_current_center():
    d = populated()
    d.evaluate(300, referee(300, active=True, radius=20), 'red')
    heartbeat(d, 303)
    command = d.evaluate(303, referee(303, active=True, radius=20), 'red')
    assert command is not None
    assert d.status['proposal']['radius_mm'] == 20
    assert d.status['proposal']['radius_source'] == 'referee'
    assert d.status['proposal']['current_coverage'] == 0
    assert d.status['proposal']['improvement_lower'] > d.config.change_margin


def test_stale_input_does_not_publish_current_area_estimate():
    d = populated()
    d.evaluate(310, referee(310), 'red')
    assert d.status['statistics_fresh'] is False
    assert d.status['estimate_valid'] is False
    assert d.status['proposal']['radius_mm'] is None


def test_recorded_status_config_does_not_reconstruct_hits_from_display_points():
    rows = [{'kind': 'topic', 'topic': '/rm_radar_algorithm/adaptive_armor', 'timestamp': 100,
             'data': {'config': config(mode='adaptive_coverage').to_dict(),
                      'hit_points': [{'x_mm': 50, 'y_mm': 50}]}}]
    status = list(replay(rows))[0]
    assert status['mode'] == 'adaptive_coverage'
    assert status['own_received_hit_count'] is None


def test_damage_weights_cannot_silently_mix_unknown_and_known_damage():
    cfg, stats = config(), ArmorStatistics()
    stats.ingest(event('session', origin='live', side='red', complete_from_start=True,
                       weight_basis='baseline_damage'), 100, cfg)
    hit = event('hit', 110, direction='incoming', event_id='R1:0:1',
                confirmed=True, x_mm=20, y_mm=20)
    with pytest.raises(ValueError, match='每发均提供'):
        stats.ingest(hit, 110, cfg)
    stats.ingest(dict(hit, baseline_damage=100), 110, cfg)
    assert stats.hits['incoming'][0].damage == 100
    other = ArmorStatistics()
    session(other, cfg)
    other.ingest(dict(hit, baseline_damage=100), 110, cfg)
    assert other.hits['incoming'][0].damage == 1  # count mode always stays comparable


@pytest.mark.parametrize('change', [
    {'mode': 'cluster'}, {'automatic_requests': 'false'}, {'plate_width_mm': float('nan')},
    {'confidence': 1}, {'unknown': 1}, {'validation_sec': 120},
])
def test_configuration_rejects_invalid_values(change):
    with pytest.raises(ValueError):
        ArmorConfig.from_dict(change)


def test_recorded_live_session_is_replay_only_and_reproducible():
    rows = [
        {'kind': 'event', 'timestamp': 100, 'data': event(
            'session', origin='live', side='red', complete_from_start=True)},
        {'kind': 'referee', 'timestamp': 101, 'data': referee(101)},
        {'kind': 'tick', 'timestamp': 102},
    ]
    first = list(replay(rows, config()))
    assert json.dumps(first, sort_keys=True) == json.dumps(list(replay(rows, config())), sort_keys=True)
    assert all(s['origin'] == 'replay' and s['pending'] is None for s in first)
    assert rows[0]['data']['origin'] == 'live'  # do not mutate recorded evidence


def test_recording_topic_envelope_and_rejected_arrival_time_rollback():
    row = {'kind': 'topic', 'topic': '/rm_radar_algorithm/armor_events', 'timestamp': 100,
           'data': event('session', origin='live', side='blue', complete_from_start=False)}
    assert list(replay([row], config()))[0]['origin'] == 'replay'
    with pytest.raises(ValueError):
        list(replay([row, {'kind': 'tick', 'timestamp': 99}], config()))
