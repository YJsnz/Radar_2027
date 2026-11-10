import gzip
import json
from dataclasses import replace
from pathlib import Path

import pytest

from rm_radio_ros.core.adaptive_armor import AdaptiveArmorDecision, Hit, legal_center
from rm_radio_ros.core.adaptive_armor_benchmark import METHODS, benchmark, candidate_centers, main
from test_adaptive_armor import config, event, referee


def rows(origin='synthetic', future_center=(11, 86), weight_basis='count'):
    result = [{'kind': 'event', 'timestamp': 100, 'data': event(
        'session', origin=origin, side='red', complete_from_start=True, weight_basis=weight_basis)}]
    for i in range(24):
        for direction, center in (('incoming', (10, 85)), ('outgoing', (40, 40))):
            result.append({'kind': 'event', 'timestamp': 101+i//2, 'data': event(
                'hit', 101+i//2, direction=direction, event_id=f'{direction}:{i}', confirmed=True,
                x_mm=center[0]+i%3, y_mm=center[1]+i//3%3, baseline_damage=1)})
    result.extend([
        {'kind': 'referee', 'timestamp': 120, 'data': referee(120, active=True, radius=5)},
        {'kind': 'event', 'timestamp': 120, 'data': event(
            'shots', 120, own_shots=1000, opponent_shots=1000, data_complete=True)},
    ])
    for i in range(10):
        result.append({'kind': 'event', 'timestamp': 121+i, 'data': event(
            'hit', 121+i, direction='incoming', event_id=f'future:{i}', confirmed=True,
            x_mm=future_center[0], y_mm=future_center[1], baseline_damage=1)})
    return result


def run(data=None, **kwargs):
    return benchmark(rows() if data is None else data, config(), horizon_sec=10, **kwargs)


def two_sessions():
    data = rows()
    ended = referee(131)
    ended['last_game_status']['match_running'] = False
    data.append({'kind': 'referee', 'timestamp': 131, 'data': ended})
    def shift(value):
        if not isinstance(value, dict):
            return value
        return {k: v+100 if k == 'timestamp' else 'match-2' if k == 'match_id'
                else shift(v) for k, v in value.items()}
    data.extend(shift(row) for row in rows(future_center=(90, 10)))
    return data


def test_past_only_methods_compare_same_frozen_radius_on_future_hits():
    report = run()
    session = report['sessions'][0]
    assert session['recorded_origin'] == 'synthetic'
    assert session['evaluation_origin'] == 'replay'
    window = session['windows'][0]
    assert window['cutoff'] == 120
    assert window['end'] == 130
    assert window['radius_mm'] == 5
    assert window['radius_source'] == 'referee'
    assert window['train_count'] == 24
    assert window['future_hit_count'] == 10
    assert window['evaluated'] is True
    assert window['methods']['fixed_center']['future_coverage'] == 0
    assert window['methods']['maximum_coverage']['future_coverage'] == 1
    assert window['methods']['kde_coverage']['future_coverage'] == 1
    for method in METHODS:
        result = window['methods'][method]
        assert result['expected_damage_multiplier'] == 1.5-result['future_coverage']
        assert session['summary'][method]['evaluated_windows'] == 1


def test_future_drift_changes_scores_but_cannot_change_saved_centers():
    first = run()['sessions'][0]['windows'][0]
    shifted = run(rows(future_center=(90, 10)))['sessions'][0]['windows'][0]
    for name in METHODS:
        assert first['methods'][name]['center_mm'] == shifted['methods'][name]['center_mm']
        assert shifted['methods'][name]['future_coverage'] == 0


def test_duplicates_and_delayed_historical_hits_are_not_future_evidence():
    data = rows()
    data.insert(-1, {**data[2], 'timestamp': 129})
    data.insert(-1, {'kind': 'event', 'timestamp': 129, 'data': event(
        'hit', 110, direction='incoming', event_id='late-old-hit', confirmed=True, x_mm=50, y_mm=50)})
    window = run(data)['sessions'][0]['windows'][0]
    assert window['future_hit_count'] == 10
    assert window['methods']['fixed_center']['future_coverage'] == 0


def test_late_arriving_future_outcome_is_scored_without_retraining_prediction():
    data = rows()
    data.append({'kind': 'event', 'timestamp': 140, 'data': event(
        'hit', 125, direction='incoming', event_id='late-future-hit', confirmed=True, x_mm=50, y_mm=50)})
    window = run(data)['sessions'][0]['windows'][0]
    assert window['future_hit_count'] == 11
    assert window['methods']['maximum_coverage']['future_coverage'] == pytest.approx(10/11)


def test_clock_lead_known_at_cutoff_is_not_future_evidence():
    data = rows()
    data.insert(-10, {'kind': 'event', 'timestamp': 120, 'data': event(
        'hit', 120.4, direction='incoming', event_id='early-clock-lead', confirmed=True,
        x_mm=50, y_mm=50)})
    window = run(data)['sessions'][0]['windows'][0]
    assert window['train_count'] == 24
    assert window['future_hit_count'] == 10


@pytest.mark.parametrize('case', ['partial', 'empty', 'incomplete', 'configuration'])
def test_invalid_or_censored_future_window_is_not_reported_as_zero_coverage(case):
    data = rows()
    if case == 'partial':
        data.pop()
    elif case == 'empty':
        data = data[:-10] + [{'kind': 'tick', 'timestamp': 130}]
    elif case == 'incomplete':
        data.insert(-1, {'kind': 'event', 'timestamp': 129, 'data': event(
            'shots', 129, own_shots=1000, opponent_shots=1000, data_complete=False)})
    else:
        data.insert(-1, {'kind': 'config', 'timestamp': 129,
                         'data': config(plate_width_mm=101).to_dict()})
    session = run(data)['sessions'][0]
    window = session['windows'][0]
    assert window['evaluated'] is False
    for name in METHODS:
        assert window['methods'][name]['future_coverage'] is None
        assert session['summary'][name]['evaluated_windows'] == 0
        assert session['summary'][name]['future_coverage'] is None


def test_damage_weighted_future_scoring_is_not_time_decayed():
    data = rows(weight_basis='baseline_damage')
    # An uncovered, heavier, late projectile outweighs all ten covered ones.
    data.append({'kind': 'event', 'timestamp': 130, 'data': event(
        'hit', 130, direction='incoming', event_id='heavy', confirmed=True,
        x_mm=50, y_mm=50, baseline_damage=90)})
    session = run(data)['sessions'][0]
    scores = session['summary']
    assert session['weight_basis'] == 'baseline_damage'
    assert scores['maximum_coverage']['future_coverage'] == pytest.approx(0.1)
    assert scores['fixed_center']['future_coverage'] == pytest.approx(0.9)


def test_config_change_at_same_arrival_time_after_forecast_invalidates_window():
    data = rows()
    data.insert(-10, {'kind': 'config', 'timestamp': 120,
                      'data': config(plate_width_mm=101).to_dict()})
    window = run(data)['sessions'][0]['windows'][0]
    assert window['configuration_changed'] is True
    assert window['evaluated'] is False


@pytest.mark.parametrize('boundary', ['clip', 'contained'])
def test_all_candidates_are_integer_mm_and_honor_same_geometry(boundary):
    cfg = config(boundary_policy=boundary, plate_width_mm=80, plate_height_mm=40)
    hits = [Hit(100+i, i%3, 39, 1) for i in range(12)]
    for center in candidate_centers(hits, cfg, 10, 120).values():
        assert center is not None
        assert all(isinstance(x, int) for x in center)
        assert legal_center(cfg, 10, center)


def test_no_contained_solution_is_explicit_and_never_shrinks_radius():
    cfg = config(boundary_policy='contained')
    centers = candidate_centers([Hit(100, 50, 50, 1)], cfg, 60, 110)
    assert all(c is None for c in centers.values())


def test_short_decay_with_old_history_still_has_finite_candidates():
    cfg = config(decay_sec=1)
    centers = candidate_centers([Hit(100, 10+i%2, 85, 1) for i in range(12)], cfg, 5, 1000)
    assert all(c is not None for c in centers.values())
    mixed = candidate_centers([Hit(100, 10, 85, 1), Hit(1000, 90, 10, 1)], cfg, 5, 1000)
    assert all(c is not None for c in mixed.values())


def test_sessions_are_separate_and_the_future_never_crosses_match_end():
    report = run(two_sessions())
    assert [s['match_id'] for s in report['sessions']] == ['match-1', 'match-2']
    assert report['sessions'][0]['summary']['maximum_coverage']['future_coverage'] == 1
    assert report['sessions'][1]['summary']['maximum_coverage']['future_coverage'] == 0
    assert all(s['recorded_origin'] == 'synthetic' for s in report['sessions'])


def test_early_match_end_cannot_borrow_next_sessions_time_or_origin():
    data = [row for row in rows() if row['timestamp'] <= 125]
    ended = referee(126)
    ended['last_game_status']['match_running'] = False
    data.append({'kind': 'referee', 'timestamp': 126, 'data': ended})
    for row in two_sessions():
        if row['timestamp'] >= 200:
            if row.get('data', {}).get('kind') == 'session':
                row['data']['origin'] = 'live'
            data.append(row)
    first, second = run(data)['sessions']
    assert first['recorded_origin'] == 'synthetic'
    assert second['recorded_origin'] == 'live'
    assert first['observed_until'] == 126
    assert first['windows'][0]['evaluated'] is False
    assert first['summary']['maximum_coverage']['future_coverage'] is None
    assert second['summary']['maximum_coverage']['future_coverage'] == 0


def test_replay_cannot_emit_requests_even_for_recorded_live_input(monkeypatch):
    original = AdaptiveArmorDecision.evaluate
    def assert_no_command(self, *args, **kwargs):
        assert original(self, *args, **kwargs) is None
        assert self.pending is None
    monkeypatch.setattr(AdaptiveArmorDecision, 'evaluate', assert_no_command)
    report = run(rows(origin='live'))
    assert report['sessions'][0]['recorded_origin'] == 'live'
    assert 'rclpy' not in vars(__import__('rm_radio_ros.core.adaptive_armor_benchmark', fromlist=['']))


def test_unconfirmed_geometry_and_missing_raw_events_have_no_fabricated_scores():
    report = benchmark(rows(), replace(config(), geometry_confirmed=False), horizon_sec=10)
    assert report['sessions'][0]['windows'] == []
    assert report['sessions'][0]['skipped_reasons']
    with pytest.raises(ValueError, match='session'):
        benchmark([{'kind': 'tick', 'timestamp': 100}], config())


def test_report_is_deterministic_and_json_finite():
    first = json.dumps(run(), sort_keys=True, allow_nan=False)
    assert first == json.dumps(run(), sort_keys=True, allow_nan=False)


def test_committed_synthetic_fixture_has_stable_and_drifting_sessions():
    path = Path(__file__).parent / 'fixtures/adaptive_armor/synthetic_comparison.jsonl'
    report = benchmark([json.loads(line) for line in path.read_text().splitlines()], horizon_sec=10)
    first, drift = report['sessions']
    assert first['recorded_origin'] == drift['recorded_origin'] == 'synthetic'
    assert first['summary']['maximum_coverage']['future_coverage'] == 1
    assert drift['summary']['maximum_coverage']['future_coverage'] == 0
    assert first['summary']['maximum_coverage']['opportunities_consumed'] is None


def test_cli_gzip_input_and_snapshot_configuration(tmp_path, monkeypatch):
    path = tmp_path / 'events.jsonl.gz'
    with gzip.open(path, 'wt') as stream:
        for row in rows():
            # Same envelope as the existing recorder for source events.
            if row['kind'] == 'event':
                row = dict(row, kind='topic', topic='/rm_radar_algorithm/armor_events')
            stream.write(json.dumps(row) + '\n')
    (tmp_path / 'adaptive_armor_config_snapshot.json').write_text(json.dumps(config().to_dict()))
    output = tmp_path / 'report.json'
    monkeypatch.setattr('sys.argv', ['benchmark', '--input', str(path), '--output', str(output),
                                    '--horizon-sec', '10'])
    main()
    assert json.loads(output.read_text()) == json.loads(json.dumps(run()))


@pytest.mark.parametrize('change', [{'horizon_sec': 0}, {'horizon_sec': float('nan')},
                                   {'clusters': 0}, {'bandwidth_mm': 0}])
def test_invalid_parameters_fail_before_processing(change):
    with pytest.raises(ValueError):
        benchmark([], config(), **change)
