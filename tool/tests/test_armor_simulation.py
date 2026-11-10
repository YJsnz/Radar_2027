import json

import pytest

from tool.armor_simulation import SimulatedMatch
from rm_radio_ros.core.adaptive_armor import AdaptiveArmorDecision, ArmorConfig
from rm_radio_ros.core.adaptive_armor_replay import replay


def prepared(origin='synthetic'):
    match = SimulatedMatch(100, 100, 'red', 1000)
    config = ArmorConfig.from_dict({'mode': 'adaptive_coverage', 'plate_width_mm': 100,
                                   'plate_height_mm': 100})
    decision = AdaptiveArmorDecision(config)
    event = match.session()
    event['origin'] = origin
    decision.ingest(event, 1000)
    return match, decision


def test_example_uses_existing_event_contract_and_radar_calculates_circle():
    match, decision = prepared()
    for event in match.example(1000, 15, 85, 3):
        decision.ingest(event, 1000)
    decision.ingest(match.heartbeat(1000), 1000)
    assert decision.evaluate(1000, {}, 'red') is None
    status = decision.status
    assert status['origin'] == 'synthetic'
    assert status['own_scored_hit_count'] == 40
    assert status['own_received_hit_count'] == 100
    assert match.shots == {'outgoing': 200, 'incoming': 500}
    assert status['estimate_valid'] is True
    assert status['proposal']['train_count'] == 70
    assert status['proposal']['validation_count'] == 30
    assert status['proposal']['radius_mm'] > 0
    assert status['proposal']['center_x_mm'] < 40
    assert status['proposal']['center_y_mm'] > 60
    assert decision.pending is None


@pytest.mark.parametrize('change', [('incoming', -1, 0, 5), ('incoming', 101, 0, 5),
                                   ('outgoing', 1.5, 0, 5), ('incoming', 0, 0, 0),
                                   ('invalid', 0, 0, 5)])
def test_invalid_manual_hit_cannot_change_local_counters(change):
    match, _ = prepared()
    with pytest.raises(ValueError):
        match.hit(*change[:3], 1000, shots=change[3])
    assert match.sequence == 0
    assert not any(match.shots.values())


def test_unique_ids_and_same_coordinates_are_still_distinct_hits():
    match, decision = prepared()
    events = [match.hit('incoming', 10, 20, 1000) for _ in range(3)]
    assert len({e['event_id'] for e in events}) == 3
    for event in events:
        decision.ingest(event, 1000)
    decision.ingest(events[0], 1000)
    assert len(decision.statistics.hits['incoming']) == 3
    assert decision.statistics.duplicates == 1


def test_matching_simulation_can_end_and_start_again_without_fake_referee():
    match, decision = prepared()
    decision.ingest(match.hit('incoming', 10, 20, 1000), 1000)
    decision.ingest(match.event('end_session', 1001), 1001)
    assert not decision.statistics.match_id
    second = SimulatedMatch(100, 100, 'blue', 1002)
    decision.ingest(second.session(), 1002)
    assert decision.statistics.match_id == second.match_id


@pytest.mark.parametrize('case', ['live', 'wrong_source', 'wrong_match', 'future', 'frame', 'pending'])
def test_end_event_never_clears_live_unrelated_or_unresolved_session(case):
    match, decision = prepared('live' if case == 'live' else 'synthetic')
    event = match.event('end_session', 1001)
    if case == 'wrong_source': event['source'] = 'other'
    if case == 'wrong_match': event['match_id'] = 'other'
    if case == 'future': event['timestamp'] = 1002
    if case == 'frame': event['coordinate_frame'] = 'map_cm'
    if case == 'pending': decision.pending = {'request_id': 'unresolved'}
    with pytest.raises(ValueError):
        decision.ingest(event, 1001)
    assert decision.statistics.match_id == match.match_id


def test_recorded_end_event_is_replayable_and_preserves_input_evidence():
    match, _ = prepared()
    rows = [{'kind': 'event', 'timestamp': 1000, 'data': match.session()},
            {'kind': 'event', 'timestamp': 1001, 'data': match.event('end_session', 1001)}]
    states = list(replay(rows))
    assert states[0]['origin'] == 'replay'
    assert states[1]['match_id'] is None
    assert rows[0]['data']['origin'] == 'synthetic'
    assert json.dumps(states, allow_nan=False)
