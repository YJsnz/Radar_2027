from pathlib import Path
import sys
from types import SimpleNamespace

import numpy as np
import pytest


pytest.importorskip('gnuradio')
from gnuradio import blocks, digital, filter, gr  # noqa: E402


FLOWGRAPH_DIR = Path(__file__).resolve().parents[1] / 'flowgraphs' / 'gfsk'
PACKAGE_DIR = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PACKAGE_DIR))
sys.path.insert(0, str(FLOWGRAPH_DIR))
from continuous_gfsk_demod import ContinuousGfskDemod  # noqa: E402
from RX import RadioRx  # noqa: E402
from rm_radio_ros.core.rm_protocol import (  # noqa: E402
    ACCESS_CODES,
    AIR_LENGTH_FIELD,
    RADAR_AIR_DATA_LENGTHS,
    AccessCodeAirPacketExtractor,
    RefereeFrameAssembler,
    build_referee_frame,
    bytes_to_bits,
)


BROADCAST_ACCESS = '0010111101101111010011000111010010111001000101000100100100101110'
AIR_LENGTH = '00000000000011110000000000001111'


def _minimum_hamming(bits: np.ndarray, pattern: np.ndarray) -> int:
    if bits.size < pattern.size:
        return int(pattern.size)
    return min(
        int(np.count_nonzero(bits[index : index + pattern.size] != pattern))
        for index in range(bits.size - pattern.size + 1)
    )


def _run_demodulator(
    input_bits: np.ndarray,
    tx_sps: int,
) -> tuple[np.ndarray, ContinuousGfskDemod]:
    input_sensitivity = 1.5628 / 2.0
    top = gr.top_block()
    source = blocks.vector_source_b(input_bits.tolist(), False)
    modulator = digital.gfsk_mod(
        samples_per_symbol=int(tx_sps),
        sensitivity=input_sensitivity,
        bt=0.35,
        verbose=False,
        log=False,
        do_unpack=False,
    )
    demodulator = ContinuousGfskDemod(
        input_sps=94,
        input_sensitivity=input_sensitivity,
        bt=0.35,
    )
    sink = blocks.vector_sink_b()
    top.connect(source, modulator, demodulator, sink)
    top.run()
    return np.asarray(sink.data(), dtype=np.uint8), demodulator


def _demodulated_bits(tx_sps: int) -> tuple[np.ndarray, ContinuousGfskDemod]:
    payload = ''.join(f'{value:08b}' for value in range(15))
    packet = BROADCAST_ACCESS + AIR_LENGTH + payload
    input_bits = np.asarray(
        ([0, 1] * 200) + ([int(bit) for bit in packet] * 6),
        dtype=np.uint8,
    )
    return _run_demodulator(input_bits, tx_sps)


@pytest.mark.parametrize('tx_sps', [93, 94, 95])
def test_continuous_demod_recovers_access_with_symbol_clock_error(tx_sps):
    output, _demodulator = _demodulated_bits(tx_sps)
    access = np.asarray([int(bit) for bit in BROADCAST_ACCESS], dtype=np.uint8)

    # The M&M loop is configured for 94 input samples/symbol, but continues
    # to recover a perfect access code across a +/-1 sample transmitter error.
    assert _minimum_hamming(output, access) == 0


def test_information_receiver_recovers_crc_frames_with_strong_adjacent_channel():
    frame = build_referee_frame(0x0A02, bytes(range(12)), seq=29)
    packet_bits = []
    for offset in range(0, len(frame), 15):
        packet_bits.extend(bytes_to_bits(
            ACCESS_CODES['broadcast'] + b'\x00\x0f\x00\x0f'
            + frame[offset:offset + 15].ljust(15, b'\0')
        ))
    bits = [0, 1] * 200 + packet_bits * 20

    def modulate(values, sensitivity):
        tb = gr.top_block()
        source = blocks.vector_source_b(values, False)
        mod = digital.gfsk_mod(samples_per_symbol=94, sensitivity=sensitivity,
                               bt=.35, do_unpack=False)
        sink = blocks.vector_sink_c()
        tb.connect(source, mod, sink)
        tb.run()
        return np.asarray(sink.data(), dtype=np.complex64)

    signal = modulate(bits, 1.5628 / 2)
    rng = np.random.default_rng(2027)
    neighbour = modulate(rng.integers(0, 2, len(bits)).tolist(), .6517 / 2)
    # A deterministic level-3-like neighbour at -400 kHz, 60 dB stronger.
    # No clipping or random packet fabrication is used to satisfy the test.
    neighbour *= np.exp(-2j * np.pi * 400_000 * np.arange(len(signal)) / 2_000_000)
    iq = 1e-4 * signal + .1 * neighbour
    iq += 1e-6 * (rng.normal(size=len(iq)) + 1j * rng.normal(size=len(iq)))

    ctx = SimpleNamespace(side='red', sample_rate=2_000_000.0, LowPass=260_000.0)
    tb = gr.top_block()
    source = blocks.vector_source_c(iq.tolist(), False)
    bpf = filter.fir_filter_ccf(1, RadioRx._broadcast_band_pass_taps(ctx))
    guard = filter.fir_filter_ccf(1, RadioRx._broadcast_guard_taps(ctx))
    demod = ContinuousGfskDemod()
    sink = blocks.vector_sink_b()
    tb.connect(source, bpf, guard, demod, sink)
    tb.run()
    extractor = AccessCodeAirPacketExtractor(
        access_codes=[ACCESS_CODES['broadcast'][-5:]],
        max_access_hamming=6, max_length_hamming=2,
    )
    assembler = RefereeFrameAssembler(allowed_cmd_lengths=RADAR_AIR_DATA_LENGTHS)
    decoded = []
    output = sink.data()
    for offset in range(0, len(output), 256):
        for payload in extractor.push_bit_bytes(output[offset:offset + 256]):
            decoded.extend(assembler.push_air_payload(payload))
    assert len(decoded) >= 16
    assert all(f.cmd_id == 0x0A02 and f.data == bytes(range(12)) for f in decoded)


def test_rx1_single_band_pass_recovers_broadcast_access_code():
    input_sensitivity = 1.5628 / 2.0
    access = np.asarray([int(bit) for bit in BROADCAST_ACCESS], dtype=np.uint8)
    input_bits = np.asarray(
        ([0, 1] * 200) + (access.tolist() * 8),
        dtype=np.uint8,
    )
    receiver = SimpleNamespace(side='red', sample_rate=2_000_000.0, LowPass=260_000.0)

    top = gr.top_block()
    source = blocks.vector_source_b(input_bits.tolist(), False)
    modulator = digital.gfsk_mod(
        samples_per_symbol=94,
        sensitivity=input_sensitivity,
        bt=0.35,
        verbose=False,
        log=False,
        do_unpack=False,
    )
    band_pass = filter.fir_filter_ccf(
        1,
        RadioRx._broadcast_band_pass_taps(receiver),
    )
    demodulator = ContinuousGfskDemod(
        input_sps=94,
        input_sensitivity=input_sensitivity,
        bt=0.35,
    )
    sink = blocks.vector_sink_b()

    top.connect(source, modulator, band_pass, demodulator, sink)
    top.run()

    output = np.asarray(sink.data(), dtype=np.uint8)
    assert _minimum_hamming(output, access) == 0


def test_continuous_demod_uses_rule_book_rate_after_decimation():
    _output, demodulator = _demodulated_bits(94)
    config = demodulator.configuration()

    assert config['input_sps'] == 94
    assert config['decimation'] == 2
    assert config['output_sps'] == 47
    assert config['output_sensitivity'] == pytest.approx(1.5628)
    assert config['bt'] == pytest.approx(0.35)
    assert config['timing_error_detector'] == 'mueller_and_muller'


def test_continuous_demod_feeds_existing_air_and_crc_parsers():
    frame = build_referee_frame(0x0A02, bytes(range(12)), seq=7)
    padded_stream = frame + bytes(30 - len(frame))
    air_packets = []
    for offset in (0, 15):
        air_packets.extend(bytes_to_bits(ACCESS_CODES['broadcast']))
        air_packets.extend(bytes_to_bits(AIR_LENGTH_FIELD))
        air_packets.extend(bytes_to_bits(padded_stream[offset : offset + 15]))
    input_bits = np.asarray(([0, 1] * 200) + (air_packets * 3), dtype=np.uint8)
    output, _demodulator = _run_demodulator(input_bits, tx_sps=94)

    extractor = AccessCodeAirPacketExtractor(
        access_codes=(ACCESS_CODES['broadcast'],),
        max_access_hamming=3,
        max_length_hamming=0,
        allow_inverted=True,
    )
    payloads = extractor.push_bit_bytes(output)
    assembler = RefereeFrameAssembler(
        max_buffer_size=4096,
        allowed_cmd_lengths=RADAR_AIR_DATA_LENGTHS,
    )
    decoded = []
    for payload in payloads:
        decoded.extend(assembler.push_air_payload(payload))

    assert any(item.cmd_id == 0x0A02 and item.data == bytes(range(12)) for item in decoded)
