import threading

import numpy as np
from gnuradio import analog, digital, filter, gr
from gnuradio.filter import firdes
from gnuradio.fft import window
from .profiles import BT, PHY_FRAGMENT_BYTES, PHY_HEADER, SAMPLE_RATE, SPS
from .protocol import RadarProtocolDecoder


DEMOD_INPUT_RATE = 2_000_000


def bits_from_bytes(data: bytes) -> list[int]:
    return [
        (value >> shift) & 1
        for value in data
        for shift in range(7, -1, -1)
    ]


def bytes_from_bits(bits: list[int]) -> bytes:
    result = bytearray()

    for offset in range(0, len(bits), 8):
        value = 0

        for bit in bits[offset:offset + 8]:
            value = (value << 1) | (int(bit) & 1)

        result.append(value)

    return bytes(result)


class GfskDemodulator(gr.hier_block2):
    """
    输入：2 MS/s complex IQ
    输出：每个样点为一个硬判决 bit（0 或 1）
    """

    def __init__(self, sensitivity: float, profile=None):
        gr.hier_block2.__init__(
            self,
            "RM Radar 2-GFSK Demodulator",
            gr.io_signature(1, 1, gr.sizeof_gr_complex),
            gr.io_signature(1, 1, gr.sizeof_char),
        )

        self.sensitivity = float(sensitivity)
        self.is_information = profile is None or profile.kind == "info"
        if not self.is_information:
            # SHARK RX2: two low-pass stages, then the installed GNU Radio
            # gfsk_demod (legacy path). Keep 2 MS/s here: a 500 kHz cutoff
            # must not be designed at a 1 MS/s sample rate (Nyquist boundary).
            self.lowpass_cutoff_hz = 160_000 if profile.bandwidth_hz <= 250_000 else 500_000
            taps = firdes.low_pass(1.0, DEMOD_INPUT_RATE, self.lowpass_cutoff_hz,
                                   50_000, window.WIN_HAMMING, 6.76)
            self.channel_filter = filter.fir_filter_ccf(1, taps)
            self.channel_filter_second = filter.fir_filter_ccf(1, taps)
            self.legacy = digital.gfsk_demod(
                samples_per_symbol=SPS * 2,
                sensitivity=self.sensitivity / 2.0,
            )
            self.connect(self, self.channel_filter, self.channel_filter_second,
                         self.legacy, self)
            return

        # SHARK RX1: real taps preserve BOTH [-260,-5] and [5,260] kHz.
        # Select the weak information channel before digital AGC/discrimination.
        self.channel_filter = filter.fir_filter_ccf(1, firdes.band_pass(
            1.0, DEMOD_INPUT_RATE, 5_000.0, 260_000.0, 50_000.0,
            window.WIN_BLACKMAN_hARRIS, 6.76,
        ))
        # Extra channel isolation beyond the report's 50 kHz BPF transition.
        # A strong level-2/3 neighbour can leak through that transition before
        # the nonlinear discriminator; suppress it before either digital AGC.
        self.adjacent_guard = filter.fir_filter_ccf(1, firdes.low_pass(
            1.0, DEMOD_INPUT_RATE, 260_000.0, 20_000.0,
            window.WIN_BLACKMAN_hARRIS, 6.76,
        ))
        # This additional decimator filter prevents aliasing at 2 -> 1 MS/s;
        # it is not the channel-select filter.
        anti_alias_taps = firdes.low_pass(
            1.0,
            DEMOD_INPUT_RATE,
            470_000.0,
            20_000.0,
            window.WIN_HAMMING,
            6.76,
        )

        self.sensitivity = float(sensitivity)

        self.complex_agc = analog.agc_cc(1e-4, 1.0, 1.0)
        self.complex_agc.set_max_gain(4.0)

        self.decimator = filter.rational_resampler_ccc(
            interpolation=1,
            decimation=2,
            taps=anti_alias_taps,
            fractional_bw=0.0,
        )

        self.discriminator = analog.quadrature_demod_cf(
            1.0 / self.sensitivity
        )

        self.gaussian_filter = filter.fir_filter_fff(
            1,
            firdes.gaussian(1.2, SPS, BT, 60),
        )

        self.float_agc = analog.agc_ff(1e-4, 1.0, 1.0)
        self.float_agc.set_max_gain(65536.0)

        self.symbol_sync = digital.symbol_sync_ff(
            digital.TED_MOD_MUELLER_AND_MULLER,
            SPS,
            0.046,
            1.0,
            0.85,
            1.5,
            1,
            digital.constellation_bpsk().base(),
            digital.IR_PFB_NO_MF,
            32,
            [],
        )

        self.slicer = digital.binary_slicer_fb()

        self.connect(
            self,
            self.channel_filter,
            self.adjacent_guard,
            self.complex_agc,
            self.decimator,
            self.discriminator,
            self.gaussian_filter,
            self.float_agc,
            self.symbol_sync,
            self.slicer,
            self,
        )

    def set_sensitivity(self, sensitivity: float) -> None:
        self.sensitivity = float(sensitivity)
        if self.is_information:
            self.discriminator.set_gain(1.0 / self.sensitivity)
        else:
            self.legacy.fmdemod.set_gain(2.0 / self.sensitivity)


class PhyDeframer(gr.sync_block):
    """
    接收硬判决 bit，恢复：

        Access Code (8 B)
        00 0F 00 0F (4 B)
        Fragment (15 B)

    然后将 Fragment 交给 RadarProtocolDecoder，
    重组成 A5 / CRC8 / cmd_id / data / CRC16 标准雷达帧。
    """

    def __init__(self, profile):
        gr.sync_block.__init__(
            self,
            name="RM Radar PHY Deframer",
            in_sig=[np.uint8],
            out_sig=None,
        )

        self.lock = threading.Lock()

        self.bits: list[int] = []
        self.frames = []

        self.access_count = 0
        self.header_reject_count = 0
        self.fragment_count = 0
        self.best_access_distance = 64
        self.best_access_hex = "--"
        self.best_access_inverted = False

        self.protocol = RadarProtocolDecoder()

        self.set_profile(profile)

    def set_profile(self, profile) -> None:
        self.profile = profile
        # With the full-deviation level-1 PAN3120 waveform, timing acquisition
        # consistently corrupts the first three access bytes (the capture
        # repeatedly showed 55 77 15 1C 71 2D instead of
        # D3 77 15 1C 71 2D).  The final five bytes are already stable.  A
        # 40-bit suffix with <=6 errors is still followed by the fixed PHY
        # header and referee CRCs, which independently reject false matches.
        self.access_bytes = profile.access_code[-5:]
        self.access_max_errors = 6
        self.access_bits = bits_from_bytes(self.access_bytes)
        self.inverted_access_bits = [
            1 - value for value in self.access_bits
        ]

        self.header_bits = bits_from_bytes(PHY_HEADER)

        self.bits = []
        self.protocol.reset()

        with self.lock:
            self.frames = []
            self.access_count = 0
            self.header_reject_count = 0
            self.fragment_count = 0
            self.best_access_distance = 64
            self.best_access_hex = "--"
            self.best_access_inverted = False

    @staticmethod
    def distance(left: list[int], right: list[int]) -> int:
        return sum(
            int(left_bit) != int(right_bit)
            for left_bit, right_bit in zip(left, right)
        )

    def find_access(self):
        access_len = len(self.access_bits)
        last_start = len(self.bits) - access_len

        for start in range(last_start + 1):
            candidate = self.bits[start:start + access_len]
            normal_distance = self.distance(candidate, self.access_bits)
            inverted_distance = self.distance(
                candidate, self.inverted_access_bits
            )

            candidate_best = min(normal_distance, inverted_distance)
            if candidate_best < self.best_access_distance:
                candidate_inverted = inverted_distance < normal_distance
                normalized_candidate = [
                    bit ^ int(candidate_inverted) for bit in candidate
                ]
                with self.lock:
                    self.best_access_distance = candidate_best
                    self.best_access_hex = bytes_from_bits(
                        normalized_candidate
                    ).hex().upper()
                    self.best_access_inverted = candidate_inverted

            if normal_distance <= self.access_max_errors:
                return start, False

            if inverted_distance <= self.access_max_errors:
                return start, True

        return None

    def extract_fragments(self) -> None:
        access_len = len(self.access_bits)

        total_bits = (
            len(self.access_bytes)
            + len(PHY_HEADER)
            + PHY_FRAGMENT_BYTES
        ) * 8

        while True:
            found = self.find_access()

            if found is None:
                # 保留尾部，避免接入码跨越两次 GNU Radio 回调。
                if len(self.bits) > total_bits - 1:
                    self.bits = self.bits[-(total_bits - 1):]

                return

            start, inverted = found

            if start:
                del self.bits[:start]

            if len(self.bits) < total_bits:
                return

            normalized = [
                bit ^ int(inverted)
                for bit in self.bits[:total_bits]
            ]

            header_start = access_len
            header_end = header_start + len(self.header_bits)

            # At +/-448.7 kHz the PAN3120 operates outside its documented
            # modulation-index range, so an otherwise aligned fixed header may
            # contain an occasional hard-decision error.  Accept at most two;
            # the 40-bit access marker and referee CRCs still gate valid data.
            header_distance = self.distance(
                normalized[header_start:header_end], self.header_bits
            )
            if header_distance > 2:
                with self.lock:
                    self.header_reject_count += 1

                del self.bits[0]
                continue

            fragment_bits = normalized[header_end:total_bits]
            fragment = bytes_from_bits(fragment_bits)

            frames = self.protocol.feed_fragment(fragment)

            with self.lock:
                self.access_count += 1
                self.fragment_count += 1
                self.frames.extend(frames)

            del self.bits[:total_bits]

    def take_frames(self):
        """由主界面定时调用，取走已解析的业务帧。"""
        with self.lock:
            frames = self.frames[:]
            self.frames.clear()

        return frames

    def status(self) -> dict:
        with self.lock:
            return {
                "access": self.access_count,
                "best_access_distance": self.best_access_distance,
                "access_bits": len(self.access_bits),
                "best_access_hex": self.best_access_hex,
                "best_access_inverted": self.best_access_inverted,
                "fragments": self.fragment_count,
                "header_reject": self.header_reject_count,
                "crc8_errors": self.protocol.crc8_errors,
                "crc16_errors": self.protocol.crc16_errors,
                "valid_frames": self.protocol.valid_frames,
            }

    def work(self, input_items, output_items):
        self.bits.extend(
            int(value) & 1
            for value in input_items[0]
        )

        self.extract_fragments()

        return len(input_items[0])
