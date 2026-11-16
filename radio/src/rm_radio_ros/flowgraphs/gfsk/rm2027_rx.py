"""RM2027 RX1 source/graph lifecycle with a post-decoding ROS adapter.

DSP, acquisition and payload parsing live in the vendored reference files.
No project Python IQ callback or ROS publisher runs on the streaming path.
"""
import hashlib
import os
from pathlib import Path

from PyQt5 import Qt, QtCore
from gnuradio import blocks, gr, iio, qtgui
from gnuradio.fft import window
import sip

from rm_radio_ros.vendor.rm2027.phy import GfskDemodulator, PhyDeframer
from rm_radio_ros.vendor.rm2027.profiles import PROFILES


class RM2027InformationRx(gr.top_block, Qt.QWidget):
    def __init__(self):
        gr.top_block.__init__(self, 'RM Radar Receiver', catch_exceptions=True)
        Qt.QWidget.__init__(self)
        self.rx_uri = os.environ.get('RM_RADIO_RX_URI', 'ip:192.168.2.1').strip()
        if not self.rx_uri.startswith(('ip:', 'usb:')) or any(c.isspace() for c in self.rx_uri):
            raise ValueError('RM2027 RX1 requires an explicit Pluto ip: or usb: URI')
        side = os.environ.get('RM_RADIO_SIDE', 'red').strip().lower()
        self.profile = PROFILES['蓝方信息波' if side == 'blue' else '红方信息波']
        self.samp_rate = 2_000_000
        self.sensitivity = self.profile.sensitivity
        self.Gain = float(os.environ.get('BROADCAST_RX_GAIN', '10'))
        self.receiving = False
        self.source = None
        # Same URI lease name as the supplied GUIs; do not contend with them.
        key = hashlib.sha256(self.rx_uri.lower().encode('utf-8')).hexdigest()
        self.device_lease = QtCore.QLockFile(str(Path(QtCore.QDir.tempPath()) / ('rm2027-pluto-' + key + '.lock')))
        if not self.device_lease.tryLock(0):
            raise RuntimeError('该 Pluto URI 正被 RM2027 窗口使用，请先断开参考程序接收。')
        try:
            self.demod = GfskDemodulator(self.sensitivity, self.profile)
            self.deframer = PhyDeframer(self.profile)
            self.spectrum = qtgui.freq_sink_c(2048, window.WIN_BLACKMAN_hARRIS,
                                              self.profile.frequency_hz, self.samp_rate, '实时频谱', 1, None)
            self.spectrum.set_y_axis(-140, -20)
            self.spectrum.enable_grid(True)
            self.waterfall = qtgui.waterfall_sink_c(1024, window.WIN_BLACKMAN_hARRIS,
                                                   self.profile.frequency_hz, self.samp_rate, '瀑布图', 1, None)
            self.waterfall.set_intensity_range(-140, 10)
            layout = Qt.QVBoxLayout(self)
            layout.addWidget(sip.wrapinstance(self.spectrum.qwidget(), Qt.QWidget))
            layout.addWidget(sip.wrapinstance(self.waterfall.qwidget(), Qt.QWidget))
            # Match Receiver.connect_device() and its ordering exactly.
            self.source = iio.fmcomms2_source_fc32(self.rx_uri, [True, True, False, False], 32768)
            self.source.set_samplerate(self.samp_rate)
            self.source.set_frequency(self.profile.frequency_hz)
            self.source.set_filter_params('Auto', '', 0, 0)
            self.source.set_gain_mode(0, 'manual')
            self.source.set_gain(0, self.Gain)
            self.source.set_quadrature(True)
            self.source.set_rfdc(True)
            self.source.set_bbdc(True)
            self.make_connections()
        except Exception:
            self.disconnect_all()
            self.source = None
            self.device_lease.unlock()
            raise

    def make_connections(self):
        self.connect((self.source, 0), (self.spectrum, 0))
        self.connect((self.source, 0), (self.waterfall, 0))
        self.connect((self.source, 0), (self.demod, 0))
        self.connect((self.demod, 0), (self.deframer, 0))
        # C++ latest-only probe for the Web view; polled by ROS, never calls
        # Python on the streaming thread. It is not a continuous IQ recording.
        self.web_vector = blocks.stream_to_vector(gr.sizeof_gr_complex, 4096)
        self.web_probe = blocks.probe_signal_vc(4096)
        self.connect((self.source, 0), self.web_vector, self.web_probe)

    def start(self, *args, **kwargs):
        result = super().start(*args, **kwargs)
        self.receiving = True
        return result

    def stop(self):
        result = super().stop()
        self.receiving = False
        return result

    def close_receiver(self):
        if self.receiving:
            self.stop()
            self.wait()
        self.disconnect_all()
        self.source = None
        self.device_lease.unlock()

    def apply_reference_setters(self, setters):
        frequency = int(setters.get('cen_f', self.profile.frequency_hz))
        profiles = [p for p in PROFILES.values() if p.kind == 'info' and p.frequency_hz == frequency]
        if not profiles:
            raise ValueError('RM2027 RX1 only accepts the reference information-wave frequencies')
        profile = profiles[0]
        sensitivity = float(setters.get('sen_re', self.sensitivity / 2)) * 2
        gain = float(setters.get('Gain', self.Gain))
        if setters.get('GainMode', 'manual') != 'manual':
            raise ValueError('RM2027 RX1 uses manual gain')
        expected = {'LowPass': 260000, 'bw_re': 540000, 'BW': 540000,
                    'sps': 94, 'sample_rate': 2000000, 'rx_tracking': True,
                    'rx_profile': 'broadcast'}
        for key, value in expected.items():
            if key in setters and setters[key] != value:
                raise ValueError(f'RM2027 RX1 fixes {key}={value}')
        allowed = set(expected) | {'Gain', 'GainMode', 'cen_f', 'center_F', 'sen_re'}
        if set(setters) - allowed:
            raise ValueError(f'unsupported RM2027 RX1 setters: {sorted(set(setters) - allowed)}')
        if profile != self.profile:
            running = self.receiving
            previous_profile, previous_sensitivity = self.profile, self.sensitivity
            if running:
                self.stop()
                self.wait()
            # Same lifecycle as Receiver.apply_profile()/rebuild_receive_chain():
            # no AGC, symbol clock or fragment state crosses a profile change.
            try:
                self.source.set_frequency(profile.frequency_hz)
                self.rebuild_receive_chain(profile, sensitivity)
                if running:
                    self.start()
            except Exception:
                if self.receiving:
                    self.stop()
                    self.wait()
                self.source.set_frequency(previous_profile.frequency_hz)
                self.rebuild_receive_chain(previous_profile, previous_sensitivity)
                if running:
                    self.start()
                raise
        elif sensitivity != self.sensitivity:
            # Matches the reference sensitivity spinbox callback.
            self.demod.set_sensitivity(sensitivity)
            self.sensitivity = sensitivity
        if gain != self.Gain:
            self.source.set_gain(0, gain)
            self.Gain = gain

    def rebuild_receive_chain(self, profile, sensitivity):
        self.disconnect_all()
        self.profile, self.sensitivity = profile, sensitivity
        self.demod = GfskDemodulator(sensitivity, profile)
        self.deframer = PhyDeframer(profile)
        self.make_connections()
        self.spectrum.set_frequency_range(profile.frequency_hz, self.samp_rate)
        self.waterfall.set_frequency_range(profile.frequency_hz, self.samp_rate)

    def take_reference_frames(self):
        return self.deframer.take_frames()

    def reference_status(self):
        return {'implementation': 'RM2027/receiver', 'profile': self.profile.name,
                'frequency_hz': self.profile.frequency_hz, **self.deframer.status()}

    def get_cen_f(self):
        return self.profile.frequency_hz

    def get_demod_mode(self):
        return 'rm2027'

    def get_frontend_profile(self):
        return 'broadcast'

    def get_demod_configuration(self):
        return {'input_sample_rate': self.samp_rate, 'output_sps': 47,
                'output_sensitivity': self.sensitivity, 'implementation': 'RM2027/receiver/phy.py'}

    def get_rx_gain_diagnostics(self):
        return {'profile': 'broadcast', 'backend': 'iio', 'uri': self.rx_uri,
                'hardware_sample_rate': self.samp_rate, 'dsp_sample_rate': self.samp_rate,
                'mode': 'manual', 'gain_db': self.Gain, 'value_source': 'configured',
                'configured_min_db': -1, 'configured_max_db': 73}
