"""Exercise the vendored reference, not the former project assembler."""
import os
from pathlib import Path
import subprocess
import sys

import numpy as np
import pytest

pytest.importorskip('gnuradio')

from rm_radio_ros.core.rm2027_adapter import adapt_reference_frame
from rm_radio_ros.core.rm_protocol import build_referee_frame
from rm_radio_ros.vendor.rm2027.phy import PhyDeframer, bits_from_bytes
from rm_radio_ros.vendor.rm2027.profiles import PROFILES, PHY_HEADER
from rm_radio_ros.vendor.rm2027.protocol import crc8_rm


def _air_bits(raw):
    profile = PROFILES['红方信息波']
    padded = raw.ljust((len(raw) + 14) // 15 * 15, b'\0')
    return [bit for offset in range(0, len(padded), 15)
            for bit in bits_from_bytes(profile.access_code + PHY_HEADER + padded[offset:offset + 15])]


@pytest.mark.parametrize('chunk_bits', [1, 22, 256, 20000])
@pytest.mark.parametrize('inverted', [False, True])
def test_reference_recovers_after_invalid_length_with_all_chunk_sizes(chunk_bits, inverted):
    # CRC8-correct false SOF must not block the subsequent valid frames.
    header = bytes((0xA5, 255, 255, 0))
    bits = _air_bits(header + bytes((crc8_rm(header),)))
    originals = [build_referee_frame(0x0A03, bytes((i % 165,)) * 10, seq=i) for i in range(100)]
    for raw in originals:
        bits.extend(_air_bits(raw))
    data = np.asarray(bits, dtype=np.uint8) ^ int(inverted)
    decoder = PhyDeframer(PROFILES['红方信息波'])
    for offset in range(0, len(data), chunk_bits):
        decoder.work([data[offset:offset + chunk_bits]], [])
    frames = decoder.take_frames()
    assert len(frames) == 100
    adapted = [adapt_reference_frame(frame) for frame in frames]
    assert [frame.raw for frame in adapted] == originals
    assert all(frame.to_dict()['rm2027_decoded'] == original.decoded
               for frame, original in zip(adapted, frames))
    assert decoder.status()['valid_frames'] == 100


def test_reference_crc_rejection_and_ros_slot_mapping():
    raw = bytearray(build_referee_frame(0x0A01, bytes(range(24)), seq=7))
    bad = raw.copy()
    bad[8] ^= 1
    decoder = PhyDeframer(PROFILES['红方信息波'])
    decoder.work([np.asarray(_air_bits(bytes(bad)) + _air_bits(bytes(raw)), dtype=np.uint8)], [])
    frames = decoder.take_frames()
    assert len(frames) == 1
    assert decoder.status()['crc16_errors'] == 1
    reference = frames[0].decoded['positions']
    assert [position['name'] for position in reference] == ['重装', '步兵3', '步兵4', '哨兵']
    frame = adapt_reference_frame(frames[0])
    assert frame.raw == bytes(raw)
    assert frame.to_dict()['crc_valid'] is True


def test_reference_graph_initialization_and_reconfiguration_without_hardware():
    directory = Path(__file__).resolve().parents[1] / 'flowgraphs/gfsk'
    script = r'''
import os,sys
from PyQt5.QtWidgets import QApplication
from gnuradio import gr,blocks,iio
sys.path.insert(0,sys.argv[1])
from rm2027_rx import RM2027InformationRx
app=QApplication([])
class Hardware(gr.hier_block2):
    def __init__(self):
        gr.hier_block2.__init__(self,'mock Pluto',gr.io_signature(0,0,0),gr.io_signature(1,1,gr.sizeof_gr_complex))
        self.samples=blocks.vector_source_c([.01+.01j]*32768,False)
        self.connect(self.samples,self)
        self.calls=[]
        self.fail_frequency=None
    def __getattr__(self,name):
        if name.startswith('set_'):
            def call(*args):
                self.calls.append((name,*args))
                if name=='set_frequency' and args[0]==self.fail_frequency:
                    self.fail_frequency=None
                    raise RuntimeError('synthetic tune failure')
            return call
        raise AttributeError(name)
hardware=Hardware()
factory_args=[]
def factory(*args):
    factory_args.append(args)
    return hardware if len(factory_args)==1 else Hardware()
iio.fmcomms2_source_fc32=factory
os.environ.update(RM_RADIO_RX_URI='ip:reference-unittest',RM_RADIO_SIDE='red',BROADCAST_RX_GAIN='10')
tb=RM2027InformationRx()
assert factory_args==[('ip:reference-unittest',[True,True,False,False],32768)]
assert hardware.calls==[('set_samplerate',2000000),('set_frequency',433200000),
    ('set_filter_params','Auto','',0,0),('set_gain_mode',0,'manual'),('set_gain',0,10.),
    ('set_quadrature',True),('set_rfdc',True),('set_bbdc',True)]
assert not hasattr(tb,'set_iq_callback') and not hasattr(tb,'set_demod_bits_callback')
initial=tb.demod
tb.apply_reference_setters({'Gain':12})
assert tb.demod is initial
hardware.fail_frequency=433920000
try:tb.apply_reference_setters({'cen_f':433920000})
except RuntimeError:pass
else:raise AssertionError('tune must fail')
assert tb.get_cen_f()==433200000 and hardware.calls[-1]==('set_frequency',433200000)
tb.apply_reference_setters({'cen_f':433920000})
assert tb.get_cen_f()==433920000 and tb.demod is not initial
assert tb.deframer.status()['valid_frames']==0
tb.start();tb.wait();tb.stop();tb.wait()
tb.close_receiver()
# Closing/reloading must release the same lease used by the supplied GUIs.
other=RM2027InformationRx();other.close_receiver()
'''
    result = subprocess.run([sys.executable, '-X', 'faulthandler', '-c', script, str(directory)],
                            env={**os.environ, 'QT_QPA_PLATFORM': 'offscreen'},
                            capture_output=True, text=True, timeout=30)
    assert result.returncode == 0, result.stdout + result.stderr
