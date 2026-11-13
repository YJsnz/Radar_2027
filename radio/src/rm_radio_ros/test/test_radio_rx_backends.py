"""Build the real DSP graph against mock hardware; no USB devices are opened."""
from pathlib import Path
import os
import subprocess
import sys


def test_rtl_and_iio_sources_share_dsp_and_keep_hardware_controls_separate():
    directory = Path(__file__).resolve().parents[1] / 'flowgraphs/gfsk'
    script = r'''
import sys,os
import numpy as np
sys.path.insert(0,sys.argv[1])
from PyQt5.QtWidgets import QApplication
from gnuradio import gr,blocks,soapy,iio
from RX import RadioRx
app=QApplication([])
class GainRange:
    def minimum(self):return 0.0
    def maximum(self):return 49.6
class Hardware(gr.hier_block2):
    def __init__(self,backend):
        gr.hier_block2.__init__(self,'mock hardware',gr.io_signature(0,0,0),gr.io_signature(1,1,gr.sizeof_gr_complex))
        self.source=blocks.vector_source_c([.1+.1j]*2048,False)
        self.connect(self.source,self)
        self.backend=backend;self.calls=[];self.gain=0.0
    def get_gain_range(self,ch):return GainRange()
    def set_gain(self,ch,value):self.gain=value;self.calls.append(('gain',ch,value))
    def get_gain(self,ch):return self.gain
    def __getattr__(self,name):
        if name.startswith('set_'):
            def call(*args):self.calls.append((name,*args))
            return call
        raise AttributeError(name)
sources=[]
def factory(backend,*args):
    source=Hardware(backend);sources.append(source);return source
soapy.source=lambda *a:factory('rtl',*a)
iio.fmcomms2_source_fc32=lambda *a:factory('iio',*a)
for profile in ('broadcast','interference'):
    for backend,uri in [('rtl','rtlsdr:00000001'),('iio','ip:synthetic')]:
        os.environ.update(RM_RADIO_RX_URI=uri,RM_RADIO_RX_PROFILE=profile,RM_RADIO_DISABLE_GUI_SINKS='1')
        tb=RadioRx();hw=sources[-1]
        tb.set_sample_rate(2000000)
        tb.set_cen_f(433200000);tb.set_bw_re(540000)
        tb.set_GainMode('manual');tb.set_Gain(32.8);tb.set_rx_tracking(True)
        diag=tb.get_rx_gain_diagnostics()
        assert diag['uri']==uri and diag['gain_db']==32.8
        if backend=='rtl':
            assert diag['hardware_sample_rate']==2048000 and diag['dsp_sample_rate']==2000000
            assert ('set_sample_rate',0,2048000) in hw.calls
            assert ('set_frequency',0,433200000.0) in hw.calls
            assert ('set_gain_mode',0,False) in hw.calls
            assert not any(c[0] in ('set_quadrature','set_rfdc','set_bbdc','set_filter_params','set_samplerate') for c in hw.calls)
            try:tb.set_sample_rate(1000000)
            except ValueError:pass
            else:raise AssertionError('Must reject changing the DSP domain without changing the resampler')
            assert tb.get_sample_rate()==2000000
            samples=[];tb.set_iq_callback(lambda data:samples.extend(data))
            tb.start();tb.wait()
            assert 1900<len(samples)<=2000, len(samples) # actual 125/128 resampler output
        else:
            assert ('set_samplerate',2000000) in hw.calls
            assert ('set_frequency',433200000) in hw.calls
            assert ('set_gain_mode',0,'manual') in hw.calls
            assert ('set_rfdc',True) in hw.calls
        tb.close()
'''
    result = subprocess.run([sys.executable, '-c', script, str(directory)], capture_output=True,
                            text=True, env={**os.environ, 'QT_QPA_PLATFORM': 'offscreen'}, timeout=30)
    assert result.returncode == 0, result.stdout + result.stderr
