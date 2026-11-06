"""
rfsoc_dashboard.py  --  interactive AWG control panel + scope for rfsoc_awg.py
==============================================================================

Tabs: DAC_A, DAC_B (per-slot settings) and "All (common)", whose single control set is
copied to all four DAC/slot pulses at once.

Gives you sliders / number boxes for every pulse, a live plot of what the DACs
will output, and a scope + spectrum view (of the AWG preview or of a real ADC
capture).  Works inside Jupyter, or as a clean web page with Voila.

Install once on the board (needs internet):
    pip3 install ipywidgets plotly anywidget voila

Use (in a notebook cell):
    from rfsoc_awg import *
    from rfsoc_dashboard import AWGDashboard
    awg = AWG(window=6.65*us)
    awg.connect()
    dash = AWGDashboard(awg)          # add capture=my_capture_function for the ADC scope
    dash.ui                           # <- last line of the cell shows the dashboard

Serve as a web app (code cells hidden, any browser on your network):
    voila AWG_dashboard.ipynb --Voila.ip=0.0.0.0 --port=8866

Hooking up the ADC scope
------------------------
Option 1 (recommended): pass your `AdcScope` object -- `AWGDashboard(awg, scope=sc)`.
  It must offer  sc.configure(trigger=..., level=..., pre_ns=..., length_ns=...)
  and            t_ns, y = sc.acquire(timeout=...)     (y in fraction of ADC full scale)
  The dashboard then shows trigger / level / pre-trigger / length controls.
  If you know the ADC full-scale peak voltage, pass `adc_vfs=<volts>` to get volts and
  dBm; otherwise the scope shows full-scale units (FS) and dBFS.
Option 2: pass `capture=f`, where `f()` returns `(samples_in_volts, fs_hz)`.

Then pick "ADC capture" in the scope's Source menu and press Run.
`DemoScope(awg)` is a simulated ADC with the same API for testing without hardware.
"""

from __future__ import annotations

import inspect
import threading
import time
import warnings
from typing import Callable, Optional, Tuple

import numpy as np
import ipywidgets as w
import plotly.graph_objects as go
from plotly.subplots import make_subplots

from rfsoc_awg import (DAC_SR, LOAD_OHMS, MAX_DBM, MAX_VPK, STREAM_ORDER, Pulse, dbm_to_vpk, vpp_to_dbm,
                       render_pulses, us, ns, MHz)

__all__ = ["AWGDashboard", "DemoScope"]

SLOT_NAMES = (("z", 0), ("x", 1))
# Starting values: the four continuous tones of the original notebook.
DEFAULTS = {("DAC_B", "z"): (20, -25), ("DAC_B", "x"): (40, -15),
            ("DAC_A", "z"): (60, -20), ("DAC_A", "x"): (70, -10)}   # (MHz, dBm)


# --------------------------------------------------------------------------- #
#  Plot helpers
# --------------------------------------------------------------------------- #
def _decimate(t: np.ndarray, y: np.ndarray, max_pts: int = 4000):
    """Min/max decimation: keeps every peak visible while sending <= max_pts
    points to the browser (the full 65k-sample waveform would be sluggish)."""
    n = len(y)
    if n <= max_pts:
        return t, y
    edges = np.linspace(0, n, max_pts // 2 + 1).astype(int)[:-1]
    lo, hi = np.minimum.reduceat(y, edges), np.maximum.reduceat(y, edges)
    tt = np.repeat(t[edges], 2)
    yy = np.empty(2 * len(edges))
    yy[0::2], yy[1::2] = lo, hi
    return tt, yy


def _spectrum_dbm(y: np.ndarray, fs: float, unit: str = "V"):
    """Amplitude spectrum.  Returns (freq [MHz], level).
    unit "V":  y in volts, level in dBm (a sine of power P dBm into 50 ohm peaks at P dBm).
    unit "FS": y in fractions of full scale, level in dBFS (a full-scale sine = 0 dBFS)."""
    win = np.hanning(len(y))
    apk = 2 * np.abs(np.fft.rfft((y - y.mean()) * win)) / win.sum()
    apk = np.maximum(apk, 1e-9)
    level = (10 * np.log10(apk ** 2 / (2 * LOAD_OHMS) / 1e-3) if unit == "V"
             else 20 * np.log10(apk))
    return np.fft.rfftfreq(len(y), 1 / fs) / 1e6, level


class DemoScope:
    """Simulated ADC with the same API as `AdcScope`, for testing without hardware.
    It 'measures' DAC_A (as if looped back): the slot that is playing, repeated
    every window like the real hardware, plus noise.  t = 0 at the slot start."""

    MODES = ["awg2", "awg0", "immediate", "rising", "falling", "ctrl_rise", "ctrl_fall", "manual"]

    def __init__(self, awg, fs: float = 2.4576e9, noise: float = 2e-3, latency_ns: float = 35.0):
        self.awg, self.fs, self.noise, self.latency_ns = awg, fs, noise, latency_ns
        self.cfg = dict(trigger="awg2", level=0.0, pre_ns=10, length_ns=1000)
        self.last = {}

    def configure(self, trigger="awg2", level=0.0, pre_ns=10, length_ns=1000):
        self.cfg = dict(trigger=trigger, level=level, pre_ns=pre_ns, length_ns=length_ns)

    def calibrate_awg0(self, *args, **kw):
        """Simulated calibration: returns (offset_ns, spread_ns), always locked."""
        time.sleep(0.2)
        return -self.latency_ns, 0.0

    def acquire(self, timeout=5.0, **kw):
        c = self.cfg
        n = max(2, int(c["length_ns"] * 1e-9 * self.fs))
        t_ns = np.arange(n) / self.fs * 1e9 - c["pre_ns"]
        v = self.awg._waves.get(("DAC_A", self.awg.slot_playing), np.zeros(self.awg.n_samples))
        if self.awg.state not in ("playing", "stale") or not self.awg._running:
            v = np.zeros(self.awg.n_samples)                  # outputs off -> nothing to see
        period = self.awg.max_points * 16                     # samples per loop
        idx = np.round((t_ns - self.latency_ns) * 1e-9 * DAC_SR).astype(int) % period   # DAC->ADC delay
        y = np.where(idx < len(v), v[np.clip(idx, 0, len(v) - 1)], 0.0) + self.noise * np.random.randn(n)
        time.sleep(0.05)
        self.last = {"triggered": True, **c}
        return t_ns, y


# --------------------------------------------------------------------------- #
#  The dashboard
# --------------------------------------------------------------------------- #
class AWGDashboard:
    """
    Interactive front-end for an `AWG` object.  Show it with `dash.ui`.

    awg      an `rfsoc_awg.AWG` (connected, or dry_run=True to just play around)
    scope    your `AdcScope` (configure()/acquire()) -> full ADC scope with trigger controls
    capture  alternative: function returning (samples_in_volts, fs_hz)
    adc_vfs  ADC full-scale PEAK voltage [V]; None = show FS units / dBFS
    template plotly theme: "plotly_white" (default) or "plotly_dark"
    """

    def __init__(self, awg, scope=None,
                 capture: Optional[Callable[[], Tuple[np.ndarray, float]]] = None,
                 adc_vfs: Optional[float] = None, template: str = "plotly_white"):
        self.awg, self.scope, self.capture, self.template = awg, scope, capture, template
        self.adc_vfs = adc_vfs
        self.has_adc = scope is not None or capture is not None
        self._cfg = None                        # last configuration sent to the scope
        self._scope_thread: Optional[threading.Thread] = None
        self._scope_stop = threading.Event()
        self._busy = False                      # blocks re-entrant updates
        self._lock = threading.RLock()          # one hardware access at a time (upload vs scope)
        self._build_controls()
        self._apply(draw=False)                 # push defaults to the AWG first ...
        self._build_plots()                     # ... so the figures are BORN with data
        self._build_ui()                        # (data added to a figure before it is
                                                #  displayed would never reach the browser)

    # ---------------------------------------------------------------- controls
    @staticmethod
    def _num(label, value, lo, hi, step, unit):
        """Slider + exact number box, linked in the browser.  Returns (row, slider)."""
        slider = w.FloatSlider(value=value, min=lo, max=hi, step=step, continuous_update=False,
                               readout=False, layout=w.Layout(width="190px"))
        box = w.BoundedFloatText(value=value, min=lo, max=hi, step=step,
                                 layout=w.Layout(width="95px"))
        w.jslink((slider, "value"), (box, "value"))
        row = w.HBox([w.Label(label, layout=w.Layout(width="95px")), slider, box,
                      w.Label(unit, layout=w.Layout(width="38px"))])
        return row, slider

    def _build_controls(self):
        win_us = self.awg.window / us
        win_ns = self.awg.window / ns
        self.ctl = {}                           # (channel, slot) -> dict of widgets
        tabs, titles = [], []
        for ch in ("DAC_A", "DAC_B"):
            boxes = []
            for name, _ in SLOT_NAMES:
                f0, p0 = DEFAULTS[(ch, name)]
                en = w.Checkbox(value=True, description=f"slot {name}: play", indent=False)
                r_f, f = self._num("Frequency", f0, 0, 2000, 0.1, "MHz")     # 0 = baseband pulse
                r_p, p = self._num("Amplitude", round(dbm_to_vpk(p0), 4),
                                   -round(MAX_VPK, 4), round(MAX_VPK, 4), 0.001, "V")  # signed peak
                r_s, s = self._num("Start", 0, 0, win_ns, 0.1, "ns")
                r_d, d = self._num("Duration", win_ns, 0.1, win_ns, 0.1, "ns")
                r_ph, ph = self._num("Phase", 0, -180, 180, 1, "deg")
                shape = w.Dropdown(options=["rect", "gauss", "cos", "flat"], value="rect",
                                   description="Shape", layout=w.Layout(width="190px"))
                rise = w.BoundedFloatText(value=20, min=1, max=10000, step=1, description="Rise [ns]",
                                          layout=w.Layout(width="190px"))
                info = w.HTML()
                wd = dict(en=en, f=f, p=p, s=s, d=d, ph=ph, shape=shape, rise=rise)
                self.ctl[(ch, name)] = wd
                for wid in wd.values():
                    wid.observe(self._on_change, names="value")
                wd["info"] = info
                boxes.append(w.VBox([en, r_f, r_p, r_s, r_d, r_ph,
                                     w.HBox([shape, rise]), info],
                                    layout=w.Layout(border="1px solid #8888", padding="6px",
                                                    margin="0 8px 0 0")))
            tabs.append(w.HBox(boxes))
            titles.append(ch)

        # ---- third tab: ONE set of controls that drives all four DAC/slot control sets
        en = w.Checkbox(value=True, description="all slots: play", indent=False)
        r_f, f = self._num("Frequency", 20, 0, 2000, 0.1, "MHz")
        r_p, p = self._num("Amplitude", round(dbm_to_vpk(-20), 4),
                           -round(MAX_VPK, 4), round(MAX_VPK, 4), 0.001, "V")
        r_s, s = self._num("Start", 0, 0, win_ns, 0.1, "ns")
        r_d, d = self._num("Duration", win_ns, 0.1, win_ns, 0.1, "ns")
        r_ph, ph = self._num("Phase", 0, -180, 180, 1, "deg")
        shape = w.Dropdown(options=["rect", "gauss", "cos", "flat"], value="rect",
                           description="Shape", layout=w.Layout(width="190px"))
        rise = w.BoundedFloatText(value=20, min=1, max=10000, step=1, description="Rise [ns]",
                                  layout=w.Layout(width="190px"))
        self.common = dict(en=en, f=f, p=p, s=s, d=d, ph=ph, shape=shape, rise=rise)
        for wid in self.common.values():
            wid.observe(self._on_common, names="value")
        note = w.HTML("<span style='color:#7f8c8d;font-size:12px'>Any change here is copied to "
                      "DAC_A and DAC_B, slots z and x, and uploaded as one update.</span>")
        tabs.append(w.HBox([w.VBox([note, en, r_f, r_p, r_s, r_d, r_ph, w.HBox([shape, rise])],
                                   layout=w.Layout(border="1px solid #8888", padding="6px",
                                                   margin="0 8px 0 0"))]))
        titles.append("All (common)")

        self.tabs = w.Tab(children=tabs)
        for i, t in enumerate(titles):
            self.tabs.set_title(i, t)

        self.range = w.FloatRangeSlider(value=[0, win_us], min=0, max=win_us, step=0.001,
                                        description="Zoom [µs]", continuous_update=False,
                                        layout=w.Layout(width="650px"))
        self.range.observe(lambda _: self._draw(), names="value")

        self.btn_play = w.Button(description="Upload & play", button_style="success", icon="play")
        self.btn_stop = w.Button(description="Stop", button_style="danger", icon="stop")
        self.live = w.Checkbox(value=False, description="Live: re-upload on every change", indent=False)
        self.btn_play.on_click(lambda _: self._hw(self.awg.play, "upload & play"))
        self.btn_stop.on_click(lambda _: self._hw(self.awg.stop, "stop"))
        self.slot_sel = w.ToggleButtons(options=[("slot z", 0), ("slot x", 1)],
                                        value=self.awg.slot_playing, description="Playing:",
                                        tooltip="Instant switch (axi_control), no re-upload",
                                        style={"button_width": "70px", "description_width": "60px"})
        self.slot_sel.observe(self._on_slot, names="value")
        self.live.observe(lambda ch: ch["new"] and self._hw(self.awg.play, "live: upload & play"),
                          names="value")
        self.status = w.HTML()

        slots = [f"{c} {s}" for c in STREAM_ORDER for s, _ in SLOT_NAMES]
        sources = slots + (["ADC capture"] if self.has_adc else [])
        self.source = w.Dropdown(options=sources, value=sources[-1] if self.has_adc else sources[0],
                                 description="Scope source", style={"description_width": "90px"})
        self.source.observe(lambda _: (self._draw_scope_preview(), self._update_overlay()), names="value")
        self.run = w.ToggleButton(value=False, description="Run scope", icon="play",
                                  disabled=not self.has_adc,
                                  tooltip="Needs scope=... in AWGDashboard(...)")
        # trigger / acquisition settings (only used with scope=)
        narrow = w.Layout(width="150px")
        modes = self._trigger_modes()
        self.trig = w.Dropdown(value="awg2" if "awg2" in modes else modes[0], options=modes,
                               description="Trigger", layout=w.Layout(width="170px"),
                               style={"description_width": "55px"})
        self.level = w.BoundedFloatText(value=-0.05, min=-1, max=1, step=0.01, description="Level [FS]",
                                        layout=w.Layout(width="150px"), style={"description_width": "70px"})
        self.pre_ns = w.BoundedFloatText(value=10, min=-1e6, max=1e6, step=1, description="Pre [ns]",
                                         layout=narrow, style={"description_width": "55px"})
        self.len_ns = w.BoundedFloatText(value=1000, min=1, max=1e6, step=10, description="Length [ns]",
                                         layout=w.Layout(width="165px"), style={"description_width": "75px"})
        self.timeout = w.BoundedFloatText(value=0.5, min=0.05, max=600, step=0.5, description="Timeout [s]",
                                          layout=w.Layout(width="160px"), style={"description_width": "80px"})
        self.adc_ch = w.Dropdown(options=[("ADC 0", 0), ("ADC 1", 1)], value=0, description="ADC",
                                 layout=w.Layout(width="130px"), style={"description_width": "35px"},
                                 tooltip="Which ADC channel is displayed and used for calibration")
        self.adc_ch.observe(lambda _: setattr(self, "_cfg", None), names="value")
        self.cal_pulse = w.Checkbox(value=True, indent=False,
                                    description="Calibrate with a 10 ns baseband pulse (restores your settings after)")
        self.btn_cal = w.Button(description="Calibrate", icon="crosshairs",
                                tooltip="Repeat scope.calibrate_awg0() until locked (spread <= 0.01 ns), then "
                                        "set trigger=awg0 and Pre = -offset")
        self.btn_cal.on_click(lambda _: self._calibrate())
        self.cal_info = w.HTML()
        self.overlay = w.Dropdown(options=["none"] + slots, value="none", description="Overlay planned",
                                  style={"description_width": "105px"})
        self.overlay.observe(lambda _: self._update_overlay(), names="value")
        self.offset = w.BoundedFloatText(value=0.0, min=-1e5, max=1e5, step=0.1, description="Delay [ns]",
                                         layout=w.Layout(width="160px"), style={"description_width": "70px"})
        self.offset.observe(lambda _: self._update_overlay(), names="value")
        self.btn_align = w.Button(description="Auto-align", icon="arrows-h",
                                  tooltip="Find the DAC->ADC delay by cross-correlating the last capture "
                                          "with the planned waveform")
        self.btn_align.on_click(lambda _: self._auto_align())
        self._last_capture = None               # (t_us, y) of the last ADC capture
        self.run.observe(self._on_run, names="value")
        self.autoplay = w.Checkbox(value=True, indent=False,
                                   description="Auto-play the AWG when the scope starts")
        self.rate = w.FloatSlider(value=5, min=0.5, max=20, step=0.5, description="Refresh [Hz]",
                                  style={"description_width": "90px"})

    # ------------------------------------------------------------------- plots
    def _build_plots(self):
        lay = dict(template=self.template, margin=dict(l=55, r=15, t=35, b=40), height=380)
        # --- programmed output: 4 traces, created with their data
        t0, t1 = self.range.value
        traces = []
        for (ch, name), (x, y) in zip(self._slots(), self._out_xy()):
            traces.append(go.Scatter(x=x, y=y, name=self._trace_name(ch, name), mode="lines",
                                     line=dict(width=1), opacity=self._trace_opacity(name)))
        red = dict(color="red", dash="dot", width=1)
        self.fig_out = go.FigureWidget(
            data=traces,
            layout=dict(lay, legend=dict(orientation="h", y=-0.22), title="AWG output (programmed)",
                        xaxis=dict(title="time [µs]", range=[t0, t1]), yaxis_title="voltage [V]",
                        hovermode="x",
                        shapes=[dict(type="line", xref="paper", x0=0, x1=1, y0=v, y1=v, line=red)
                                for v in (MAX_VPK, -MAX_VPK)]))
        # --- scope: time (row 1) + spectrum (row 2)
        base = make_subplots(rows=2, cols=1, vertical_spacing=0.18,
                             specs=[[{"secondary_y": True}], [{}]])
        t_us, y, fs, title = self._scope_preview_data()
        f_mhz, dbm = _spectrum_dbm(y, fs)
        base.add_trace(go.Scatter(x=_decimate(t_us, y)[0], y=_decimate(t_us, y)[1], mode="lines",
                                  line=dict(width=1)), row=1, col=1)
        base.add_trace(go.Scatter(x=f_mhz, y=dbm, mode="lines", line=dict(width=1)), row=2, col=1)
        base.add_trace(go.Scatter(x=[], y=[], mode="lines", name="planned",   # overlay (index 2)
                                  line=dict(width=1, color="#e67e22", dash="dot")),
                       row=1, col=1, secondary_y=True)
        base.update_layout(**dict(lay, margin=dict(l=55, r=55, t=35, b=40)), showlegend=False, title=title)
        base.update_yaxes(title_text="planned DAC [V]", showgrid=False, range=[-0.3, 0.3],
                          secondary_y=True, row=1, col=1)
        base.update_xaxes(title_text="time [µs]", row=1, col=1)
        base.update_yaxes(title_text="voltage [V]", secondary_y=False, row=1, col=1)
        base.update_xaxes(title_text="frequency [MHz]", row=2, col=1)
        base.update_yaxes(title_text="level [dBm]", range=[-110, 0], row=2, col=1)
        self.fig_scope = go.FigureWidget(base)

    def _build_ui(self):
        hw = w.HBox([self.btn_play, self.btn_stop, self.slot_sel, self.live])
        scope_bar = [w.HBox([self.source, self.run, self.rate]), self.autoplay]
        if self.scope is not None:
            scope_bar += [w.HBox([self.adc_ch, self.trig, self.level, self.pre_ns, self.len_ns, self.timeout]),
                          w.HBox([self.btn_cal, self.cal_pulse, self.cal_info]),
                          w.HBox([self.overlay, self.offset, self.btn_align])]
        self.ui = w.VBox([
            w.HTML("<h2>RFSoC 4x2 AWG &amp; scope</h2>"),
            self.tabs, hw, self.status, self.range,
            w.HBox([self.fig_out, w.VBox(scope_bar + [self.fig_scope])]),
        ])

    # ---------------------------------------------------------------- callbacks
    def _on_change(self, _change):
        if self._busy:
            return
        self._apply()
        if self.live.value:
            self._hw(self.awg.play, "live upload")

    def _on_common(self, _change):
        """Common tab: copy every common value into all four DAC/slot control sets,
        then apply (and re-upload if Live) exactly once."""
        if self._busy:
            return
        self._busy = True                       # silence the per-slot observers during the copy
        try:
            for c in self.ctl.values():
                for key, wid in self.common.items():
                    c[key].value = wid.value
        finally:
            self._busy = False
        self._apply()
        if self.live.value:
            self._hw(self.awg.play, "live upload")

    def _apply(self, draw=True):
        """Read widgets -> Pulse objects -> AWG, then redraw."""
        msgs, win = [], self.awg.window
        for (ch, name), c in self.ctl.items():
            if not c["en"].value:
                self.awg.set_waveform(ch, name, None)
                continue
            amp = c["p"].value
            c["info"].value = (f"<span style='color:#7f8c8d;font-size:12px'>"
                               f"{'baseband pulse (0 MHz)' if c['f'].value == 0 else 'RF pulse'} · "
                               f"peak {amp*1e3:.1f} mV"
                               + (f" ≙ {vpp_to_dbm(2*abs(amp)):.1f} dBm sine" if amp else "")
                               + "</span>")
            start, dur = c["s"].value * ns, c["d"].value * ns
            if start + dur >= win - 0.5 / DAC_SR:           # reaches the end -> "until end"
                dur = None
            try:
                self.awg.set_waveform(ch, name, Pulse(
                    freq=c["f"].value * MHz, amp=amp, start=start, duration=dur,
                    phase_deg=c["ph"].value, shape=c["shape"].value, rise=c["rise"].value * ns))
            except ValueError as exc:
                self.awg.set_waveform(ch, name, None)
                msgs.append(f"<b>{ch} {name}:</b> {exc}")
        self._set_status(msgs)
        if draw:
            self._draw()
            self._draw_scope_preview()

    def _hw(self, action, label=None):
        """Run a hardware action (never at the same time as a scope capture),
        showing any error in the status line."""
        label = label or getattr(action, "__name__", "action")
        try:
            with self._lock, warnings.catch_warnings(record=True) as caught:
                warnings.simplefilter("always")
                action()
            msgs = [str(x.message) for x in caught
                    if "close no longer functional" not in str(x.message)]   # PYNQ-internal noise
            self._set_status(msgs, ok=f"{label} done at {time.strftime('%H:%M:%S')} "
                                      f"- AWG: {self._state_text()}")
            self._draw()                                    # playing-slot highlight
        except Exception as exc:                            # show, don't crash the page
            self._set_status([f"{type(exc).__name__}: {exc}"])

    def _on_slot(self, change):
        self._hw(lambda: self.awg.select_slot(change["new"]), f"slot {'zx'[change['new']]}")

    def _trace_name(self, ch, name):
        return f"{ch} {name}" + (" ▶" if dict(SLOT_NAMES)[name] == self.awg.slot_playing else "")

    def _trace_opacity(self, name):
        return 1.0 if dict(SLOT_NAMES)[name] == self.awg.slot_playing else 0.3

    def _trigger_modes(self):
        """Trigger names offered by the scope (AdcScope's MODES), with a fallback list."""
        import sys
        mod = sys.modules.get(type(self.scope).__module__) if self.scope is not None else None
        modes = getattr(self.scope, "MODES", None) or getattr(mod, "MODES", None)
        return list(modes) if modes else ["awg2", "awg0", "immediate", "rising", "falling"]

    def _state_text(self):
        return {"playing": f"PLAYING slot {'zx'[self.awg.slot_playing]}",
                "stale": "playing OLD waveform - press Upload & play",
                "ready": "uploaded, outputs off", "stopped": "stopped (nothing sent to the DAC yet)"
                }[self.awg.state]

    def _set_status(self, msgs, ok=None):
        if msgs:
            self.status.value = "".join(f"<div style='color:#c0392b'>⚠ {m}</div>" for m in msgs)
        else:
            self.status.value = (f"<div style='color:#27ae60'>✔ {ok}</div>" if ok else
                                 f"<div style='color:#7f8c8d'>AWG: {self._state_text()} · "
                                 f"window {self.awg.window/us:.4f} µs · {self.awg.n_samples} samples · "
                                 f"MAX_POINTS {self.awg.max_points}</div>")

    # ------------------------------------------------------------------ drawing
    @staticmethod
    def _slots():
        return [(c, s) for c in STREAM_ORDER for s, _ in SLOT_NAMES]

    def _slot_volts(self, ch, name):
        return self.awg._waves.get((ch, dict(SLOT_NAMES)[name]))

    def _out_xy(self):
        """(x, y) for each of the 4 slots, zoomed to the range slider and decimated."""
        t0, t1 = self.range.value
        sel = slice(int(t0 * us * DAC_SR), int(np.ceil(t1 * us * DAC_SR)) + 1)
        t = self.awg.t[sel] / us
        out = []
        for ch, name in self._slots():
            v = self._slot_volts(ch, name)
            out.append(([], []) if v is None else _decimate(t, v[sel]))
        return out

    def _draw(self):
        """Programmed AWG output, zoomed to the range slider."""
        with self.fig_out.batch_update():
            for trace, (ch, name), (x, y) in zip(self.fig_out.data, self._slots(), self._out_xy()):
                trace.x, trace.y = x, y
                trace.name, trace.opacity = self._trace_name(ch, name), self._trace_opacity(name)
            self.fig_out.update_xaxes(range=list(self.range.value))

    def _show_scope(self, t_us, y, fs, unit="V"):
        f_mhz, level = _spectrum_dbm(y, fs, unit)
        with self.fig_scope.batch_update():
            self.fig_scope.data[0].x, self.fig_scope.data[0].y = _decimate(t_us, y)
            self.fig_scope.data[1].x, self.fig_scope.data[1].y = f_mhz, level
            self.fig_scope.update_yaxes(title_text="voltage [V]" if unit == "V" else "ADC [FS]",
                                        secondary_y=False, row=1, col=1)
            self.fig_scope.update_yaxes(title_text="level [dBm]" if unit == "V" else "level [dBFS]",
                                        row=2, col=1)

    def _update_overlay(self):
        """Planned DAC waveform (orange, right axis) on top of the ADC trace.
        The slot loops forever, so it is repeated over the whole capture.  It is shifted by
        the 'Delay' (DAC pipeline + converters + cable); use Auto-align to measure it."""
        tr = self.fig_scope.data[2]
        if self.source.value != "ADC capture" or self.overlay.value == "none":
            tr.x, tr.y = [], []
            return
        ch, name = self.overlay.value.split()
        v = self._slot_volts(ch, name)
        if v is None:
            tr.x, tr.y = [], []
            return
        win_us = self.awg.window / us
        t_end = self._last_capture[0][-1] if self._last_capture else win_us
        t_beg = self._last_capture[0][0] if self._last_capture else 0.0
        delay_us = self.offset.value / 1e3
        k0 = int(np.floor((t_beg - delay_us) / win_us))
        k1 = int(np.ceil((t_end - delay_us) / win_us))
        k0, k1 = max(k0, -50), min(max(k1, k0 + 1), k0 + 60)          # keep it light
        t_one = self.awg.t / us
        xs = np.concatenate([t_one + k * win_us + delay_us for k in range(k0, k1)])
        ys = np.tile(v, k1 - k0)
        keep = (xs >= t_beg) & (xs <= t_end)
        tr.x, tr.y = _decimate(xs[keep], ys[keep]) if keep.any() else ([], [])

    def _auto_align(self):
        """Estimate the DAC->ADC delay: slide the (periodic) planned waveform over the last
        capture and keep the shift with the best normalised correlation."""
        if self._last_capture is None or self.overlay.value == "none":
            self._set_status(["Auto-align: capture first (Run scope on 'ADC capture') and pick "
                              "a slot in 'Overlay planned'."])
            return
        ch, name = self.overlay.value.split()
        v = self._slot_volts(ch, name)
        t_ns, y = self._last_capture[0] * 1e3, self._last_capture[1]
        if v is None or np.ptp(y) == 0 or not np.any(v):
            self._set_status(["Auto-align: nothing to correlate (flat capture or empty slot)."])
            return
        win_ns, tv = self.awg.window / ns, self.awg.t / ns
        y0 = y - y.mean()
        best = (-2.0, 0.0)
        for d in np.arange(-50.0, min(400.0, win_ns), 0.1):          # candidate delays [ns]
            p = np.interp((t_ns - d) % win_ns, tv, v)
            p = p - p.mean()
            den = np.linalg.norm(p) * np.linalg.norm(y0)
            if den > 0:
                c = float(p @ y0 / den)
                if c > best[0]:
                    best = (c, float(d))
        self.offset.value = round(best[1], 1)
        self._set_status([] if best[0] > 0.5 else
                         [f"Auto-align: weak match (correlation {best[0]:.2f}); check the loopback."],
                         ok=f"delay DAC→ADC = {best[1]:.1f} ns (correlation {best[0]:.2f})")

    def _scope_preview_data(self):
        """Computed scope data for the selected slot -> (t_us, volts, fs, title)."""
        src = self.source.value
        if src == "ADC capture":                    # nothing measured yet: show first slot
            src = f"{STREAM_ORDER[0]} z"
        ch, name = src.split()
        v = self._slot_volts(ch, name)
        if v is None:
            v = np.zeros(self.awg.n_samples)
        return self.awg.t / us, v, DAC_SR, f"Scope – {src} (computed, not measured)"

    def _draw_scope_preview(self):
        """Scope in 'computed' mode: show the programmed waveform of one slot."""
        if self.source.value == "ADC capture":
            return
        t_us, y, fs, title = self._scope_preview_data()
        self.fig_scope.update_layout(title=title)
        self._show_scope(t_us, y, fs)

    # ---------------------------------------------------------------- ADC scope
    def _calibrate(self, tol_ns: float = 0.01, max_tries: int = 50):
        """Repeat scope.calibrate_awg0() until locked (spread <= tol_ns), then switch the
        scope to trigger='awg0' with Pre = -offset (same as the manual script)."""
        if self.scope is None or not hasattr(self.scope, "calibrate_awg0"):
            self._set_status(["Calibrate: the scope object has no calibrate_awg0()."])
            return
        self.btn_cal.disabled = True
        self.cal_info.value = "<span style='color:#7f8c8d'>calibrating...</span>"

        def work():
            used_pulse = False
            try:
                ch = self.adc_ch.value
                if self.cal_pulse.value:
                    # A continuous tone has no edge to lock on: play a short baseband pulse instead.
                    cal = Pulse(freq=0.0, amp=0.1, start=0.0, duration=10 * ns, shape="rect")
                    for dac in ("DAC_A", "DAC_B"):
                        for name, _ in SLOT_NAMES:
                            self.awg.set_waveform(dac, name, cal)
                    used_pulse = True
                    self._hw(self.awg.play, "calibration pulse")
                elif self.autoplay.value and self.awg.state != "playing":
                    self._hw(self.awg.play)         # the DAC must be running to calibrate
                kw = {}
                try:                                # V5 AdcScope.calibrate_awg0(channel=...) if present
                    if "channel" in inspect.signature(self.scope.calibrate_awg0).parameters:
                        kw["channel"] = ch
                except (TypeError, ValueError):
                    pass
                with self._lock:                    # no scope capture during calibration
                    # calibration must trigger on the AWG start, not on whatever the scope was left on
                    self.scope.configure(trigger="awg0", pre_ns=max(self.pre_ns.value, 20.0),
                                         length_ns=max(self.len_ns.value, 200.0))
                    offset_ns, spread_ns = self.scope.calibrate_awg0(**kw)
                    tries = 1
                    while spread_ns > tol_ns and tries < max_tries:     # not locked -> again
                        offset_ns, spread_ns = self.scope.calibrate_awg0(**kw)
                        tries += 1
                locked = spread_ns <= tol_ns
                modes = list(self.trig.options)
                if "awg0" in modes:
                    self.trig.value = "awg0"
                self.pre_ns.value = -offset_ns
                self._cfg = None                    # force re-configure on the next capture
                color = "#27ae60" if locked else "#c0392b"
                self.cal_info.value = (f"<span style='color:{color}'>offset DAC→ADC = {offset_ns:.3f} ns, "
                                       f"spread {spread_ns:.4f} ns, {tries} attempt(s)"
                                       f"{'' if locked else ' - NOT locked' }</span>")
                self._set_status([] if locked else
                                 [f"Calibrate: not locked after {tries} attempts (spread {spread_ns:.3f} ns)."],
                                 ok=f"calibrated: offset {offset_ns:.3f} ns after {tries} attempt(s)")
            except Exception as exc:
                self.cal_info.value = ""
                self._set_status([f"Calibrate (ADC {self.adc_ch.value}): {type(exc).__name__}: {exc}"
                                  " - is the loopback cable on this ADC?"])
            finally:
                if used_pulse:                      # put the user's pulses back on the DACs
                    self._apply(draw=True)
                    self._hw(self.awg.play, "restore waveforms")
                self.btn_cal.disabled = False

        threading.Thread(target=work, daemon=True).start()

    def _on_run(self, change):
        if change["new"]:
            if self.autoplay.value and self.awg.state != "playing":
                self._hw(self.awg.play)             # the DAC must be running to have a signal
            self._scope_stop.clear()
            self._scope_thread = threading.Thread(target=self._scope_loop, daemon=True)
            self._scope_thread.start()
            self.run.description, self.run.icon = "Stop scope", "stop"
        else:
            self._scope_stop.set()
            self.run.description, self.run.icon = "Run scope", "play"

    def _acquire(self):
        """One ADC capture -> (t_us, y, fs_hz, unit, description)."""
        with self._lock:
            return self._acquire_unlocked()

    def _acquire_unlocked(self):
        if self.scope is not None:
            trig = self.trig.value.strip()
            cfg = (trig, self.level.value, self.pre_ns.value, self.len_ns.value)
            if cfg != self._cfg:                    # only reconfigure when something changed
                kw = dict(trigger=trig, pre_ns=self.pre_ns.value, length_ns=self.len_ns.value)
                if trig != "awg2":                  # awg2 triggers on the AWG, no level needed
                    kw["level"] = self.level.value
                self.scope.configure(**kw)
                self._cfg = cfg
            t_ns, y = self.scope.acquire(timeout=self.timeout.value)
            t_ns, y = np.asarray(t_ns, float).ravel(), np.asarray(y, float)
            if y.ndim > 1:                          # several ADC channels: (n_ch, N) or (N, n_ch)
                ax = 0 if y.shape[-1] == len(t_ns) else 1
                y = np.take(y, min(self.adc_ch.value, y.shape[ax] - 1), axis=ax)
            y = y.ravel()
            if len(y) != len(t_ns):
                raise ValueError(f"scope returned {len(y)} samples for {len(t_ns)} time points")
            fs = 1e9 / float(np.median(np.diff(t_ns)))
            last = getattr(self.scope, "last", None)
            trig_txt = ""
            if isinstance(last, dict) and "triggered" in last:
                trig_txt = ", triggered" if last["triggered"] else ", NOT triggered (timeout)"
            if self.adc_vfs:
                return t_ns / 1e3, y * self.adc_vfs, fs, "V", f"trigger={trig}{trig_txt}"
            return t_ns / 1e3, y, fs, "FS", f"trigger={trig}{trig_txt}"
        y, fs = self.capture()
        y = np.asarray(y, float).ravel()
        return np.arange(len(y)) / fs / us, y, fs, "V", ""

    def _grab_and_show(self):
        t_us, y, fs, unit, note = self._acquire()
        self.fig_scope.update_layout(title=f"Scope – ADC ({len(y)} samples @ {fs/1e9:.4f} GS/s"
                                           f"{', ' + note if note else ''})")
        self._show_scope(t_us, y, fs, unit)
        self._last_capture = (t_us, y)
        self._update_overlay()

    def _scope_loop(self):
        while not self._scope_stop.is_set():
            t_start = time.time()
            try:
                if self.source.value == "ADC capture":
                    self._grab_and_show()
                else:
                    self._draw_scope_preview()
            except Exception as exc:
                self._set_status([f"Scope: {type(exc).__name__}: {exc}"])
                self.run.value = False
                return
            self._scope_stop.wait(max(0.0, 1 / self.rate.value - (time.time() - t_start)))

    def single(self):
        """Grab one capture (e.g. dash.single()) without starting the loop."""
        self._grab_and_show()
