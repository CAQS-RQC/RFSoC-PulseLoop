"""adc_scope.py  (v5)  -  triggered ADC scope for the AWG overlay (PYNQ, RFSoC 4x2)

Hardware (fpga/tcl/add_adc.tcl): ADC D (vin0_01) [+ vin0_23 on v5] -> trigger engine -> BRAM ring buffer.
v5 bitstream (awg_adc_v5.bit): TWO channels captured together, sample-aligned (same ADC tile):
    ch0 = ADC D (slice 00, vin0_01)      ch1 = slice 02 (vin0_23, the other input of tile 224)
    acquire() then returns y with shape (2, N): y[0] = ch0, y[1] = ch1.
    configure(..., channel=1) makes the level trigger look at ch1.
Single-channel bitstreams (v3, v4) still work: y is then 1-D as before.
The ADC side never stalls, so every capture is gap-free, and it can be re-armed forever.
Current bitstream: awg_adc_v4.bit, raw 4.9152 GSps (no decimation), 12 samples/beat,
196608 samples (40 us), ADC clock locked to the AWG ch0 clock.
The driver reads the setup from the core, so it also still runs the older 2.4576 GSps
bitstreams (awg_adc.bit, awg_adc_v3.bit).

    from adc_scope import AdcScope
    ol = Overlay("awg_adc_v4.bit")         # after xrfclk.set_ref_clks(...) as usual
    sc = AdcScope(ol)                      # starts the ADC clock (MMCM) and checks the ADC streams
    sc.configure(trigger="rising", level=0.05, pre_ns=50, length_ns=1000)
    t, y = sc.acquire()                    # t in ns, 0 = trigger; y in fraction of full scale
    sc.plot(t, y)
    sc.live()                              # refreshing view, stop with the Jupyter stop button

Trigger sources:
    "immediate"  free-running capture
    "rising"     ADC level, rising edge  (level in fraction of full scale, sub-sample timing)
    "falling"    ADC level, falling edge
    "awg0"       AWG channel 0 sequence start (DAC tile 0)
                   locked, no jitter: t = 0 is the same ADC sample for every sequence start
                   (see calibrate_awg0() to put t = 0 on the DAC output edge)
                   older 2.4576 GSps bitstreams: jitter +-1 beat (3.3 ns)
    "awg2"       AWG channel 2 sequence start (DAC tile 2)       - jitter +-1 beat
    "ctrl_rise"  control_trigger rising edge (pin or axi_control) - jitter +-1 beat
    "ctrl_fall"  control_trigger falling edge
    "manual"     only sc.force()
If no trigger arrives within `timeout`, acquire() forces one when auto=True (like a scope's AUTO).
"""
import time
import numpy as np

FULL_SCALE = 32768.0

R_CTRL, R_MODE, R_LEVEL, R_PRE, R_POST = 0x00, 0x04, 0x08, 0x0C, 0x10
R_STATUS, R_TADDR, R_TSUB, R_TTIME = 0x14, 0x18, 0x1C, 0x20
R_DEPTH, R_ID, R_ALIVE = 0x24, 0x28, 0x2C
R_CFG, R_FS_KHZ, R_CLKCTL = 0x30, 0x34, 0x38
CORE_ID_V3 = 0xADC50003       # older 2.4576 GSps core (awg_adc.bit, awg_adc_v3.bit), no CFG register
CORE_ID = 0xADC50004          # v4: 1 channel
CORE_ID_V5 = 0xADC50005       # v5: 2 channels (CFG[27:24])
MODES = {"immediate": 0, "rising": 1, "falling": 2, "awg0": 3, "awg2": 4,
         "ctrl_rise": 5, "ctrl_fall": 6, "manual": 7}
CH_NAMES = {0: "ch0 ADC D (vin0_01)", 1: "ch1 vin0_23"}
STATES = {0: "idle", 1: "filling pre-trigger", 2: "waiting for trigger", 3: "post-trigger", 4: "done"}


def rfdc(ol, name="usp_rf_data_converter_0"):
    """The RFDC driver object, even if PYNQ left the IP as a DefaultIP."""
    ip = getattr(ol, name)
    if hasattr(ip, "adc_tiles"):
        return ip
    import xrfdc
    return xrfdc.RFdc(ol.ip_dict[name])


def setup_adc(ol, tile=0, block=0, mixer_hz=0.0):
    """ADC fine mixer at 0 Hz -> the I stream is the plain input waveform (0 .. ~1 GHz)."""
    import xrfdc
    b = rfdc(ol).adc_tiles[tile].blocks[block]
    b.MixerSettings["Freq"] = mixer_hz / 1e6
    b.UpdateEvent(xrfdc.EVENT_MIXER)


def restart_adc_tile(ol, tile=0):
    """Restart the RF-ADC tile so its output FIFO starts clean on the (new) stream clock."""
    t = rfdc(ol).adc_tiles[tile]
    t.ShutDown()
    t.StartUp()


class AdcScope:
    def __init__(self, ol, core="adc", mem="axi_bram_ctrl_adc", setup=True):
        from pynq import MMIO
        def info(names):
            for name in names:
                for d in (ol.ip_dict, getattr(ol, "mem_dict", {})):
                    if name in d:
                        return d[name]
            raise KeyError(f"{names[0]} not found in ip_dict/mem_dict")
        c = info([core, "adc_scope"])                    # older bitstreams used the *_scope names
        m = info([mem, "axi_bram_ctrl_scope"])
        self.ol = ol
        self.regs = MMIO(c["phys_addr"], c["addr_range"])
        self.mem = MMIO(m["phys_addr"], m.get("addr_range", m.get("size", 262144)))
        ident = self.regs.read(R_ID)
        self.nch = 1
        if ident in (CORE_ID, CORE_ID_V5):
            cfg = self.regs.read(R_CFG)
            self.nch = max(1, (cfg >> 24) & 0xF)
            self.spb = cfg & 0xFF                        # samples per beat
            self.wpb = (cfg >> 8) & 0xFF                 # 32-bit words per beat in memory
            self.awg0_locked = bool(cfg & 0x10000)
            self.fs = self.regs.read(R_FS_KHZ) * 1e3
        elif ident == CORE_ID_V3:
            self.spb, self.wpb, self.awg0_locked, self.fs = 8, 4, False, 2.4576e9
        else:
            raise RuntimeError(f"ADC core ID is {ident:#x}, expected {CORE_ID_V5:#x}: wrong bitstream?")
        self.full = self.awg0_locked
        self.beat_ns = self.spb / self.fs * 1e9
        self.awg0_offset_ns = 0.0                        # set by calibrate_awg0()
        self.depth = self.regs.read(R_DEPTH)             # beats
        try:
            self.words = self.mem.array                  # uint32 view of the BRAM
        except AttributeError:
            self.words = np.frombuffer(self.mem.mem, np.uint32)
        if setup:
            if self.full:
                self.start_clock()
            else:
                setup_adc(ol)
        self.configure()

    def __repr__(self):
        mode = "raw, AWG-locked" if self.full else "2.4576 GSps decimated (old bitstream)"
        return (f"AdcScope({mode}, {self.nch} channel(s), {self.fs / 1e9:.4f} GSps, {self.spb} samples/beat, "
                f"{self.depth * self.spb} samples = {self.depth * self.beat_ns / 1000:.1f} us)")

    # ---------------------------------------------------------------- ADC clock (MMCM)
    def locked(self):
        return bool(self.regs.read(R_STATUS) & 4)

    def start_clock(self, restart_tile=True):
        """(Re)start the MMCM that makes the 409.6 MHz ADC clock from the DAC0 clock.
        Call it after the RFDC clocks run (xrfclk.set_ref_clks); __init__ does it for you."""
        self.regs.write(R_CLKCTL, 1)
        time.sleep(0.001)
        self.regs.write(R_CLKCTL, 0)
        t0 = time.time()
        while not self.locked():
            if time.time() - t0 > 1.0:
                raise RuntimeError("ADC clock MMCM does not lock: is the DAC0 clock running? "
                                   "(xrfclk.set_ref_clks() before AdcScope)")
            time.sleep(0.001)
        if restart_tile:
            try:
                restart_adc_tile(self.ol)
            except Exception as e:                       # driver differences between PYNQ versions
                print(f"note: ADC tile restart skipped ({e})")
        time.sleep(0.01)
        if not self.alive():
            raise RuntimeError("ADC clock locked but no ADC data: check the RFDC ADC tile state")

    # ---------------------------------------------------------------- settings
    def configure(self, trigger="rising", level=0.05, pre_ns=100.0, length_ns=2000.0, channel=0):
        """level: fraction of full scale (-1..1). pre_ns: time shown before the trigger.
        channel: which input the level trigger ("rising"/"falling") looks at (v5: 0 = ADC D, 1 = vin0_23).
        length_ns: total window (max depth * beat: 40 us; 53 us on the old 2.4576 GSps bitstreams)."""
        if trigger not in MODES:
            raise ValueError(f"trigger must be one of {list(MODES)}")
        total = int(np.ceil(length_ns / self.beat_ns))
        pre = int(np.ceil(pre_ns / self.beat_ns)) + 1      # +1: room for sub-beat trigger position
        total = max(total, pre + 2)
        if total > self.depth:
            raise ValueError(f"window too long: max {self.depth * self.beat_ns / 1000:.1f} us")
        if not 0 <= channel < self.nch:
            raise ValueError(f"channel must be 0..{self.nch - 1} on this bitstream")
        self.trigger, self.level, self.trig_ch = trigger, float(level), int(channel)
        self.pre, self.post = pre, total - pre
        code = int(np.clip(round(level * FULL_SCALE), -32768, 32767)) & 0xFFFF
        self.regs.write(R_CTRL, 4)                         # abort anything running
        self.regs.write(R_MODE, MODES[trigger] | (8 if self.trig_ch else 0))
        self.regs.write(R_LEVEL, code)
        self.regs.write(R_PRE, self.pre)
        self.regs.write(R_POST, self.post)

    # ---------------------------------------------------------------- low level
    def status(self):
        s = self.regs.read(R_STATUS)
        return {"done": bool(s & 1), "busy": bool(s & 2), "state": STATES.get((s >> 4) & 7, "?")}

    def alive(self, dt=0.01):
        """True if the ADC is streaming into the scope (beat counter moving)."""
        a = self.regs.read(R_ALIVE)
        time.sleep(dt)
        return self.regs.read(R_ALIVE) != a

    def arm(self):
        self.regs.write(R_CTRL, 1)
        t0 = time.time()
        while self.regs.read(R_STATUS) & 2:                # wait until the ADC side took the arm
            if time.time() - t0 > 0.1:
                raise RuntimeError("scope did not take the ARM: ADC clock not running? (check sc.alive())")

    def force(self):
        self.regs.write(R_CTRL, 2)

    def wait(self, timeout=1.0):
        t0 = time.time()
        while not (self.regs.read(R_STATUS) & 1):
            if time.time() - t0 > timeout:
                return False
        return True

    def read_window(self):
        """Samples of the last capture (int16), oldest first, plus trigger info."""
        taddr = self.regs.read(R_TADDR)
        tsub = self.regs.read(R_TSUB)
        ttime = self.regs.read(R_TTIME)
        n = self.pre + self.post
        W = self.wpb
        start = (taddr - self.pre) % self.depth
        if start + n <= self.depth:
            w = np.array(self.words[W * start: W * (start + n)])
        else:
            k = self.depth - start
            w = np.concatenate([np.array(self.words[W * start: W * self.depth]),
                                np.array(self.words[0: W * (n - k)])])
        # each memory word: nch blocks of (2W/nch) int16, the first spb of each block are samples
        blk = 2 * W // self.nch
        x = w.view(np.int16).reshape(n, self.nch, blk)[:, :, :self.spb]
        x = x.transpose(1, 0, 2).reshape(self.nch, -1)          # (nch, n*spb)
        return (x[0] if self.nch == 1 else x), tsub, ttime

    # ---------------------------------------------------------------- user level
    def acquire(self, timeout=0.5, auto=True, raw=False):
        """One capture. Returns (t_ns, y): t = 0 at the trigger, y in fraction of full scale.
        Level triggers are interpolated between samples.
        awg0 (v4 bitstream): t = 0 is the AWG ch0 sequence start, minus awg0_offset_ns."""
        self.arm()
        triggered = self.wait(timeout)
        if not triggered:
            if not auto:
                raise TimeoutError("no trigger")
            self.force()
            if not self.wait(0.2):
                raise RuntimeError(f"capture did not finish ({self.status()})")
        x, tsub, ttime = self.read_window()
        k = self.pre * self.spb + (tsub if triggered else 0)   # index of the trigger sample
        t0 = float(k)
        xt = x if x.ndim == 1 else x[self.trig_ch]               # the channel the trigger looked at
        if triggered and self.trigger in ("rising", "falling") and k > 0:
            lv = self.level * FULL_SCALE
            a, b = float(xt[k - 1]), float(xt[k])
            if b != a:
                t0 = (k - 1) + np.clip((lv - a) / (b - a), 0.0, 1.0)
        t = (np.arange(xt.size) - t0) / self.fs * 1e9
        if triggered and self.full and self.trigger == "awg0":
            t = t - self.awg0_offset_ns
        self.last = {"triggered": triggered, "trig_time_ns": ttime * self.beat_ns, "t0_index": t0}
        y = x if raw else x / FULL_SCALE
        return t, y

    def average(self, n=16, **kw):
        """Average n triggered captures on a common time grid (needs a stable trigger)."""
        t_ref, acc = None, None
        for _ in range(n):
            t, y = self.acquire(**kw)
            if t_ref is None:
                t_ref, acc = t, y.astype(np.float64)
            elif y.ndim == 1:
                acc += np.interp(t_ref, t, y)
            else:
                acc += np.stack([np.interp(t_ref, t, row) for row in y])
        return t_ref, acc / n

    def calibrate_awg0(self, edge_ns=0.0, n=8, frac=0.5, channel=0, **kw):
        """Loopback cable DAC ch0 -> ADC: play a waveform with a sharp rising edge
        at edge_ns after the sequence start, then call this. Afterwards t = 0 of awg0 captures is the
        sequence start as it appears at the ADC input. Returns (offset_ns, spread_ns).
        A spread of a few ps confirms the lock; redo it after each RFDC restart / power-up."""
        if not self.full:
            raise RuntimeError("needs the AWG-locked bitstream (awg_adc_v4.bit)")
        old = self.awg0_offset_ns
        self.awg0_offset_ns = 0.0
        starts = []
        try:
            for _ in range(n):
                t, y = self.acquire(auto=False, **kw)
                p = find_pulses(t, y if y.ndim == 1 else y[channel], frac)
                if not p:
                    raise RuntimeError("no edge found: check the cable, the waveform and the window")
                starts.append(p[0]["start_ns"])
        except Exception:
            self.awg0_offset_ns = old
            raise
        starts = np.array(starts)
        self.awg0_offset_ns = float(starts.mean() - edge_ns)
        return self.awg0_offset_ns, float(starts.std())

    @staticmethod
    def plot(t, y, title=None):
        import matplotlib.pyplot as plt
        plt.figure(figsize=(10, 3.5))
        y = np.asarray(y)
        if y.ndim == 1:
            plt.plot(t, y, lw=0.8)
        else:
            for c, row in enumerate(y):
                plt.plot(t, row, lw=0.8, label=CH_NAMES.get(c, f"ch{c}"))
            plt.legend(loc="upper right")
        plt.axvline(0, color="k", lw=0.5, ls=":")
        plt.xlabel("time from trigger (ns)")
        plt.ylabel("ADC (fraction of full scale)")
        if title:
            plt.title(title)
        plt.grid(True)
        plt.tight_layout()
        plt.show()

    def live(self, frames=None, avg=1, ylim=None, **kw):
        """Refreshing scope view in the notebook. Stop with the Jupyter stop button (or frames=N)."""
        import matplotlib.pyplot as plt
        from IPython.display import display
        fig, ax = plt.subplots(figsize=(10, 3.5))
        lines = [ax.plot([], [], lw=0.8, label=CH_NAMES.get(c, f"ch{c}"))[0] for c in range(self.nch)]
        if self.nch > 1:
            ax.legend(loc="upper right")
        ax.axvline(0, color="k", lw=0.5, ls=":")
        ax.set_xlabel("time from trigger (ns)")
        ax.set_ylabel("ADC (fraction of full scale)")
        ax.grid(True)
        h = display(fig, display_id=True)
        plt.close(fig)
        i, t_last = 0, time.time()
        try:
            while frames is None or i < frames:
                t, y = self.average(avg, **kw) if avg > 1 else self.acquire(**kw)
                rows = [y] if y.ndim == 1 else list(y)
                for ln, row in zip(lines, rows):
                    ln.set_data(t, row)
                ax.set_xlim(t[0], t[-1])
                if ylim:
                    ax.set_ylim(*ylim)
                else:
                    ax.relim(); ax.autoscale_view(scalex=False)
                now = time.time()
                p = find_pulses(t, rows[min(getattr(self, "trig_ch", 0), len(rows) - 1)])
                txt = f"{self.trigger}  {1 / max(now - t_last, 1e-3):.1f} fps"
                if not self.last["triggered"]:
                    txt += "  [AUTO - no trigger]"
                if p:
                    txt += f"  |  pulse: width {p[0]['width_ns']:.2f} ns, height {p[0]['height']:.3f}"
                ax.set_title(txt)
                h.update(fig)
                t_last, i = now, i + 1
        except KeyboardInterrupt:
            pass


def find_pulses(t, y, frac=0.5):
    """Pulses found by thresholding at frac*peak above the median baseline.
    Returns list of {start_ns, width_ns, height} with times interpolated between samples."""
    t = np.asarray(t, dtype=np.float64)
    y = np.asarray(y, dtype=np.float64)
    y = y - np.median(y)
    sign = 1.0 if y.max() >= -y.min() else -1.0
    y = sign * y
    peak = y.max()
    if peak < 5e-3:
        return []
    th = frac * peak
    above = y > th
    d = np.diff(above.astype(np.int8))
    rise = np.flatnonzero(d == 1)
    fall = np.flatnonzero(d == -1)

    def cross(i):                         # linear interpolation between sample i and i+1
        a, b = y[i], y[i + 1]
        return t[i] + (th - a) / (b - a) * (t[i + 1] - t[i])

    out = []
    for r in rise:
        f = fall[fall > r]
        if len(f) == 0:
            break
        ts, te = cross(r), cross(f[0])
        out.append({"start_ns": ts, "width_ns": te - ts,
                    "height": float(sign * y[r + 1:f[0] + 1].max())})
    return out
