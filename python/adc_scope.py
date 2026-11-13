"""adc_scope.py  -  triggered 2-channel ADC scope for the AWG overlay (PYNQ, RFSoC 4x2)

Bitstream: awg_adc_v5.bit (fpga/tcl/add_adc.tcl). Two inputs of ADC tile 224, captured together and
sample-aligned, raw 4.9152 GSps, 12 samples/beat, 196608 samples (40 us), ADC clock locked to the DAC0 clock:
    ADC "D" = slice 00 (vin0_01)      ADC "C" = slice 02 (vin0_23, the other input of tile 224)
acquire() returns (t, y) with y = {"D": array, "C": array}.
The ADC side never stalls, so every capture is gap-free, and it can be re-armed forever.

    from adc_scope import AdcScope
    ol = Overlay("awg_adc_v5.bit")         # after xrfclk.set_ref_clks(...) as usual
    sc = AdcScope(ol)                      # starts the ADC clock (MMCM) and checks the ADC streams
    sc.calibrate_awg("B", adc="D")         # loopback DAC B -> ADC D, B playing a sharp edge
    sc.configure(trigger="dac_b", pre_ns=5, length_ns=20, channel="D")
    t, y = sc.acquire()                    # t in ns, 0 = DAC B output start; y in fraction of full scale
    sc.plot(t, y)
    sc.live()                              # refreshing view, stop with the Jupyter stop button

Trigger sources:
    "immediate"  free-running capture
    "rising"     ADC level, rising edge on `channel`  (level in fraction of full scale, sub-sample timing)
    "falling"    ADC level, falling edge
    "dac_b"      DAC B sequence start (DAC tile 0): locked, no jitter
    "dac_a"      DAC A sequence start (DAC tile 2): jitter +-1 beat (2.4 ns)
    "ctrl_rise"  control_trigger rising edge (pin or axi_control)            - jitter +-1 beat
    "ctrl_fall"  control_trigger falling edge
    "manual"     only sc.force()
    (DAC letter -> tile comes from rfsoc_awg.CHANNELS, the one table to flip if the labels are swapped.)
If no trigger arrives within `timeout`, acquire() forces one when auto=True (like a scope's AUTO).

Calibrated time: after calibrate_awg(), captures on that DAC's trigger have t = 0 at the DAC output
start as seen at the ADC, and pre_ns / length_ns are in that time: configure(pre_ns=5, length_ns=20)
shows -5 .. +15 ns around the DAC edge even though it reaches the ADC ~100+ ns after the trigger.
"""
import time
import numpy as np

from rfsoc_awg import CHANNELS

FULL_SCALE = 32768.0

R_CTRL, R_MODE, R_LEVEL, R_PRE, R_POST = 0x00, 0x04, 0x08, 0x0C, 0x10
R_STATUS, R_TADDR, R_TSUB, R_TTIME = 0x14, 0x18, 0x1C, 0x20
R_DEPTH, R_ID, R_ALIVE = 0x24, 0x28, 0x2C
R_CFG, R_FS_KHZ, R_CLKCTL = 0x30, 0x34, 0x38
CORE_ID = 0xADC50005          # v5: 2 channels (CFG[27:24])
ADCS = {"D": 0, "C": 1}       # ADC letter -> row of the capture memory (slice 00 / slice 02)
ADC_NAMES = {"D": "ADC D (vin0_01)", "C": "ADC C (vin0_23)"}
NCH = len(ADCS)
_TILE_MODE = {0: 3, 2: 4}     # DAC tile -> MODE code of its sequence-start trigger in the core
DAC_TRIGGERS = {f"dac_{d.lower()}": _TILE_MODE[tile] for d, tile in sorted(CHANNELS.items())}
MODES = {"immediate": 0, "rising": 1, "falling": 2, **DAC_TRIGGERS,
         "ctrl_rise": 5, "ctrl_fall": 6, "manual": 7}
LOCKED = tuple(f"dac_{d.lower()}" for d, tile in CHANNELS.items() if tile == 0)   # no jitter
STATES = {0: "idle", 1: "filling pre-trigger", 2: "waiting for trigger", 3: "post-trigger", 4: "done"}
POLL_S = 50e-6                                # register polling interval (keeps a CPU core free)


def rfdc(ol, name="usp_rf_data_converter_0"):
    """The RFDC driver object, even if PYNQ left the IP as a DefaultIP."""
    ip = getattr(ol, name)
    if hasattr(ip, "adc_tiles"):
        return ip
    import xrfdc
    return xrfdc.RFdc(ol.ip_dict[name])


def restart_adc_tile(ol, tile=0):
    """Restart the RF-ADC tile so its output FIFO starts clean on the (new) stream clock."""
    t = rfdc(ol).adc_tiles[tile]
    t.ShutDown()
    t.StartUp()


def dac_trigger(dac):
    """'A' / 'B' -> the trigger name of that DAC's sequence start ('dac_a' / 'dac_b')."""
    if not (isinstance(dac, str) and dac.upper() in CHANNELS):
        raise ValueError(f"dac must be 'A' or 'B', got {dac!r}")
    return f"dac_{dac.lower()}"


def adc_letter(adc):
    """Check an ADC name: 'D' or 'C'."""
    if not (isinstance(adc, str) and adc.upper() in ADCS):
        raise ValueError(f"ADC must be 'D' or 'C', got {adc!r}")
    return adc.upper()


class AdcScope:
    def __init__(self, ol, core="adc", mem="axi_bram_ctrl_adc", setup=True):
        from pynq import MMIO
        for name in (core, mem):
            if name not in ol.ip_dict and name not in getattr(ol, "mem_dict", {}):
                raise KeyError(f"{name} not found in the overlay: is this awg_adc_v5.bit?")
        c = ol.ip_dict.get(core) or ol.mem_dict[core]
        m = ol.ip_dict.get(mem) or ol.mem_dict[mem]
        self.ol = ol
        self.regs = MMIO(c["phys_addr"], c["addr_range"])
        self.mem = MMIO(m["phys_addr"], m.get("addr_range") or m["size"])
        ident = self.regs.read(R_ID)
        if ident != CORE_ID:
            raise RuntimeError(f"ADC core ID is {ident:#x}, expected {CORE_ID:#x}: load awg_adc_v5.bit")
        cfg = self.regs.read(R_CFG)
        if (cfg >> 24) & 0xF != NCH or not cfg & 0x10000:
            raise RuntimeError(f"ADC core CFG {cfg:#x}: expected {NCH} channels locked to the AWG")
        self.spb = cfg & 0xFF                            # samples per beat
        self.wpb = (cfg >> 8) & 0xFF                     # 32-bit words per beat in memory
        self.fs = self.regs.read(R_FS_KHZ) * 1e3
        self.beat_ns = self.spb / self.fs * 1e9
        self.depth = self.regs.read(R_DEPTH)             # beats
        self.words = self.mem.array                      # uint32 view of the BRAM
        self.offset_ns = {t: 0.0 for t in DAC_TRIGGERS}  # per DAC trigger, set by calibrate_awg()
        self.settings = {}                               # what configure() last wrote
        self.last = {"triggered": False}
        if setup:
            self.start_clock()
        self.configure()

    def __repr__(self):
        return (f"AdcScope({NCH} channels, {self.fs / 1e9:.4f} GSps, {self.spb} samples/beat, "
                f"{self.depth * self.spb} samples = {self.depth * self.beat_ns / 1000:.1f} us, "
                f"settings {self.settings})")

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
    def configure(self, trigger="rising", level=0.05, pre_ns=100.0, length_ns=2000.0, channel="D"):
        """level: fraction of full scale (-1..1). pre_ns: time shown before t = 0 (>= 0).
        length_ns: total window shown (pre_ns included), max 40 us.
        channel: ADC the level trigger ("rising"/"falling") looks at, "D" or "C".
        On a calibrated DAC trigger, t = 0 is the DAC output start: the capture is placed
        around it however long the DAC -> ADC delay is (the extra samples are cropped)."""
        if trigger not in MODES:
            raise ValueError(f"trigger must be one of {list(MODES)}, got {trigger!r}")
        channel = adc_letter(channel)
        level, pre_ns, length_ns = float(level), float(pre_ns), float(length_ns)
        if not np.isfinite([level, pre_ns, length_ns]).all():
            raise ValueError("level, pre_ns and length_ns must be finite numbers")
        if not -1.0 <= level <= 1.0:
            raise ValueError(f"level must be in -1..1 (fraction of full scale), got {level}")
        if pre_ns < 0:
            raise ValueError(f"pre_ns must be >= 0, got {pre_ns}")
        if length_ns <= 0:
            raise ValueError(f"length_ns must be > 0, got {length_ns}")
        off = self.offset_ns.get(trigger, 0.0)
        start_raw = off - pre_ns                           # wanted window, in ns after the trigger
        end_raw = start_raw + length_ns
        pre = int(np.ceil(max(0.0, -start_raw) / self.beat_ns)) + 1   # +1: sub-beat trigger position
        post = int(np.ceil(max(0.0, end_raw) / self.beat_ns)) + 1
        if pre + post > self.depth:
            raise ValueError(f"window too long: {(pre + post) * self.beat_ns / 1000:.1f} us needed "
                             f"(pre + length + {off:.1f} ns calibrated delay), max "
                             f"{self.depth * self.beat_ns / 1000:.1f} us")
        self.trigger, self.level, self.trig_ch = trigger, level, channel
        self.pre, self.post = pre, post
        self._win = (-pre_ns, length_ns - pre_ns)          # shown range of t [ns]
        self._cfg_offset = off
        code = int(np.clip(round(level * FULL_SCALE), -32768, 32767)) & 0xFFFF
        self.regs.write(R_CTRL, 4)                         # abort anything running
        self.regs.write(R_MODE, MODES[trigger] | (8 if ADCS[channel] else 0))
        self.regs.write(R_LEVEL, code)
        self.regs.write(R_PRE, self.pre)
        self.regs.write(R_POST, self.post)
        self.settings = dict(trigger=trigger, level=level, pre_ns=pre_ns, length_ns=length_ns,
                             channel=channel)

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
            time.sleep(POLL_S)

    def force(self):
        self.regs.write(R_CTRL, 2)

    def wait(self, timeout=1.0):
        t0 = time.time()
        while not (self.regs.read(R_STATUS) & 1):
            if time.time() - t0 > timeout:
                return False
            time.sleep(POLL_S)
        return True

    def read_window(self):
        """Samples of the last capture, int16 array (rows in ADCS order, N), oldest first, plus trigger info."""
        taddr = self.regs.read(R_TADDR)
        tsub = self.regs.read(R_TSUB)
        ttime = self.regs.read(R_TTIME)
        if taddr >= self.depth or tsub >= self.spb:
            raise RuntimeError(f"bad trigger position from the core (addr {taddr}, sub {tsub})")
        n = self.pre + self.post
        W = self.wpb
        start = (taddr - self.pre) % self.depth
        if start + n <= self.depth:
            w = np.array(self.words[W * start: W * (start + n)])
        else:
            k = self.depth - start
            w = np.concatenate([np.array(self.words[W * start: W * self.depth]),
                                np.array(self.words[0: W * (n - k)])])
        # each memory word: NCH blocks of (2W/NCH) int16, the first spb of each block are samples
        blk = 2 * W // NCH
        x = w.view(np.int16).reshape(n, NCH, blk)[:, :, :self.spb]
        return x.transpose(1, 0, 2).reshape(NCH, -1), tsub, ttime

    # ---------------------------------------------------------------- user level
    def acquire(self, timeout=0.5, auto=True, raw=False):
        """One capture. Returns (t_ns, y): y = {"D": array, "C": array} in fraction of full scale
        (int16 codes with raw=True), t = 0 at the trigger, cropped to the configured window.
        Level triggers are interpolated between samples.
        DAC triggers: t = 0 is the DAC output start once calibrate_awg() has run."""
        if self.offset_ns.get(self.trigger, 0.0) != self._cfg_offset:
            self.configure(**self.settings)                # offset changed: re-place the window
        self.arm()
        triggered = self.wait(timeout)
        if not triggered:
            if not auto:
                raise TimeoutError(f"no trigger within {timeout} s (trigger={self.trigger})")
            self.force()
            if not self.wait(0.2):
                raise RuntimeError(f"capture did not finish ({self.status()})")
        x, tsub, ttime = self.read_window()
        k = self.pre * self.spb + (tsub if triggered else 0)   # index of the trigger sample
        t0 = float(k)
        xt = x[ADCS[self.trig_ch]]                             # the ADC the trigger looked at
        if triggered and self.trigger in ("rising", "falling") and k > 0:
            lv = self.level * FULL_SCALE
            a, b = float(xt[k - 1]), float(xt[k])
            if b != a:
                t0 = (k - 1) + np.clip((lv - a) / (b - a), 0.0, 1.0)
        t = (np.arange(xt.size) - t0) / self.fs * 1e9 - self.offset_ns.get(self.trigger, 0.0)
        keep = (t >= self._win[0]) & (t < self._win[1])
        if not keep.any():
            keep[:] = True
        self.last = {"triggered": triggered, "trig_time_ns": ttime * self.beat_ns, "t0_index": t0}
        y = {name: (x[row][keep] if raw else x[row][keep] / FULL_SCALE) for name, row in ADCS.items()}
        return t[keep], y

    def average(self, n=16, max_tries=None, **kw):
        """Average n TRIGGERED captures on a common time grid (needs a stable trigger).
        Captures without a trigger are skipped; TimeoutError if none of max_tries (3n) triggers."""
        kw.pop("auto", None)
        max_tries = max_tries or 3 * n
        t_ref, acc, got = None, None, 0
        for _ in range(max_tries):
            try:
                t, y = self.acquire(auto=False, **kw)
            except TimeoutError:
                continue
            if t_ref is None:
                t_ref, acc = t, {c: v.astype(np.float64) for c, v in y.items()}
            else:
                for c in acc:
                    acc[c] += np.interp(t_ref, t, y[c])
            got += 1
            if got == n:
                break
        if not got:
            raise TimeoutError(f"average: no trigger in {max_tries} attempts (trigger={self.trigger})")
        return t_ref, {c: v / got for c, v in acc.items()}

    def calibrate_awg(self, dac="B", adc="D", edge_ns=0.0, n=8, frac=0.5,
                      pre_ns=20.0, length_ns=2000.0, timeout=0.5):
        """Loopback cable DAC <dac> -> ADC <adc>: play a waveform with a sharp rising edge
        at edge_ns after the sequence start, then call this. It triggers on that DAC's sequence
        start ("dac_a" / "dac_b"), finds the edge n times, and afterwards t = 0 of those
        captures is the sequence start as it appears at the ADC input. The search window
        (length_ns) must contain the DAC -> ADC delay. Your scope settings are restored after,
        re-placed for the new offset. Returns (offset_ns, spread_ns).
        The tile-0 DAC (LOCKED) gives a spread of a few ps; the other one has +-1 beat trigger
        jitter, so expect ~1 ns there. Redo it after each RFDC restart / power-up."""
        trig = dac_trigger(dac)
        adc = adc_letter(adc)
        if n < 2:
            raise ValueError("n must be >= 2 to measure the spread")
        saved = dict(self.settings)
        old, new, starts = self.offset_ns[trig], None, []
        try:
            self.offset_ns[trig] = 0.0
            self.configure(trigger=trig, pre_ns=pre_ns, length_ns=length_ns, channel=adc)
            for _ in range(n):
                try:
                    t, y = self.acquire(timeout=timeout, auto=False)
                except TimeoutError as e:
                    raise TimeoutError(f"{e}: is DAC {dac.upper()} playing?") from None
                p = find_pulses(t, y[adc], frac)
                if not p:
                    raise RuntimeError(f"no edge found on ADC {adc}: check the cable from DAC "
                                       f"{dac.upper()}, the waveform and the window ({length_ns} ns)")
                starts.append(p[0]["start_ns"])
            new = float(np.mean(starts) - edge_ns)
        finally:
            self.offset_ns[trig] = old if new is None else new
            if saved:
                self.configure(**saved)
        return new, float(np.std(starts))

    @staticmethod
    def plot(t, y, title=None, ylabel="ADC (fraction of full scale)"):
        import matplotlib.pyplot as plt
        plt.figure(figsize=(10, 3.5))
        if isinstance(y, dict):
            for c, row in y.items():
                plt.plot(t, row, lw=0.8, label=ADC_NAMES.get(c, c))
            plt.legend(loc="upper right")
        else:
            plt.plot(t, y, lw=0.8)
        plt.axvline(0, color="k", lw=0.5, ls=":")
        plt.xlabel("time from trigger (ns)")
        plt.ylabel(ylabel)
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
        lines = {c: ax.plot([], [], lw=0.8, label=ADC_NAMES[c])[0] for c in ADCS}
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
                for c, ln in lines.items():
                    ln.set_data(t, y[c])
                ax.set_xlim(t[0], t[-1])
                if ylim:
                    ax.set_ylim(*ylim)
                else:
                    ax.relim(); ax.autoscale_view(scalex=False)
                now = time.time()
                p = find_pulses(t, y[self.trig_ch])
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
