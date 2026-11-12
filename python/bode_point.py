"""
bode_point.py -- measure one point at a time with an EXTERNAL generator and add it to a Bode plot.

    from bode_point import Bode
    b = Bode(sc, gen_vpk=0.1, adc_vfs=None)     # gen_vpk: peak volts the generator sends (constant!)
    b.point()          # generator set to a new frequency -> call this, a point appears on the plot
    b.point(f=50e6)    # same, but give the frequency yourself (skips auto-detection)
    b.save("bode.csv"); b.clear()

Gain = measured peak amplitude / generator peak.  Without adc_vfs the plot is in dB relative to the
generator assuming 1.0 FS = 1 V peak, so set adc_vfs (volts peak for 1.0 FS) for real numbers.
Points whose amplitude is below `threshold` (FS) or that clip are NOT plotted (they are listed).
"""
import csv
import numpy as np
import matplotlib.pyplot as plt

# ADC_FS = 2.4576e9          # fallback only: the real rate is read from scope.fs (4.9152e9 on v4/v5)


def _fit(t_s, y, f):
    """Sine fit at known f using the 3x3 normal equations (a few dot products: fast on the board's ARM)."""
    w = (2 * np.pi * f) * t_s
    sn, cs = np.sin(w), np.cos(w)
    n = float(len(y))
    G = np.array([[sn @ sn, sn @ cs, sn.sum()], [sn @ cs, cs @ cs, cs.sum()], [sn.sum(), cs.sum(), n]])
    a, b, c = np.linalg.solve(G, np.array([sn @ y, cs @ y, y.sum()]))
    r = y - (a * sn + b * cs + c)
    return float(np.hypot(a, b)), float(np.sqrt(np.mean(r * r)))


def find_freq(t_s, y):
    """Frequency of the dominant sine: zero-padded FFT peak, then refined by least squares."""
    n = len(y)
    dt = float(np.median(np.diff(t_s)))
    win = np.hanning(n)
    nfft = 1 << int(np.ceil(np.log2(n * 8)))
    sp = np.abs(np.fft.rfft((y - y.mean()) * win, nfft))
    sp[0] = 0
    k = int(np.argmax(sp))
    f0, df = k / (nfft * dt), 1 / (nfft * dt)
    if 0 < k < len(sp) - 1 and sp[k - 1] > 0 and sp[k + 1] > 0:       # parabolic peak interpolation
        a, b, c = np.log(sp[k - 1:k + 2] + 1e-30)
        f0 += 0.5 * (a - c) / (a - 2 * b + c + 1e-30) * df
    if n > 20000:                                                    # long capture: FFT is accurate enough
        return f0
    best = (np.inf, f0)
    for f in np.linspace(f0 - 4 * df, f0 + 4 * df, 81):          # fine search around the peak
        if f > 0:
            _, r = _fit(t_s, y, f)
            if r < best[0]:
                best = (r, f)
    return best[1]


class Bode:
    def __init__(self, scope, gen_vpk=0.1, adc_vfs=None, threshold=0.005, channel=0,
                 length_ns=2000.0, averages=4, show=True):
        self.sc, self.gen_vpk, self.adc_vfs = scope, gen_vpk, adc_vfs
        self.threshold, self.channel, self.length_ns, self.averages = threshold, channel, length_ns, averages
        self.rows = []                                   # (f_hz, amp_fs, gain_db, resid_fs, ok)
        self.fs = float(getattr(scope, "fs", ADC_FS))
        self.fig = self.ax = None
        if show:
            self._init_plot()

    def _acquire(self, length_ns):
        self.sc.configure(trigger="immediate", pre_ns=0, length_ns=length_ns)
        t, y = self.sc.acquire()
        y = np.asarray(y, float)
        if y.ndim > 1:
            y = y[self.channel]
        return np.asarray(t, float) * 1e-9, y

    def point(self, f=None):
        t, y = self._acquire(self.length_ns)
        if f is None:
            if np.ptp(y) < self.threshold:                  # nothing there: do not guess a frequency
                self.rows.append((np.nan, np.ptp(y) / 2, np.nan, np.nan, False))
                print(f"no signal (peak {np.ptp(y)/2:.4f} FS < threshold {self.threshold}): not plotted")
                return None
            f = find_freq(t, y)
        cycles = f * (t[-1] - t[0])
        if cycles < 5:                                       # too few cycles: capture longer
            t, y = self._acquire(min(6e9 / f, 5e4))
        amps, ress, clip = [], [], False
        for i in range(max(1, self.averages)):
            if i:
                t, y = self._acquire(len(y) / self.fs * 1e9)
            a, r = _fit(t, y, f)
            amps.append(a); ress.append(r)
            clip |= bool(np.max(np.abs(y)) > 0.95)
        a, r = float(np.mean(amps)), float(np.mean(ress))
        vfs = self.adc_vfs or 1.0
        gain = 20 * np.log10(max(a, 1e-12) * vfs / self.gen_vpk)
        ok = a >= self.threshold and not clip and r < 0.3 * a
        self.rows.append((f, a, gain, r, ok))
        note = "" if ok else ("  CLIPPING" if clip else "  below threshold / poor fit")
        print(f"{f/1e6:10.4f} MHz   {a:.4f} FS   gain {gain:+.2f} dB   resid {r:.4f}{note}")
        self._draw()
        return f, gain

    def _max_length_ns(self):
        depth = getattr(self.sc, "depth", None)
        spb = getattr(self.sc, "spb", None)
        if depth and spb:
            return 0.97 * depth * spb / self.fs * 1e9
        return 38000.0

    def scan(self, duration_s=20.0, max_points=400, fmax=None):
        """Generator in sweep mode: phase 1 only ACQUIRES (as fast as the scope allows, raw data kept in
        memory), phase 2 analyses up to `max_points` of the captures that contain a signal, evenly spread
        in time. Splitting them keeps the slow analysis from starving the capture rate.
        Start the generator's sweep, then call this. Stop button ends phase 1 early."""
        import time
        fmax = fmax or 0.98 * self.fs / 2
        max_ns = self._max_length_ns()
        L, caps, self.trace = max_ns, [], []
        t_acq = 0.0
        t0 = time.time()
        try:
            while time.time() - t0 < duration_s:
                ta = time.time()
                t, y = self._acquire(L)
                t_acq += time.time() - ta
                dt = float(t[1] - t[0])
                pk = float(np.ptp(y) / 2)
                self.trace.append((time.time() - t0, pk))
                if pk < self.threshold:
                    L = max_ns                                # nothing seen: go long (low frequencies)
                    continue
                caps.append((time.time() - t0, y.astype(np.float32), dt, pk))
                yc = y - y.mean()                                          # cheap frequency estimate,
                st = np.where(yc > 0.3 * pk, 1, np.where(yc < -0.3 * pk, -1, 0))   # with hysteresis vs noise
                st = st[st != 0]
                z = int(np.count_nonzero(np.diff(st)))
                f_est = z / (2 * len(y) * dt) if z >= 2 else 1.0
                L = float(np.clip(10e9 / max(f_est, 1.0), self.length_ns, max_ns))   # ~10 cycles next
        except KeyboardInterrupt:
            pass
        total, dt_scan = len(self.trace), max(time.time() - t0, 1e-9)
        print(f"acquired {total} captures in {dt_scan:.1f} s ({total/dt_scan:.1f}/s, "
              f"{1e3*t_acq/max(total,1):.0f} ms per acquire); {len(caps)} contain a signal "
              f"(largest peak {max([p for _, p in self.trace] or [0]):.4f} FS, channel {self.channel})")
        if not caps:
            print("No signal: check the channel (channel=0/1), the generator output, or lower threshold=.")
            return
        idx = np.unique(np.linspace(0, len(caps) - 1, min(max_points, len(caps))).astype(int))
        t1, n_ok, high = time.time(), 0, 0
        for i in idx:
            _, y, dt, _ = caps[i]
            y = y.astype(float)
            tt = np.arange(len(y)) * dt          # dt is already in seconds
            f = find_freq(tt, y)
            if f > fmax:
                high += 1
                continue
            a, r = _fit(tt, y, f)
            clip = bool(np.max(np.abs(y)) > 0.95)
            gain = 20 * np.log10(max(a, 1e-12) * (self.adc_vfs or 1.0) / self.gen_vpk)
            cycles = f * tt[-1]
            ok = a >= self.threshold and not clip and r < 0.3 * a and cycles >= 3
            self.rows.append((f, a, gain, r, ok))
            n_ok += ok
        print(f"analysed {len(idx)} captures in {time.time()-t1:.1f} s: {n_ok} plotted, {high} above "
              f"{fmax/1e6:.0f} MHz")
        fs_ = [r[0] for r in self.rows if r[4]]
        if fs_:
            print(f"frequencies: {min(fs_)/1e6:.3f} .. {max(fs_)/1e6:.1f} MHz")
        self._draw()

    def plot_trace(self):
        """Peak amplitude of every capture of the last scan() versus time: shows WHEN the signal is there."""
        tr = np.array(self.trace)
        fig, ax = plt.subplots(figsize=(8, 3.5))
        ax.semilogy(tr[:, 0], np.maximum(tr[:, 1], 1e-5), ".", ms=3)
        ax.axhline(self.threshold, color="r", ls=":", label="threshold")
        ax.set_xlabel("time since scan start [s]"); ax.set_ylabel("peak [FS]")
        ax.grid(True, which="both", alpha=0.4); ax.legend()
        print(f"noise floor (median peak): {np.median(tr[:, 1]):.5f} FS")
        plt.show()

    # ---- plot / io
    def _init_plot(self):
        self.fig, self.ax = plt.subplots(figsize=(8, 4))
        self.ax.set_xscale("log"); self.ax.grid(True, which="both", alpha=0.4)
        self.ax.set_xlabel("frequency [Hz]")
        self.ax.set_ylabel("gain [dB]" + ("" if self.adc_vfs else "  (1 FS = 1 V assumed)"))
        self.line, = self.ax.plot([], [], "o-", ms=5)

    def _draw(self):
        if self.ax is None:
            return
        ok = sorted((r for r in self.rows if r[4]), key=lambda r: r[0])
        self.line.set_data([r[0] for r in ok], [r[2] for r in ok])
        self.ax.relim(); self.ax.autoscale_view()
        self.fig.canvas.draw_idle()
        try:
            from IPython.display import display, clear_output
            clear_output(wait=True); display(self.fig)
        except Exception:
            pass

    def clear(self):
        self.rows.clear(); self._draw()

    def save(self, path="bode.csv"):
        with open(path, "w", newline="") as fh:
            w = csv.writer(fh); w.writerow(["freq_hz", "amp_fs", "gain_db", "resid_fs", "accepted"])
            w.writerows(sorted(self.rows, key=lambda r: (np.isnan(r[0]), r[0])))