"""
rfsoc_awg.py  --  easy pulse / arbitrary-waveform generator for the RFSoC 4x2
==============================================================================

What this module does
---------------------
It hides all the "plumbing" (sample interleaving, int16 conversion, DMA upload,
GPIO registers) behind a small API so you only think about *pulses*:

    from rfsoc_awg import *

    awg = AWG(window=6.65*us)                       # length of one waveform slot
    awg.set_waveform("DAC_A", "z",                  # which output / which slot
                     Pulse(freq=60*MHz, power_dbm=-20,
                           start=1*us, duration=2*us, shape="gauss"))
    awg.plot()                                      # look before you play
    awg.connect(); awg.play()                       # talk to the board
    ... edit waveforms ...;   awg.play()            # re-upload: takes effect immediately
    awg.select_slot("x")                            # switch slot instantly, no upload

Hardware model (bitstreams `awg_clock_syncd.bit` / `awg_adc.bit`, RTL fpga/src/awg_BRAM_dual.v)
-----------------------------------------------------------------------------------------------
* 2 outputs:  "DAC_B" (hardware channel 0) and "DAC_A" (hardware channel 2).
* The DAC runs at 9.8304 GS/s  ->  1 sample = 101.7 ps.
* The fabric hands the DAC 16 samples per clock cycle (614.4 MHz -> 1.6276 ns).
  => the length of a waveform is always a multiple of 16 samples.
* Every output owns TWO waveform "slots" (named "z" and "x" here, or 0 and 1),
  each up to 2**18 samples = 26.67 us long.  Both slots use the same length
  (the `window`), because the bitstream has a single MAX_POINTS register.
* ONE slot plays at a time, looping forever: `select_slot("z")` / `select_slot("x")`
  (register axi_control, OR-ed with the external control_trigger pin).
* Samples are 16-bit signed integers (two's complement).

Units
-----
Everything is in SI base units (seconds, Hz, volts).  Use the multipliers
`ns, us, kHz, MHz, GHz` to keep code readable:  `Pulse(freq=70*MHz, start=1.5*us)`.

Dry-run mode
------------
`AWG(..., dry_run=True)` never touches pynq, so you can design and plot pulses
on any laptop.  Only `connect()/upload()/start()` need the real board.
"""

from __future__ import annotations

import time
import warnings
from dataclasses import dataclass
from typing import Iterable, Optional, Sequence, Union

import numpy as np

__all__ = [
    "AWG", "Pulse", "chirp", "render_pulses", "pack_channel",
    "dbm_to_vpk", "dbm_to_vpp", "vpp_to_dbm", "print_clocks",
    "ns", "us", "ms", "kHz", "MHz", "GHz",
    "DAC_SR", "SLOT_SAMPLES", "SAMPLES_PER_CLK",
]

# --------------------------------------------------------------------------- #
#  Unit multipliers (so you can write 5*us instead of 5e-6)
# --------------------------------------------------------------------------- #
ns, us, ms = 1e-9, 1e-6, 1e-3
kHz, MHz, GHz = 1e3, 1e6, 1e9

# --------------------------------------------------------------------------- #
#  Hardware constants -- change only if the bitstream / board changes
# --------------------------------------------------------------------------- #
DAC_SR = 9.8304e9            # DAC sample rate [samples/s]
SAMPLES_PER_CLK = 16         # DAC samples delivered per fabric clock cycle
INTERLEAVE = 8               # samples per block when splitting into the two RAMs
SLOT_SAMPLES = 2 ** 18       # RAM depth reserved for ONE slot of ONE channel
N_SLOTS = 2                  # slots per channel ("z" and "x")
DAC_FULL_SCALE_CODE = 2 ** 15   # int16 range is [-32768, 32767]

# Calibration taken from the original script:  code_peak = 2 * Vpp * 2**15,
# i.e. a *peak* code of 2**15 gives 0.5 Vpp  (=> +-0.25 V) at the output.
# Re-measure with a scope / spectrum analyser and edit this if your hardware
# (balun, attenuators, ...) differs.
VPP_AT_FULL_SCALE = 0.5      # [V peak-to-peak]
LOAD_OHMS = 50.0             # impedance used for dBm <-> volt conversions

# Names -> hardware channel index (the number in the IP names `enable_ch<N>`)
CHANNELS = {"DAC_B": 0, "DAC_A": 2}
# Order in which the channels are laid out in the DMA stream (must match RTL).
STREAM_ORDER = ("DAC_B", "DAC_A")
# Friendly slot names
SLOTS = {"z": 0, "x": 1, 0: 0, 1: 1}


# --------------------------------------------------------------------------- #
#  Small conversion helpers
# --------------------------------------------------------------------------- #
def dbm_to_vpk(dbm: float) -> float:
    """Peak voltage [V] of a sine carrying `dbm` into 50 ohm."""
    return float(np.sqrt(2.0 * LOAD_OHMS * 1e-3 * 10 ** (dbm / 10.0)))


def dbm_to_vpp(dbm: float) -> float:
    """Peak-to-peak voltage [V] of a sine carrying `dbm` into 50 ohm.
    (0.63246 * 10**(dbm/20), the `dBm2Vpp` of the old notebook.)"""
    return 2.0 * dbm_to_vpk(dbm)


def vpp_to_dbm(vpp: float) -> float:
    """Power [dBm] of a sine of `vpp` volts peak-to-peak into 50 ohm."""
    return float(10 * np.log10((vpp / 2) ** 2 / (2 * LOAD_OHMS) / 1e-3))


MAX_VPK = VPP_AT_FULL_SCALE / 2            # largest peak voltage the DAC can make
MAX_DBM = vpp_to_dbm(VPP_AT_FULL_SCALE)    # ... and its CW power (about -2 dBm)


# --------------------------------------------------------------------------- #
#  Pulse description
# --------------------------------------------------------------------------- #
@dataclass
class Pulse:
    """
    One pulse:  envelope(t) * carrier(t).

    The carrier is, in order of priority:
      * `wave`          your own signal (callable or array, see below), or
      * sin(2*pi*freq*t + phase)   if freq > 0, or
      * a constant (baseband / "DC" pulse, = cos(phase), i.e. 1)   if freq = 0
        (the default).  So with the defaults you just get an envelope pulse.

    Parameters
    ----------
    freq        Carrier frequency [Hz] (0 <= freq < 4.9152 GHz). Default 0 =
                baseband pulse (no carrier).
    power_dbm   Peak power in dBm into 50 ohm (same convention as the old
                notebook).  Give EXACTLY ONE of `power_dbm`, `vpp`, `amp`.
    vpp         Peak-to-peak voltage [V] at the peak of the envelope.
    amp         Peak voltage [V], SIGNED (amp=-0.1 gives an inverted pulse).
    start       When the pulse begins, measured from the start of the slot [s].
    duration    Length of the pulse [s].  None = until the end of the window.
    phase_deg   Carrier phase [deg].
    shape       "rect"  : constant amplitude (hard edges)
                "gauss" : Gaussian, sigma = `sigma` (default duration/6)
                "cos"   : raised-cosine (sin^2) bump, smooth edges
                "flat"  : constant amplitude with raised-cosine rise/fall of
                          length `rise` (flat-top pulse)
    rise        Rise/fall time [s] for shape="flat".
    sigma       Gaussian sigma [s] for shape="gauss".
    wave        Custom signal that gets windowed by `start`/`duration`/`shape`.
                * callable  wave(t, T) -> array in [-1, 1].  `t` = time since
                  the pulse start [s] (array), `T` = pulse duration [s].
                  Use the ready-made `chirp(f0, f1)` or write your own.
                  `duration=None` -> runs until the end of the window.
                * numpy array in [-1, 1], one value per DAC sample.  Its length
                  sets the duration (or must match `duration` if you give it).
                `freq` must stay 0 when `wave` is used.
    phase_ref   "window": phase is referenced to t=0 of the slot, so pulses of
                          the same frequency stay phase-coherent with each other
                          and with the other channel (best for qubit control).
                "pulse" : phase restarts at 0 at the beginning of every pulse.

    Times are rounded to the nearest DAC sample (101.7 ps).
    """
    freq: float = 0.0
    power_dbm: Optional[float] = None
    vpp: Optional[float] = None
    amp: Optional[float] = None
    start: float = 0.0
    duration: Optional[float] = None
    phase_deg: float = 0.0
    shape: str = "rect"
    rise: float = 0.0
    sigma: Optional[float] = None
    phase_ref: str = "window"
    wave: object = None

    def __post_init__(self):
        n_level = sum(x is not None for x in (self.power_dbm, self.vpp, self.amp))
        if n_level != 1:
            raise ValueError("Pulse: give exactly one of power_dbm=, vpp= or amp=")
        if self.wave is not None and self.freq != 0:
            raise ValueError("Pulse: with wave=..., leave freq at 0 (put the frequency in the wave)")
        if not (0 <= self.freq < DAC_SR / 2):
            raise ValueError(f"Pulse: freq must be in [0, {DAC_SR/2/1e9:.4f} GHz), got {self.freq}")
        if self.shape not in ("rect", "gauss", "cos", "flat"):
            raise ValueError("Pulse: shape must be 'rect', 'gauss', 'cos' or 'flat'")
        if self.phase_ref not in ("window", "pulse"):
            raise ValueError("Pulse: phase_ref must be 'window' or 'pulse'")
        if self.start < 0:
            raise ValueError("Pulse: start must be >= 0")

    @property
    def vpk(self) -> float:
        """Peak voltage of the carrier at the top of the envelope."""
        if self.amp is not None:
            return self.amp
        return self.vpp / 2 if self.vpp is not None else dbm_to_vpk(self.power_dbm)


def chirp(f0: float, f1: float, phase_deg: float = 0.0):
    """
    Linear frequency sweep from `f0` to `f1` [Hz] over the whole pulse duration.
    Use as  Pulse(wave=chirp(50*MHz, 70*MHz), duration=10*us, ...).
    """
    for f in (f0, f1):
        if not (0 <= f < DAC_SR / 2):
            raise ValueError(f"chirp: frequencies must be in [0, {DAC_SR/2/1e9:.4f} GHz)")

    def _chirp(t, T):
        k = (f1 - f0) / T
        return np.sin(2 * np.pi * (f0 * t + 0.5 * k * t ** 2) + np.deg2rad(phase_deg))
    return _chirp


def _envelope(p: Pulse, m: int) -> np.ndarray:
    """Amplitude envelope (0..1) of pulse `p` sampled on `m` DAC samples."""
    if p.shape == "rect":
        return np.ones(m)
    k = np.arange(m)
    if p.shape == "gauss":
        d = m / DAC_SR
        sigma = (p.sigma if p.sigma else d / 6) * DAC_SR      # in samples
        return np.exp(-0.5 * ((k - (m - 1) / 2) / sigma) ** 2)
    if p.shape == "cos":
        return np.sin(np.pi * (k + 0.5) / m) ** 2
    # "flat": flat-top with cosine edges
    r = int(round(p.rise * DAC_SR))
    if r < 1 or 2 * r > m:
        raise ValueError("Pulse(shape='flat'): need 0 < rise and 2*rise <= duration")
    env = np.ones(m)
    ramp = 0.5 * (1 - np.cos(np.pi * (np.arange(r) + 0.5) / r))
    env[:r], env[-r:] = ramp, ramp[::-1]
    return env


def render_pulses(n_samples: int, pulses: Union[Pulse, Iterable[Pulse]]) -> np.ndarray:
    """
    Render one or several pulses into an array of instantaneous VOLTS with
    `n_samples` points (sample i happens at t = i / DAC_SR).  Overlapping
    pulses are simply added.
    """
    if isinstance(pulses, Pulse):
        pulses = [pulses]
    out = np.zeros(n_samples)
    for p in pulses:
        i0 = int(round(p.start * DAC_SR))
        arr = None if (p.wave is None or callable(p.wave)) else np.asarray(p.wave, float).ravel()
        if arr is not None:
            m = arr.size
            if p.duration is not None and max(1, int(round(p.duration * DAC_SR))) != m:
                raise ValueError(f"Pulse: wave array has {m} samples ({m/DAC_SR/us:.4f} us) "
                                 f"but duration={p.duration/us:.4f} us; drop `duration`.")
            i1 = i0 + m
        else:
            i1 = n_samples if p.duration is None else i0 + max(1, int(round(p.duration * DAC_SR)))
        if i1 > n_samples:
            raise ValueError(
                f"Pulse ends at {i1/DAC_SR/us:.4f} us but the window is only "
                f"{n_samples/DAC_SR/us:.4f} us long (increase `window` or shorten the pulse).")
        if i0 >= i1:
            raise ValueError("Pulse starts after the end of the window.")
        t = np.arange(i0, i1) / DAC_SR
        if arr is not None:
            carrier = arr
        elif p.wave is not None:
            carrier = np.asarray(p.wave(t - t[0], (i1 - i0) / DAC_SR), float)
            if carrier.shape != t.shape:
                raise ValueError("Pulse: wave(t, T) must return an array the same shape as t")
        elif p.freq == 0:
            carrier = np.full(t.shape, np.cos(np.deg2rad(p.phase_deg)))
        else:
            t_carrier = t if p.phase_ref == "window" else t - t[0]
            carrier = np.sin(2 * np.pi * p.freq * t_carrier + np.deg2rad(p.phase_deg))
        out[i0:i1] += p.vpk * _envelope(p, i1 - i0) * carrier
    return out


# --------------------------------------------------------------------------- #
#  Packing into the layout the FPGA wants
# --------------------------------------------------------------------------- #
def pack_channel(slot0: np.ndarray, slot1: np.ndarray) -> np.ndarray:
    """
    Rearrange the two slots of ONE channel into the memory layout of the FPGA.

    The channel's samples (slot0 followed by slot1) are cut into blocks of 8.
    Even-numbered blocks go to RAM "A", odd-numbered blocks to RAM "B" (the
    hardware reads 8 samples from each RAM per word to feed 16 DAC samples per
    clock).  The result is  [RAM_A, RAM_B]  (each 2**18 int16 values).
    """
    full = np.concatenate([slot0, slot1])
    blocks = full.reshape(-1, 2, INTERLEAVE)        # (pair, A/B, 8 samples)
    ram_a = blocks[:, 0, :].reshape(-1)
    ram_b = blocks[:, 1, :].reshape(-1)
    return np.concatenate([ram_a, ram_b])


# --------------------------------------------------------------------------- #
#  The AWG class
# --------------------------------------------------------------------------- #
class AWG:
    """
    High-level interface to the RFSoC 4x2 AWG bitstream.

    Typical use (see the notebook for a full example):

        awg = AWG(window=6.65*us)
        awg.set_waveform("DAC_B", "z", Pulse(freq=20*MHz, power_dbm=-25))
        awg.plot()
        awg.connect()
        awg.upload()
        awg.start()
        ...
        awg.stop()
    """

    def __init__(self, window: float, bitfile="./awg_clock_syncd.bit",
                 dry_run: bool = False, lmk_freq: float = 245.76,
                 lmx_freq: float = 491.52, verbose: bool = True):
        """
        window    Length of ONE waveform slot [s] (max 26.67 us).  It is rounded
                  to a multiple of 16 samples (1.6276 ns); see `awg.window`.
        bitfile   Path to the FPGA bitstream (.bit, with its .hwh next to it),
                  OR an already-loaded pynq `Overlay` (then connect() reuses it:
                  it does not reload the bitstream nor reprogram the clocks, so
                  call xrfclk.set_ref_clks(...) yourself first).
        dry_run   True = never touch the hardware (design/plot on a laptop).
        lmk_freq, lmx_freq
                  Reference clocks [MHz] programmed into the board's LMK04828
                  and LMX2594 chips.  The register files come from
                  github.com/Xilinx/RFSoC-PYNQ (boards/RFSoC4x2/.../tics).
        """
        self.bitfile, self.dry_run, self.verbose = bitfile, dry_run, verbose
        self.lmk_freq, self.lmx_freq = lmk_freq, lmx_freq
        self._waves: dict = {}          # (channel name, slot index) -> volts
        self._gpio: dict = {}
        self._dma = None
        self._ol = None
        self._uploaded = False
        self._running = False
        self.slot_playing = 0           # 0 = "z", 1 = "x" (axi_control)
        self.set_window(window)

    # ---- window / time base ------------------------------------------------ #
    def set_window(self, duration: float) -> None:
        """(Re)define the slot length.  Clears all waveforms if it changes."""
        n = int(round(duration * DAC_SR / SAMPLES_PER_CLK)) * SAMPLES_PER_CLK
        if not (SAMPLES_PER_CLK <= n <= SLOT_SAMPLES):
            raise ValueError(
                f"window must be between {SAMPLES_PER_CLK/DAC_SR/ns:.2f} ns and "
                f"{SLOT_SAMPLES/DAC_SR/us:.2f} us, got {duration/us:.3f} us")
        if self._waves and n != getattr(self, "n_samples", n):
            warnings.warn("Window length changed: all waveforms were cleared.")
            self._waves.clear()
        self.n_samples = n
        self._uploaded = False
        self._log(f"Window = {self.window/us:.4f} us  ({n} samples, "
                  f"{self.max_points} clock cycles)  "
                  f"[requested {duration/us:.4f} us]")

    @property
    def window(self) -> float:
        """Actual slot length [s] after rounding."""
        return self.n_samples / DAC_SR

    @property
    def max_points(self) -> int:
        """Clock cycles (16-sample words) per slot.  The MAX_POINTS register gets
        this minus 1, because the hardware read counter includes its end value."""
        return self.n_samples // SAMPLES_PER_CLK

    @property
    def t(self) -> np.ndarray:
        """Time axis [s] of one slot (use it to build custom waveforms)."""
        return np.arange(self.n_samples) / DAC_SR

    def coherent_freq(self, freq: float) -> float:
        """Nearest frequency that fits an integer number of cycles in the window
        (handy for CW tones: no phase jump if the window is repeated)."""
        cycles = max(1, round(freq * self.window))
        return cycles / self.window

    # ---- defining waveforms ------------------------------------------------ #
    def set_waveform(self, channel, slot, content) -> None:
        """
        Define what a slot of a channel plays.

        channel  "DAC_A" or "DAC_B"   (also accepted: hardware index 2 / 0)
        slot     "z" or "x"           (also accepted: 0 / 1)
        content  * a `Pulse`, or a list of `Pulse` objects (summed), or
                 * a numpy array of instantaneous VOLTS (len <= window samples;
                   shorter arrays are padded with zeros), or
                 * None to clear the slot (outputs zero).
        """
        ch, sl = self._channel(channel), self._slot(slot)
        if content is None:
            self._waves.pop((ch, sl), None)
        elif isinstance(content, (Pulse, list, tuple)):
            self._waves[(ch, sl)] = render_pulses(self.n_samples, content)
        else:
            arr = np.asarray(content, dtype=float).ravel()
            if arr.size > self.n_samples:
                raise ValueError(f"Array has {arr.size} samples but the window holds "
                                 f"{self.n_samples}. Increase `window` or shorten it.")
            self._waves[(ch, sl)] = np.pad(arr, (0, self.n_samples - arr.size))
        self._uploaded = False

    def clear(self) -> None:
        """Remove every waveform (all slots output zero after the next upload)."""
        self._waves.clear()
        self._uploaded = False

    # ---- inspection -------------------------------------------------------- #
    def summary(self) -> None:
        """Print peak levels of every slot and warn about clipping."""
        print(f"Window: {self.window/us:.4f} us = {self.n_samples} samples "
              f"= {self.max_points} clock cycles (MAX_POINTS)")
        print(f"DAC full scale: +-{MAX_VPK:.3f} V peak (CW {MAX_DBM:.2f} dBm into 50 ohm)")
        for ch in STREAM_ORDER:
            for name, sl in (("z", 0), ("x", 1)):
                v = self._waves.get((ch, sl))
                if v is None:
                    print(f"  {ch} slot {name}: (empty)")
                    continue
                pk = np.max(np.abs(v))
                dbm = vpp_to_dbm(2 * pk) if pk > 0 else float("-inf")
                flag = "  <-- CLIPS!" if pk > MAX_VPK else ""
                print(f"  {ch} slot {name}: peak {pk*1e3:8.2f} mV  ({dbm:6.2f} dBm){flag}")

    def plot(self, tmin: float = 0.0, tmax: Optional[float] = None):
        """
        Plot what will be played.  `tmin`/`tmax` [s] zoom into a time range,
        e.g. awg.plot(1*us, 1.05*us) to see individual carrier cycles.
        """
        import matplotlib.pyplot as plt
        tmax = self.window if tmax is None else tmax
        sel = slice(int(tmin * DAC_SR), int(np.ceil(tmax * DAC_SR)))
        fig, axes = plt.subplots(len(STREAM_ORDER), 1, sharex=True, figsize=(10, 5))
        for ax, ch in zip(axes, STREAM_ORDER):
            for name, sl in (("z", 0), ("x", 1)):
                v = self._waves.get((ch, sl))
                if v is not None:
                    ax.plot(self.t[sel] / us, v[sel], lw=0.8, label=f"slot {name}")
            ax.axhline(MAX_VPK, color="r", ls=":", lw=0.8)
            ax.axhline(-MAX_VPK, color="r", ls=":", lw=0.8)
            ax.set_ylabel(f"{ch} [V]")
            ax.legend(loc="upper right")
        axes[-1].set_xlabel("time [us]  (dotted red = DAC full scale)")
        fig.tight_layout()
        return fig

    # ---- hardware ---------------------------------------------------------- #
    #
    # What the registers really do (from fpga/src/awg_BRAM_dual.v + fpga/bd/block_design.tcl):
    #   rst_n        -> proc_sys_reset_2 -> awg_BRAM_dual.rst_n. Resets ONLY the RAM
    #                   *write pointer*. It must be pulsed before EVERY upload: after one
    #                   full upload the pointer parks on the last RAM word, so a second
    #                   upload without reset is written into that single word and the
    #                   waveform never changes.
    #   write_enable 1 = RAM write mode (DMA data goes to RAM; DAC outputs read zeros).
    #   MAX_POINTS   read pointer runs 0..MAX_POINTS *inclusive*, so it must be set to
    #                (words per slot - 1). It loops forever, free running.
    #   axi_control  SLOT SELECT, not start/stop: 0 plays slot "z", 1 plays slot "x".
    #                It is OR-ed with the external control_trigger pin, so the pin can
    #                switch slots in real time.
    #   enable_ch0/2 output enable of DAC_B / DAC_A (0 = output zeros).
    #
    def connect(self):
        """
        Get access to the AWG registers and DMA.  With a bitfile path it also loads
        the bitstream and programs the clocks; with an Overlay object it reuses it.
        """
        if self.dry_run:
            self._log("[dry-run] connect(): nothing to do")
            return self
        try:
            from pynq import PL, Overlay
            from pynq.lib import AxiGPIO
            import xrfclk
        except ImportError as exc:
            raise ImportError("pynq / xrfclk not found. Run this on the RFSoC board, "
                              "or create the AWG with dry_run=True.") from exc

        if isinstance(self.bitfile, (str, bytes)) or hasattr(self.bitfile, "__fspath__"):
            PL.reset()
            self._ol = Overlay(str(self.bitfile))              # download bitstream
            xrfclk.set_ref_clks(lmk_freq=self.lmk_freq, lmx_freq=self.lmx_freq)
        else:                                                  # already-loaded Overlay
            self._ol = self.bitfile

        needed = ["rst_n", "write_enable", "MAX_POINTS", "axi_control",
                  "enable_ch0", "enable_ch2"]
        missing = [n for n in needed if n not in self._ol.ip_dict]
        if missing:
            raise RuntimeError(f"Bitstream has no GPIO block(s) named {missing}; "
                               f"is {self.bitfile} the right file?")
        self._gpio = {n: AxiGPIO(self._ol.ip_dict[n]).channel1 for n in needed}
        self._dma = self._ol.axi_dma_0

        for idx in CHANNELS.values():                          # known state: outputs off,
            self._write(f"enable_ch{idx}", 0)
        self._write("axi_control", self.slot_playing)          # slot z
        self._reset_writer()
        self._uploaded = False
        self._running = False
        return self

    def _reset_writer(self) -> None:
        """Pulse rst_n: rewinds the RAM write pointer (see the register notes above)."""
        self._write("rst_n", 0)
        time.sleep(0.001)                                      # >> 16 clocks at 100 MHz
        self._write("rst_n", 1)
        time.sleep(0.001)

    def upload(self) -> None:
        """
        Send all waveforms (both channels, both slots) to the board RAM and set the
        slot length.  Safe to call any time, as often as you like: the write pointer
        is reset first.  The outputs read zeros for the ~10 ms the transfer takes,
        then continue with the NEW waveforms (enables are left as they were).
        """
        image = self._dma_image().view(np.uint16)      # 2**20 words for 2 channels
        if self.dry_run:
            self._log(f"[dry-run] upload(): would DMA {image.size} words "
                      f"({image.nbytes/1024:.0f} KiB), MAX_POINTS={self.max_points - 1}")
            self._uploaded = True
            return
        self._need_connection()
        import pynq

        self._reset_writer()                           # write pointer back to RAM 0, word 0
        self._write("write_enable", 1)                 # RAM write mode ON (outputs -> 0)
        try:
            with pynq.allocate(shape=(image.size,), dtype=np.uint16) as buf:  # freed on exit
                buf[:] = image
                t0 = time.time()
                with warnings.catch_warnings():        # PYNQ's own SG-DMA wait() calls a
                    warnings.filterwarnings(            # deprecated .close(): harmless noise
                        "ignore", message=r".*close no longer functional")
                    self._dma.sendchannel.transfer(buf)
                    self._dma.sendchannel.wait()
                self._log(f"DMA upload took {time.time() - t0:.4f} s")
            self._write("MAX_POINTS", self.max_points - 1, 0xFFFFFFFF)   # inclusive counter
        finally:
            self._write("write_enable", 0)             # RAM write mode OFF -> play again
        self._uploaded = True

    def select_slot(self, slot) -> None:
        """Choose which slot plays: "z" (0) or "x" (1).  Instant, no re-upload.
        (If the external control_trigger pin is high, slot "x" plays regardless.)"""
        self.slot_playing = self._slot(slot)
        self._write("axi_control", self.slot_playing)

    def start(self, channels: Optional[Sequence] = None, slot=None) -> None:
        """
        Turn outputs on.  `channels` = e.g. ["DAC_A"]; default = every channel that
        has a waveform.  `slot` = "z"/"x" to also choose the slot (default: keep).
        """
        if not self._uploaded:
            raise RuntimeError("Nothing uploaded yet (or waveforms changed): call upload() first.")
        if slot is not None:
            self.select_slot(slot)
        if channels is None:
            channels = [c for c in STREAM_ORDER if any(k[0] == c for k in self._waves)]
        wanted = {self._channel(c) for c in channels}
        for name in ("DAC_A", "DAC_B"):
            self._write(f"enable_ch{CHANNELS[name]}", 1 if name in wanted else 0)
        self._running = bool(wanted)
        self._log(f"Playing slot {'zx'[self.slot_playing]} on {sorted(wanted)}")

    def stop(self) -> None:
        """Outputs off (they output zeros).  The RAM content is kept."""
        for idx in CHANNELS.values():
            self._write(f"enable_ch{idx}", 0)
        self._running = False
        self._log("Stopped.")

    @property
    def state(self) -> str:
        """"playing" (outputs on, waveforms current), "stale" (outputs on but you edited
        waveforms since the last upload), "ready" (uploaded, outputs off) or "stopped"."""
        if self._running:
            return "playing" if self._uploaded else "stale"
        return "ready" if self._uploaded else "stopped"

    def play(self, channels: Optional[Sequence] = None, slot=None) -> None:
        """Upload the current waveforms and turn the outputs on (use after any edit)."""
        self.upload()
        self.start(channels, slot)

    # ---- internals --------------------------------------------------------- #
    def _dma_image(self) -> np.ndarray:
        """Full int16 stream sent over DMA: [DAC_B (RAM A, RAM B), DAC_A (RAM A, RAM B)]."""
        parts = []
        for ch in STREAM_ORDER:
            slots = []
            for sl in range(N_SLOTS):
                padded = np.zeros(SLOT_SAMPLES, dtype=np.int16)    # zero padding to RAM depth
                v = self._waves.get((ch, sl))
                if v is not None:
                    padded[: self.n_samples] = self._to_codes(v, f"{ch} slot {sl}")
                slots.append(padded)
            parts.append(pack_channel(*slots))
        return np.concatenate(parts)

    @staticmethod
    def _to_codes(volts: np.ndarray, label: str) -> np.ndarray:
        """Volts -> int16 DAC codes, clipping (with a warning) if too large."""
        codes = volts / MAX_VPK * DAC_FULL_SCALE_CODE
        peak = np.max(np.abs(codes))
        if peak > DAC_FULL_SCALE_CODE - 1:
            warnings.warn(f"{label}: peak {np.max(np.abs(volts)):.3f} V exceeds the DAC "
                          f"full scale (+-{MAX_VPK:.3f} V, {MAX_DBM:.2f} dBm). Clipping!")
        return np.clip(np.rint(codes), -DAC_FULL_SCALE_CODE, DAC_FULL_SCALE_CODE - 1).astype(np.int16)

    def _write(self, reg: str, value: int, mask: int = 0x1) -> None:
        if self.dry_run:
            self._log(f"[dry-run] {reg} <- {value}")
        else:
            self._need_connection()
            self._gpio[reg].write(value, mask)

    def _need_connection(self) -> None:
        if not self._gpio:
            raise RuntimeError("Not connected: call awg.connect() first.")

    @staticmethod
    def _channel(ch) -> str:
        if isinstance(ch, str) and ch.upper() in CHANNELS:
            return ch.upper()
        for name, idx in CHANNELS.items():
            if ch == idx:
                return name
        raise ValueError(f"Unknown channel {ch!r}; use 'DAC_A' or 'DAC_B'")

    @staticmethod
    def _slot(slot) -> int:
        if slot in SLOTS:
            return SLOTS[slot]
        raise ValueError(f"Unknown slot {slot!r}; use 'z' or 'x' (or 0 / 1)")

    def _log(self, msg: str) -> None:
        if self.verbose:
            print(msg)


def print_clocks() -> None:
    """Diagnostic: print the PS clock frequencies (needs pynq)."""
    from pynq import ps
    print("CPU Clock Frequency:", ps.Clocks.cpu_mhz, "MHz")
    for i in range(4):
        print(f"FCLK{i} Clock Frequency:", getattr(ps.Clocks, f"fclk{i}_mhz"), "MHz")
