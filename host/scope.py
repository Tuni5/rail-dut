"""The RTM2034 for the server: one open connection, one lock, one capture per load phase.

All instrument work is done by bench/scope.py from power-rail-bench, unchanged
(verified there on this RTM2034 on 2026-09-20: socket, error queue, PNG
screenshot, ASCII waveform). This module adds only what the sequence needs:
  - the connection stays open between captures
  - capture(): freeze the current acquisition, save screenshot and waveform, run again;
    if the connection is gone (bench 2026-11-25: WinError 10054, the scope reset the socket
    and every later capture failed the same way), open it once more and repeat the capture
  - analyse(): switching frequency and duty cycle from the waveform

The switching frequency is NOT the scope's own FREQuency measurement: in
discontinuous mode the ringing after each pulse counts as extra edges (bench
2026-11-22: the scope showed 270-282 kHz on the fakes, the real pulse rate is
about 40-45 kHz). analyse() uses hysteresis and a minimum pulse width instead.
"""

import contextlib
import logging
import threading
import time
from pathlib import Path

import numpy as np

import sim
from bench import config
from bench import scope as rtm
from bench.scope import ScopeError

LOG = logging.getLogger("scope")

CHANNEL = 1                      # the probe on the switch node goes to scope channel 1
MIN_HIGH_S = 0.5e-6              # high for less than this is ringing, not a switching pulse
# Thresholds as fractions of the span from the low level (diode drop) to VIN. Both sit high: in
# discontinuous mode the node rings around VOUT, at 8 V in that is most of the span, and the first
# ring peaks can even exceed VIN. So the top is VIN as the INA226 measured it during the phase, not
# the record's maximum. A pulse starts above HI and ends below LO; ringing crosses HI only for a
# few 100 ns and is dropped by MIN_HIGH_S.
HYST_HI, HYST_LO = 0.85, 0.6
# Time base fitted per capture, so the window shows enough pulses for f_sw and the period spread:
# the fakes run at 34-89 kHz with skipped pulses, the genuine parts at 150-164 kHz (bench
# 2026-11-25: at 5 us/div the fakes showed 1-2 pulses). Aim for ~8, accept 5..20; the scope
# keeps the last time base, so the next phase usually fits at once.
TDIV_STEPS = [2e-6, 5e-6, 10e-6, 20e-6, 50e-6, 100e-6]
PULSES_MIN, PULSES_MAX, PULSES_AIM = 5, 20, 8
FIT_TRIES = 3
RUN_SETTLE_S = 0.6               # after a new time base: a few acquisitions before STOP


class Scope:
    def __init__(self):
        self._lock = threading.Lock()
        self._stack: contextlib.ExitStack | None = None
        self._inst = None
        self.resource: str | None = None
        self.identity: str | None = None

    @property
    def connected(self) -> bool:
        return self._inst is not None

    def open(self, resource: str | None) -> str:
        """VISA resource, "SIM", or None for config.SCOPE_RESOURCE."""
        self.close()
        resource = (resource or "").strip() or config.SCOPE_RESOURCE
        with self._lock:
            stack = contextlib.ExitStack()
            try:
                if resource.upper() == sim.PORT:
                    inst, resource = sim.SimScope(), sim.PORT
                else:
                    inst = stack.enter_context(rtm.open_scope(resource))
                identity = rtm.idn(inst)
                if "RTM" not in identity.upper() and resource != sim.PORT:
                    raise ScopeError(f"{resource} answered {identity!r}, not an R&S RTM")
                rtm.errors(inst)                      # start with an empty error queue
            except Exception:
                stack.close()
                raise
            self._stack, self._inst = stack, inst
            self.resource, self.identity = resource, identity
        return identity

    def close(self) -> None:
        with self._lock:
            if self._inst is not None:
                with contextlib.suppress(Exception):
                    rtm.command(self._inst, rtm.CMD_RUN)    # leave it running, as found
            if self._stack is not None:
                self._stack.close()
            self._stack = self._inst = None
            self.resource = self.identity = None

    def capture(self, png_path: Path, vin: float | None = None) -> tuple[Path, np.ndarray, np.ndarray]:
        """Freeze what is on the screen, save it as PNG, read channel 1, run again.
        The waveform and the picture are the same acquisition. A failed capture on the real scope
        is repeated once on a fresh connection; if that cannot be opened the scope counts as
        disconnected, so the remaining phases fail at once instead of each waiting for a timeout."""
        with self._lock:
            if self._inst is None:
                raise ScopeError("scope is not connected")
            try:
                return self._capture_once(png_path, vin)
            except ScopeError as first:
                if self.resource == sim.PORT:
                    raise
                LOG.warning("capture failed (%s), reconnecting to %s", first, self.resource)
                self._reopen()
                try:
                    return self._capture_once(png_path, vin)
                except ScopeError as second:
                    raise ScopeError(f"{second} (also after reconnect; first try: {first})")

    def _capture_once(self, png_path: Path, vin: float | None) -> tuple[Path, np.ndarray, np.ndarray]:
        """One capture; any failure comes back as ScopeError naming the step. The waveform is read
        first; with too few or too many pulses the time base is changed, the scope runs again and
        the capture repeats. Picture and waveform are always the same acquisition."""
        inst, step = self._inst, "STOP"
        try:
            rtm.command(inst, rtm.CMD_STOP)
            step = "waveform"
            t, v = rtm.waveform(inst, CHANNEL)
            for _ in range(FIT_TRIES):
                want = fit_timebase(t, analyse(t, v, vin))
                if want is None:
                    break
                step = f"time base {want * 1e6:g} us/div"
                span_before = t[-1] - t[0]
                rtm.setup_timebase(inst, want)
                rtm.command(inst, rtm.CMD_RUN)
                time.sleep(RUN_SETTLE_S)
                rtm.command(inst, rtm.CMD_STOP)
                step = "waveform"
                t, v = rtm.waveform(inst, CHANNEL)
                if abs((t[-1] - t[0]) - span_before) < 1e-3 * span_before:
                    break                          # the window did not change (replayed SIM data)
            step = "screenshot"
            path = rtm.screenshot(inst, str(png_path))
        except Exception as error:                 # socket errors arrive as OSError, VISA as VisaIOError
            raise ScopeError(f"{step}: {error}") from error
        finally:
            with contextlib.suppress(Exception):
                rtm.command(inst, rtm.CMD_RUN)
        return Path(path), t, v

    def _reopen(self) -> None:
        """New connection to the same resource. Caller holds the lock."""
        if self._stack is not None:
            with contextlib.suppress(Exception):
                self._stack.close()
        self._stack = self._inst = None
        stack = contextlib.ExitStack()
        try:
            inst = stack.enter_context(rtm.open_scope(self.resource))
            rtm.idn(inst)
            rtm.errors(inst)
        except Exception as error:
            stack.close()
            self.identity = None
            raise ScopeError(f"connection lost and reconnect to {self.resource} failed: {error}")
        self._stack, self._inst = stack, inst
        LOG.info("reconnected to %s", self.resource)


def fit_timebase(t, result: dict) -> float | None:
    """Next time base (s/div) when the window shows too few or too many pulses, else None."""
    if "no switching" in result.get("error", "") or len(t) < 2:
        return None
    n = result.get("pulses", 0)
    if PULSES_MIN <= n <= PULSES_MAX:
        return None
    tdiv = (t[-1] - t[0]) / 10
    need = tdiv * PULSES_AIM / max(n, 1)           # time base that would show ~PULSES_AIM pulses
    if n < PULSES_MIN:
        bigger = [s for s in TDIV_STEPS if s >= need * 0.95 and s > tdiv * 1.05]
        return bigger[0] if bigger else None
    smaller = [s for s in TDIV_STEPS if s <= need * 1.05 and s < tdiv * 0.95]
    return smaller[-1] if smaller else None


def analyse(t: np.ndarray, v: np.ndarray, vin: float | None = None) -> dict:
    """Switching node: pulse rate, duty cycle, levels. Ringing is not counted as pulses.
    vin: input voltage during the capture (INA226); without it the record's 90th percentile."""
    t, v = np.asarray(t, dtype=float), np.asarray(v, dtype=float)
    if len(v) < 10:
        return {"error": "no waveform"}
    vmin, vmax = float(v.min()), float(v.max())
    # v_mean: a buck's switch node averages VOUT (no DC across the inductor); far off means the
    # record is not a whole number of periods, or the probe is not on the switch node
    out = {"vpp": round(vmax - vmin, 3), "v_high": round(vmax, 3), "v_low": round(vmin, 3),
           "v_mean": round(float(v.mean()), 3)}
    if vmax - vmin < 1.0:
        return {**out, "error": "no switching on screen"}
    bottom = float(np.percentile(v, 2))
    # top: the on level. Not above VIN, but also not above the trace's own high plateau (98th
    # percentile): at 8 V in the switch drops ~1 V, the plateau sat at 6.8 V of 7.9 V, and a threshold
    # at 85 % of VIN missed pulses (bench 2026-11-25: 129-170 kHz reported where the scope showed
    # 153 kHz). Ringing peaks above VIN are too short to move the 98th percentile.
    top = min(vin, float(np.percentile(v, 98))) if vin and vin > bottom + 1.0 else float(np.percentile(v, 90))
    hi = bottom + HYST_HI * (top - bottom)
    lo = bottom + HYST_LO * (top - bottom)
    pulses, high, t_rise = [], bool(v[0] > hi), None
    for k in range(1, len(v)):
        if not high and v[k] > hi:
            high, t_rise = True, t[k]
        elif high and v[k] < lo:
            high = False
            if t_rise is not None and t[k] - t_rise >= MIN_HIGH_S:
                pulses.append((t_rise, t[k]))
            t_rise = None
    out["tdiv_us"] = round((t[-1] - t[0]) / 10 * 1e6, 3)
    out["pulses"] = len(pulses)
    if len(pulses) < 3:
        return {**out, "error": f"{len(pulses)} pulses on screen, need 3: slower time base"}
    rises = np.array([p[0] for p in pulses])
    periods = np.diff(rises)
    period = float(periods.mean())
    width = float(np.mean([p[1] - p[0] for p in pulses]))
    # Period spread: a genuine LM2596 runs on a fixed oscillator (spread well under 1 %); the fakes
    # skip pulses and change their period from pulse to pulse (bench 2026-11-25: 12-30 us on M1).
    return {**out, "f_sw": round(1 / period, 1), "duty": round(width / period, 4),
            "period_min_us": round(float(periods.min()) * 1e6, 3), "period_max_us": round(float(periods.max()) * 1e6, 3),
            "spread": round(float(periods.max() - periods.min()) / period, 4)}


def save_csv(path: Path, t: np.ndarray, v: np.ndarray) -> None:
    np.savetxt(path, np.column_stack([t, v]), delimiter=",", header="t_s,v", comments="", fmt="%.9g")
