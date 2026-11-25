"""Sampler and channel/load sequence. All numbers are computed here; the GUI only draws.

Sampler: reads both INA226 every LOG_S while the Pico is connected. That also
feeds the firmware watchdog. Each sample carries the label the sequence set.

Sequence: per channel (Load Select jumper moved by hand), per VIN point, per load phase.
Phase mean = samples of that phase after SETTLE_S, so the label lag is gone.
"""

import threading
import time
from collections import deque
from dataclasses import dataclass, field, asdict
from pathlib import Path

from pico import PicoError, PicoLink
import runfile
from scope import Scope, ScopeError, analyse
from supply import Supply
from bench.psu import PsuError

LOG_S = 0.25
SETTLE_S = 0.4            # samples this soon after a phase change are not averaged
ZERO_S = 1.0              # Iload offset: averaged this long with EN and LOAD off
SUPPLY_STEP_S = 1.5       # typical time for the supply to settle on a new setpoint (plan only)
SCOPE_CAPTURE_S = 2.0     # screenshot + ASCII waveform over LAN: plan until the first capture is timed
VIN_SETTLE_TIMEOUT_S = 10.0   # supply readout must reach a new setpoint within this
# Stepping VIN down: the OWON cannot sink current, so with no load the input capacitors
# stay charged for minutes (24 V -> 8 V failed after 2 s on the bench, 2026-11-22).
# So the next, lower setpoint is sent while the last load of a point is still on, and the
# load stays on until VIN is down (Load Select still sits on this channel at that moment).
# If VIN is still too high when the next point starts, _set_vin discharges with LOAD1.
DISCHARGE_MARGIN_V = 0.3      # discharge until VIN is this close above the new setpoint
DISCHARGE_TIMEOUT_S = 20.0
ZERO_MAX_A = 0.010        # an offset larger than this is a fault, not an offset
EN_SETTLE_S = 0.6         # after EN on, before the output check
MIN_VLOAD_V = 3.0         # below this after EN: no output on the load (Load Select? regulator?)

# what each phase switches and the LED color it shows
# red is reserved for faults (watchdog, limit, run error), so no phase uses it
PHASES = {"L1": ((1,), "magenta"), "L2": ((2,), "orange"), "L12": ((1, 2), "white")}
PHASE_NAMES = {"L1": "LOAD1", "L2": "LOAD2", "L12": "LOAD1+2"}


class Sampler(threading.Thread):
    def __init__(self, pico: PicoLink, limits: dict):
        super().__init__(daemon=True)
        self.pico = pico
        self.limits = limits
        self.samples = deque(maxlen=20000)
        self.count = 0                    # index of the next sample, never resets
        self.label: dict = {}
        self.error: str | None = None
        self.trip: str | None = None      # set when a limit switched everything off
        self.on_sample = None             # run writes its raw log through this
        self.iload_offset = 0.0           # A, subtracted from every Iload reading (zero())
        self._lock = threading.Lock()
        self._t0 = time.time()

    def run(self):
        while True:
            t = time.time()
            if self.pico.connected:
                try:
                    vin, iin, vload, iload = self.pico.read()
                    iload = round(iload - self.iload_offset, 5)
                    s = {"i": self.count, "t": round(t - self._t0, 3), "vin": vin, "iin": iin,
                         "vload": vload, "iload": iload, **self.label}
                    with self._lock:
                        self.samples.append(s)
                        self.count += 1
                    self.error = None
                    self._check(iin, iload)
                    if self.on_sample:
                        try:
                            self.on_sample(s)
                        except ValueError:
                            pass              # raw log closed in between
                except PicoError as error:
                    self.error = str(error)
            time.sleep(max(0.0, LOG_S - (time.time() - t)))

    def _check(self, iin, iload):
        if iin > self.limits["iin_max_a"] or iload > self.limits["iload_max_a"]:
            try:
                self.pico.off()
                self.pico.led("red")
            except PicoError:
                pass
            self.trip = f"limit: Iin {iin:.3f} A / Iload {iload:.3f} A, everything switched off"

    def since(self, index: int, limit: int = 2000) -> list[dict]:
        with self._lock:
            if not self.samples:
                return []
            first = self.samples[0]["i"]
            start = max(index, first, self.count - limit)
            return list(self.samples)[start - first:]

    def window(self, t_from: float, t_to: float) -> list[dict]:
        with self._lock:
            return [s for s in self.samples if t_from <= s["t"] < t_to]

    def now(self) -> float:
        return round(time.time() - self._t0, 3)

    def zero(self) -> float:
        """Iload offset with everything off. Caller makes sure EN and LOAD are off."""
        old = self.iload_offset
        t0 = self.now() + 0.3
        time.sleep(0.3 + ZERO_S)
        got = self.window(t0, self.now())
        if len(got) < 2:
            raise PicoError("Iload zero: no samples")
        raw = old + sum(s["iload"] for s in got) / len(got)
        if abs(raw) > ZERO_MAX_A:
            raise PicoError(f"Iload zero: {raw*1000:.1f} mA with the load off, more than "
                            f"{ZERO_MAX_A*1000:.0f} mA; something is still drawing current")
        self.iload_offset = raw
        return raw

    def mean_last(self, seconds: float) -> dict:
        got = self.window(self.now() - seconds, self.now() + 1)
        if not got:
            raise PicoError("no samples")
        return {k: sum(s[k] for s in got) / len(got) for k in ("vin", "iin", "vload", "iload")}


@dataclass
class Settings:
    channels: list[int]
    use_supply: bool
    vin_points: list[float]
    phases: list[str]
    phase_s: float = 2.5
    gap_s: float = 1.0
    jumper_prompt: bool = True
    supply_off_at_end: bool = True
    scope_capture: bool = False   # one RTM2034 snapshot per load phase, probe on the channel's switch node
    name: str = ""                # free text, e.g. "Board_5-TI_LM2596_adj"; the folder stays the start time
    board: int | None = None      # board number of the campaign (board.toml [boards]); sets CH4's name


@dataclass
class Run:
    settings: Settings
    stamp: str
    state: str = "running"            # running, waiting, done, aborted, error
    message: str = ""
    step: int = 0
    steps: int = 0
    current: dict = field(default_factory=dict)
    channel_state: dict = field(default_factory=dict)
    rows: list = field(default_factory=list)
    iload_offset: float | None = None
    t0: float = 0.0                   # sampler time at the start: x = 0 of the run graphs
    plan_s: float = 0.0               # planned duration without jumper waits
    waited_s: float = 0.0             # time spent waiting for the jumper so far
    capture_s: list = field(default_factory=list)   # measured duration of each scope capture
    t_end: float | None = None        # sampler time when the run ended
    samples: list = field(default_factory=list)     # every INA226 sample of the run
    waves: dict = field(default_factory=dict)       # scope waveform per snapshot, key = picture name
    meta: dict = field(default_factory=dict)        # channel names, instruments, as at the start
    merge_into: str | None = None     # repeat run: stamp of the run whose channel it replaces when done


class Sequencer:
    def __init__(self, pico: PicoLink, supply: Supply, sampler: Sampler, runs_dir: Path, channels: dict,
                 scope: Scope):
        self.pico, self.supply, self.sampler, self.scope = pico, supply, sampler, scope
        self.runs_dir = runs_dir
        self.channels = channels
        self.run: Run | None = None
        self._thread = None
        self._stop = threading.Event()
        self._go = threading.Event()
        self._skip = False

    @property
    def active(self) -> bool:
        return self.run is not None and self.run.state in ("running", "waiting")

    # --- control from the API -------------------------------------------------
    def start(self, settings: Settings, channels: dict | None = None, merge_into: str | None = None) -> Run:
        """channels: names for this run (CH4 from the chosen board); default the board file's.
        merge_into: stamp of a finished run whose channel this (one-channel) run replaces when done."""
        if self.active:
            raise RuntimeError("a run is already active")
        if not self.pico.connected:
            raise RuntimeError("Pico is not connected")
        if settings.use_supply and not self.supply.connected:
            raise RuntimeError("supply is not connected (or untick 'Use supply')")
        if settings.scope_capture and not self.scope.connected:
            raise RuntimeError("scope is not connected (or untick 'scope snapshot per phase')")
        stamp = time.strftime("%Y%m%d-%H%M%S")
        n_vin = len(settings.vin_points) if settings.use_supply else 1
        run = Run(settings=settings, stamp=stamp,
                  steps=len(settings.channels) * n_vin * len(settings.phases),
                  channel_state={ch: "waiting" for ch in settings.channels})
        run.meta = {"channels": channels or self.channels, "pico": self.pico.identity, "pico_info": self.pico.info,
                    "supply": self.supply.identity if settings.use_supply else None,
                    "scope": self.scope.identity if settings.scope_capture else None,
                    # simulated scope: pictures are replayed or modelled, never measurements
                    "scope_sim": bool(settings.scope_capture and self.scope.resource == "SIM")}
        run.merge_into = merge_into
        run.t0 = self.sampler.now()
        run.plan_s = plan_seconds(settings)
        self.run = run
        self.sampler.trip = None
        self._stop.clear()
        self._thread = threading.Thread(target=self._main, daemon=True)
        self._thread.start()
        return run

    def stop(self):
        self._stop.set()
        self._go.set()

    def proceed(self, skip: bool = False):
        self._skip = skip
        self._go.set()

    # --- the run --------------------------------------------------------------
    def _main(self):
        run, s = self.run, self.run.settings
        final = ("error", "unexpected stop")
        self.sampler.on_sample = None
        try:
            self.pico.off()
            run.current = {"phase": "zero"}
            run.iload_offset = self.sampler.zero()
            self.sampler.on_sample = run.samples.append
            self._save()
            if s.use_supply:
                self.supply.prepare()
            for ch in s.channels:
                self._channel(ch)
                if self._stop.is_set():
                    break
            final = ("aborted", "stopped") if self._stop.is_set() else ("done", "")
            self.pico.led("green" if final[0] == "done" else "blue")
        except (PicoError, PsuError, Abort) as error:
            final = ("error", str(error))
            try:
                self.pico.led("red")
            except PicoError:
                pass
        except Exception as error:
            final = ("error", f"{type(error).__name__}: {error}")
        finally:
            self._safe_off()
            if s.use_supply and s.supply_off_at_end and self.supply.connected:
                try:
                    self.supply.output(False)
                    self.supply.read_state()
                except PsuError:
                    pass
            self.sampler.label = {}
            self.sampler.on_sample = None
            run.current = {}
            run.t_end = self.sampler.now()
            run.state, run.message = final
            self._save()
            self._merge()

    def _channel(self, ch: int):
        run, s = self.run, self.run.settings
        run.channel_state[ch] = "running"
        if s.jumper_prompt:
            probe = f" and the scope probe on the CH{ch} switch node" if s.scope_capture else ""
            answer = self._ask(ch, "jumper", f"Set the Load Select jumper to CH{ch}{probe}, then continue")
            if answer == "stop":
                return
            if answer == "skip":
                run.channel_state[ch] = "skipped"
                run.step += (len(s.vin_points) if s.use_supply else 1) * len(s.phases)
                return
        points = s.vin_points if s.use_supply else [None]
        later = s.channels[s.channels.index(ch) + 1:]
        for k, vin_set in enumerate(points):
            if self._stop.is_set():
                return
            if vin_set is not None:
                self._set_vin(ch, vin_set)
                if self._stop.is_set():
                    return
            # the setpoint after this one: next VIN of this channel, or the first of the next channel
            nxt = points[k + 1] if k + 1 < len(points) else (points[0] if later else None)
            result = self._point(ch, vin_set, nxt)
            if result == "stop":
                return
            if result == "skip":
                run.channel_state[ch] = "skipped"
                run.step += len(s.phases) * (len(points) - points.index(vin_set))
                return
        run.channel_state[ch] = "done"

    def _set_vin(self, ch: int, vin_set: float):
        """New VIN setpoint. Going down, discharge the input through this channel and LOAD1
        (Load Select is on this channel here, the prompt came before)."""
        run = self.run
        self.supply.set_voltage(vin_set)
        was_on = self.supply.is_on()
        # Also with the output off: right after a run that ended at 24 V the input capacitors still
        # hold ~20 V, and switching on at 8 V then fails in psu.py's ramp check (2026-11-25, a repeat
        # started right after its run). So discharge first in both cases.
        vin_now = self.pico.read()[0]
        if vin_now > vin_set + DISCHARGE_MARGIN_V:
            self.sampler.label = {"ch": ch, "vin_set": vin_set, "phase": "discharge"}
            run.current = {"ch": ch, "vin_set": vin_set, "phase": "discharge"}
            self.pico.led("blue")
            self.pico.en(ch, True)
            self.pico.load(1, True)
            t0 = time.time()
            try:
                while vin_now > vin_set + DISCHARGE_MARGIN_V:
                    if time.time() - t0 > DISCHARGE_TIMEOUT_S:
                        raise Abort(f"VIN still {vin_now:.2f} V after {DISCHARGE_TIMEOUT_S:.0f} s of discharging "
                                    f"towards {vin_set} V (Load Select on CH{ch}?)")
                    self._sleep(0.1)
                    if self._stop.is_set():
                        return
                    vin_now = self.pico.read()[0]
            finally:
                self.pico.load(1, False)
                self.pico.en(ch, False)
        if not was_on:
            self.supply.output(True)                  # ramps up and waits (psu.py)
            self.supply.read_state()
            return
        self.supply.wait_for(vin_set, VIN_SETTLE_TIMEOUT_S)
        self.supply.read_state()

    def _point(self, ch: int, vin_set, next_vin=None):
        run, s = self.run, self.run.settings
        label = {"ch": ch, "vin_set": vin_set}
        # No output on the load after EN: mostly Load Select not moved to this channel. The run
        # does not go on to the next channel by itself; it waits for retry (after fixing) or skip.
        while True:
            self.sampler.label = {**label, "phase": "EN"}
            run.current = {**label, "phase": "EN"}
            self.pico.en(ch, True)
            self._sleep(EN_SETTLE_S)
            if self._stop.is_set():
                return "stop"
            vload = self.pico.read()[2]
            if vload >= MIN_VLOAD_V:
                break
            self.pico.en(ch, False)
            answer = self._ask(ch, "no_output", f"No output on the load from CH{ch}: VLOAD {vload:.3f} V. "
                               f"Is Load Select on CH{ch}? Fix it, then retry", vin_set=vin_set, vload=round(vload, 3))
            if answer == "stop":
                return "stop"
            if answer == "skip":
                run.rows.append({**label, "phase": "EN", "error": f"skipped: no output on the load "
                                 f"(VLOAD {vload:.3f} V; Load Select on CH{ch}? regulator on?)"})
                return "skip"
        for phase in s.phases:
            if self._stop.is_set():
                break
            loads, color = PHASES[phase]
            self.sampler.label = {**label, "phase": phase}
            run.current = {**label, "phase": phase}
            self.pico.led(color)
            for n in (1, 2):
                self.pico.load(n, n in loads)
            t0 = self.sampler.now()
            self._sleep(s.phase_s)
            t1 = self.sampler.now()
            row = {**label, "phase": phase, **mean(self.sampler.window(t0 + SETTLE_S, t1))}
            if s.scope_capture:
                tc = time.time()
                row["scope"] = self._snapshot(ch, vin_set, phase, row.get("vin"))   # load still on: same operating point
                # the time axis follows the real capture time instead of the 2 s guess
                run.capture_s.append(time.time() - tc)
                run.plan_s = plan_seconds(s, sum(run.capture_s) / len(run.capture_s))
            run.rows.append(row)
            run.step += 1
            self._save()
            if phase == s.phases[-1] and next_vin is not None and vin_set is not None and next_vin < vin_set:
                self._step_down_under_load(ch, vin_set, next_vin)
            self.pico.load(1, False)
            self.pico.load(2, False)
        self.pico.en(ch, False)
        self.pico.led("blue")
        self.sampler.label = {**label, "phase": "gap"}
        run.current = {**label, "phase": "gap"}
        self._sleep(s.gap_s)
        return True

    def _snapshot(self, ch: int, vin_set, phase: str, vin) -> dict:
        """Screenshot and waveform of the switch node for this phase. A failed capture is
        recorded with the phase and does not stop the run."""
        name = f"CH{ch}_{'manual' if vin_set is None else f'{vin_set:g}V'}_{PHASE_NAMES[phase]}"
        folder = runfile.folder(self.runs_dir, self.run.stamp)
        folder.mkdir(parents=True, exist_ok=True)
        try:
            path, t, v = self.scope.capture(folder / f"{name}.png", vin)
            self.run.waves[path.name] = runfile.wave_entry(t, v)
            return {"png": path.name, **analyse(t, v, vin)}
        except (ScopeError, OSError) as error:
            return {"error": f"capture failed: {error}"}

    def _step_down_under_load(self, ch: int, vin_set: float, target: float):
        """Send the lower setpoint while the load is still on and keep it on until VIN follows.
        Not reaching it in time is no error here: _set_vin finishes the job at the next point."""
        self.sampler.label = {"ch": ch, "vin_set": vin_set, "phase": "discharge"}
        self.run.current = {"ch": ch, "vin_set": vin_set, "phase": "discharge"}
        self.supply.set_voltage(target)
        t0 = time.time()
        while time.time() - t0 < DISCHARGE_TIMEOUT_S and not self._stop.is_set():
            if self.pico.read()[0] <= target + DISCHARGE_MARGIN_V:
                return
            self._sleep(0.1)

    def _sleep(self, seconds):
        end = time.time() + seconds
        while time.time() < end:
            if self._stop.is_set():
                return
            if self.sampler.trip:
                raise Abort(self.sampler.trip)
            time.sleep(0.05)

    def _safe_off(self):
        try:
            self.pico.off()
        except PicoError:
            pass

    def _ask(self, ch: int, wait: str, message: str, **extra) -> str:
        """Hold the run until the user answers in the GUI: "go" (continue / retry), "skip" or "stop".
        The waiting time is no measurement time: labelled "jumper", cut out of the graphs."""
        run = self.run
        run.state, run.message = "waiting", message
        run.current = {"ch": ch, "wait": wait, **extra}
        self.pico.led("cyan")
        self.sampler.label = {"phase": "jumper"}
        self._skip = False
        self._go.clear()
        t_wait = time.time()
        self._go.wait()
        run.waited_s += time.time() - t_wait
        run.state, run.message = "running", ""
        if self._stop.is_set():
            return "stop"
        return "skip" if self._skip else "go"

    def rename(self, name: str):
        """Name of the current run; written to its file at once."""
        self.run.settings.name = name
        self._save()

    def _merge(self):
        """Repeat run: when its channel was measured completely, it replaces that channel in the
        original run. Otherwise the original stays untouched (the repeat is still saved on its own)."""
        run = self.run
        if not run.merge_into:
            return
        ch = run.settings.channels[0]
        if run.state != "done" or run.channel_state.get(ch) != "done":
            why = f" ({run.message})" if run.message else ""
            run.message = f"CH{ch} not complete{why}: run {run.merge_into} left as it was"
        else:
            try:
                note = runfile.merge_channel(self.runs_dir, run.merge_into, to_file(run), ch, table, line_regulation)
                run.message = f"{note}: replaces CH{ch} in {run.merge_into}"
            except (OSError, ValueError, KeyError) as error:
                run.message = f"merge into {run.merge_into} failed: {error}"
        self._save()

    def _save(self):
        """Write the run file. A failed write is reported in the run message, the run goes on."""
        try:
            runfile.save(runfile.file(self.runs_dir, self.run.stamp), to_file(self.run))
        except OSError as error:
            self.run.message = f"run file not written: {error}"


def to_file(run: Run) -> dict:
    """Everything of a run for runs/<stamp>/run_<stamp>.json."""
    return {"format": runfile.FORMAT, "stamp": run.stamp, "state": run.state, "message": run.message,
            "settings": asdict(run.settings), **run.meta, "merge_into": run.merge_into,
            "steps": run.steps, "step": run.step, "channel_state": run.channel_state,
            "iload_offset": run.iload_offset, "t0": run.t0, "t_end": run.t_end, "plan_s": run.plan_s,
            "waited_s": round(run.waited_s, 1), "capture_s": [round(x, 2) for x in run.capture_s],
            "rows": run.rows, "table": table(run.rows, run.settings.phases),
            "line_reg": line_regulation(run.rows),
            "samples": list(run.samples), "waves": dict(run.waves)}


class Abort(Exception):
    pass


def plan_seconds(s: Settings, capture_s: float = SCOPE_CAPTURE_S) -> float:
    """Expected run time without the jumper waits: what the graphs use as their time axis.
    capture_s: time per scope snapshot, the guess at the start, the measured mean once known."""
    points = len(s.vin_points) if s.use_supply else 1
    per_phase = s.phase_s + (capture_s if s.scope_capture else 0)
    per_point = EN_SETTLE_S + len(s.phases) * per_phase + s.gap_s + (SUPPLY_STEP_S if s.use_supply else 0)
    return round(0.3 + ZERO_S + len(s.channels) * points * per_point, 1)


def mean(samples: list[dict]) -> dict:
    n = len(samples)
    if not n:
        return {"n": 0, "error": "no samples in this phase"}
    avg = {k: sum(s[k] for s in samples) / n for k in ("vin", "iin", "vload", "iload")}
    pin, pout = avg["vin"] * avg["iin"], avg["vload"] * avg["iload"]
    return {"n": n, **{k: round(v, 5) for k, v in avg.items()}, "pin": round(pin, 4), "pout": round(pout, 4),
            "eta": round(pout / pin, 4) if pin > 0.01 else None}


def table(rows: list[dict], phases: list[str]) -> list[dict]:
    """One line per channel and VIN point: VLOAD and eta per phase, load regulation."""
    out, index = [], {}
    for r in rows:
        key = (r["ch"], r["vin_set"])
        if key not in index:
            index[key] = {"ch": r["ch"], "vin_set": r["vin_set"], "vin": None, "phases": {}, "error": None}
            out.append(index[key])
        line = index[key]
        if r.get("error"):
            line["error"] = r["error"]
            continue
        line["vin"] = r["vin"]
        line["phases"][r["phase"]] = {"vload": r["vload"], "iload": r["iload"], "eta": r["eta"],
                                      "scope": r.get("scope")}
    for line in out:
        # load regulation: VLOAD from the lightest to the heaviest measured load, whichever phases those are
        p = list(line["phases"].values())
        light = min(p, key=lambda x: x["iload"]) if p else None
        heavy = max(p, key=lambda x: x["iload"]) if p else None
        if light and heavy and light is not heavy:
            line["load_reg_mv"] = round((heavy["vload"] - light["vload"]) * 1000, 1)
            line["load_reg_di"] = round(heavy["iload"] - light["iload"], 3)
    return out


def line_regulation(rows: list[dict]) -> dict:
    """Per channel and phase: VLOAD spread over the VIN points, in mV."""
    acc: dict = {}
    for r in rows:
        if r.get("error") or r.get("vin_set") is None:
            continue
        acc.setdefault(r["ch"], {}).setdefault(r["phase"], []).append(r["vload"])
    return {ch: {ph: round((max(v) - min(v)) * 1000, 1) for ph, v in d.items() if len(v) > 1}
            for ch, d in acc.items()}
