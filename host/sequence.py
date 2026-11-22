"""Sampler and channel/load sequence. All numbers are computed here; the GUI only draws.

Sampler: reads both INA226 every LOG_S while the Pico is connected. That also
feeds the firmware watchdog. Each sample carries the label the sequence set.

Sequence: per channel (J2 jumper moved by hand), per VIN point, per load phase.
Phase mean = samples of that phase after SETTLE_S, so the label lag is gone.
"""

import csv
import json
import threading
import time
from collections import deque
from dataclasses import dataclass, field, asdict
from pathlib import Path

from pico import PicoError, PicoLink
from supply import Supply
from bench.psu import PsuError

LOG_S = 0.25
SETTLE_S = 0.4            # samples this soon after a phase change are not averaged
ZERO_S = 1.0              # Iload offset: averaged this long with EN and LOAD off
ZERO_MAX_A = 0.010        # an offset larger than this is a fault, not an offset
EN_SETTLE_S = 0.6         # after EN on, before the output check
MIN_VLOAD_V = 3.0         # below this after EN: no output on the load (J2? regulator?)

# what each phase switches and the LED color it shows
PHASES = {"L1": ((1,), "magenta"), "L2": ((2,), "orange"), "L12": ((1, 2), "red")}
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


class Sequencer:
    def __init__(self, pico: PicoLink, supply: Supply, sampler: Sampler, runs_dir: Path, channels: dict):
        self.pico, self.supply, self.sampler = pico, supply, sampler
        self.runs_dir = runs_dir
        self.channels = channels
        self.run: Run | None = None
        self._thread = None
        self._stop = threading.Event()
        self._go = threading.Event()
        self._skip = False
        self._raw = None

    @property
    def active(self) -> bool:
        return self.run is not None and self.run.state in ("running", "waiting")

    # --- control from the API -------------------------------------------------
    def start(self, settings: Settings) -> Run:
        if self.active:
            raise RuntimeError("a run is already active")
        if not self.pico.connected:
            raise RuntimeError("Pico is not connected")
        if settings.use_supply and not self.supply.connected:
            raise RuntimeError("supply is not connected (or untick 'Use supply')")
        stamp = time.strftime("%Y%m%d-%H%M%S")
        n_vin = len(settings.vin_points) if settings.use_supply else 1
        run = Run(settings=settings, stamp=stamp,
                  steps=len(settings.channels) * n_vin * len(settings.phases),
                  channel_state={ch: "waiting" for ch in settings.channels})
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
        self.runs_dir.mkdir(exist_ok=True)
        self._raw = open(self.runs_dir / f"{run.stamp}_rail.jsonl", "w")
        self.sampler.on_sample = None
        try:
            self.pico.off()
            run.current = {"phase": "zero"}
            run.iload_offset = self.sampler.zero()
            self._raw.write(json.dumps({"head": asdict(s), "channels": self.channels,
                                        "pico": self.pico.identity, "supply": self.supply.identity,
                                        "iload_offset_a": run.iload_offset}) + "\n")
            self.sampler.on_sample = lambda smp: self._raw.write(json.dumps(smp) + "\n")
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
            self._raw.close()
            self._write_csv()
            run.current = {}
            run.state, run.message = final

    def _channel(self, ch: int):
        run, s = self.run, self.run.settings
        run.channel_state[ch] = "running"
        if s.jumper_prompt:
            run.state, run.message = "waiting", f"Put the J2 jumper on CH{ch}, then continue"
            run.current = {"ch": ch}
            self.pico.led("cyan")
            self._go.clear()
            self._go.wait()
            run.state, run.message = "running", ""
            if self._stop.is_set():
                return
            if self._skip:
                run.channel_state[ch] = "skipped"
                run.step += (len(s.vin_points) if s.use_supply else 1) * len(s.phases)
                return
        points = s.vin_points if s.use_supply else [None]
        for vin_set in points:
            if self._stop.is_set():
                return
            if vin_set is not None:
                self.supply.set_voltage(vin_set)
                if not self.supply.is_on():
                    self.supply.output(True)
                else:
                    self.supply.wait_for(vin_set)
                self.supply.read_state()
            ok = self._point(ch, vin_set)
            if not ok:
                run.channel_state[ch] = "fail"
                run.step += len(s.phases) * (len(points) - points.index(vin_set))
                return
        run.channel_state[ch] = "done"

    def _point(self, ch: int, vin_set):
        run, s = self.run, self.run.settings
        label = {"ch": ch, "vin_set": vin_set}
        self.sampler.label = {**label, "phase": "EN"}
        run.current = {**label, "phase": "EN"}
        self.pico.en(ch, True)
        self._sleep(EN_SETTLE_S)
        vload = self.pico.read()[2]
        if vload < MIN_VLOAD_V:
            self.pico.en(ch, False)
            run.rows.append({**label, "phase": "EN", "error": f"no output on LOAD: VLOAD {vload:.3f} V "
                             f"(J2 on CH{ch}? regulator on?)"})
            return False
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
            self.pico.load(1, False)
            self.pico.load(2, False)
            run.rows.append({**label, "phase": phase, **mean(self.sampler.window(t0 + SETTLE_S, t1))})
            run.step += 1
        self.pico.en(ch, False)
        self.pico.led("blue")
        self.sampler.label = {**label, "phase": "gap"}
        run.current = {**label, "phase": "gap"}
        self._sleep(s.gap_s)
        return True

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

    def _write_csv(self):
        run = self.run
        path = self.runs_dir / f"{run.stamp}_rail.csv"
        keys = ["ch", "name", "vin_set", "phase", "n", "vin", "iin", "vload", "iload", "pin", "pout", "eta", "error"]
        with open(path, "w", newline="") as f:
            w = csv.DictWriter(f, fieldnames=keys, extrasaction="ignore")
            w.writeheader()
            for r in run.rows:
                w.writerow({**r, "name": self.channels.get(r["ch"], {}).get("name", ""),
                            "phase": PHASE_NAMES.get(r["phase"], r["phase"])})


class Abort(Exception):
    pass


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
        line["phases"][r["phase"]] = {"vload": r["vload"], "iload": r["iload"], "eta": r["eta"]}
    for line in out:
        p = line["phases"]
        light = p.get("L2") or p.get("L1")
        heavy = p.get("L12") or p.get("L1")
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
