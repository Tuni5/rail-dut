"""HTTP API between the browser GUI and the rail-dut bench.

All computation happens here; the GUI only draws what it receives.

    python host/server.py   ->   http://localhost:8001
"""

import json
import logging
import time
import tomllib
from contextlib import asynccontextmanager
from pathlib import Path

import uvicorn
from fastapi import FastAPI, HTTPException
from fastapi.responses import FileResponse, Response
from pydantic import BaseModel, Field
from serial.tools import list_ports

import runfile
import sim
from bench import config
from bench import config as bench_config
from bench.psu import PsuError
from pico import PicoError, PicoLink
from sequence import PHASE_NAMES, Sampler, Sequencer, Settings, line_regulation, table, to_file
from scope import Scope, analyse
from bench.scope import ScopeError
from supply import Supply

ROOT = Path(__file__).resolve().parent.parent
UI_DIR = ROOT / "ui"
RUNS_DIR = ROOT / "runs"
BOARD = tomllib.loads((Path(__file__).resolve().parent / "board.toml").read_text())
CHANNELS = {int(k): v for k, v in BOARD["channels"].items()}
BOARDS_FILE = Path(__file__).resolve().parent / "boards.json"


def load_boards() -> dict:
    """Boards of the measurement campaign: number -> {ch4: DUT on CH4, name: default run name}."""
    if not BOARDS_FILE.exists():
        return {}
    return {int(k): v for k, v in json.loads(BOARDS_FILE.read_text(encoding="utf-8"))["boards"].items()}


def save_boards(boards: dict) -> None:
    data = json.loads(BOARDS_FILE.read_text(encoding="utf-8")) if BOARDS_FILE.exists() else {}
    data["boards"] = {str(k): v for k, v in sorted(boards.items())}
    tmp = BOARDS_FILE.with_suffix(".json.tmp")
    tmp.write_text(json.dumps(data, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
    tmp.replace(BOARDS_FILE)


BOARDS = load_boards()


def channels_for(board: int | None) -> dict:
    """Channel names of a run: CH4 is the DUT of the chosen board."""
    ch = {k: dict(v) for k, v in CHANNELS.items()}
    if board in BOARDS:
        ch[4]["name"] = BOARDS[board]["ch4"]
    return ch

pico = PicoLink()
supply = Supply()
sampler = Sampler(pico, BOARD["limits"])
scope = Scope()
seq = Sequencer(pico, supply, sampler, RUNS_DIR, CHANNELS, scope)


def sim_scope_state():
    """What a SIM scope shows: the channel, VIN and phase of the running sequence, so it also works
    next to a real Pico and supply. Without a run it follows the simulated board (Pico SIM)."""
    run = seq.run
    if run is not None and run.state in ("running", "waiting") and run.current.get("ch"):
        last = sampler.samples[-1] if sampler.samples else None
        switching = run.current.get("phase") in ("EN", "L1", "L2", "L12", "discharge")
        return (run.current["ch"], (last["vin"] if last else None), switching,
                (last["iload"] if last else None), (last["vload"] if last else None))
    if pico.port_name == sim.PORT:
        ch = sim.board.jumper
        _, _, vload, iload = sim.board.read()
        return ch, sim.board.vin(), bool(sim.board.en[ch - 1]), iload, vload
    return None, None, False, None, None


sim.scope_state = sim_scope_state

_saved_cache: dict = {}


def saved_run(stamp: str) -> dict:
    """A saved run file, parsed once per change (the files are a few MB with all waveforms)."""
    mtime = runfile.file(RUNS_DIR, stamp).stat().st_mtime
    hit = _saved_cache.get(stamp)
    if hit is None or hit[0] != mtime:
        hit = (mtime, upgrade_scope(runfile.load(RUNS_DIR, stamp)))
        _saved_cache[stamp] = hit
    return hit[1]


def upgrade_scope(data: dict) -> dict:
    """Runs saved before the period spread (and the 8 V threshold fix): their scope analysis is
    recomputed from the stored waveforms for the GUI. The file itself is not changed; to write it,
    python host/reanalyse_runs.py <stamp>."""
    import numpy as np
    waves = data.get("waves") or {}
    for r in data["rows"]:
        sc = r.get("scope") or {}
        w = waves.get(sc.get("png"))
        if w and "tdiv_us" not in sc and "spread" not in sc:
            v = np.asarray(w["v"], dtype=float)
            r["scope"] = {"png": sc["png"], **analyse(w["t0"] + np.arange(len(v)) * w["dt"], v, r.get("vin"))}
    return data


def sim_scope_recording():
    """For a SIM scope: the real snapshot of the same board, channel and load phase from the newest
    measured run of that board (nearest VIN if that point is missing). None: use the model."""
    run = seq.run
    if run is None or run.state not in ("running", "waiting") or not run.settings.board:
        return None
    cur = run.current
    if not cur.get("ch") or cur.get("phase") not in PHASE_NAMES:
        return None
    for d in sorted((p for p in RUNS_DIR.iterdir() if p.is_dir()), reverse=True) if RUNS_DIR.exists() else []:
        if d.name == run.stamp:
            continue
        try:
            data = saved_run(d.name)
        except (OSError, ValueError):
            continue
        if data["settings"].get("board") != run.settings.board or data.get("scope_sim"):
            continue
        waves = data.get("waves") or {}
        rows = [r for r in data["rows"] if r["ch"] == cur["ch"] and r["phase"] == cur["phase"]
                and (r.get("scope") or {}).get("png") in waves]
        if not rows:
            continue
        r = min(rows, key=lambda x: abs((x["vin_set"] or 0) - (cur.get("vin_set") or 0)))
        png = r["scope"]["png"]
        w = waves[png]
        return {"png": (runfile.folder(RUNS_DIR, d.name) / png).read_bytes(),
                "t": [w["t0"] + k * w["dt"] for k in range(len(w["v"]))], "v": w["v"],
                "source": f"{data['settings'].get('name') or d.name}, {png[:-4]}"}
    return None


sim.scope_recording = sim_scope_recording


@asynccontextmanager
async def lifespan(app: FastAPI):
    sampler.start()
    yield
    seq.stop()
    pico.close()      # all off, LED off
    supply.close()    # output stays as it is
    scope.close()     # left running


app = FastAPI(title="rail-dut", lifespan=lifespan)


def fail(status: int, error):
    raise HTTPException(status, str(error))


def idle():
    if seq.active:
        fail(409, "a run is active")


class ConnectRequest(BaseModel):
    port: str | None = None


class Switch(BaseModel):
    n: int
    on: bool


class SupplySet(BaseModel):
    volts: float | None = None
    amps: float | None = None


class OutputRequest(BaseModel):
    on: bool


class RunRequest(BaseModel):
    channels: list[int] = Field(min_length=1)
    use_supply: bool = True
    vin_points: list[float] = [8.0, 16.0, 24.0]
    phases: list[str] = ["L1", "L2", "L12"]
    phase_s: float = Field(2.5, ge=1.0, le=60)
    gap_s: float = Field(1.0, ge=0, le=30)
    jumper_prompt: bool = True
    supply_off_at_end: bool = True
    scope_capture: bool = False
    name: str = Field("", max_length=80)
    board: int | None = None


class BoardRequest(BaseModel):
    ch4: str = Field(min_length=1, max_length=60)
    name: str = Field("", max_length=80)


class RepeatRequest(BaseModel):
    ch: int


class NameRequest(BaseModel):
    name: str = Field("", max_length=80)


class VoltCheck(BaseModel):
    which: str = Field(pattern="^(vin|vload)$")
    dmm: float = Field(gt=0)


class Proceed(BaseModel):
    skip: bool = False


@app.get("/")
def index():
    # no caching: after an update a normal reload must get the new page
    return FileResponse(UI_DIR / "index.html", headers={"Cache-Control": "no-store"})


@app.get("/api/ports")
def ports():
    # real ports first, so the list never opens on the simulator; SIM stays available at the end
    return [p.device for p in sorted(list_ports.comports(), key=lambda p: p.device)] + [sim.PORT]


@app.get("/api/board")
def board():
    return {"channels": CHANNELS, "loads": BOARD["loads"], "limits": BOARD["limits"],
            "phases": PHASE_NAMES, "boards": BOARDS, "vin_range": [config.RAIL_MIN_VIN_V, config.RAIL_MAX_VIN_V],
            "defaults": BOARD["defaults"]}


@app.post("/api/boards")
def boards_add(req: BoardRequest):
    """New board of the campaign: next free number, CH4 DUT, default run name."""
    ch4 = req.ch4.strip()
    if not ch4:
        fail(400, "CH4 DUT is empty")
    n = max(BOARDS, default=0) + 1
    BOARDS[n] = {"ch4": ch4, "name": req.name.strip() or f"Board_{n}-{'_'.join(ch4.split())}"}
    try:
        save_boards(BOARDS)
    except OSError as error:
        del BOARDS[n]
        fail(500, f"boards.json not written: {error}")
    return {"board": n, **BOARDS[n]}


@app.get("/api/status")
def status(since: int = 0):
    run = seq.run
    run_view = None
    if run is not None:
        run_view = {"stamp": run.stamp, "state": run.state, "message": run.message, "step": run.step,
                    "steps": run.steps, "current": run.current, "channel_state": run.channel_state,
                    "settings": run.settings.__dict__, "rows": run.rows,
                    "table": table(run.rows, run.settings.phases), "line_reg": line_regulation(run.rows),
                    "iload_offset": run.iload_offset, "t0": run.t0, "plan_s": run.plan_s,
                    "waited_s": round(run.waited_s, 1), "t_end": run.t_end, "saved": False,
                    "channels": run.meta.get("channels") or CHANNELS,
                    "scope_sim": bool(run.meta.get("scope_sim")), "merge_into": run.merge_into}
    pico_state = None
    if pico.connected and not seq.active:
        try:
            pico_state = pico.state()
        except PicoError:
            pass
    return {
        "pico": {"connected": pico.connected, "port": pico.port_name, "identity": pico.identity, "info": pico.info,
                 "error": sampler.error, "state": pico_state},
        "supply": {"connected": supply.connected, "port": supply.port_name, "identity": supply.identity,
                   "last": supply.last},
        "scope": {"connected": scope.connected, "resource": scope.resource, "identity": scope.identity,
                  "default": bench_config.SCOPE_RESOURCE},
        "trip": sampler.trip,
        "iload_offset": sampler.iload_offset,
        "checks": checks[-6:],
        "run": run_view,
        "samples": sampler.since(since),
        "next": sampler.count,
    }


@app.post("/api/pico/connect")
def pico_connect(req: ConnectRequest):
    if not req.port:
        fail(400, "choose a port")
    try:
        ident = pico.open(req.port)
        pico.led("blue")
        return {"identity": ident}
    except PicoError as error:
        fail(502, error)


@app.post("/api/pico/disconnect")
def pico_disconnect():
    seq.stop()
    pico.close()
    return {}


@app.post("/api/pico/en")
def pico_en(req: Switch):
    idle()
    try:
        pico.en(req.n, req.on)
    except PicoError as error:
        fail(502, error)
    return {}


@app.post("/api/pico/load")
def pico_load(req: Switch):
    idle()
    try:
        pico.load(req.n, req.on)
    except PicoError as error:
        fail(502, error)
    return {}


@app.post("/api/pico/off")
def pico_off():
    """Always allowed: stops a run as well."""
    seq.stop()
    sampler.trip = None
    try:
        pico.off()
        pico.led("blue")
    except PicoError as error:
        fail(502, error)
    return {}


@app.post("/api/pico/zero")
def pico_zero():
    """Iload offset, with everything switched off first."""
    idle()
    try:
        pico.off()
        return {"iload_offset": sampler.zero()}
    except PicoError as error:
        fail(502, error)


checks: list[dict] = []
CHECK_OK_PCT = 0.3     # UT61E 0.1 % + 2 digits plus INA226 0.1 %, with margin


@app.post("/api/check/voltage")
def check_voltage(req: VoltCheck):
    """Compare the board reading (mean of the last second) with a multimeter value. Nothing is corrected."""
    try:
        board_v = sampler.mean_last(1.0)[req.which]
    except PicoError as error:
        fail(502, error)
    if board_v < 0.5:
        fail(409, f"board reads {board_v:.3f} V on {req.which.upper()}: supply on? "
                  + ("channel enabled and Load Select set?" if req.which == "vload" else ""))
    dev = (board_v - req.dmm) / req.dmm * 100
    entry = {"time": time.strftime("%Y-%m-%d %H:%M:%S"), "which": req.which, "board": round(board_v, 4),
             "dmm": req.dmm, "dev_pct": round(dev, 3), "ok": abs(dev) <= CHECK_OK_PCT}
    checks.append(entry)
    RUNS_DIR.mkdir(exist_ok=True)
    with open(RUNS_DIR / "voltage_checks.jsonl", "a") as f:
        f.write(json.dumps(entry) + "\n")
    return entry


@app.post("/api/supply/connect")
def supply_connect(req: ConnectRequest):
    try:
        return {"identity": supply.open(req.port or None)}
    except PsuError as error:
        fail(502, error)


@app.post("/api/supply/disconnect")
def supply_disconnect():
    idle()
    supply.close()
    return {}


@app.get("/api/supply/read")
def supply_read():
    try:
        return supply.read_state()
    except PsuError as error:
        fail(502, error)


@app.post("/api/supply/set")
def supply_set(req: SupplySet):
    idle()
    try:
        if req.amps is not None:
            supply.set_current(req.amps)
        if req.volts is not None:
            supply.set_voltage(req.volts)
        return supply.read_state()
    except PsuError as error:
        fail(400, error)


@app.post("/api/supply/output")
def supply_output(req: OutputRequest):
    if req.on:
        idle()
    try:
        if req.on:
            supply.prepare()
        supply.output(req.on)
        return supply.read_state()
    except PsuError as error:
        fail(502, error)


@app.post("/api/scope/connect")
def scope_connect(req: ConnectRequest):
    try:
        return {"identity": scope.open(req.port)}
    except (ScopeError, OSError) as error:
        fail(502, error)


@app.post("/api/scope/disconnect")
def scope_disconnect():
    idle()
    scope.close()
    return {}


# --- saved runs: runs/<stamp>/run_<stamp>.json plus the scope pictures -----------------------
def run_data(stamp: str) -> dict:
    """The run as in its file; the live run from memory, so it is complete up to the last sample."""
    if seq.run is not None and seq.run.stamp == stamp:
        data = to_file(seq.run)
        data["channel_state"] = dict(seq.run.channel_state)
        return data
    try:
        return saved_run(stamp)
    except (OSError, ValueError) as error:
        fail(404, f"no saved run {stamp}: {error}")


@app.get("/api/runs")
def runs_list():
    return runfile.listing(RUNS_DIR, saved_run)


@app.get("/api/runs/{stamp}")
def runs_open(stamp: str):
    """A saved run in the shape of /api/status's run, plus its samples, for the GUI to show."""
    d = run_data(stamp)
    s = d["settings"]
    view = {"stamp": d["stamp"], "state": d["state"], "message": d["message"], "step": d["step"],
            "steps": d["steps"], "current": {}, "channel_state": d["channel_state"], "settings": s,
            "rows": d["rows"], "table": table(d["rows"], s["phases"]), "line_reg": line_regulation(d["rows"]),
            "iload_offset": d["iload_offset"], "t0": d["t0"], "plan_s": d["plan_s"], "waited_s": d["waited_s"],
            "t_end": d["t_end"], "saved": True, "channels": d.get("channels") or CHANNELS,
            "scope_sim": bool(d.get("scope_sim")), "repeats": d.get("repeats", []),
            "instruments": {"pico": d.get("pico"), "supply": d.get("supply"), "scope": d.get("scope")}}
    return {"run": view, "samples": d["samples"]}


@app.post("/api/runs/{stamp}/name")
def runs_rename(stamp: str, req: NameRequest):
    name = req.name.strip()
    if seq.run is not None and seq.run.stamp == stamp:
        seq.rename(name)
    else:
        try:
            runfile.rename(RUNS_DIR, stamp, name)
        except (OSError, ValueError) as error:
            fail(404, f"no saved run {stamp}: {error}")
    return {"name": name}


@app.post("/api/runs/{stamp}/repeat")
def runs_repeat(stamp: str, req: RepeatRequest):
    """Measure one channel of a finished run again, with the run's own settings and board. When the
    repeat completes, its channel replaces the old one in that run (see runfile.merge_channel)."""
    if seq.active:
        fail(409, "a run is active")
    d = run_data(stamp)
    if d["state"] in ("running", "waiting"):
        fail(409, "that run is still active")
    s = dict(d["settings"])
    if req.ch not in s["channels"]:
        fail(400, f"CH{req.ch} is not part of run {stamp}")
    s["channels"] = [req.ch]
    s["name"] = f"{s.get('name') or stamp} · CH{req.ch} repeat"
    known = Settings.__dataclass_fields__
    settings = Settings(**{k: v for k, v in s.items() if k in known})
    try:
        run = seq.start(settings, d.get("channels") and {int(k): v for k, v in d["channels"].items()}
                        or channels_for(settings.board), merge_into=stamp)
    except RuntimeError as error:
        fail(409, error)
    return {"stamp": run.stamp}


@app.get("/api/runs/{stamp}/csv")
def runs_csv(stamp: str):
    return Response(runfile.csv_text(run_data(stamp)), media_type="text/csv",
                    headers={"Content-Disposition": f'attachment; filename="rail_{stamp}.csv"'})


@app.get("/api/runs/{stamp}/file/{name}")
def runs_picture(stamp: str, name: str):
    """Scope picture of a run (live or saved), by the name stored in its phase row."""
    bad = lambda x: not x or "/" in x or "\\" in x or ".." in x
    if bad(stamp) or bad(name) or not name.endswith(".png"):
        fail(404, "no such picture")
    path = runfile.folder(RUNS_DIR, stamp) / name
    if not path.exists():
        fail(404, "no such picture")
    return FileResponse(path, media_type="image/png")


@app.post("/api/run/start")
def run_start(req: RunRequest):
    bad = [p for p in req.phases if p not in PHASE_NAMES]
    if bad:
        fail(400, f"unknown phase {bad}")
    if any(c not in CHANNELS for c in req.channels):
        fail(400, "channels are 1..4")
    if req.use_supply:
        if not req.vin_points:
            fail(400, "no VIN points")
        lo, hi = config.RAIL_MIN_VIN_V, config.RAIL_MAX_VIN_V
        out = [v for v in req.vin_points if not lo <= v <= hi]
        if out:
            fail(400, f"VIN {out} outside {lo}..{hi} V")
    req.name = req.name.strip()
    if BOARDS and req.board not in BOARDS:
        fail(400, "choose the board")
    try:
        run = seq.start(Settings(**req.model_dump()), channels_for(req.board))
        return {"stamp": run.stamp}
    except RuntimeError as error:
        fail(409, error)


@app.post("/api/run/continue")
def run_continue(req: Proceed):
    seq.proceed(req.skip)
    return {}


@app.post("/api/run/stop")
def run_stop():
    seq.stop()
    return {}


if __name__ == "__main__":
    logging.basicConfig(level=logging.INFO, format="%(asctime)s %(levelname)-7s %(name)s: %(message)s",
                        datefmt="%H:%M:%S")
    logging.getLogger("pyvisa").setLevel(logging.WARNING)
    uvicorn.run(app, host="127.0.0.1", port=8001, log_level="warning")
