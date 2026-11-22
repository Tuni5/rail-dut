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
from fastapi.responses import FileResponse
from pydantic import BaseModel, Field
from serial.tools import list_ports

import sim
from bench import config
from bench.psu import PsuError
from pico import PicoError, PicoLink
from sequence import PHASE_NAMES, Sampler, Sequencer, Settings, line_regulation, table
from supply import Supply

ROOT = Path(__file__).resolve().parent.parent
UI_DIR = ROOT / "ui"
RUNS_DIR = ROOT / "runs"
BOARD = tomllib.loads((Path(__file__).resolve().parent / "board.toml").read_text())
CHANNELS = {int(k): v for k, v in BOARD["channels"].items()}

pico = PicoLink()
supply = Supply()
sampler = Sampler(pico, BOARD["limits"])
seq = Sequencer(pico, supply, sampler, RUNS_DIR, CHANNELS)


@asynccontextmanager
async def lifespan(app: FastAPI):
    sampler.start()
    yield
    seq.stop()
    pico.close()      # all off, LED off
    supply.close()    # output stays as it is


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
            "phases": PHASE_NAMES, "vin_range": [config.RAIL_MIN_VIN_V, config.RAIL_MAX_VIN_V],
            "defaults": BOARD["defaults"]}


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
                    "waited_s": round(run.waited_s, 1), "t_end": run.t_end}
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
    try:
        run = seq.start(Settings(**req.model_dump()))
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


@app.get("/api/run/csv")
def run_csv():
    if seq.run is None:
        fail(404, "no run yet")
    path = RUNS_DIR / f"{seq.run.stamp}_rail.csv"
    if not path.exists():
        fail(404, "the CSV is written when the run ends")
    return FileResponse(path, filename=path.name, media_type="text/csv")


if __name__ == "__main__":
    logging.basicConfig(level=logging.INFO, format="%(asctime)s %(levelname)-7s %(name)s: %(message)s",
                        datefmt="%H:%M:%S")
    logging.getLogger("pyvisa").setLevel(logging.WARNING)
    uvicorn.run(app, host="127.0.0.1", port=8001, log_level="warning")
