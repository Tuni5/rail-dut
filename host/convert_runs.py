"""Convert runs from the old layout into one folder per run.

Old (until 2026-11-25):  runs/<stamp>_rail.jsonl   settings line + all samples
                         runs/<stamp>_rail.csv     phase rows
                         runs/<stamp>_scope/       <name>.png + <name>.csv (waveform)
New:                     runs/<stamp>/run_<stamp>.json + <name>.png

The scope analysis (f_sw, duty, levels) is recomputed from the stored waveform with the same
analyse() the run used, so the numbers are the same. The old files are left in place.

    python host/convert_runs.py                  all old runs that have no new folder yet
    python host/convert_runs.py 20261125-151740  just this one
"""

import csv
import json
import shutil
import sys
from pathlib import Path

import numpy as np

import runfile
from scope import analyse
from sequence import Settings, line_regulation, plan_seconds, table

RUNS_DIR = Path(__file__).resolve().parent.parent / "runs"
PHASE_KEYS = {v: k for k, v in runfile.PHASE_NAMES.items()}


def num(x, kind=float):
    return None if x in ("", None) else kind(float(x)) if kind is int else kind(x)


def convert(stamp: str, runs_dir: Path = RUNS_DIR) -> Path:
    raw = runs_dir / f"{stamp}_rail.jsonl"
    with open(raw, encoding="utf-8") as f:
        lines = [json.loads(x) for x in f if x.strip()]
    head, samples = lines[0], lines[1:]
    settings = Settings(**head["head"])
    channels = {int(k): v for k, v in head["channels"].items()}

    out_dir = runfile.folder(runs_dir, stamp)
    out_dir.mkdir(parents=True, exist_ok=True)
    rows, waves = [], {}
    old_csv = runs_dir / f"{stamp}_rail.csv"
    scope_dir = runs_dir / f"{stamp}_scope"
    if old_csv.exists():
        with open(old_csv, encoding="utf-8") as f:
            for r in csv.DictReader(f):
                row = {"ch": int(r["ch"]), "vin_set": num(r["vin_set"]),
                       "phase": PHASE_KEYS.get(r["phase"], r["phase"])}
                err = r.get("error") or None
                if r.get("n"):
                    row["n"] = num(r["n"], int)
                    for k in ("vin", "iin", "vload", "iload", "pin", "pout", "eta"):
                        row[k] = num(r[k])
                png = r.get("scope_png") or None
                if png and (scope_dir / png).exists():
                    shutil.copy2(scope_dir / png, out_dir / png)
                    wave = scope_dir / (Path(png).stem + ".csv")
                    if wave.exists():
                        tv = np.loadtxt(wave, delimiter=",", skiprows=1)
                        t, v = tv[:, 0], tv[:, 1]
                        waves[png] = runfile.wave_entry(t, v)
                        row["scope"] = {"png": png, **analyse(t, v, row.get("vin"))}
                    else:
                        row["scope"] = {"png": png, "error": "waveform missing in the old run"}
                    if err and err.startswith("capture failed"):
                        err = None
                elif err and err.startswith("capture failed"):
                    row["scope"], err = {"error": err}, None
                if err:
                    row["error"] = err
                rows.append(row)

    n_vin = len(settings.vin_points) if settings.use_supply else 1
    steps = len(settings.channels) * n_vin * len(settings.phases)
    measured = [r for r in rows if r["phase"] in settings.phases]
    channel_state = {}
    for ch in settings.channels:
        mine = [r for r in rows if r["ch"] == ch]
        channel_state[ch] = ("fail" if any(r.get("error") for r in mine)
                             else "done" if len([r for r in mine if r["phase"] in settings.phases]) == n_vin * len(settings.phases)
                             else "skipped" if not mine else "aborted")
    t0 = samples[0]["t"] if samples else 0.0
    data = {"format": runfile.FORMAT, "stamp": stamp,
            "state": "done" if len(measured) == steps else "aborted",
            "message": "converted from the old run layout",
            "settings": head["head"], "channels": channels,
            "pico": head.get("pico"), "pico_info": head.get("pico_info"), "supply": head.get("supply"),
            "scope": None, "steps": steps, "step": len(measured), "channel_state": channel_state,
            "iload_offset": head.get("iload_offset_a"), "t0": t0,
            "t_end": samples[-1]["t"] if samples else None, "plan_s": plan_seconds(settings),
            "waited_s": 0.0, "capture_s": [],
            "rows": rows, "table": table(rows, settings.phases), "line_reg": line_regulation(rows),
            "samples": samples, "waves": waves}
    path = runfile.file(runs_dir, stamp)
    runfile.save(path, data)
    return path


def old_stamps(runs_dir: Path) -> list[str]:
    return sorted(p.name[:-len("_rail.jsonl")] for p in runs_dir.glob("*_rail.jsonl"))


if __name__ == "__main__":
    wanted = sys.argv[1:] or [s for s in old_stamps(RUNS_DIR) if not runfile.file(RUNS_DIR, s).exists()]
    for stamp in wanted:
        try:
            print(f"{stamp} -> {convert(stamp)}")
        except Exception as error:
            print(f"{stamp}: not converted: {type(error).__name__}: {error}")
