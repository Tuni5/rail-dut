"""Recompute the scope analysis (f_sw, duty, levels) of saved runs from their stored waveforms,
with the current analyse(). For runs measured before a fix in analyse().

    python host/reanalyse_runs.py 20261125-151740 [more stamps]
"""

import sys
from pathlib import Path

import numpy as np

import runfile
from scope import analyse
from sequence import line_regulation, table

RUNS_DIR = Path(__file__).resolve().parent.parent / "runs"


def reanalyse(stamp: str, runs_dir: Path = RUNS_DIR) -> list[tuple]:
    data = runfile.load(runs_dir, stamp)
    changed = []
    for r in data["rows"]:
        sc = r.get("scope") or {}
        w = data.get("waves", {}).get(sc.get("png"))
        if not w:
            continue
        v = np.asarray(w["v"], dtype=float)
        t = w["t0"] + np.arange(len(v)) * w["dt"]
        new = {"png": sc["png"], **analyse(t, v, r.get("vin"))}
        if new.get("f_sw") != sc.get("f_sw"):
            changed.append((r["ch"], r["vin_set"], r["phase"], sc.get("f_sw"), new.get("f_sw")))
        r["scope"] = new
    data["table"] = table(data["rows"], data["settings"]["phases"])
    data["line_reg"] = line_regulation(data["rows"])
    runfile.save(runfile.file(runs_dir, stamp), data)
    return changed


if __name__ == "__main__":
    for stamp in sys.argv[1:]:
        for ch, vin, ph, old, new in reanalyse(stamp):
            print(f"{stamp} CH{ch} {vin} V {ph}: f_sw {old} -> {new}")
