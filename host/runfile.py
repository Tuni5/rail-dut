"""One run = one folder: runs/<stamp>/ with run_<stamp>.json and the scope pictures next to it.

The JSON holds everything the GUI shows and everything needed to recompute it:
settings, channel names, instruments, phase rows (incl. scope analysis), all INA226 samples,
and the switch-node waveform of every scope snapshot. The GUI opens a saved run from it.

The file is rewritten after every phase (written to a temporary file first, then swapped
in), so an aborted or crashed run is saved up to its last phase.
"""

import csv
import io
import json
import os
from pathlib import Path

FORMAT = "rail-dut run 1"
PHASE_NAMES = {"L1": "LOAD1", "L2": "LOAD2", "L12": "LOAD1+2"}


def folder(runs_dir: Path, stamp: str) -> Path:
    return runs_dir / stamp


def file(runs_dir: Path, stamp: str) -> Path:
    return folder(runs_dir, stamp) / f"run_{stamp}.json"


def save(path: Path, data: dict) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    tmp = path.with_suffix(".json.tmp")
    with open(tmp, "w", encoding="utf-8") as f:
        json.dump(data, f, separators=(",", ":"))
    os.replace(tmp, path)


def load(runs_dir: Path, stamp: str) -> dict:
    if not stamp or any(c in stamp for c in "/\\.:"):
        raise FileNotFoundError(stamp)
    with open(file(runs_dir, stamp), encoding="utf-8") as f:
        data = json.load(f)
    if data.get("format") != FORMAT:
        raise ValueError(f"{stamp}: unknown run format {data.get('format')!r}")
    data["settings"].setdefault("name", "")
    data["settings"].setdefault("board", None)
    # JSON keys are strings; the GUI and table() use channel numbers
    data["channel_state"] = {int(k): v for k, v in data.get("channel_state", {}).items()}
    data["channels"] = {int(k): v for k, v in data.get("channels", {}).items()}
    return data


def listing(runs_dir: Path, loader=None) -> list[dict]:
    """Saved runs, newest first, with what the run selector shows. loader(stamp): a caching load."""
    out = []
    if not runs_dir.exists():
        return out
    for d in sorted((p for p in runs_dir.iterdir() if p.is_dir()), reverse=True):
        try:
            data = loader(d.name) if loader else load(runs_dir, d.name)
        except (OSError, ValueError, json.JSONDecodeError):
            continue
        s = data["settings"]
        out.append({"stamp": data["stamp"], "name": s["name"], "board": s["board"], "state": data["state"],
                    "channels": s["channels"],
                    "vin_points": s["vin_points"] if s["use_supply"] else [],
                    "scope": s.get("scope_capture", False), "scope_sim": bool(data.get("scope_sim")),
                    "rows": len(data["rows"])})
    return out


def rename(runs_dir: Path, stamp: str, name: str) -> None:
    data = load(runs_dir, stamp)
    data["settings"]["name"] = name
    save(file(runs_dir, stamp), data)


def wave_entry(t, v) -> dict:
    """Waveform as start, step and values (the RTM2034 record is equally spaced)."""
    n = len(v)
    dt = float(t[-1] - t[0]) / (n - 1) if n > 1 else 0.0
    return {"t0": float(t[0]), "dt": dt, "v": [round(float(x), 4) for x in v]}


def csv_text(data: dict) -> str:
    """Result rows as CSV, same columns as before the one-file format."""
    keys = ["ch", "name", "vin_set", "phase", "n", "vin", "iin", "vload", "iload", "pin", "pout", "eta",
            "f_sw", "duty", "period_min_us", "period_max_us", "spread", "tdiv_us", "sw_vpp", "scope_png", "error"]
    buf = io.StringIO()
    w = csv.DictWriter(buf, fieldnames=keys, extrasaction="ignore", lineterminator="\n")
    w.writeheader()
    channels = data.get("channels", {})
    for r in data["rows"]:
        sc = r.get("scope") or {}
        w.writerow({**r, "name": (channels.get(r["ch"]) or channels.get(str(r["ch"])) or {}).get("name", ""),
                    "phase": PHASE_NAMES.get(r["phase"], r["phase"]),
                    "f_sw": sc.get("f_sw"), "duty": sc.get("duty"), "period_min_us": sc.get("period_min_us"),
                    "period_max_us": sc.get("period_max_us"), "spread": sc.get("spread"), "tdiv_us": sc.get("tdiv_us"),
                    "sw_vpp": sc.get("vpp"),
                    "scope_png": sc.get("png"), "error": r.get("error") or sc.get("error")})
    return buf.getvalue()


def merge_channel(runs_dir: Path, target: str, repeat: dict, ch: int, table_fn, line_reg_fn) -> str:
    """Put channel ch of a repeat run into the target run, in place of the old channel ch.

    Rows, scope pictures and waveforms of ch are replaced. In the samples the new ch block takes the
    old block's place on the time axis; everything after it moves by the difference in length.
    Nothing is lost: the old rows, samples and waveforms go into the run file under "replaced",
    the old pictures into the subfolder replaced_<repeat stamp>. Returns a one-line note."""
    import shutil
    data = load(runs_dir, target)
    folder_t, folder_r = folder(runs_dir, target), folder(runs_dir, repeat["stamp"])
    new_rows = [r for r in repeat["rows"] if r["ch"] == ch]
    old_rows = [r for r in data["rows"] if r["ch"] == ch]

    # rows: the new ones where the old ones were (channel order of the run)
    order = data["settings"]["channels"]
    keep = [r for r in data["rows"] if r["ch"] != ch]
    at = next((k for k, r in enumerate(keep) if order.index(r["ch"]) > order.index(ch)), len(keep))
    data["rows"] = keep[:at] + new_rows + keep[at:]

    # pictures and waveforms
    keep_dir = folder_t / f"replaced_{repeat['stamp']}"
    old_waves = {}
    for r in old_rows:
        png = (r.get("scope") or {}).get("png")
        if png:
            if (folder_t / png).exists():
                keep_dir.mkdir(exist_ok=True)
                shutil.move(str(folder_t / png), str(keep_dir / png))
            if png in data.get("waves", {}):
                old_waves[png] = data["waves"].pop(png)
    for r in new_rows:
        png = (r.get("scope") or {}).get("png")
        if png:
            shutil.copy2(folder_r / png, folder_t / png)
            if png in repeat.get("waves", {}):
                data.setdefault("waves", {})[png] = repeat["waves"][png]

    # samples: swap the ch block, shift what follows
    old = data["samples"]
    idx = [k for k, s in enumerate(old) if s.get("ch") == ch]
    new = [dict(s) for s in repeat["samples"] if s.get("ch") == ch]
    if idx:
        i0, i1 = idx[0], idx[-1] + 1
        t_start, old_len = old[i0]["t"], old[i1 - 1]["t"] - old[i0]["t"]
    else:                                    # channel had no samples (skipped): before the next channel
        i0 = i1 = next((k for k, s in enumerate(old) if s.get("ch") in order
                        and order.index(s["ch"]) > order.index(ch)), len(old))
        t_start, old_len = (old[i0]["t"] if i0 < len(old) else (old[-1]["t"] if old else 0.0)), 0.0
    new_len = (new[-1]["t"] - new[0]["t"]) if new else 0.0
    if new:
        t0 = new[0]["t"]
        for s in new:
            s["t"] = round(s["t"] - t0 + t_start, 3)
    delta = new_len - old_len
    tail = [dict(s, t=round(s["t"] + delta, 3)) for s in old[i1:]]
    replaced_samples = old[i0:i1]
    data["samples"] = old[:i0] + new + tail
    for k, s in enumerate(data["samples"]):  # sample ids unique within the run again
        s["i"] = k
    if data.get("t_end") is not None:
        data["t_end"] = round(data["t_end"] + delta, 3)

    # state
    data["channel_state"][ch] = repeat["channel_state"].get(ch) or repeat["channel_state"].get(str(ch))
    done = all(v == "done" for v in data["channel_state"].values())
    data["state"] = "done" if done else data["state"]
    data["step"] = len([r for r in data["rows"] if r["phase"] in data["settings"]["phases"]])
    data["table"] = table_fn(data["rows"], data["settings"]["phases"])
    data["line_reg"] = line_reg_fn(data["rows"])
    when = repeat["stamp"]
    note = f"CH{ch} repeated in run {when}"
    data.setdefault("repeats", []).append({"ch": ch, "by": when})
    data.setdefault("replaced", []).append({"ch": ch, "by": when, "rows": old_rows, "samples": replaced_samples,
                                            "waves": old_waves, "pictures": keep_dir.name if keep_dir.exists() else None})
    data["message"] = "; ".join(x for x in (data.get("message"), note) if x)
    save(file(runs_dir, target), data)
    return note
