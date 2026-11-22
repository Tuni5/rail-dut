# rail-dut software

Bench GUI for rail-dut rev A: four LM2596 channels, two switched loads, two INA226.
The Pico switches and measures, the host runs the sequence and computes, the browser shows it.
Same structure as shmoo-dut; the OWON driver (`host/bench/psu.py`, `common.py`) is taken over unchanged.

## Start

```powershell
py -3.12 -m venv .venv
.venv\Scripts\Activate.ps1
pip install -r requirements.txt

mpremote connect COM9 cp firmware/pico/main.py : + reset
python host/server.py                          # then open http://localhost:8001
```

Port 8001, so it can run next to shmoo-dut (8000). Port `SIM` (Pico and supply)
runs everything against a simulated board; CH4 is modelled as a bad part. For the GUI, not for numbers.

## Sequence

Per channel: GUI asks to move the J2 jumper → per VIN point (supply) → EN on, output check
(VLOAD > 3 V, otherwise the channel is marked fail) → load phases LOAD1, LOAD2, LOAD1+2 → EN off, gap.
Phase mean leaves out the first 0.4 s after each switch. Results per channel and VIN:
VLOAD, Iload, η per phase, load regulation (LOAD2 → LOAD1+2), line regulation (spread over VIN).
Without "use supply" one point is measured at whatever VIN is set by hand.

Files per run in `runs/`: `<stamp>_rail.jsonl` (every sample with its label) and `<stamp>_rail.csv` (phase means).

## Safety

- Firmware watchdog: no command for 3 s → all EN and LOAD off, LED red. The host reads every 250 ms.
- Host limit: Iin or Iload above 1.6 A (`host/board.toml`) → everything off, run stops, red banner.
- Supply: OVP 24.5 V, OCP 2.0 A, CC 1.5 A before every run; VIN setpoints only 6..24 V (`host/bench/config.py`).
- "All off" in the header always works, also during a run.

## Firmware commands (`firmware/pico/main.py`)

`IDN`, `EN <1-4> <0|1>`, `LOAD <1|2> <0|1>` (pulses TRIG GP16 for the scope), `OFF`,
`READ` → `OK vin iin vload iload`, `STATE`, `LED <name>`.

## Board

Channel names, load values and limits: `host/board.toml`. Change CH4 there when the chip changes.
