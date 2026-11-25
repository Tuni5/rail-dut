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

Per channel: GUI asks to move the Load Select jumper → per VIN point (supply) → EN on, output check
(VLOAD > 3 V) → load phases LOAD1, LOAD2, LOAD1+2 → EN off, gap. No output after EN (mostly Load Select
not moved): the run waits with "No output · CHx" and Retry / Skip channel; a skipped channel gets an error
row in the results. It never moves on to the next channel by itself.
Repeat a channel: open a finished run, pick the channel next to "Repeat channel". Only that channel runs
again, with the run's settings and board. When it completes, it replaces the old channel in that run
(rows, pictures, waveforms; in the graphs the new block sits where the old one was). The old data stays in
the run file under "replaced", its pictures in `replaced_<stamp>/`; the repeat is also saved as its own run.
Phase mean leaves out the first 0.4 s after each switch.
Stepping VIN down (24 → 8 V): the OWON cannot sink current, so the channel under test with LOAD1
discharges the input until VIN is within 0.3 V of the new setpoint (at most 20 s; also before switching
the output on, since after a run that ended at 24 V the input capacitors still hold ~20 V), then the supply
readout must reach it within 10 s.
LED: blue ready, cyan waiting (Load Select or no output), magenta LOAD1, orange LOAD2, white LOAD1+2, green done,
red only for faults (watchdog, current limit, run error). Results per channel and VIN:
VLOAD, Iload, η per phase, load regulation (lightest → heaviest load), line regulation (spread over VIN).
Without "use supply" one point is measured at whatever VIN is set by hand.

Boards: CH1..CH3 are the same on every board, CH4 is the DUT. `host/boards.json` lists the boards of
the campaign (number, CH4 part, default run name); the GUI's Sequence card needs one chosen before Start
(it names CH4 and the run) and adds new ones with "Add board".

One folder per run: `runs/<stamp>/` holds `run_<stamp>.json` and the scope pictures. The JSON has
settings, channel names, instruments, the phase rows (incl. scope analysis), every sample with its label
and the switch-node waveform of each snapshot. It is rewritten after every phase, so an aborted run is
kept up to its last phase. The GUI opens any saved run with the Run selector in the Live card; the CSV
button exports the shown run's phase rows. Runs from before 2026-11-25 (`<stamp>_rail.jsonl/.csv`,
`<stamp>_scope/`) are converted with `python host/convert_runs.py`; the old files stay.

## Scope (R&S RTM2034, LAN)

Header field: VISA resource, default `config.SCOPE_RESOURCE` (`TCPIP0::192.168.77.1::5025::SOCKET`), or `SIM`.
Instrument I/O is `host/bench/scope.py` from power-rail-bench, unchanged (verified there on this RTM2034).
With "scope snapshot per load phase" the run freezes the scope at the end of every load phase (load still on),
saves the screenshot as `runs/<stamp>/CH2_16V_LOAD1.png` and the channel 1 waveform into the run file, and runs it again.
The probe on scope CH1 moves by hand with Load Select; the prompt says so. The time base is fitted per capture:
with fewer than 5 or more than 20 pulses on screen it moves along 2/5/10/20/50/100 µs/div (aim ~8 pulses), the
scope runs again and the capture repeats (at most 3 times); the scope keeps the last time base. Needed for the
fakes, which run at 34–89 kHz with skipped pulses (5 µs/div showed 1–2 pulses).
f_sw and duty are computed from the waveform (hysteresis at 85 %/60 % between the diode drop and the on level:
VIN as the INA226 measured it, but not above the trace's own 98th percentile, since at 8 V in the switch drops
~1 V; pulses shorter than 0.5 µs ignored), not taken from the scope's FREQuency measurement: that one counts the ringing of the
fakes in discontinuous mode (bench: 270–282 kHz shown, ~45 kHz real).
Period spread: shortest and longest period on screen and (max − min) / mean. A genuine LM2596 runs on a fixed
oscillator (bench: 0.07–0.8 %); the M1 fake changes its period from pulse to pulse (11.5–19.1 µs, 49 %).
Above 5 % the GUI shows it in red. Runs saved before this are recomputed from their waveforms when opened.

Scope `SIM` follows the running sequence (channel, VIN, phase), so it also works next to a real Pico and
supply. Its switch-node shapes are taken from the bench photos of 2026-11-22: CH1 154.0 kHz, CH2 154.9 kHz,
CH3 162.1 kHz continuous; CH4 fake 1 (board 1) ~89 kHz, or fake 2 (board 2) ~41 kHz with `SIM_FAKE = 2` in
`host/sim.py`.
With a board chosen and a measured run of that board saved, scope `SIM` replays that run's real picture and
waveform for the same channel and load phase (nearest VIN). Every SIM picture is stamped SIMULATED DATA (needs
Pillow), the run is marked as SIM scope and the GUI shows a red SIMULATED DATA tag on its pictures.
Click a picture in the scope strip to open the gallery: ‹ › or ← →, film strip by channel and VIN.
`python host/reanalyse_runs.py <stamp>` recomputes f_sw and duty of a saved run from its stored waveforms
after a change to the analysis (done for 20261125-151740 after the 8 V fix).

## Safety

- Firmware watchdog: no command for 3 s → all EN and LOAD off, LED red. The host reads every 250 ms.
- Host limit: Iin or Iload above 1.6 A (`host/board.toml`) → everything off, run stops, red banner.
- Supply: OVP 24.5 V, OCP 2.0 A, CC 1.5 A before every run; VIN setpoints only 6..24 V (`host/bench/config.py`).
- "All off" in the header always works, also during a run.

## Firmware commands (`firmware/pico/main.py`)

`IDN` (firmware version, MCU, board, unique ID, clock, MicroPython version, INA226 IDs), `EN <1-4> <0|1>`, `LOAD <1|2> <0|1>` (pulses TRIG GP16 for the scope), `OFF`,
`READ` → `OK vin iin vload iload`, `STATE`, `LED <name>`.

## Board

Channel names, load values and limits: `host/board.toml`. Change CH4 there when the chip changes.
