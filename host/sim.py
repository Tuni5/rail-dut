"""Simulated board and supply, port name "SIM". For the GUI, not for numbers.

The board model: four bucks with made-up output resistance and line
regulation, CH4 worse (the suspect chip). The Load Select jumper follows the
channel that was enabled last, as if you always moved it correctly.
"""

import random
import threading
import time

import pyvisa
from pyvisa import constants

PORT = "SIM"

R_LOAD = {1: 10.1, 2: 5.04}     # as fitted on the board (host/board.toml)
# per channel: VOUT at no load and 12 V in, output resistance, line regulation V/V, efficiency
CH = {1: (4.860, 0.085, 0.0010, 0.80),
      2: (4.845, 0.065, 0.0015, 0.80),
      3: (4.850, 0.090, 0.0020, 0.81),
      4: (5.020, 0.250, 0.0080, 0.68)}


class Board:
    def __init__(self):
        self.lock = threading.Lock()
        self.en = [0, 0, 0, 0]
        self.load = [0, 0]
        self.jumper = 1
        self.jumper_forgotten = set()   # channels whose Load Select move is "forgotten" (to try retry/skip)
        self.supply = None            # SimOwon when the simulated supply is connected
        self.fixed_vin = 12.0         # otherwise a bench supply set by hand

    def vin(self) -> float:
        if self.supply is not None:
            return self.supply.vout()
        return self.fixed_vin

    def read(self):
        with self.lock:
            vin = self.vin()
            ch = self.jumper
            vload = iload = 0.0
            pout = 0.0
            if self.en[ch - 1] and vin > 6.0:
                v0, r_out, line, _ = CH[ch]
                g = sum(1 / R_LOAD[n + 1] for n in range(2) if self.load[n])
                v_nl = v0 + line * (vin - 12.0)
                vload = v_nl / (1 + r_out * g)
                iload = vload * g
                pout = vload * iload
            iin = 0.001 + 0.004 * sum(self.en)
            for i, on in enumerate(self.en):
                if on and i + 1 == ch and vin > 0:
                    iin += pout / (CH[ch][3] * vin)
            n = random.gauss
            # 0.6 mA Iload offset, so the zero function has something to find
            return (vin + n(0, 0.002), iin + n(0, 0.0005), vload + n(0, 0.001) if vload else 0.0,
                    iload + 0.0006 + n(0, 0.0002))


board = Board()


class SimPicoPort:
    """Stands in for the serial port: same lines in, same lines out."""

    def __init__(self):
        self._out = []

    def write(self, data: bytes):
        line = data.decode().strip()
        p = line.split()
        cmd = p[0].upper() if p else ""
        b = board
        if cmd == "IDN":
            r = ("OK rail-dut,revA,0.1,mcu=SIM,board=simulated,uid=0000000000000000,f_sys=125000000,"
                 "mpy=sim,ina_in=5449/2260,ina_load=5449/2260")
        elif cmd == "EN":
            ch, on = int(p[1]), int(p[2])
            b.en[ch - 1] = on
            if on and ch not in b.jumper_forgotten:
                b.jumper = ch
            r = "OK"
        elif cmd == "LOAD":
            b.load[int(p[1]) - 1] = int(p[2])
            r = "OK"
        elif cmd == "OFF":
            b.en, b.load = [0, 0, 0, 0], [0, 0]
            r = "OK"
        elif cmd == "READ":
            r = "OK %.4f %.5f %.4f %.5f" % b.read()
        elif cmd == "STATE":
            r = "OK en=%s load=%s wdt=0" % ("".join(map(str, b.en)), "".join(map(str, b.load)))
        elif cmd == "LED":
            r = "OK"
        else:
            r = "ERR CMD " + cmd
        self._out.append((r + "\n").encode())

    def readline(self) -> bytes:
        return self._out.pop(0) if self._out else b""

    def reset_input_buffer(self):
        self._out.clear()

    def close(self):
        b = board
        b.en, b.load = [0, 0, 0, 0], [0, 0]


class SimOwon:
    """Stands in for the pyvisa instrument psu.py talks to."""

    def __init__(self):
        self.timeout = 3000
        self._pending = []
        self._v, self._ovp = 12.0, 24.5
        self._i, self._ocp = 1.5, 2.0
        self._on = False
        self._v_out, self._t = 0.0, time.time()
        board.supply = self

    def vout(self) -> float:
        """Rises at once; falls only as fast as the load discharges the capacitors."""
        now = time.time()
        dt, self._t = now - self._t, now
        target = self._v if self._on else 0.0
        if target >= self._v_out:
            self._v_out = target
        else:
            loaded = any(board.en) and any(board.load)
            self._v_out = max(target, self._v_out - (12.0 if loaded else 0.3) * dt)
        return self._v_out

    def write(self, cmd: str) -> None:
        name, _, arg = cmd.partition(" ")
        name = name.upper()
        if name == "MEASURE:CURRENT?":
            amps = board.read()[1] if self._on else 0.0
        answers = {
            "*IDN?": "OWON,SPE3102,SIMULATED,FV:sim",
            "VOLTAGE?": f"{self._v:.3f}", "CURRENT?": f"{self._i:.3f}",
            "VOLTAGE:LIMIT?": f"{self._ovp:.3f}", "CURRENT:LIMIT?": f"{self._ocp:.3f}",
            "OUTPUT?": "ON" if self._on else "OFF",
            "MEASURE:VOLTAGE?": f"{self.vout() + (random.gauss(0, 0.003) if self._on else 0):.3f}",
            "MEASURE:CURRENT?": f"{amps if name == 'MEASURE:CURRENT?' else 0:.3f}",
        }
        if name in answers:
            self._pending.append(answers[name])
            return
        if name == "VOLTAGE":
            self._v = float(arg)
        elif name == "CURRENT":
            self._i = float(arg)
        elif name == "VOLTAGE:LIMIT":
            self._ovp = float(arg)
        elif name == "CURRENT:LIMIT":
            self._ocp = float(arg)
        elif name == "OUTPUT":
            self._on = arg.upper() == "ON"
        else:
            self._pending.append("ERR")
        if self._on and self._v > self._ovp:
            self._on = False

    def read(self) -> str:
        if not self._pending:
            raise pyvisa.VisaIOError(constants.StatusCode.error_timeout)
        return self._pending.pop(0)

    def clear(self) -> None:
        self._pending.clear()

    def close(self) -> None:
        board.supply = None


# --- RTM2034 stand-in ---------------------------------------------------------
# Answers exactly the commands bench/scope.py sends. The waveform is the switch
# node of the channel the Load Select jumper is on: CH1..3 continuous at 150 kHz,
# CH4 (the suspect) discontinuous at 45 kHz with ringing after each pulse.

import math
import struct
import zlib


# Switch node per channel. Frequencies and the fake pulse shapes are read off the bench photos of
# 2026-11-22 (RTM2034, VIN 13 V, probe 10:1):
#   CH1 TI 154.0 kHz, CH2 UMW 154.9 kHz, CH3 TECH PUBLIC 162.1 kHz: continuous, clean square
#   fake 1 (board 1, CH4): ~89 kHz, ~3 us high at 13 V, diode phase, ringing around VOUT
#   fake 2 (board 2, CH4): ~41 kHz, ~6 us high at 13 V, same pattern, longer ringing
# Everything else follows from the regulator holding VOUT: the inductor carries no DC voltage, so
# the switch node averages VOUT at every VIN and load (volt-second balance):
#   continuous:     D = (VOUT + Vd) / (VIN + Vd)
#   discontinuous:  t_on * (VIN - VOUT) = t_diode * (VOUT + Vd), then ringing around VOUT
# The fake's on time at another VIN or load scales like a buck in discontinuous mode,
# t_on ~ sqrt(Iout / (VIN * (VIN - VOUT))), from the photo point (13 V, load assumed ~0.5 A: the
# photos do not say which load was on). If t_on + t_diode no longer fits, it runs continuous.
SW_MODEL = {
    1: {"f": 154.0e3},
    2: {"f": 154.9e3},
    3: {"f": 162.1e3},
    4: {"f": 89e3, "t_on_ref": 3.0e-6, "ring_hz": 2.5e6, "ring_tau": 6.0e-6},   # fake 1
}
FAKE_2 = {"f": 41e3, "t_on_ref": 6.0e-6, "ring_hz": 2.0e6, "ring_tau": 12.0e-6}
# Ringing: the photos show it barely decayed when the next pulse comes, with peaks reaching most of
# the way to VIN (first swing VOUT + Vd above VOUT). ring_tau is set long accordingly.
SIM_FAKE = 1                     # 2: put fake 2 (board 2) on CH4 instead
VIN_REF, I_REF = 13.0, 0.5       # operating point of the fake photos (load assumed)
V_DIODE = 1.6                    # the node sits at -V_DIODE while the catch diode conducts (photos)
VOUT = 5.0

# Where the simulated scope looks. None: the simulated board (Pico SIM). The server sets a function
# returning (channel, vin, switching, iload, vout) from the running sequence, so a SIM scope also works
# next to a real Pico and supply.
scope_state = None
# Recorded data instead of the model: the server sets a function returning
# {"png": bytes, "t": [...], "v": [...], "source": str} from a real run of the same board, channel,
# VIN and load phase, or None (then the model above is used). Every simulated picture is stamped.
scope_recording = None


def _sw_wave(n=5000, span=100e-6):
    if scope_state is not None:
        ch, vin, on, iload, vout = scope_state()
    else:
        ch, vin = board.jumper, board.vin()
        _, _, vout, iload = board.read()
        on = bool(board.en[ch - 1])
    vout = vout if vout and vout > 1.0 else VOUT    # the node averages what the INA226 sees at the load
    on = bool(on and ch and vin and vin > vout + 1.0)
    t = [(-span / 2) + k * span / (n - 1) for k in range(n)]
    if not on:
        return t, [random.gauss(0, 0.02) for _ in t]
    m = FAKE_2 if (ch == 4 and SIM_FAKE == 2) else SW_MODEL[ch]
    f = m["f"]
    period = 1 / f
    t_on = t_d = None
    if "t_on_ref" in m:
        load = max(iload or 0.0, 0.02)
        t_on = m["t_on_ref"] * math.sqrt((load / I_REF) * VIN_REF * (VIN_REF - VOUT) / (vin * (vin - vout)))
        t_d = t_on * (vin - vout) / (vout + V_DIODE)
        if t_on + t_d >= period:
            t_on = None                             # does not fit: continuous like the others
    duty = (vout + V_DIODE) / (vin + V_DIODE)
    v = []
    for x in t:
        tp = (x * f % 1.0) / f                      # time since the last rising edge
        if t_on is None:
            v.append((vin if tp < duty * period else -V_DIODE) + random.gauss(0, 0.06))
        elif tp < t_on:
            v.append(vin + random.gauss(0, 0.06))
        elif tp < t_on + t_d:
            v.append(-V_DIODE + random.gauss(0, 0.06))
        else:                                       # inductor empty: rings around VOUT from the diode drop
            d = tp - t_on - t_d
            amp = vout + V_DIODE
            v.append(vout - amp * math.exp(-d / m["ring_tau"]) * math.cos(2 * math.pi * m["ring_hz"] * d)
                     + random.gauss(0, 0.06))
    return t, v


def _png(t, v, w=640, h=400):
    """Minimal scope-looking picture: dark screen, grid, yellow trace."""
    px = bytearray([16, 18, 22] * w * h)

    def dot(x, y, c):
        if 0 <= x < w and 0 <= y < h:
            i = 3 * (y * w + x)
            px[i:i + 3] = bytes(c)
    for gx in range(0, w, w // 10):
        for y in range(h):
            dot(gx, y, (50, 54, 60))
    for gy in range(0, h, h // 8):
        for x in range(w):
            dot(x, gy, (50, 54, 60))
    lo, hi = -5.0, 30.0
    prev = None
    for k in range(len(t)):
        x = int(k * (w - 1) / (len(t) - 1))
        y = int((1 - (v[k] - lo) / (hi - lo)) * (h - 1))
        if prev:
            for yy in range(min(prev[1], y), max(prev[1], y) + 1):
                dot(x, yy, (240, 220, 40))
        dot(x, y, (240, 220, 40))
        prev = (x, y)
    raw = b"".join(b"\x00" + bytes(px[3 * w * r:3 * w * (r + 1)]) for r in range(h))

    def chunk(tag, data):
        return struct.pack(">I", len(data)) + tag + data + struct.pack(">I", zlib.crc32(tag + data))
    return (b"\x89PNG\r\n\x1a\n" + chunk(b"IHDR", struct.pack(">IIBBBBB", w, h, 8, 2, 0, 0, 0))
            + chunk(b"IDAT", zlib.compress(raw, 6)) + chunk(b"IEND", b""))


def _stamp(png: bytes, source: str | None = None) -> bytes:
    """Write SIMULATED DATA across the picture, so a simulated snapshot is never taken for a measurement,
    also outside the GUI. Needs Pillow; without it the picture stays as is (the GUI still marks it)."""
    try:
        import io
        from PIL import Image, ImageDraw, ImageFont
        im = Image.open(io.BytesIO(png)).convert("RGB")
        w, h = im.size
        d = ImageDraw.Draw(im)
        try:
            big, small = ImageFont.load_default(size=max(18, h // 9)), ImageFont.load_default(size=max(11, h // 32))
        except TypeError:                       # Pillow < 10.1: fixed small font only
            big = small = ImageFont.load_default()
        red, dark = (235, 40, 40), (0, 0, 0)
        d.rectangle([2, 2, w - 3, h - 3], outline=red, width=max(3, h // 120))
        d.text((w / 2, h * 0.30), "SIMULATED DATA", font=big, fill=red, anchor="mm", stroke_width=3, stroke_fill=dark)
        line = "replayed from " + source if source else "model, not a measurement"
        d.text((w / 2, h * 0.30 + h // 11), line, font=small, fill=red, anchor="mm", stroke_width=2, stroke_fill=dark)
        out = io.BytesIO()
        im.save(out, format="PNG")
        return out.getvalue()
    except Exception:
        return png


class SimScope:
    """Stands in for the pyvisa instrument bench/scope.py talks to."""

    def __init__(self):
        self.timeout = 5000
        self.read_termination = self.write_termination = "\n"
        self._pending = []
        self._frozen = None          # (t, v) while STOPped
        self._frozen_png = None
        self._tdiv = 10e-6           # TIMebase:SCALe, the record spans 10 divisions

    def _freeze(self):
        rec = scope_recording() if scope_recording is not None else None
        if rec:
            self._frozen = (rec["t"], rec["v"])
            self._frozen_png = _stamp(rec["png"], rec.get("source"))
        else:
            self._frozen = _sw_wave(span=10 * self._tdiv)
            self._frozen_png = None

    def _wave(self):
        return self._frozen or _sw_wave(span=10 * self._tdiv)

    def write(self, cmd: str) -> None:
        c = cmd.strip().upper()
        if c == "*IDN?":
            self._pending.append("Rohde&Schwarz,RTM2034,SIMULATED,sim")
        elif c.startswith("SYST") and c.endswith("ERR?") or c == "SYSTEM:ERROR?":
            self._pending.append('0,"No error"')
        elif c == "*OPC?":
            self._pending.append("1")
        elif c.startswith("TIM") and "SCAL" in c and not c.endswith("?"):
            self._tdiv = float(c.split()[-1])
        elif c == "STOP":
            self._freeze()
        elif c == "RUN":
            self._frozen = self._frozen_png = None
        elif c.endswith(":DATA:HEADER?"):
            t, _ = self._wave()
            self._pending.append(f"{t[0]:.6e},{t[-1]:.6e},{len(t)},1")
        elif c.endswith(":DATA?") and c.startswith("CHAN"):
            _, v = self._wave()
            self._pending.append(",".join(f"{x:.3f}" for x in v))

    def read(self) -> str:
        if not self._pending:
            raise pyvisa.VisaIOError(constants.StatusCode.error_timeout)
        return self._pending.pop(0)

    def query(self, cmd: str) -> str:
        self.write(cmd)
        return self.read()

    def query_binary_values(self, cmd, datatype="B", container=bytes, **kw):
        if self._frozen_png is not None:
            return self._frozen_png
        t, v = self._wave()
        return _stamp(_png(t, v))

    def clear(self) -> None:
        self._pending.clear()

    def close(self) -> None:
        pass
