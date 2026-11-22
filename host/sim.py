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
            if on:
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
