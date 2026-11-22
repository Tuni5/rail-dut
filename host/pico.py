"""Serial link to the rail-dut Pico (firmware/pico/main.py). One lock, one line per answer."""

import threading
import time

import serial

import sim


class PicoError(Exception):
    pass


class PicoLink:
    def __init__(self, timeout_s: float = 1.0):
        self._lock = threading.Lock()
        self._port = None
        self._timeout_s = timeout_s
        self.port_name: str | None = None
        self.identity: str | None = None

    @property
    def connected(self) -> bool:
        return self._port is not None

    def open(self, port_name: str) -> str:
        self.close()
        if port_name == sim.PORT:
            port = sim.SimPicoPort()
        else:
            try:
                port = serial.Serial(port_name, 115200, timeout=self._timeout_s)
            except serial.SerialException as error:
                raise PicoError(f"cannot open {port_name}: {error}") from error
            time.sleep(0.1)
            port.reset_input_buffer()
        self._port, self.port_name = port, port_name
        try:
            self.identity = self._ask("IDN").removeprefix("OK ")
        except PicoError:
            self.close()
            raise
        if not self.identity.startswith("rail-dut"):
            ident = self.identity
            self.close()
            raise PicoError(f"{port_name} answered {ident!r}, not rail-dut firmware")
        return self.identity

    def close(self) -> None:
        with self._lock:
            if self._port is not None:
                try:
                    self._port.write(b"OFF\n")
                    self._port.readline()
                    self._port.write(b"LED off\n")
                    self._port.readline()
                except Exception:
                    pass
                try:
                    self._port.close()
                except Exception:
                    pass
            self._port = None
            self.port_name = self.identity = None

    # --- commands -----------------------------------------------------------
    def en(self, ch: int, on: bool) -> None:
        self._ask(f"EN {ch} {int(on)}")

    def load(self, n: int, on: bool) -> None:
        self._ask(f"LOAD {n} {int(on)}")

    def off(self) -> None:
        self._ask("OFF")

    def led(self, name: str) -> None:
        self._ask(f"LED {name}")

    def read(self) -> tuple[float, float, float, float]:
        """VIN, Iin, VLOAD, Iload in V and A."""
        vals = self._ask("READ").split()[1:]
        return tuple(float(v) for v in vals)

    def state(self) -> dict:
        fields = dict(f.split("=") for f in self._ask("STATE").split()[1:])
        return {"en": [c == "1" for c in fields["en"]], "load": [c == "1" for c in fields["load"]],
                "wdt": fields["wdt"] == "1"}

    def _ask(self, line: str) -> str:
        with self._lock:
            if self._port is None:
                raise PicoError("Pico is not connected")
            try:
                self._port.write((line + "\n").encode("ascii"))
                raw = self._port.readline()
            except serial.SerialException as error:
                self._port = None
                raise PicoError(f"serial port lost ({error}); unplugged or reset?") from error
        if not raw:
            raise PicoError(f"no answer to {line.split()[0]} within {self._timeout_s} s")
        reply = raw.decode("ascii", errors="replace").strip()
        if not reply.startswith("OK"):
            raise PicoError(reply)
        return reply
