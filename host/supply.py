"""The OWON supply for the server: one open connection, one lock, rail-dut limits.

All instrument work is done by bench/psu.py from power-rail-bench, unchanged.
Closing never touches the output. Switching off is a deliberate action.
"""

import contextlib
import threading

import sim
from bench import config, psu
from bench.psu import PsuError


class Supply:
    def __init__(self):
        self._lock = threading.Lock()
        self._stack: contextlib.ExitStack | None = None
        self._inst = None
        self.identity: str | None = None
        self.port_name: str | None = None
        self.last: dict | None = None

    @property
    def connected(self) -> bool:
        return self._inst is not None

    def open(self, port: str | None) -> str:
        self.close()
        with self._lock:
            stack = contextlib.ExitStack()
            try:
                if port == sim.PORT:
                    inst, name = sim.SimOwon(), sim.PORT
                else:
                    name = psu.resolve_port(port)
                    inst = stack.enter_context(psu.open_psu(name))
                identity = psu.idn(inst)
                if not identity.startswith(config.PSU_IDN_PREFIX):
                    raise PsuError(f"{name} answered {identity!r}, not {config.PSU_IDN_PREFIX}")
            except Exception:
                stack.close()
                raise
            self._stack, self._inst = stack, inst
            self.identity, self.port_name = identity, name
        return identity

    def close(self) -> None:
        with self._lock:
            if self._inst is not None and self.port_name == sim.PORT:
                self._inst.close()
            if self._stack is not None:
                self._stack.close()
            self._stack = self._inst = None
            self.identity = self.port_name = self.last = None

    def call(self, function, *args):
        with self._lock:
            if self._inst is None:
                raise PsuError("supply is not connected")
            return function(self._inst, *args)

    # --- with the board limits -----------------------------------------------
    def set_voltage(self, volts: float) -> float:
        if volts != 0 and not config.RAIL_MIN_VIN_V <= volts <= config.RAIL_MAX_VIN_V:
            raise PsuError(f"VIN {volts} V outside the board range "
                           f"{config.RAIL_MIN_VIN_V}..{config.RAIL_MAX_VIN_V} V")
        return self.call(psu.set_voltage, volts)

    def set_current(self, amps: float) -> float:
        return self.call(psu.set_current, amps)

    def prepare(self) -> None:
        """Fixed protection for every run: OVP, OCP, CC limit."""
        self.call(psu.set_ovp, config.RAIL_OVP_V)
        self.call(psu.set_ocp, config.RAIL_OCP_A)
        self.call(psu.set_current, config.RAIL_CURRENT_LIMIT_A)

    def output(self, on: bool) -> None:
        self.call(psu.output, on)

    def is_on(self) -> bool:
        return self.call(psu.output_is_on)

    def wait_for(self, volts: float, timeout_s: float = psu.ON_TIMEOUT_S) -> None:
        self.call(psu.wait_for_voltage, volts, timeout_s)

    def read_state(self) -> dict:
        with self._lock:
            if self._inst is None:
                raise PsuError("supply is not connected")
            inst = self._inst
            v_set = float(psu.query(inst, psu.CMD_GET_VOLT))
            i_set = float(psu.query(inst, psu.CMD_GET_CURR))
            on = psu.output_is_on(inst)
            v, i = psu.measure(inst)
        self.last = {"v_set": v_set, "i_set": i_set, "on": on, "v": v, "i": i}
        return self.last
