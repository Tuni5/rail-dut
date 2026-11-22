"""Bench configuration: what is true about THIS table.

psu.py and common.py come unchanged from power-rail-bench and read their
settings from here. The rail-dut limits at the bottom are enforced by
host/supply.py before anything reaches psu.py.
"""

# --- Supply: OWON SPE3102, USB-serial ----------------------------------------
PSU_IDN_PREFIX = "OWON,SPE3102" # what *IDN? must start with
PSU_MAX_VOLT = 30.0             # rating; setpoints above are refused by psu.py
PSU_MAX_CURR = 10.0
PSU_PORT = None                 # "COM7" to pin it; None = find by USB vendor ID
PSU_PROBE_VIDS = {0x1A86}       # CH340 bridge inside the supply
PSU_BAUD = 115200               # instrument menu setting

# --- rail-dut board limits -----------------------------------------------------
# VIN of the board: ON/OFF pins of the LM2596 are rated 25 V, so 24 V is the top.
# Below about 7 V a 5 V buck has no headroom left; lower setpoints are refused.
RAIL_MIN_VIN_V = 6.0
RAIL_MAX_VIN_V = 24.0
RAIL_OVP_V = 24.5               # one fixed OVP for every run
RAIL_CURRENT_LIMIT_A = 1.5      # CC limit; 1.4 A load at 5 V is ~0.45 A at 24 V, ~1 A at 8 V
RAIL_OCP_A = 2.0                # trip above that, as a second line
