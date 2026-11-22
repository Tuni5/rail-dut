# rail-dut rev A - command firmware (MicroPython, Pico)
# One command per line over USB serial, one answer per line: "OK ..." or "ERR ...".
#
#   IDN              -> OK rail-dut,revA,0.1,mcu=RP2040,board=Raspberry Pi Pico with RP2040,
#                          uid=E6614103E7..., f_sys=125000000,mpy=1.24.1,ina_in=5449/2260,ina_load=5449/2260
#   EN <1-4> <0|1>   regulator CHx on/off
#   LOAD <1|2> <0|1> load switch; every change pulses TRIG (GP16) for the scope
#   OFF              all EN and LOAD off
#   READ             -> OK <vin> <iin> <vload> <iload>   (V, A)
#   STATE            -> OK en=0010 load=10 wdt=0
#   LED <name>       off red green blue magenta orange cyan white
#
# Watchdog: no command for WDT_MS -> everything off, LED red. Any command re-arms it.
# EN1..4 = GP6..9, LOAD1/2 = GP14/15, LED R/B/G = GP11/12/13, TRIG = GP16
# U5 input (R20) I2C1 GP2/3, U6 load (R21) I2C0 GP20/21, both 0x40

from machine import Pin, I2C, PWM
import machine
import os
import sys
import select
import time

FW_VERSION = "0.1"
WDT_MS = 3000
LED_DUTY = 0.05          # max brightness 1/20
R_SHUNT_IN = 0.05        # R20
R_SHUNT_LOAD = 0.05      # R21
INA_CONFIG = 0x4527      # AVG 16, 1.1 ms bus + shunt, continuous

en = [Pin(p, Pin.OUT, value=0) for p in (6, 7, 8, 9)]
load = [Pin(p, Pin.OUT, value=0) for p in (14, 15)]
trig = Pin(16, Pin.OUT, value=0)


class Led:
    def __init__(self, pin):
        self.pwm = PWM(Pin(pin))
        self.pwm.freq(1000)
        self.set(0)

    def set(self, level):
        self.pwm.duty_u16(int(65535 * LED_DUTY * level))


R, B, G = Led(11), Led(12), Led(13)
COLORS = {
    "off": (0, 0, 0), "red": (1, 0, 0), "green": (0, 1, 0), "blue": (0, 0, 1),
    "magenta": (1, 0, 1), "orange": (1, 0.25, 0), "cyan": (0, 1, 1), "white": (1, 1, 1),
}


def led(name):
    r, g, b = COLORS[name]
    R.set(r)
    G.set(g)
    B.set(b)


class INA226:
    def __init__(self, bus, shunt, addr=0x40):
        self.bus, self.shunt, self.addr = bus, shunt, addr
        self.ok = False
        try:
            self.ids = "%04X/%04X" % (self.rd(0xFE), self.rd(0xFF))
            self.wr(0x00, INA_CONFIG)
            self.ok = True
        except OSError:
            self.ids = "none"

    def rd(self, reg):
        b = self.bus.readfrom_mem(self.addr, reg, 2)
        return (b[0] << 8) | b[1]

    def wr(self, reg, val):
        self.bus.writeto_mem(self.addr, reg, bytes((val >> 8, val & 0xFF)))

    def vbus(self):
        return self.rd(0x02) * 1.25e-3

    def current(self):
        v = self.rd(0x01)
        if v & 0x8000:
            v -= 0x10000
        return v * 2.5e-6 / self.shunt


u5 = INA226(I2C(1, sda=Pin(2), scl=Pin(3), freq=400_000), R_SHUNT_IN)
u6 = INA226(I2C(0, sda=Pin(20), scl=Pin(21), freq=400_000), R_SHUNT_LOAD)

wdt_tripped = False


def all_off():
    for p in en + load:
        p.value(0)


def pulse_trig():
    trig.value(1)
    time.sleep_us(20)
    trig.value(0)


def mcu_info():
    """What the Pico says about itself: board, chip, unique ID, clock, MicroPython version."""
    board = getattr(sys.implementation, "_machine", os.uname().machine).replace(",", " ")
    chip = board.split()[-1] if " with " in board else "unknown"
    uid = "".join("%02X" % b for b in machine.unique_id())
    return "mcu=%s,board=%s,uid=%s,f_sys=%d,mpy=%s" % (chip, board, uid, machine.freq(), os.uname().release)


MCU = mcu_info()


def handle(line):
    global wdt_tripped
    parts = line.split()
    if not parts:
        return None
    cmd = parts[0].upper()
    if cmd == "IDN":
        return "OK rail-dut,revA,%s,%s,ina_in=%s,ina_load=%s" % (FW_VERSION, MCU, u5.ids, u6.ids)
    if cmd == "EN":
        ch, on = int(parts[1]), int(parts[2])
        if not 1 <= ch <= 4:
            return "ERR EN channel 1..4"
        en[ch - 1].value(1 if on else 0)
        wdt_tripped = False
        return "OK"
    if cmd == "LOAD":
        n, on = int(parts[1]), int(parts[2])
        if n not in (1, 2):
            return "ERR LOAD 1 or 2"
        load[n - 1].value(1 if on else 0)
        pulse_trig()
        wdt_tripped = False
        return "OK"
    if cmd == "OFF":
        all_off()
        return "OK"
    if cmd == "READ":
        if not (u5.ok and u6.ok):
            return "ERR READ INA missing (in=%s load=%s)" % (u5.ids, u6.ids)
        return "OK %.4f %.5f %.4f %.5f" % (u5.vbus(), u5.current(), u6.vbus(), u6.current())
    if cmd == "STATE":
        return "OK en=%s load=%s wdt=%d" % ("".join(str(p.value()) for p in en),
                                           "".join(str(p.value()) for p in load), wdt_tripped)
    if cmd == "LED":
        name = parts[1].lower() if len(parts) > 1 else ""
        if name not in COLORS:
            return "ERR LED " + " ".join(COLORS)
        led(name)
        return "OK"
    return "ERR CMD " + cmd


def main():
    global wdt_tripped
    poll = select.poll()
    poll.register(sys.stdin, select.POLLIN)
    buf = ""
    last = time.ticks_ms()
    led("blue")
    while True:
        if poll.poll(20):
            c = sys.stdin.read(1)
            if c in "\r\n":
                if buf:
                    try:
                        reply = handle(buf)
                    except (ValueError, IndexError):
                        reply = "ERR ARGS " + buf
                    except OSError as e:
                        reply = "ERR I2C %s" % e
                    if reply:
                        sys.stdout.write(reply + "\n")
                    buf = ""
                    last = time.ticks_ms()
            else:
                buf += c
                if len(buf) > 80:
                    buf = ""
        if not wdt_tripped and time.ticks_diff(time.ticks_ms(), last) > WDT_MS:
            if any(p.value() for p in en + load):
                all_off()
                led("red")
                wdt_tripped = True


try:
    main()
finally:
    all_off()
    led("off")
