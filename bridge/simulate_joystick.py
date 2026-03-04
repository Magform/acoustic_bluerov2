import time
from evdev import UInput, AbsInfo, ecodes as e

cap = {
    e.EV_KEY: [e.BTN_JOYSTICK],
    e.EV_ABS: {
        e.ABS_X: AbsInfo(0, -32767, 32767, 0, 0, 0),
        e.ABS_Y: AbsInfo(0, -32767, 32767, 0, 0, 0),
    }
}

ui = UInput(
    cap,
    name="VirtualTestJoystick",
    vendor=0x1234,
    product=0x5678,
    bustype=e.BUS_USB,
)

i=0
def set_axis(x, y):
    global i
    ui.write(e.EV_ABS, e.ABS_X, x)
    ui.write(e.EV_ABS, e.ABS_Y, y)
    ui.syn()
    i += 1
    print(f"[{i}] Sent X={x} Y={y}")

while True:
    set_axis(0, -32767)
    time.sleep(10)

    set_axis(0, 0)
    time.sleep(10)

    set_axis(32767, 0)
    time.sleep(10)

    set_axis(0, 0)
    time.sleep(10)

    set_axis(0, 32767)
    time.sleep(10)

    set_axis(0, 0)
    time.sleep(10)

    set_axis(-32767, 0)
    time.sleep(10)

    set_axis(0, 0)
    time.sleep(10)