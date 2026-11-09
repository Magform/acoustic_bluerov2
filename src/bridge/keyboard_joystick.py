import sys
import termios
import tty
import yaml
from evdev import UInput, AbsInfo, ecodes as e

# Virtual joystick driven by the keyboard (step mode).
# Axes are created in the same order as an Xbox pad, so the xbox node
# sees it exactly like the real controller (run it with no pad plugged in).

STEP = 0.25  # effective stick step, after the dead zone

KEYS = {
    'w': ('left_v', -1), 's': ('left_v', +1),     # forward / backward
    'a': ('left_h', -1), 'd': ('left_h', +1),     # strafe
    'UP': ('right_v', -1), 'DOWN': ('right_v', +1),   # emerge / submerge
    'LEFT': ('right_h', -1), 'RIGHT': ('right_h', +1),  # yaw
}

AXES = {
    'left_h':  e.ABS_X,   # SDL axis 0
    'left_v':  e.ABS_Y,   # SDL axis 1
    'right_h': e.ABS_RX,  # SDL axis 3
    'right_v': e.ABS_RY,  # SDL axis 4
}

ARROWS = {'A': 'UP', 'B': 'DOWN', 'C': 'RIGHT', 'D': 'LEFT'}

def load_dead_zone():
    try:
        with open('config.yaml') as f:
            return float(yaml.safe_load(f)['dead_zone'])
    except Exception:
        return 0.2

def to_raw(level, dead_zone):
    # Inverse of the controller's dead zone, so every step really moves the ROV
    if level == 0:
        return 0
    raw = dead_zone + abs(level) * (1.0 - dead_zone)
    return int((1 if level > 0 else -1) * raw * 32767)

def read_key():
    ch = sys.stdin.read(1)
    if ch == '\x1b':
        if sys.stdin.read(1) == '[':
            return ARROWS.get(sys.stdin.read(1))
        return None
    return ch.lower()

def main():
    dead_zone = load_dead_zone()
    abs_info = AbsInfo(0, -32767, 32767, 0, 0, 0)
    cap = {
        e.EV_KEY: [e.BTN_SOUTH],
        e.EV_ABS: [(code, abs_info) for code in
                   (e.ABS_X, e.ABS_Y, e.ABS_Z, e.ABS_RX, e.ABS_RY, e.ABS_RZ)],
    }
    ui = UInput(cap, name='KeyboardJoystick', vendor=0x1234, product=0x5678, bustype=e.BUS_USB)

    levels = {name: 0.0 for name in AXES}

    def send():
        for name, code in AXES.items():
            ui.write(e.EV_ABS, code, to_raw(levels[name], dead_zone))
        ui.syn()
        status = '  '.join(f'{n}={v:+.2f}' for n, v in levels.items())
        print(f'\r{status}   ', end='', flush=True)

    print('W/S forward  A/D strafe  arrows up-down/yaw  SPACE stop  Q quit')
    old = termios.tcgetattr(sys.stdin)
    try:
        tty.setcbreak(sys.stdin)
        send()
        while True:
            key = read_key()
            if key == 'q':
                break
            if key == ' ':
                levels = {name: 0.0 for name in AXES}
            elif key in KEYS:
                name, direction = KEYS[key]
                levels[name] = max(-1.0, min(1.0, levels[name] + direction * STEP))
            send()
    finally:
        levels = {name: 0.0 for name in AXES}
        send()
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old)
        ui.close()
        print()

if __name__ == '__main__':
    main()
