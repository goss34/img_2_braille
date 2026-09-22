#!/usr/bin/env python3
"""Standalone Canute 360 test tool: no ROS, no camera.

Talks to the board with the same HDLC serial framing as braille_server_node and logs
every frame in both directions, with timestamps, so we can see what the board really does.

It only sends what braille_server_node already sends (SEND_LINE, SEND_BUTTONS) plus
read-only queries (GET_CHARS, GET_ROWS; VERSION and POLL only with `probe --extended`).

Stop braille_server_node (or the whole launch) first: two programs sharing one serial
port corrupt each other's frames.

  ports                 list serial ports with their USB vendor:product IDs
  probe                 ask the board for its size and button state; log the raw replies
  pattern NAME          send a test page, log replies, and time the refresh
  buttons               log button changes while you press them

Patterns:
  blank          every pin down
  full           every pin up; use it to measure dot spacing with calipers
  dots           groups of six cells raising one dot each, bit 0..5; checks dot order
  rows           line N has N+1 full cells from the left; checks line order and left/right
  border         a one-dot frame around the whole 27 x 80 dot area (6-dot cells)
  legacy-border  the same frame drawn the way the pipeline does today (36 dot rows,
                 8-dot cells, then masked to 6 bits); expect the bottom edge to be missing

Examples:
  python3 tools/canute_test.py ports
  python3 tools/canute_test.py probe
  python3 tools/canute_test.py pattern dots
  python3 tools/canute_test.py pattern full --rows 0
  python3 tools/canute_test.py buttons --seconds 30
  python3 tools/canute_test.py probe --port /dev/cu.usbmodem1101     (macOS)
"""
import argparse
import os
import subprocess
import sys
import threading
import time
from collections import namedtuple
from datetime import datetime

# Use the repo's copy of the HDLC code when run from the repo; otherwise fall back to the
# installed braille_interface package (source the workspace first).
_REPO_PKG = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'src', 'braille_interface')
if os.path.isdir(_REPO_PKG):
    sys.path.insert(0, os.path.abspath(_REPO_PKG))

try:
    import serial
except ImportError:
    sys.exit('pyserial is missing. Install it with: pip install pyserial crcmod')
try:
    from braille_interface.util import comms_codes as cc
    from braille_interface.util.simple_hdlc import HDLC, calcCRC
except ImportError as e:
    sys.exit(f'Could not import braille_interface.util ({e}).\n'
             'Run from the repo, or source the workspace, and make sure crcmod is installed.')

ROWS, CELLS = 9, 40

# Copied from braille_server_node.py (importing it needs rclpy).
BUTTON_MAP_X = {0: 'ROW_0', 1: 'ROW_1', 2: 'ROW_2', 3: 'ROW_3', 4: 'ROW_4', 5: 'ROW_5', 6: 'ROW_6', 7: 'ROW_7'}
BUTTON_MAP_Y = {3: 'LEFT', 4: 'CENTER', 5: 'RIGHT'}

CMD_NAMES = {
    cc.CMD_GET_CHARS: 'GET_CHARS', cc.CMD_GET_ROWS: 'GET_ROWS', cc.CMD_SEND_PAGE: 'SEND_PAGE',
    cc.CMD_SEND_VERSION: 'SEND_VERSION', cc.CMD_SEND_ERROR: 'SEND_ERROR', cc.CMD_SEND_OK: 'SEND_OK',
    cc.CMD_SEND_LINE: 'SEND_LINE', cc.CMD_RESET: 'RESET', cc.CMD_WARMUP: 'WARMUP', cc.CMD_LOWER: 'LOWER',
    cc.CMD_SEND_BUTTONS: 'SEND_BUTTONS', cc.CMD_POLL: 'POLL',
}

Frame = namedtuple('Frame', 't data crc_ok')

# Replies to queries braille_server_node never sends, so they can't be misread by it.
NODE_NEVER_ASKS = {cc.CMD_GET_CHARS, cc.CMD_GET_ROWS, cc.CMD_SEND_VERSION, cc.CMD_POLL}


# ---------------------------------------------------------------- patterns
# Each pattern is 9 lines of 40 cell bytes. A cell byte uses bits 0-5 for dots 1-6 in
# Unicode braille order: dots 1-3 run down the left column, dots 4-6 down the right.

SIX_DOT_BIT = {(0, 0): 0, (1, 0): 1, (2, 0): 2, (0, 1): 3, (1, 1): 4, (2, 1): 5}  # (dy, dx) -> bit
# braille_server_node.occupancy_to_braille's 8-dot mapping; send_line then keeps only bits 0-5.
LEGACY_BIT = {(0, 0): 0, (1, 0): 1, (2, 0): 2, (3, 0): 6, (0, 1): 3, (1, 1): 4, (2, 1): 5, (3, 1): 7}


def pack(img, cell_h, bit_map):
    lines = []
    for r in range(ROWS):
        cells = []
        for c in range(CELLS):
            v = 0
            for (dy, dx), bit in bit_map.items():
                if img[r * cell_h + dy][c * 2 + dx]:
                    v |= 1 << bit
            cells.append(v & 0x3F)
        lines.append(cells)
    return lines


def outline(h, w):
    img = [[False] * w for _ in range(h)]
    for x in range(w):
        img[0][x] = img[h - 1][x] = True
    for y in range(h):
        img[y][0] = img[y][w - 1] = True
    return img


def pattern_dots():
    group = [1 << bit for bit in range(6)] + [0]
    line = [group[i % len(group)] for i in range(CELLS)]
    return [list(line) for _ in range(ROWS)]


PATTERNS = {
    'blank': lambda: [[0] * CELLS for _ in range(ROWS)],
    'full': lambda: [[0x3F] * CELLS for _ in range(ROWS)],
    'dots': pattern_dots,
    'rows': lambda: [[0x3F] * (r + 1) + [0] * (CELLS - r - 1) for r in range(ROWS)],
    'border': lambda: pack(outline(ROWS * 3, CELLS * 2), 3, SIX_DOT_BIT),
    'legacy-border': lambda: pack(outline(ROWS * 4, CELLS * 2), 4, LEGACY_BIT),
}


def to_unicode(cells):
    return ''.join(chr(0x2800 + (b & 0x3F)) for b in cells)


def decode_buttons(data):
    """Decode a frame the way braille_server_node.handle_button_frame does, plus unmapped bits."""
    if len(data) < 3:
        return []
    names = []
    for bit in range(8):
        if data[1] & (1 << bit):
            names.append(BUTTON_MAP_X.get(bit, f'x-bit{bit}(unmapped)'))
        if data[2] & (1 << bit):
            names.append(BUTTON_MAP_Y.get(bit, f'y-bit{bit}(unmapped)'))
    return names


# ---------------------------------------------------------------- serial link

class Log:
    def __init__(self, path):
        os.makedirs(os.path.dirname(path) or '.', exist_ok=True)
        self.path = path
        self.file = open(path, 'w', encoding='utf-8')
        self.t0 = time.monotonic()

    def __call__(self, msg='', t=None):
        t = time.monotonic() if t is None else t
        line = f'{t - self.t0:8.3f}s  {msg}'
        print(line, flush=True)
        self.file.write(line + '\n')
        self.file.flush()


class Link:
    """HDLC over serial. Sends frames and records every received frame with its arrival time.

    Uses its own frame reader rather than HDLC.readFrame so that it can log CRC failures,
    stray bytes and back-to-back frames instead of hiding them.
    """

    def __init__(self, port, baud, log):
        self.ser = serial.Serial(port, baud, timeout=0.01)
        self.log = log
        self.stray_bytes = 0
        self._buf = bytearray()
        self._in_frame = False
        self._escape = False

    def send(self, payload, quiet=False):
        self.ser.write(HDLC._encode(bytearray(payload)))
        if not quiet:
            self.log(f'TX  {describe_tx(bytes(payload))}')

    def listen(self, seconds, quiet_after=None, stop=None, log_rx=True):
        """Collect frames for up to `seconds`. Stops early after `quiet_after` seconds without
        a frame, or when stop() returns True."""
        frames = []
        start = time.monotonic()
        last = start
        while True:
            now = time.monotonic()
            if now - start >= seconds or (quiet_after is not None and now - last >= quiet_after):
                break
            if stop is not None and stop():
                break
            for b in self.ser.read(self.ser.in_waiting or 1):
                frame = self._feed(b)
                if frame is not None:
                    frames.append(frame)
                    last = frame.t
                    if log_rx:
                        self.log_rx(frame)
        return frames

    def log_rx(self, frame):
        name = CMD_NAMES.get(frame.data[0], f'0x{frame.data[0]:02x}?')
        msg = f'RX  {name} [{frame.data.hex(" ")}] {"crc ok" if frame.crc_ok else "CRC BAD"}'
        if frame.crc_ok and frame.data[0] not in NODE_NEVER_ASKS | {cc.CMD_SEND_BUTTONS}:
            ghost = decode_buttons(frame.data)
            if ghost:
                msg += f'  <- braille_server_node would read this as button(s): {", ".join(ghost)}'
        self.log(msg, t=frame.t)

    def _feed(self, b):
        if b == 0x7E:  # flag: ends the current frame (if any) and starts the next
            frame = self._finish() if self._in_frame and self._buf else None
            self._in_frame, self._buf, self._escape = True, bytearray(), False
            return frame
        if not self._in_frame:
            self.stray_bytes += 1
            return None
        if b == 0x7D:
            self._escape = True
            return None
        if self._escape:
            b ^= 0x20
            self._escape = False
        self._buf.append(b)
        return None

    def _finish(self):
        raw = bytes(self._buf)
        if len(raw) < 3:  # one data byte plus two CRC bytes at minimum
            self.log(f'RX  malformed frame, {len(raw)} byte(s): [{raw.hex(" ")}]')
            return None
        data, crc = raw[:-2], raw[-2:]
        return Frame(time.monotonic(), data, bytes(calcCRC(data)) == crc)

    def close(self):
        self.ser.close()


def describe_tx(data):
    name = CMD_NAMES.get(data[0], f'0x{data[0]:02x}?')
    if data[0] == cc.CMD_SEND_LINE and len(data) >= 2:
        return f'{name} row={data[1]}  {to_unicode(data[2:])}'
    return f'{name} [{data.hex(" ")}]'


# ---------------------------------------------------------------- commands

def cmd_ports(_args):
    from serial.tools import list_ports
    ports = sorted(list_ports.comports(), key=lambda p: p.device)
    if not ports:
        print('No serial ports found.')
    for p in ports:
        ids = f'{p.vid:04x}:{p.pid:04x}' if p.vid is not None else '----:----'
        print(f'{p.device:28s} {ids}  {p.manufacturer or ""} {p.product or p.description or ""}'
              f'  serial={p.serial_number or "-"}')
    return 0


def cmd_probe(args, link, log):
    queries = [
        (cc.CMD_GET_CHARS, 'cells per line (expect 40)'),
        (cc.CMD_GET_ROWS, 'number of lines (expect 9)'),
        (cc.CMD_SEND_BUTTONS, 'button state (press nothing)'),
    ]
    if args.extended:
        queries += [
            (cc.CMD_SEND_VERSION, 'firmware version'),
            (cc.CMD_POLL, 'status flags; comms_codes says bit 0 means "moving"'),
        ]
    for cmd, what in queries:
        log('')
        log(f'--- {CMD_NAMES[cmd]}: {what}')
        link.send([cmd])
        frames = link.listen(args.wait)
        if not frames:
            log('    (no reply)')
        for f in frames:
            if cmd in (cc.CMD_GET_CHARS, cc.CMD_GET_ROWS) and f.crc_ok and f.data[0] == cmd and len(f.data) >= 3:
                log(f'    bytes 1-2 as a little-endian number: {f.data[1] | f.data[2] << 8}')


def cmd_pattern(args, link, log):
    lines = PATTERNS[args.name]()
    rows = parse_rows(args.rows)
    log(f'Pattern "{args.name}". What the board should show (6-dot Unicode braille, line 0 on top):')
    for r, cells in enumerate(lines):
        log(f'    line {r}: {to_unicode(cells)}{"" if r in rows else "   (not sent)"}')

    start = time.monotonic()
    for r in rows:
        link.send(bytes([cc.CMD_SEND_LINE, r]) + bytes(lines[r]))
        link.listen(args.line_delay)  # braille_server_node waits 50 ms between lines too
    log(f'Sent {len(rows)} line(s) in {time.monotonic() - start:.2f} s.')

    pressed = []
    if sys.stdin.isatty() and not args.no_prompt:
        log('Press Enter the moment the board stops moving (Ctrl+C to give up).')
        threading.Thread(target=lambda: (sys.stdin.readline(), pressed.append(time.monotonic())),
                         daemon=True).start()
        # Keep logging replies until a second after Enter.
        link.listen(args.listen, stop=lambda: bool(pressed) and time.monotonic() - pressed[0] > 1.0)
        if pressed:
            log(f'Board finished about {pressed[0] - start:.1f} s after the first line was sent '
                f'({len(rows)} line(s)).', t=pressed[0])
        else:
            log(f'No Enter within {args.listen:.0f} s.')
    else:
        log(f'Listening for replies (until {args.quiet:.0f} s of silence, at most {args.listen:.0f} s).')
        link.listen(args.listen, quiet_after=args.quiet)


def cmd_buttons(args, link, log):
    log(f'Asking for button state every {args.interval:.2f} s for {args.seconds:.0f} s; only changes are logged.')
    log('Press each button once, slowly, one at a time. Note which physical button you pressed.')
    end = time.monotonic() + args.seconds
    last = None
    other = 0
    while time.monotonic() < end:
        link.send([cc.CMD_SEND_BUTTONS], quiet=True)
        for f in link.listen(args.interval, log_rx=False):
            if not f.crc_ok or f.data[0] != cc.CMD_SEND_BUTTONS:
                other += 1
                link.log_rx(f)
            elif f.data != last:
                names = ', '.join(decode_buttons(f.data)) or '(none)'
                log(f'RX  SEND_BUTTONS [{f.data.hex(" ")}]  -> {names}', t=f.t)
                last = f.data
    log(f'Done. {other} frame(s) that were not button replies.')


def parse_rows(text):
    if text == 'all':
        return list(range(ROWS))
    rows = []
    for part in text.split(','):
        lo, _, hi = part.partition('-')
        rows += list(range(int(lo), int(hi or lo) + 1))
    bad = [r for r in rows if not 0 <= r < ROWS]
    if bad:
        raise SystemExit(f'--rows must be between 0 and {ROWS - 1}, got {bad}')
    return rows


def port_in_use_check(force):
    if force:
        return
    try:
        pids = subprocess.run(['pgrep', '-f', 'braille_server_node'],
                              capture_output=True, text=True).stdout.split()
    except FileNotFoundError:
        return
    if pids:
        raise SystemExit(f'braille_server_node seems to be running (pid {", ".join(pids)}).\n'
                         'Stop it first: two programs on one serial port corrupt each other. '
                         'Use --force to run anyway.')


# ---------------------------------------------------------------- main

def build_parser():
    common = argparse.ArgumentParser(add_help=False)
    common.add_argument('--port', default='/dev/ttyACM0', help='serial port (default: %(default)s)')
    common.add_argument('--baud', type=int, default=57600, help='baud rate (default: %(default)s)')
    common.add_argument('--settle', type=float, default=2.0,
                        help='seconds to listen after opening the port, before sending anything')
    common.add_argument('--log-dir', default='canute_logs', help='where log files go (default: %(default)s)')
    common.add_argument('--force', action='store_true', help='run even if braille_server_node seems to be running')

    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = ap.add_subparsers(dest='cmd', required=True)
    sub.add_parser('ports', help='list serial ports')

    p = sub.add_parser('probe', parents=[common], help='query board size and buttons')
    p.add_argument('--extended', action='store_true', help='also send VERSION and POLL queries')
    p.add_argument('--wait', type=float, default=1.0, help='seconds to collect replies per query')

    p = sub.add_parser('pattern', parents=[common], help='send a test page')
    p.add_argument('name', choices=list(PATTERNS))
    p.add_argument('--rows', default='all', help='lines to send, e.g. 0 or 0,4,8 or 0-3 (default: all)')
    p.add_argument('--line-delay', type=float, default=0.05, help='seconds between lines (default: %(default)s)')
    p.add_argument('--listen', type=float, default=60.0, help='max seconds to wait after sending')
    p.add_argument('--quiet', type=float, default=5.0,
                   help='without the Enter prompt: stop after this many seconds with no reply')
    p.add_argument('--no-prompt', action='store_true', help="don't ask you to press Enter")

    p = sub.add_parser('buttons', parents=[common], help='log button presses')
    p.add_argument('--seconds', type=float, default=30.0)
    p.add_argument('--interval', type=float, default=0.15)
    return ap


def main():
    args = build_parser().parse_args()
    if args.cmd == 'ports':
        return cmd_ports(args)

    port_in_use_check(args.force)
    name = args.cmd + (f'_{args.name}' if args.cmd == 'pattern' else '')
    log = Log(os.path.join(args.log_dir, f'{datetime.now():%Y%m%d_%H%M%S}_{name}.log'))
    log(f'canute_test.py {" ".join(sys.argv[1:])}')

    try:
        link = Link(args.port, args.baud, log)
    except serial.SerialException as e:
        log(f'Could not open {args.port}: {e}')
        log('List ports with: python3 tools/canute_test.py ports')
        return 1

    try:
        log(f'Opened {args.port}. Listening {args.settle:.1f} s for anything the board sends on connect.')
        link.listen(args.settle)
        {'probe': cmd_probe, 'pattern': cmd_pattern, 'buttons': cmd_buttons}[args.cmd](args, link, log)
    except KeyboardInterrupt:
        log('Interrupted.')
    finally:
        if link.stray_bytes:
            log(f'{link.stray_bytes} stray byte(s) arrived outside any frame.')
        link.close()
        log(f'Log saved to {log.path}')
    return 0


if __name__ == '__main__':
    sys.exit(main())
