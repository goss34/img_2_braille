import serial
import time
import numpy as np
import cv2
from utils.simple_hdlc import HDLC
from utils.comms_codes import CMD_SEND_LINE

# === Canute 360 Display Config ===
SERIAL_PORT = '/dev/ttyACM0'
BAUDRATE = 115200
ROWS = 9          # Number of Braille lines
CHARS = 40        # Characters per line
DOT_ROWS = ROWS * 4    # = 36
DOT_COLS = CHARS * 2   # = 80

# === Create full-size occupancy map: 36x80 ===
occupancy_map = np.zeros((DOT_ROWS, DOT_COLS), dtype=np.uint8)

# Fill with a horizontal stripe pattern (every other row)
for y in range(DOT_ROWS):
    for x in range(DOT_COLS):
        occupancy_map[y, x] = 255 if y % 2 == 0 else 0

# === Convert occupancy map to Braille character lines ===
def occupancy_to_braille(map2d):
    rows, cols = map2d.shape
    assert rows % 4 == 0 and cols % 2 == 0
    lines = []

    for br in range(0, rows, 4):
        line = ''
        for bc in range(0, cols, 2):
            dots = 0
            for dy in range(4):
                for dx in range(2):
                    y = br + dy
                    x = bc + dx
                    if map2d[y, x]:
                        dot_idx = {
                            (0,0): 0, (1,0): 1, (2,0): 2, (3,0): 6,
                            (0,1): 3, (1,1): 4, (2,1): 5, (3,1): 7
                        }[(dy, dx)]
                        dots |= (1 << dot_idx)
            braille_char = chr(0x2800 + dots)
            line += braille_char
        lines.append(line)
    return lines

# === Send one Braille line to Canute ===
def send_line(hdlc, row, data_str):
    data_bytes = bytes([ord(c) & 0x3F for c in data_str.ljust(CHARS)])
    msg = bytes([CMD_SEND_LINE, row]) + data_bytes
    hdlc.sendFrame(msg)
    print(f"[{time.strftime('%H:%M:%S')}] Sent row {row}: {data_str}")

# === Main ===
def main():
    ser = serial.Serial(SERIAL_PORT, BAUDRATE, timeout=0.2)
    hdlc = HDLC(ser)

    braille_lines = occupancy_to_braille(occupancy_map)
    print("Sending full-screen Braille map to Canute 360...")

    try:
        for row in range(ROWS):
            send_line(hdlc, row, braille_lines[row])
            time.sleep(0.05)

        # === Show image with OpenCV ===
        scale = 5  # adjust for display
        resized = cv2.resize(occupancy_map, (DOT_COLS * scale, DOT_ROWS * scale), interpolation=cv2.INTER_NEAREST)
        cv2.imshow("Occupancy Grid", resized)
        print("Press any key in the image window to exit.")
        cv2.waitKey(0)
        cv2.destroyAllWindows()

    except KeyboardInterrupt:
        print("\nInterrupted by user.")
    finally:
        ser.close()
        print("Serial port closed.")

if __name__ == '__main__':
    main()
