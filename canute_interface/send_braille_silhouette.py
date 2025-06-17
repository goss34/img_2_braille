import serial
import time
import cv2
import numpy as np
from utils.simple_hdlc import HDLC
from utils.comms_codes import CMD_SEND_LINE

# === Canute 360 dimensions ===
SERIAL_PORT = '/dev/ttyACM0'
BAUDRATE = 115200
ROWS = 9
CHARS = 40
DOT_ROWS = ROWS * 4    # = 36
DOT_COLS = CHARS * 2   # = 80

# === Image loading and processing ===
def load_silhouette(path):
    img = cv2.imread(path, cv2.IMREAD_GRAYSCALE)
    if img is None:
        raise ValueError(f"Image not found: {path}")
    _, binary = cv2.threshold(img, 127, 255, cv2.THRESH_BINARY_INV)
    resized = cv2.resize(binary, (DOT_COLS, DOT_ROWS), interpolation=cv2.INTER_NEAREST)
    return resized

def image_to_braille(occupancy_map):
    lines = []
    for br in range(0, DOT_ROWS, 4):
        line = ''
        for bc in range(0, DOT_COLS, 2):
            dots = 0
            for dy in range(4):
                for dx in range(2):
                    y = br + dy
                    x = bc + dx
                    if occupancy_map[y, x] > 0:
                        dot_idx = {
                            (0,0): 0, (1,0): 1, (2,0): 2, (3,0): 6,
                            (0,1): 3, (1,1): 4, (2,1): 5, (3,1): 7
                        }[(dy, dx)]
                        dots |= (1 << dot_idx)
            braille_char = chr(0x2800 + dots)
            line += braille_char
        lines.append(line)
    return lines

def send_line(hdlc, row, data_str):
    data_bytes = bytes([ord(c) & 0x3F for c in data_str.ljust(CHARS)])
    msg = bytes([CMD_SEND_LINE, row]) + data_bytes
    hdlc.sendFrame(msg)
    print(f"[{time.strftime('%H:%M:%S')}] Sent row {row}")

def clear_display(hdlc):
    blank = ' ' * CHARS
    for row in range(ROWS):
        send_line(hdlc, row, blank)
        time.sleep(0.02)

# === Main loop through multiple images ===
def main():
    image_files = ["images/circle.png", "images/square.png", "images/diagonal.png"]  # Replace with your own image list
    ser = serial.Serial(SERIAL_PORT, BAUDRATE, timeout=0.2)
    hdlc = HDLC(ser)

    try:
        while True:
            for path in image_files:
                print(f"\nDisplaying: {path}")
                silhouette = load_silhouette(path)
                braille_lines = image_to_braille(silhouette)

                # Optional preview
                cv2.imshow("Silhouette", silhouette)
                cv2.waitKey(500)  # Show for 0.5s

                for row in range(ROWS):
                    send_line(hdlc, row, braille_lines[row])
                    time.sleep(0.05)

                time.sleep(2.5)  # Hold on screen
                clear_display(hdlc)
                time.sleep(0.5)

    finally:
        ser.close()
        cv2.destroyAllWindows()
        print("Serial port closed.")

if __name__ == '__main__':
    main()
