import serial
import time
from utils.simple_hdlc import HDLC
from utils.comms_codes import CMD_SEND_LINE

# === Config ===
SERIAL_PORT = '/dev/ttyACM0'
BAUDRATE = 115200
ROWS = 9
CHARS = 40

# === Pattern generator with slight row variation ===
def make_test_line(row):
    return bytes([(i + row) % 64 for i in range(CHARS)])

# === Send one Braille line ===
def send_line(hdlc, row, data):
    msg = bytes([CMD_SEND_LINE, row]) + data
    hdlc.sendFrame(msg)
    print(f"[{time.strftime('%H:%M:%S')}] Sent row {row}")

# === Main function ===
def main():
    ser = serial.Serial(SERIAL_PORT, BAUDRATE, timeout=0.2)
    hdlc = HDLC(ser)

    print("Sending test Braille pattern...")
    try:
        for row in range(ROWS):
            send_line(hdlc, row, make_test_line(row))
            time.sleep(0.05)

    except KeyboardInterrupt:
        print("\nInterrupted by user.")
    finally:
        ser.close()
        print("Serial port closed.")

if __name__ == '__main__':
    main()
