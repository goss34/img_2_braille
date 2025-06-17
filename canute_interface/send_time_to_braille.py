import serial
import time
from utils.simple_hdlc import HDLC
from utils.comms_codes import CMD_SEND_LINE

# === Config ===
SERIAL_PORT = '/dev/ttyACM0'
BAUDRATE = 115200
CHARS = 40  # Characters per row (Canute 360)
ROWS = 9    # Total rows on Canute

# === Generate Braille text line for current time ===
def make_time_line():
    timestr = time.strftime("Time: %I:%M %p")
    ascii_bytes = timestr.encode('ascii')[:CHARS]
    padded = ascii_bytes.ljust(CHARS, b' ')
    return padded

# === Send one Braille line ===
def send_line(hdlc, row, data):
    msg = bytes([CMD_SEND_LINE, row]) + data
    hdlc.sendFrame(msg)
    print(f"[{time.strftime('%H:%M:%S')}] Sent to row {row}: {data.decode(errors='ignore').strip()}")

# === Main function ===
def main():
    ser = serial.Serial(SERIAL_PORT, BAUDRATE, timeout=0.2)
    hdlc = HDLC(ser)

    try:
        print("Displaying current time and clearing other lines...")

        # Send time to row 1
        send_line(hdlc, row=1, data=make_time_line())

        # Clear rows 2 through 9
        blank_line = b' ' * CHARS
        for row in range(2, ROWS + 1):
            send_line(hdlc, row=row, data=blank_line)
            time.sleep(0.02)

    except KeyboardInterrupt:
        print("\nInterrupted by user.")
    finally:
        ser.close()
        print("Serial port closed.")

if __name__ == '__main__':
    main()
