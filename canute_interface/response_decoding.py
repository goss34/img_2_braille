import serial
import time
from utils.simple_hdlc import HDLC

# === Command map ===
COMMANDS = {
    0x00: "COMMAND_N_CHARACTERS",
    0x01: "COMMAND_N_ROWS",
    0x03: "COMMAND_VERSION",
    0x04: "COMMAND_SEND_ERROR",
    0x05: "COMMAND_SEND_OK",
    0x07: "COMMAND_RESET",
}

# === Serial + HDLC setup ===
SERIAL_PORT = '/dev/ttyACM0'
BAUDRATE = 115200

ser = serial.Serial(SERIAL_PORT, BAUDRATE, timeout=1)
hdlc = HDLC(ser)

def send_command(cmd_byte):
    """Send a single-byte command over HDLC."""
    label = COMMANDS.get(cmd_byte, f"UNKNOWN_COMMAND (0x{cmd_byte:02X})")
    print(f"\n→ Sending {label}")
    hdlc.sendFrame(bytes([cmd_byte]))

def read_response(timeout=2.0):
    """Attempt to read a response frame from the device."""
    try:
        data = hdlc.readFrame(timeout=timeout)
        print("✅ Response received:", data)
        return data
    except Exception as e:
        print(f"❌ Error: {e}")
        return None

def parse_response(cmd, payload):
    """Interpret the response based on the original command."""
    if not payload or len(payload) < 1:
        print("⚠️ No or invalid payload")
        return

    try:
        if cmd == 0x00 and len(payload) == 3:
            num_chars = payload[1] | (payload[2] << 8)
            print(f"🔤 Characters per row: {num_chars} (status={payload[2]})")

        elif cmd == 0x01 and len(payload) == 3:
            row_count = payload[1]
            print(f"📟 Number of rows: {row_count} (status={payload[2]})")

        elif cmd == 0x03 and len(payload) == 5:
            version = f"{payload[1]}.{payload[2]}.{payload[3]}"
            print(f"🛠️ Firmware version: {version} (status={payload[4]})")

        elif cmd in (0x04, 0x05) and len(payload) >= 1:
            print(f"🔔 Beep acknowledged (echo={payload[0]:#04x})")

        else:
            print(f"⚠️ Unexpected response for command 0x{cmd:02X}: {list(payload)}")

    except Exception as e:
        print(f"❌ Parsing error: {e}")

def run_command_sequence():
    """Run a test sequence of commands and print parsed responses."""
    for cmd in [0x00, 0x01, 0x03, 0x04, 0x05, 0x07]:
        send_command(cmd)
        response = read_response()
        parse_response(cmd, response)
        time.sleep(0.5)

if __name__ == "__main__":
    try:
        run_command_sequence()
    finally:
        ser.close()
