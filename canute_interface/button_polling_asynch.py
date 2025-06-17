import asyncio
import serial
from utils.simple_hdlc import HDLC
from utils.comms_codes import CMD_SEND_BUTTONS

SERIAL_PORT = '/dev/ttyACM0'
BAUDRATE = 115200

# Button bit mappings for frame[1] (X byte)
BUTTON_MAP_X = {
    0: "ROW_0",   # 0b00000001
    1: "ROW_1",   # 0b00000010
    2: "ROW_2",   # 0b00000100
    3: "ROW_3",   # 0b00001000
    4: "ROW_4",   # 0b00010000
    5: "ROW_5",   # 0b00100000
    6: "ROW_6",   # 0b01000000
    7: "ROW_7",   # 0b10000000
}

# Button bit mappings for frame[2] (Y byte)
BUTTON_MAP_Y = {
    3: "LEFT",    # 0b00001000
    4: "CENTER",  # 0b00010000
    5: "RIGHT",   # 0b00100000
    # Add additional bits as needed, e.g., 5: "RIGHT"
}


class ButtonPoller:
    def __init__(self, port, baudrate):
        try:
            self.ser = serial.Serial(port, baudrate, timeout=0.1)
            self.hdlc = HDLC(self.ser)
        except serial.SerialException as e:
            raise RuntimeError(f"Failed to open serial port {port}: {e}")

    def close(self):
        if self.ser and self.ser.is_open:
            self.ser.close()

    async def poll_buttons(self):
        while True:
            self.hdlc.sendFrame(bytes([CMD_SEND_BUTTONS]))
            await asyncio.sleep(0.1)

            try:
                frame = self.hdlc.readFrame(timeout=0.5)

                if not isinstance(frame, (bytes, bytearray)):
                    raise TypeError(f"Expected bytearray, got {type(frame)}: {frame}")

                self._handle_button_frame(frame)

            except RuntimeError:
                print("[Timeout] No button response.")
            except ValueError:
                print("[CRC FAIL] Invalid frame received.")
            except Exception as e:
                print(f"[Unexpected Error] {e}")

    def _handle_button_frame(self, frame):
        if len(frame) < 3:
            print("[Invalid Frame] Too short:", list(frame))
            return

        x_byte = frame[1]
        y_byte = frame[2]
        active = []

        for bit, name in BUTTON_MAP_X.items():
            if x_byte & (1 << bit):
                active.append(name)

        for bit, name in BUTTON_MAP_Y.items():
            if y_byte & (1 << bit):
                active.append(name)

        if active:
            print(f"[Button Pressed] {', '.join(active)}")
        else:
            print("[No Button Pressed]")


async def main():
    poller = ButtonPoller(SERIAL_PORT, BAUDRATE)
    try:
        await poller.poll_buttons()
    except KeyboardInterrupt:
        print("Stopped by user.")
    finally:
        poller.close()


if __name__ == "__main__":
    asyncio.run(main())
