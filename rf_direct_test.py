import serial
import time

PORT = "COM4"
BAUD = 115200
ANGLES = [10, 45, 80, 135]


def read_window(ser: serial.Serial, seconds: float):
    end = time.time() + seconds
    lines = []
    while time.time() < end:
        line = ser.readline().decode(errors="replace").strip()
        if line:
            lines.append(line)
    return lines


with serial.Serial(PORT, BAUD, timeout=0.25) as ser:
    time.sleep(2)
    print("opened", PORT)

    for angle in ANGLES:
        cmd = f"0,0,0,{angle}\n".encode("ascii")
        ser.write(cmd)
        print("sent", angle)
        lines = read_window(ser, 1.2)
        if lines:
            print("recv", lines[-5:])
        else:
            print("recv", [])

print("done")
