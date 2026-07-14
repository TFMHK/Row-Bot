import sys, time, serial
port = sys.argv[1] if len(sys.argv) > 1 else 'COM6'
baud = int(sys.argv[2]) if len(sys.argv) > 2 else 115200
ser = serial.Serial(port, baud, timeout=1)
time.sleep(2)  # Arduino resets on open
ser.reset_input_buffer()
print(f'reading {port}@{baud} -- Ctrl+C to stop')
try:
    while True:
        line = ser.readline().decode('ascii', 'replace').strip()
        if line:
            print(time.strftime('%H:%M:%S'), line)
except KeyboardInterrupt:
    pass
finally:
    ser.close()
