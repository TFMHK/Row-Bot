import serial, time

s = serial.Serial('COM4', 115200, timeout=2)
time.sleep(2)  # המתן לאיפוס הארדואינו

print('שולח פקודה: 0,0,0,90')
s.write(b'0,0,0,90\n')

print('מחכה לתגובה (3 שניות)...')
end = time.time() + 3
while time.time() < end:
    line = s.readline()
    if line:
        print('התקבל:', line.decode(errors='replace').strip())

s.close()
print('סיום.')
