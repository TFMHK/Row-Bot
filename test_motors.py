import serial, time

PORT = 'COM7'  # arduino החוף - משדר לסירה ברדיו


def send(s, left, right, winch=0, radar=90):
    cmd = f"{left},{right},{winch},{radar}\n"
    s.write(cmd.encode())
    print(f"-> נשלח: {cmd.strip()}")


def read_telemetry(s, seconds, label):
    print(f"   מאזין לטלמטריה ({label}) למשך {seconds}s...")
    end = time.time() + seconds
    got = False
    while time.time() < end:
        line = s.readline()
        if line:
            got = True
            print("   <- טלמטריה:", line.decode(errors='replace').strip())
    if not got:
        print("   (לא התקבלה טלמטריה בחלון הזה)")
    return got


with serial.Serial(PORT, 115200, timeout=0.3) as s:
    time.sleep(2)  # המתנה לאיפוס הארדואינו לאחר פתיחת הפורט

    print("=== שני המנועים קדימה במהירות שווה ל-3 שניות ===")
    send(s, 160, 160)
    time.sleep(3)

    print("=== עצירה ===")
    send(s, 0, 0)
    read_telemetry(s, 1, "עצירה")

    print("סיום. המנועים הוחזרו לעצירה.")
