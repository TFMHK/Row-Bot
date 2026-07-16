import serial, time

PORT = 'COM4'  # arduino החוף - משדר לסירה ברדיו
SPEED = 160    # עוצמת מנוע קדימה (0..255)
DURATION = 5   # שניות


def send(s, left, right, winch=0, radar=90, mode=0):
    # הקושחה של החוף (shore_radio.cpp) דורשת 5 שדות: left,right,winch,radar,mode
    # אחרת sscanf!=5 והפקודה נזרקת בשקט (המנועים לא זזים).
    cmd = f"{left},{right},{winch},{radar},{mode}\n"
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

    print(f"=== מנוע ימין קדימה (right={SPEED}) למשך {DURATION} שניות ===")
    end = time.time() + DURATION
    # שולחים שוב ושוב כדי לשמור על השליטה (failsafe בסירה ~1.5s)
    while time.time() < end:
        send(s, 0, SPEED)          # left=0, right=SPEED -> רק מנוע ימין קדימה
        read_telemetry(s, 0.4, "ריצה")

    print("=== עצירה ===")
    send(s, 0, 0)
    read_telemetry(s, 1, "עצירה")

    print("סיום. המנוע הוחזר לעצירה.")
