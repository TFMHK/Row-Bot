"""בדיקת מנועים דרך שרת השליטה (control_server) על גבי הרדיו (COM4).

השרת מפעיל את תיקוני החומרה (shape_motor_speed, היפוך כנף) ומחזיר טלמטריה.
הבדיקה: קדימה, עצירה, שמאל בלבד, ימין בלבד, אחורה, עצירה — עם דגימת טלמטריה
בין השלבים. בסיום מחזיר את השרת למצב MOCK כפי שהיה.
"""
import json
import time
import urllib.request

BASE = "http://127.0.0.1:8765"
PORT = "COM4"


def post(path, payload):
    data = json.dumps(payload).encode()
    req = urllib.request.Request(BASE + path, data=data,
                                 headers={"Content-Type": "application/json"})
    with urllib.request.urlopen(req, timeout=5) as r:
        return json.loads(r.read().decode())


def get(path):
    with urllib.request.urlopen(BASE + path, timeout=5) as r:
        return json.loads(r.read().decode())


def cmd(left, right, winch=0, radar=90):
    post("/api/command", {"leftSpeed": left, "rightSpeed": right,
                          "winchSpeed": winch, "radarAngle": radar})
    print(f"   -> פקודה: L={left} R={right} winch={winch}")


def watch(seconds, label):
    print(f"   מאזין לטלמטריה ({label}) {seconds}s...")
    end = time.time() + seconds
    seen = 0
    last = None
    while time.time() < end:
        st = get("/api/state")["state"]
        t = st.get("telemetry", {})
        sig = (t.get("usRadar"), t.get("usFront"), t.get("usLeft"),
               t.get("usRight"), t.get("radarAngle"))
        if sig != last:
            seen += 1
            last = sig
            print("   <- טלמטריה:", t, "| connected:", st.get("connected"),
                  "| err:", st.get("lastError") or "-")
        time.sleep(0.25)
    if not seen:
        print("   (לא התקבלה טלמטריה חדשה בחלון הזה)")
    return seen


def main():
    print("=== בדיקת מנועים חיה דרך הרדיו (COM4) ===\n")

    print("[1] כיבוי מצב MOCK")
    post("/api/mock", {"enabled": False})

    print(f"[2] התחברות ל-{PORT}")
    st = post("/api/connect", {"port": PORT, "baudRate": 115200})["state"]
    print("    connected:", st.get("connected"), "| port:", st.get("serialPort"),
          "| err:", st.get("lastError") or "-")
    if not st.get("connected"):
        print("    !! ההתחברות נכשלה — ייתכן שהפורט תפוס. בדיקה מופסקת.")
        return
    time.sleep(2)  # איפוס הארדואינו לאחר פתיחת הפורט
    watch(2, "מצב מנוחה")

    print("\n[3] שני המנועים קדימה (3s)")
    cmd(100, 100)
    watch(3, "קדימה")
    cmd(0, 0)
    watch(1, "עצירה")

    print("\n[4] מנוע שמאל בלבד (2s)")
    cmd(100, 0)
    watch(2, "שמאל")
    cmd(0, 0)
    watch(1, "עצירה")

    print("\n[5] מנוע ימין בלבד (2s)")
    cmd(0, 100)
    watch(2, "ימין")
    cmd(0, 0)
    watch(1, "עצירה")

    print("\n[6] שני המנועים אחורה (2s)")
    cmd(-100, -100)
    watch(2, "אחורה")

    print("\n[7] עצירה סופית")
    cmd(0, 0)
    watch(1, "עצירה")

    print("\n[8] ניתוק והחזרת מצב MOCK")
    post("/api/disconnect", {})
    post("/api/mock", {"enabled": True})
    print("סיום הבדיקה. המנועים נעצרו והשרת חזר למצב MOCK.")


if __name__ == "__main__":
    main()
