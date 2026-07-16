#include <RH_ASK.h>
#include "rf_link.h"  // פריימים דחוסים + אימות MAC (משותף עם main.cpp)

// רדיו: speed=2000bps, rxPin=11, txPin=10 (חייב להתאים לקצב של הסירה)
// קצב נמוך => מקלט ASK רגיש יותר => טווח טוב יותר. ה-heartbeat בשרת הוגדל
RH_ASK driver(2000, 11, 10);

// החוף הוא ממסר שקוף: הקישור מחשב<->חוף נשאר CSV רגיל על ה-USB (מהימן, קווי),
// והדחיסה+האימות מתבצעים אך ורק בקפיצת ה-RF (חוף<->סירה). לכן קוד המחשב
// (control_server.py) אינו משתנה כלל.

// מוני-מחזור (anti-replay): cmdSeq = ה-seq היוצא של הפקודות; lastTelSeq = ה-seq
// האחרון שהתקבל בטלמטריה. haveTelSeq=false => פריים טלמטריה חתום ראשון מתקבל
// בלי בדיקת-רעננות כדי להסתנכרן למונה של הסירה, ואז חוסמים replay.
uint8_t cmdSeq = 0;
uint8_t lastTelSeq = 0;
bool haveTelSeq = false;

void setup() {
  Serial.begin(115200);

  if (!driver.init()) {
    Serial.println("ERROR: RF driver init failed!");
    while (1);
  }
}

void loop() {
  // 1. קריאת פקודות מהמחשב (CSV על ה-Serial), דחיסה+חתימה ושידור לסירה.
  if (Serial.available() > 0) {
    String inputData = Serial.readStringUntil('\n');
    int leftSpeed, rightSpeed, winchSpeed, radarAngle, mode;
    int parsed = sscanf(inputData.c_str(), "%d,%d,%d,%d,%d",
                        &leftSpeed, &rightSpeed, &winchSpeed, &radarAngle, &mode);
    if (parsed == 5) {
      RfCommand out;
      out.seq   = cmdSeq++;
      // המנועים מחוברים פיזית הפוך בסירה: ערוץ "ימין" הלוגי הניע את המנוע
      // השמאלי (אומת בבדיקת סריאל ישירה). מתקנים כאן, בממסר החוף, כך שהתיקון
      // חל גם על השרת וגם על בדיקות ישירות — בלי צורך לצרוב מחדש את הסירה.
      out.left  = rf_speed_encode(rightSpeed);
      out.right = rf_speed_encode(leftSpeed);
      out.winch = rf_speed_encode(winchSpeed);
      out.flags = (mode ? 0x01 : 0x00);
      // radarAngle לא נשלח: הסירה סורקת מקומית ומתעלמת ממנו ממילא.
      rf_cmd_sign(&out);
      driver.send((uint8_t*)&out, sizeof(RfCommand));
      driver.waitPacketSent();
    }
  }

  // 2. קבלת טלמטריה מהסירה: אימות MAC + seq, פענוח והעברה למחשב כ-CSV.
  //    פריים מגורם זר או פריים ישן ("replay") נדחה בשקט ולא מגיע למחשב.
  uint8_t buf[RH_ASK_MAX_MESSAGE_LEN];
  uint8_t buflen = sizeof(buf);
  if (driver.recv(buf, &buflen) && buflen == sizeof(RfTelemetry)) {
    RfTelemetry in;
    memcpy(&in, buf, sizeof(RfTelemetry));
    if (rf_tel_verify(&in) && (!haveTelSeq || rf_seq_fresh(in.seq, lastTelSeq))) {
      haveTelSeq = true;
      lastTelSeq = in.seq;
      Serial.print(rf_dist_decode(in.dRadar));  Serial.print(",");
      Serial.print(rf_dist_decode(in.dFront));  Serial.print(",");
      Serial.print(rf_dist_decode(in.dLeft));   Serial.print(",");
      Serial.print(rf_dist_decode(in.dRight));  Serial.print(",");
      Serial.println(in.angle);
    }
  }
}
