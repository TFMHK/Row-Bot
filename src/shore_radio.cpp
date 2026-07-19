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
uint8_t navSeq = 0;  // מונה-מחזור נפרד לפריים קונפיג-הניווט
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
    if (inputData.length() > 0 && inputData[0] == 'N') {
      // קונפיג-ניווט מהמחשב (כיול/כוונון בלי צריבה). פורמט:
      // N,block,clear,emergency,decision,decisionHalf,sideStandoff,bowOffset,sweepSign
      int b, c, e, d, dh, ss, bo, sg;
      int np = sscanf(inputData.c_str(), "N,%d,%d,%d,%d,%d,%d,%d,%d",
                      &b, &c, &e, &d, &dh, &ss, &bo, &sg);
      if (np == 8) {
        RfNavConfig nc;
        nc.seq            = navSeq++;
        nc.frontBlock     = rf_dist_encode(b);
        nc.frontClear     = rf_dist_encode(c);
        nc.frontEmergency = rf_dist_encode(e);
        nc.decision       = rf_dist_encode(d);
        nc.decisionHalf   = (uint8_t)constrain(dh, 0, 180);
        nc.sideStandoff   = rf_dist_encode(ss);
        nc.bowOffset      = (uint8_t)constrain(bo, 0, 180);
        nc.flags          = (sg >= 0) ? 0x01 : 0x00;
        rf_navcfg_sign(&nc);
        driver.send((uint8_t*)&nc, sizeof(RfNavConfig));
        driver.waitPacketSent();
      }
    } else {
    int leftSpeed, rightSpeed, winchSpeed, radarAngle, mode;
    int parsed = sscanf(inputData.c_str(), "%d,%d,%d,%d,%d",
                        &leftSpeed, &rightSpeed, &winchSpeed, &radarAngle, &mode);
    if (parsed == 5) {
      RfCommand out;
      out.seq   = cmdSeq++;
      // מיפוי ישר: ערוץ שמאל לוגי -> מנוע שמאל, ימין -> ימין. ההחלפה הקודמת
      // הצליבה את הערוצים (וגם את הגבלות המהירות שהשרת מחיל לכל ערוץ בנפרד),
      // כך שמחוון "מנוע ימין" הגביל בפועל את המנוע השמאלי. עכשיו שני הקצוות
      // ישרים ועקביים עם control_server.py.
      out.left  = rf_speed_encode(leftSpeed);
      out.right = rf_speed_encode(rightSpeed);
      out.winch = rf_speed_encode(winchSpeed);
      out.flags = (mode ? 0x01 : 0x00);
      // radarAngle לא נשלח: הסירה סורקת מקומית ומתעלמת ממנו ממילא.
      rf_cmd_sign(&out);
      driver.send((uint8_t*)&out, sizeof(RfCommand));
      driver.waitPacketSent();
    }
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
