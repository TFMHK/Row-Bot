#include <RH_ASK.h>
#include <Servo.h>
#include <Adafruit_NeoPixel.h>
#include "rf_link.h"  // פריימים דחוסים + אימות MAC (משותף עם shore_radio.cpp)

// רדיו: speed=2000bps, rxPin=7, txPin=4 (שני הקצות חייבים להתאים!)
// קצב נמוך => מקלט ASK רגיש יותר => טווח טוב יותר. מחיר: זמן-שידור
// לפאקטה מוכפל (~150ms) – לכן ה-heartbeat בשרת הוגדל ל-400ms כדי שה-round-trip ייכנס.
// (ה-TX עבר מ-D6 ל-D4 כדי לפנות את D6 ל-PWM של מנוע)
RH_ASK driver(2000, 7, 4);

// טבעת ניאו-פיקסל (8 לדים) על D11 לחיווי מצב + שליטה מרחוק בצבע.
// הועבר מ-D1 ל-D11: D1 הוא קו ה-TX של ה-USART ומוחזק HIGH במנוחה, ולכן אות ה-
// DATA של ה-NeoPixel (שדורש מנוחה LOW ותזמון מדויק) משתבש ואף לד לא נדלק.
// D11 הוא פין דיגיטלי פנוי ונקי בסירה.
const int PIXEL_PIN = 11;
const int PIXEL_COUNT = 8;
Adafruit_NeoPixel pixel(PIXEL_COUNT, PIXEL_PIN, NEO_GRB + NEO_KHZ800);

// סרוו המכם על D3 (מונע על-ידי Timer1 דרך ספריית Servo)
const int RADAR_SERVO_PIN = 3;
Servo radarServo;

// --- סריקת מכם מקומית ואוטונומית + החלקת תנועה ---
// הסרוו סורק בכוחות עצמו (0..90; 4 החיישנים ב-90° מכסים 360°) לפי טיימר מקומי,
// בלי תלות בפקודות מהחוף. כך אין תלות כפולה בקו התקשורת: הסריקה לא צריכה את
// קו הפקודות (חוף→סירה), והסירה משדרת זווית+מרחקים בכל צעד. התנועה מוחלקת
// בצעדים קטנים (slew) כדי שתהיה רציפה ולא קופצנית. הסרוו נשאר מחובר תמיד — לא
// מנתקים אותו (detach) כדי שלא "יפול" בכבידה ואז יקפוץ בחדות בהתחברות מחדש.
float radarServoPos = 90.0f;               // מיקום נוכחי מוחלק
int   radarServoTarget = 90;               // זווית יעד/סריקה נוכחית
const float RADAR_SLEW_STEP_DEG = 3.0f;    // גודל צעד החלקה בכל פעימה
const int   RADAR_SLEW_STEP_DELAY_MS = 6;  // השהיה בין צעדי החלקה
// פרמטרי הסריקה האוטונומית
const int RADAR_SWEEP_MIN = 0;             // גבול תחתון לזווית הסריקה
const int RADAR_SWEEP_MAX = 90;            // גבול עליון (מספיק ל-360° עם 4 חיישנים)
const int RADAR_SWEEP_STEP = 15;           // צעד סריקה בכל פעימה
const unsigned long RADAR_SWEEP_INTERVAL_MS = 90; // קצב פעימת סריקה
int radarSweepDir = 1;                      // כיוון הסריקה הנוכחי (+1/-1)
unsigned long lastRadarSweepMs = 0;         // חותם-זמן הפעימה האחרונה
unsigned long lastTelemetrySentMs = 0;      // חותם-זמן שידור טלמטריה אחרון

// סרוו הרשת על D10 (אותו Timer1 של Servo — בלי קונפליקט טיימר נוסף)
const int NET_SERVO_PIN = 10;
Servo netServo;
// מעריך את cmd.winchSpeed (טווח שרת: -255..255) לזווית סרוו 0..180;
// 0 = מרכז/ניטרלי (90°). מתאים לסרוו זוויתי וגם לסרוו רציף (90=עצירה).
const int NET_SERVO_NEUTRAL = 90;

// אולטרה-סוני: טריגר משותף ו-4 קווי Echo (המכם על A1 כדי לחסוך פין D)
// הטריגר עבר מ-D5 ל-D2 כדי לפנות את D5 ל-PWM של מנוע
const int US_TRIG_PIN = 2;
const int US_ECHO_RADAR_PIN = A5;
const int US_ECHO_FRONT_PIN = A1;
const int US_ECHO_LEFT_PIN = A2;
const int US_ECHO_RIGHT_PIN = A4;
// חלון-האזנה ל-echo. 24ms מכסה ~400ס"מ (התקרה בקוד ממילא 400ס"מ; מעבר לכך → 999),
// וקוצר מ-38ms כדי להאיץ את הלולאה כשקרן רחוקה לא מחזירה החזר.
const unsigned long US_TIMEOUT_US = 24000UL;

// שני מנועים דרך H-Bridge
// מנוע שמאל: כיוון D8/D9, מהירות PWM D5 (Timer0)
const int MOTOR1_IN1 = 8;
const int MOTOR1_IN2 = 9;
const int MOTOR1_PWM = 5;
// מנוע ימין: כיוון D12/D13, מהירות PWM D6 (Timer0)
const int MOTOR2_IN1 = 12;
const int MOTOR2_IN2 = 13;
const int MOTOR2_PWM = 6;

struct CommandPacket {
  int leftSpeed;
  int rightSpeed;
  int winchSpeed;
  int radarAngle;
  int mode;           // 0 = ידני (ירוק), 1 = אוטומטי (כחול) — צבע הטבעת
};

struct TelemetryPacket {
  int usRadar;
  int usFront;
  int usLeft;
  int usRight;
  int radarAngle;
};

CommandPacket cmd = {0, 0, 0, 90, 0};
TelemetryPacket telemetry;

// מוני-מחזור (anti-replay) של פרוטוקול ה-RF:
// lastCmdSeq = ה-seq האחרון שהתקבל מהחוף; telSeq = ה-seq היוצא של הטלמטריה.
// haveCmdSeq=false => הפריים החתום הראשון אחרי אתחול מתקבל בלי בדיקת-רעננות
// כדי להסתנכרן למונה של החוף, ואז חוסמים replay של פריימים ישנים.
uint8_t lastCmdSeq = 0;
bool haveCmdSeq = false;
uint8_t telSeq = 0;

// Failsafe: אם לא מתקבלת פקודה תקינה תוך פרק הזמן הזה (למשל אובדן קשר RF),
// המנועים נעצרים אוטומטית כדי שהסירה לא תמשיך לנוע "בעיוורון" עד לריסט.
// הועלה מ-800 ל-1500ms כדי "לרכוב מעל" דעיכות RF קצרות (טווח) בלי לעצור מנועים.
const unsigned long FAILSAFE_TIMEOUT_MS = 1500;
unsigned long lastCommandMs = 0;
bool motorsStopped = true;
// כשאבד קשר (failsafe) הטבעת מוצגת אדומה קבוע כחיווי בטיחות.
bool linkLost = false;
// המצב האחרון שהוצג על הטבעת (0=ידני,1=אוטומטי). -1 = טרם הוצג,
// מאלץ ציור ראשון. show() מתבצע רק כשהערך הזה משתנה — אירוע נדיר.
int lastMode = -1;

// --- חיווי כיוון על טבעת הלדים (כמו מצפן) ---
// הטבעת מצביעה לכיוון התנועה הנגזר מפקודות המנועים (כיוון הג'ויסטיק):
// קדימה → לד קדמי, פנייה ימינה/שמאלה → צד, אחורה → לד אחורי.
// כשהסירה עומדת (אין פקודה) — הטבעת כבויה.
// LED 0 = חזית הסירה. RING_OFFSET מסובב את המיפוי אם הטבעת מורכבת מסובבת
// פיזית (כל יחידה = 45°). RING_CW=+1 אם הלדים עולים עם כיוון השעון, -1 אם נגדו.
const int RING_OFFSET = 0;
const int RING_CW = 1;
// מתחת לסף התנועה הזה נחשבת עמידה — הטבעת כבויה.
const int MOTION_DEADZONE = 20;

// שידור-גיבוי עצמאי של טלמטריה כשאין פקודות נכנסות (uplink שותק). כשעברו יותר
// מ-UPLINK_SILENCE_MS בלי פקודה, הסירה משדרת בעצמה כל TELEMETRY_FALLBACK_MS
// כדי שהמכם ימשיך לשדר זווית+מרחק גם ללא תקשורת נכנסת.
// הועלה מ-300 ל-550ms: ה-heartbeat בשרת הוא כעת 400ms, כך שבמהלך קשר תקין
// הסף לא נחצה והשידור-גיבוי לא מתנגש עם הפקודה הבאה — הוא נכנס רק בניתוק אמיתי.
const unsigned long UPLINK_SILENCE_MS = 550;
// קריטי ב-2000bps: פריים טלמטריה = ~150ms אוויר. אם משדרים גיבוי כל 150ms
// הסירה משדרת ברציפות ומקלטה חירש (חצי-דופלקס) => לעולם לא תופסת פקודה נכנסת
// => דדלוק. הועלה ל-500ms: הסירה משדרת ~150ms ואז מאזינה ~350ms ותופסת את
// ה-heartbeat (400ms) של החוף, נכנסת ל-"פינג-פונג" והגיבוי מפסיק מעצמו.
const unsigned long TELEMETRY_FALLBACK_MS = 500;

void controlMotor(int in1, int in2, int pwmPin, int speed);
void readAllUltrasonic(int results[4]);
void showPixel(uint8_t r, uint8_t g, uint8_t b);
void showModeColor(int mode);
void slewRadarServoTo(int target);
void sendTelemetry();

void setup() {
  pinMode(MOTOR1_IN1, OUTPUT);
  pinMode(MOTOR1_IN2, OUTPUT);
  pinMode(MOTOR1_PWM, OUTPUT);
  pinMode(MOTOR2_IN1, OUTPUT);
  pinMode(MOTOR2_IN2, OUTPUT);
  pinMode(MOTOR2_PWM, OUTPUT);

  pinMode(US_TRIG_PIN, OUTPUT);
  digitalWrite(US_TRIG_PIN, LOW);
  pinMode(US_ECHO_RADAR_PIN, INPUT);
  pinMode(US_ECHO_FRONT_PIN, INPUT);
  pinMode(US_ECHO_LEFT_PIN, INPUT);
  pinMode(US_ECHO_RIGHT_PIN, INPUT);

  radarServo.attach(RADAR_SERVO_PIN);
  radarServo.write(cmd.radarAngle);
  radarServoPos = cmd.radarAngle;
  radarServoTarget = cmd.radarAngle;

  netServo.attach(NET_SERVO_PIN);
  netServo.write(NET_SERVO_NEUTRAL);

  pixel.begin();
  pixel.setBrightness(40);
  showPixel(0, 255, 0); // ירוק = בדיקת תקינות הניאו-פיקסל בהדלקה; נשאר עד תחילת פעילות

  if (!driver.init()) {
    showPixel(60, 0, 0); // אדום קבוע = כשל אתחול רדיו
    while (1);
  }
}

void loop() {
  unsigned long now = millis();

  // 1) קליטת פקודות (לא חוסמת): שולטת רק במנועים וברשת. זווית המכם כבר לא מגיעה
  //    מהפקודה — הסריקה מקומית לגמרי. מיד עם קבלת פקודה משדרים טלמטריה ("פינג-
  //    פונג"): החוף בדיוק סיים לשדר ולכן נמצא במצב האזנה, כך שהשידור נוחת אמין.
  //    כל פריים חייב לעבור אימות MAC (מפתח סודי) + בדיקת seq — פריים מגורם זר
  //    או פריים ישן ש"מנוגן מחדש" נדחה בשקט ולא משפיע על המנועים.
  uint8_t buf[RH_ASK_MAX_MESSAGE_LEN];
  uint8_t buflen = sizeof(buf);
  if (driver.recv(buf, &buflen) && buflen == sizeof(RfCommand)) {
    RfCommand in;
    memcpy(&in, buf, sizeof(RfCommand));
    if (rf_cmd_verify(&in) && (!haveCmdSeq || rf_seq_fresh(in.seq, lastCmdSeq))) {
      haveCmdSeq = true;
      lastCmdSeq = in.seq;
      cmd.leftSpeed  = rf_speed_decode(in.left);
      cmd.rightSpeed = rf_speed_decode(in.right);
      cmd.winchSpeed = rf_speed_decode(in.winch);
      cmd.mode       = (in.flags & 0x01) ? 1 : 0;
      lastCommandMs = now;
      motorsStopped = false;
      linkLost = false;

      sendTelemetry();
      lastTelemetrySentMs = now;

      controlMotor(MOTOR1_IN1, MOTOR1_IN2, MOTOR1_PWM, cmd.leftSpeed);
      controlMotor(MOTOR2_IN1, MOTOR2_IN2, MOTOR2_PWM, cmd.rightSpeed);

      // סרוו הרשת: ממפה מהירות -255..255 לזווית 0..180
      int netAngle = map(constrain(cmd.winchSpeed, -255, 255), -255, 255, 0, 180);
      netServo.write(netAngle);

      // חיווי מצב על הטבעת: צבע לפי המצב (ידני/אוטומטי) שנשלח מהחוף.
      // קוראים ל-pixel.show() (שמכבה פסיקות ~240µs) רק כשהמצב משתנה בפועל
      // — אירוע נדיר — כך שהוא לעולם לא נופל בזמן קליטת פקודות ולא תוקע את המנועים.
      if (cmd.mode != lastMode) {
        lastMode = cmd.mode;
        showModeColor(cmd.mode);
      }
    }
  }

  // 2) סריקת מכם אוטונומית מקומית — רצה תמיד לפי טיימר, גם ללא פקודות כלל.
  //    מקדמת את זווית הסריקה, מזיזה את הסרוו בתנועה רציפה, דוגמת את החיישנים
  //    ושומרת לתוך telemetry את הזווית והמרחקים שנקלטו.
  if (now - lastRadarSweepMs >= RADAR_SWEEP_INTERVAL_MS) {
    lastRadarSweepMs = now;

    // קדם את זווית הסריקה בהלוך-חזור בין הגבולות
    int nextAngle = radarServoTarget + RADAR_SWEEP_STEP * radarSweepDir;
    if (nextAngle >= RADAR_SWEEP_MAX) {
      nextAngle = RADAR_SWEEP_MAX;
      radarSweepDir = -1;
    } else if (nextAngle <= RADAR_SWEEP_MIN) {
      nextAngle = RADAR_SWEEP_MIN;
      radarSweepDir = 1;
    }

    // הזז את הסרוו בתנועה רציפה (החלקה) — משמש גם כזמן ההתייצבות לפני הדגימה
    slewRadarServoTo(nextAngle);

    // קרא את החיישנים בזווית הנוכחית ואחסן לשידור
    int usResults[4];
    readAllUltrasonic(usResults);
    telemetry.usRadar = usResults[0];
    telemetry.usFront = usResults[1];
    telemetry.usLeft  = usResults[2];
    telemetry.usRight = usResults[3];
    telemetry.radarAngle = nextAngle;

    // הערה: אין כאן רענון של הטבעת. pixel.show() מכבה פסיקות ~240µs (8 לדים)
    // ואם ירוץ בתזמון אקראי (כל ~90ms) הוא משבש את דגימת ה-RF (טיימר) ומשמיד
    // פאקטות נכנסות → failsafe עוצר מנועים. לכן מעדכנים את הטבעת רק בחלון הבטוח:
    // מיד אחרי קבלת פקודה ושידור טלמטריה (בבלוק 1), כשהחוף מאזין ולא משדר אלינו.
  }

  // 3) שידור-גיבוי עצמאי: כשקו הפקודות (חוף→סירה) שותק אין "פינג-פונג", ולכן
  //    הסירה משדרת את הזווית+המרחקים בכוחות עצמה. כך המכם ממשיך לשדר מה שהוא
  //    קולט גם ללא תקשורת נכנסת — בלי תלות כפולה בקו התקשורת.
  if (now - lastCommandMs > UPLINK_SILENCE_MS &&
      now - lastTelemetrySentMs >= TELEMETRY_FALLBACK_MS) {
    sendTelemetry();
    lastTelemetrySentMs = now;
  }

  // 4) Failsafe watchdog: אם עברו יותר מ-FAILSAFE_TIMEOUT_MS בלי פקודה תקינה,
  //    עצור את המנועים פעם אחת (millis() בחשבון unsigned עמיד ל-overflow).
  //    שים לב: המכם ממשיך לסרוק ולשדר — רק המנועים נעצרים.
  if (!motorsStopped && (now - lastCommandMs > FAILSAFE_TIMEOUT_MS)) {
    controlMotor(MOTOR1_IN1, MOTOR1_IN2, MOTOR1_PWM, 0);
    controlMotor(MOTOR2_IN1, MOTOR2_IN2, MOTOR2_PWM, 0);
    netServo.write(NET_SERVO_NEUTRAL);
    linkLost = true;
    showPixel(60, 0, 0); // אדום = אבד קשר (failsafe)
    lastMode = -1;       // בהתאוששות הקשר — לצייר מחדש את צבע המצב
    cmd.leftSpeed = 0;
    cmd.rightSpeed = 0;
    cmd.winchSpeed = 0;
    motorsStopped = true;
  }
}

// מסיע את סרוו המכם אל הזווית המבוקשת בצעדים קטנים ורציפים מתוך המיקום הנוכחי,
// כך שהתנועה חלקה ולא קופצת. הסרוו כבר מחובר (מ-setup) ונשאר מחובר.
void slewRadarServoTo(int target) {
  target = constrain(target, 0, 180);
  radarServoTarget = target;
  while ((int)(radarServoPos + 0.5f) != target) {
    float diff = target - radarServoPos;
    if (diff >  RADAR_SLEW_STEP_DEG) diff =  RADAR_SLEW_STEP_DEG;
    if (diff < -RADAR_SLEW_STEP_DEG) diff = -RADAR_SLEW_STEP_DEG;
    radarServoPos += diff;
    radarServo.write((int)(radarServoPos + 0.5f));
    delay(RADAR_SLEW_STEP_DELAY_MS);
  }
  radarServoPos = target;
  radarServo.write(target);
}

// קורא את כל 4 חיישני האולטרה-סוני במקביל: טריגר אחד, polling על כל ה-echo pins.
// results[0]=radar, [1]=front, [2]=left, [3]=right  (999 = אין מדידה)
void readAllUltrasonic(int results[4]) {
  const int echoPins[4] = {
    US_ECHO_RADAR_PIN, US_ECHO_FRONT_PIN, US_ECHO_LEFT_PIN, US_ECHO_RIGHT_PIN
  };

  // טריגר משותף
  digitalWrite(US_TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(US_TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(US_TRIG_PIN, LOW);

  unsigned long riseTime[4] = {0, 0, 0, 0};
  unsigned long pulseWidth[4] = {0, 0, 0, 0};
  bool done[4] = {false, false, false, false};

  unsigned long start = micros();
  while (micros() - start < US_TIMEOUT_US) {
    unsigned long now = micros();
    bool allDone = true;
    for (byte i = 0; i < 4; i++) {
      if (done[i]) continue;
      if (digitalRead(echoPins[i])) {
        if (riseTime[i] == 0) riseTime[i] = now;
      } else {
        if (riseTime[i] != 0) {
          pulseWidth[i] = now - riseTime[i];
          done[i] = true;
        }
      }
      if (!done[i]) allDone = false;
    }
    // יציאה מוקדמת: אם כל 4 החיישנים כבר החזירו הד, אין טעם להמתין עד ה-timeout.
    // כך משחררים את הלולאה מהר וחוזרים לקלוט RF במקום להיחסם 24ms קבועים.
    if (allDone) break;
  }

  for (byte i = 0; i < 4; i++) {
    if (pulseWidth[i] == 0) {
      results[i] = 999;
    } else {
      long cm = (long)pulseWidth[i] / 58;
      results[i] = (cm > 0 && cm <= 400) ? (int)cm : 999;
    }
  }
}

void controlMotor(int in1, int in2, int pwmPin, int speed) {
  int pwm = abs(constrain(speed, -255, 255));

  if (speed > 0) {
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
    analogWrite(pwmPin, pwm);
  } else if (speed < 0) {
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
    analogWrite(pwmPin, pwm);
  } else {
    digitalWrite(in1, LOW);
    digitalWrite(in2, LOW);
    analogWrite(pwmPin, 0);
  }
}

// חיווי על כל לדי הטבעת באותו צבע (RGB 0..255, מוגבל על-ידי setBrightness).
void showPixel(uint8_t r, uint8_t g, uint8_t b) {
  for (int i = 0; i < PIXEL_COUNT; i++) {
    pixel.setPixelColor(i, pixel.Color(r, g, b));
  }
  pixel.show();
}

// חיווי מצב על הטבעת: צבע אחיד לכל הלדים לפי המצב שנשלח מהחוף.
// mode==1 (אוטומטי) → כחול; אחרת (ידני) → ירוק. נקרא רק בעת שינוי מצב.
void showModeColor(int mode) {
  if (mode == 1) {
    showPixel(0, 0, 255);   // אוטומטי = כחול
  } else {
    showPixel(0, 255, 0);   // ידני = ירוק
  }
}

// דוחס את הטלמטריה הנוכחית לפריים RfTelemetry, חותם ב-MAC ומשדר. מרחקים
// נדחסים לבית בודד כל אחד (ס"מ/2, 255=אין הד), הזווית בבית, ומצורף seq עולה
// למניעת replay + 2 בתי MAC. החוף מאמת את החתימה לפני שהוא מעביר למחשב.
void sendTelemetry() {
  RfTelemetry out;
  out.seq    = telSeq++;
  out.dRadar = rf_dist_encode(telemetry.usRadar);
  out.dFront = rf_dist_encode(telemetry.usFront);
  out.dLeft  = rf_dist_encode(telemetry.usLeft);
  out.dRight = rf_dist_encode(telemetry.usRight);
  out.angle  = (uint8_t)constrain(telemetry.radarAngle, 0, 180);
  rf_tel_sign(&out);
  driver.send((uint8_t*)&out, sizeof(RfTelemetry));
  driver.waitPacketSent();
}
