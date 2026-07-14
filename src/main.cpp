#include <RH_ASK.h>
#include <Servo.h>
#include <Adafruit_NeoPixel.h>

// רדיו: speed=4000bps, rxPin=7, txPin=4 (הועלה מ-2000 להאצת הלינק; שני הקצוות תואמים)
// (ה-TX עבר מ-D6 ל-D4 כדי לפנות את D6 ל-PWM של מנוע)
RH_ASK driver(4000, 7, 4);

// לד ניאו-פיקסל יחיד על D1 לחיווי מצב (קשר/תנועה/failsafe).
// D1 פנוי בסירה כי אין שימוש ב-Serial (התקשורת היא RF בלבד).
const int PIXEL_PIN = 1;
const int PIXEL_COUNT = 1;
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

// אולטרה-סוני: טריגר משותף ו-4 קווי Echo (המכם על A0 כדי לחסוך פין D)
// הטריגר עבר מ-D5 ל-D2 כדי לפנות את D5 ל-PWM של מנוע
const int US_TRIG_PIN = 2;
const int US_ECHO_RADAR_PIN = A5;
const int US_ECHO_FRONT_PIN = A0;
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
};

struct TelemetryPacket {
  int usRadar;
  int usFront;
  int usLeft;
  int usRight;
  int radarAngle;
};

CommandPacket cmd = {0, 0, 0, 90};
TelemetryPacket telemetry;

// Failsafe: אם לא מתקבלת פקודה תקינה תוך פרק הזמן הזה (למשל אובדן קשר RF),
// המנועים נעצרים אוטומטית כדי שהסירה לא תמשיך לנוע "בעיוורון" עד לריסט.
const unsigned long FAILSAFE_TIMEOUT_MS = 800;
unsigned long lastCommandMs = 0;
bool motorsStopped = true;

// שידור-גיבוי עצמאי של טלמטריה כשאין פקודות נכנסות (uplink שותק). כשעברו יותר
// מ-UPLINK_SILENCE_MS בלי פקודה, הסירה משדרת בעצמה כל TELEMETRY_FALLBACK_MS
// כדי שהמכם ימשיך לשדר זווית+מרחק גם ללא תקשורת נכנסת.
const unsigned long UPLINK_SILENCE_MS = 300;
const unsigned long TELEMETRY_FALLBACK_MS = 150;

void controlMotor(int in1, int in2, int pwmPin, int speed);
void readAllUltrasonic(int results[4]);
void showPixel(uint8_t r, uint8_t g, uint8_t b);
void showProximity(int nearestCm);
void slewRadarServoTo(int target);

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
  uint8_t buf[sizeof(CommandPacket)];
  uint8_t buflen = sizeof(CommandPacket);
  if (driver.recv(buf, &buflen) && buflen == sizeof(CommandPacket)) {
    memcpy(&cmd, buf, sizeof(CommandPacket));
    lastCommandMs = now;
    motorsStopped = false;

    driver.send((uint8_t*)&telemetry, sizeof(TelemetryPacket));
    driver.waitPacketSent();
    lastTelemetrySentMs = now;

    controlMotor(MOTOR1_IN1, MOTOR1_IN2, MOTOR1_PWM, cmd.leftSpeed);
    controlMotor(MOTOR2_IN1, MOTOR2_IN2, MOTOR2_PWM, cmd.rightSpeed);

    // סרוו הרשת: ממפה מהירות -255..255 לזווית 0..180
    int netAngle = map(constrain(cmd.winchSpeed, -255, 255), -255, 255, 0, 180);
    netServo.write(netAngle);
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

    // חיווי לד קרבה: ירוק כשאין גוף קרוב מ-35ס"מ, ומאדים ככל שהגוף קרוב יותר.
    int nearest = 999;
    for (byte i = 0; i < 4; i++) {
      if (usResults[i] < nearest) nearest = usResults[i];
    }
    showProximity(nearest);
  }

  // 3) שידור-גיבוי עצמאי: כשקו הפקודות (חוף→סירה) שותק אין "פינג-פונג", ולכן
  //    הסירה משדרת את הזווית+המרחקים בכוחות עצמה. כך המכם ממשיך לשדר מה שהוא
  //    קולט גם ללא תקשורת נכנסת — בלי תלות כפולה בקו התקשורת.
  if (now - lastCommandMs > UPLINK_SILENCE_MS &&
      now - lastTelemetrySentMs >= TELEMETRY_FALLBACK_MS) {
    driver.send((uint8_t*)&telemetry, sizeof(TelemetryPacket));
    driver.waitPacketSent();
    lastTelemetrySentMs = now;
  }

  // 4) Failsafe watchdog: אם עברו יותר מ-FAILSAFE_TIMEOUT_MS בלי פקודה תקינה,
  //    עצור את המנועים פעם אחת (millis() בחשבון unsigned עמיד ל-overflow).
  //    שים לב: המכם ממשיך לסרוק ולשדר — רק המנועים נעצרים.
  if (!motorsStopped && (now - lastCommandMs > FAILSAFE_TIMEOUT_MS)) {
    controlMotor(MOTOR1_IN1, MOTOR1_IN2, MOTOR1_PWM, 0);
    controlMotor(MOTOR2_IN1, MOTOR2_IN2, MOTOR2_PWM, 0);
    netServo.write(NET_SERVO_NEUTRAL);
    showPixel(60, 0, 0); // אדום = אבד קשר (failsafe)
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
    }
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

// חיווי לד ניאו-פיקסל יחיד (RGB 0..255, מוגבל ע"י setBrightness).
void showPixel(uint8_t r, uint8_t g, uint8_t b) {
  pixel.setPixelColor(0, pixel.Color(r, g, b));
  pixel.show();
}

// חיווי קרבה: ירוק כשהגוף הקרוב ביותר >=35ס"מ, מעבר לינארי לאדום ככל שמתקרב.
void showProximity(int nearestCm) {
  const int NEAR_CM = 35;  // מעל זה = ירוק מלא (אין גוף קרוב)
  const int CLOSE_CM = 5;  // מתחת זה = אדום מלא
  int d = constrain(nearestCm, CLOSE_CM, NEAR_CM);
  int green = (long)(d - CLOSE_CM) * 60 / (NEAR_CM - CLOSE_CM);
  int red = 60 - green;
  showPixel((uint8_t)red, (uint8_t)green, 0);
}
