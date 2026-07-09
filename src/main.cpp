#include <RH_ASK.h>
#include <Servo.h>

// רדיו: speed=2000bps, rxPin=7, txPin=4
// (ה-TX עבר מ-D6 ל-D4 כדי לפנות את D6 ל-PWM של מנוע)
RH_ASK driver(2000, 7, 4);

// סרוו המכם על D3 (מונע על-ידי Timer1 דרך ספריית Servo)
const int RADAR_SERVO_PIN = 3;
Servo radarServo;

// סרוו הרשת על D10 (אותו Timer1 של Servo — בלי קונפליקט טיימר נוסף)
const int NET_SERVO_PIN = 10;
Servo netServo;
// מעריך את cmd.winchSpeed (טווח שרת: -255..255) לזווית סרוו 0..180;
// 0 = מרכז/ניטרלי (90°). מתאים לסרוו זוויתי וגם לסרוו רציף (90=עצירה).
const int NET_SERVO_NEUTRAL = 90;

// אולטרה-סוני: טריגר משותף ו-4 קווי Echo (המכם על A0 כדי לחסוך פין D)
// הטריגר עבר מ-D5 ל-D2 כדי לפנות את D5 ל-PWM של מנוע
const int US_TRIG_PIN = 2;
const int US_ECHO_RADAR_PIN = A0;
const int US_ECHO_FRONT_PIN = A1;
const int US_ECHO_LEFT_PIN = A2;
const int US_ECHO_RIGHT_PIN = A3;
const unsigned long US_TIMEOUT_US = 38000UL;

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

void controlMotor(int in1, int in2, int pwmPin, int speed);
void readAllUltrasonic(int results[4]);

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

  netServo.attach(NET_SERVO_PIN);
  netServo.write(NET_SERVO_NEUTRAL);

  if (!driver.init()) {
    while (1);
  }
}

void loop() {
  uint8_t buf[sizeof(CommandPacket)];
  uint8_t buflen = sizeof(CommandPacket);
  if (driver.recv(buf, &buflen) && buflen == sizeof(CommandPacket)) {
    memcpy(&cmd, buf, sizeof(CommandPacket));
    lastCommandMs = millis();
    motorsStopped = false;
    controlMotor(MOTOR1_IN1, MOTOR1_IN2, MOTOR1_PWM, cmd.leftSpeed);
    controlMotor(MOTOR2_IN1, MOTOR2_IN2, MOTOR2_PWM, cmd.rightSpeed);

    // סרוו הרשת: ממפה מהירות -255..255 לזווית 0..180
    int netAngle = map(constrain(cmd.winchSpeed, -255, 255), -255, 255, 0, 180);
    netServo.write(netAngle);

    // הזז את סרוו המכם לזווית המבוקשת ותן לו זמן להתייצב לפני הדגימה
    radarServo.write(constrain(cmd.radarAngle, 0, 180));
    delay(60);

    // בחומרת ASK פשוטה עדיף לענות אחרי קבלה במקום לשדר כל הזמן
    int usResults[4];
    readAllUltrasonic(usResults);
    telemetry.usRadar = usResults[0];
    telemetry.usFront = usResults[1];
    telemetry.usLeft  = usResults[2];
    telemetry.usRight = usResults[3];
    telemetry.radarAngle = cmd.radarAngle;
    driver.send((uint8_t*)&telemetry, sizeof(TelemetryPacket));
    driver.waitPacketSent();
  }

  // Failsafe watchdog: אם עברו יותר מ-FAILSAFE_TIMEOUT_MS בלי פקודה תקינה,
  // עצור את המנועים פעם אחת (millis() בחשבון unsigned עמיד ל-overflow).
  if (!motorsStopped && (millis() - lastCommandMs > FAILSAFE_TIMEOUT_MS)) {
    controlMotor(MOTOR1_IN1, MOTOR1_IN2, MOTOR1_PWM, 0);
    controlMotor(MOTOR2_IN1, MOTOR2_IN2, MOTOR2_PWM, 0);
    netServo.write(NET_SERVO_NEUTRAL);
    cmd.leftSpeed = 0;
    cmd.rightSpeed = 0;
    cmd.winchSpeed = 0;
    motorsStopped = true;
  }
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
