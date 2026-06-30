#include <RH_ASK.h>
#include <ServoTimer2.h>

// --- הגדרות פינים (מותאם ל-Arduino Mega מומלץ) ---
// רדיו: speed=2000bps, rxPin=7, txPin=6
RH_ASK driver(2000, 7, 6);

const int LED_RADIO = 5; // לד מצב קליטה

// מנועי נסיעה (L298N #1)
const int PWM_LEFT = 2; const int IN1 = 22; const int IN2 = 24;
const int PWM_RIGHT = 3; const int IN3 = 26; const int IN4 = 28;

// מנוע משיכת רשת / כננת (L298N #2)
const int PWM_WINCH = 4; const int IN5 = 30; const int IN6 = 32;

// סרוו למכ"ם
const int RADAR_SERVO_PIN = 9;
ServoTimer2 radarServo;

// חיישני אולטרה-סוני (T=Trig, E=Echo)
const int US_RADAR_T = 34; const int US_RADAR_E = 35;
const int US_FRONT_T = 36; const int US_FRONT_E = 37;
const int US_LEFT_T = 38;  const int US_LEFT_E = 39;
const int US_RIGHT_T = 40; const int US_RIGHT_E = 41;

// --- מבני נתונים (חייבים להיות זהים לממסר החוף) ---
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

CommandPacket cmd;
TelemetryPacket telemetry;

// ניהול זמנים
unsigned long previousMillis = 0;
const long TELEMETRY_INTERVAL = 100; // 10 הרץ

// הצהרות קדימה
void controlDriveMotors(int leftSpeed, int rightSpeed);
void controlWinchMotor(int winchSpeed);
int getDistance(int trigPin, int echoPin);

void setup() {
  // אתחול מנועים
  pinMode(PWM_LEFT, OUTPUT); pinMode(IN1, OUTPUT); pinMode(IN2, OUTPUT);
  pinMode(PWM_RIGHT, OUTPUT); pinMode(IN3, OUTPUT); pinMode(IN4, OUTPUT);
  pinMode(PWM_WINCH, OUTPUT); pinMode(IN5, OUTPUT); pinMode(IN6, OUTPUT);
  
  // אתחול חיישנים
  pinMode(US_RADAR_T, OUTPUT); pinMode(US_RADAR_E, INPUT);
  pinMode(US_FRONT_T, OUTPUT); pinMode(US_FRONT_E, INPUT);
  pinMode(US_LEFT_T, OUTPUT);  pinMode(US_LEFT_E, INPUT);
  pinMode(US_RIGHT_T, OUTPUT); pinMode(US_RIGHT_E, INPUT);
  
  radarServo.attach(RADAR_SERVO_PIN);
  radarServo.write(1472); // מרכז (90 מעלות)
  
  pinMode(LED_RADIO, OUTPUT);
  digitalWrite(LED_RADIO, LOW);

  // אתחול רדיו
  if (!driver.init()) {
    while (1); // שגיאת רדיו קריטית
  }
}

void loop() {
  // 1. קריאת פקודות מממסר החוף
  uint8_t buf[sizeof(CommandPacket)];
  uint8_t buflen = sizeof(CommandPacket);
  if (driver.recv(buf, &buflen)) {
    if (buflen == sizeof(CommandPacket)) {
      digitalWrite(LED_RADIO, HIGH);
      memcpy(&cmd, buf, sizeof(CommandPacket));
      controlDriveMotors(cmd.leftSpeed, cmd.rightSpeed);
      controlWinchMotor(cmd.winchSpeed);
      radarServo.write(map(cmd.radarAngle, 0, 180, 544, 2400));
      digitalWrite(LED_RADIO, LOW);
    }
  }

  // 2. קריאת חיישנים ושליחת טלמטריה בזמן קבוע
  unsigned long currentMillis = millis();
  if (currentMillis - previousMillis >= TELEMETRY_INTERVAL) {
    previousMillis = currentMillis;
    
    // דגימת החיישנים (פונקציה מוגדרת למטה)
    telemetry.usRadar = getDistance(US_RADAR_T, US_RADAR_E);
    telemetry.usFront = getDistance(US_FRONT_T, US_FRONT_E);
    telemetry.usLeft = getDistance(US_LEFT_T, US_LEFT_E);
    telemetry.usRight = getDistance(US_RIGHT_T, US_RIGHT_E);
    telemetry.radarAngle = cmd.radarAngle;
    
    // שידור הטלמטריה
    driver.send((uint8_t*)&telemetry, sizeof(TelemetryPacket));
    driver.waitPacketSent();
  }
}

// פונקציה למדידת מרחק באולטרה-סוני עם Timeout מובנה
int getDistance(int trigPin, int echoPin) {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  
  // Timeout של 20000 מיקרו-שניות = מקסימום המתנה של כ-3.4 מטרים
  // מונע מהסירה "להיתקע" אם החיישן לא מקבל הדהוד
  long duration = pulseIn(echoPin, HIGH, 20000);
  
  if (duration == 0) return 999; // 999 יסמל חוסר קליטה/רחוק מדי
  return duration * 0.034 / 2;   // מרחק בס"מ
}

// פונקציות בקרת מנועים
void controlDriveMotors(int leftSpeed, int rightSpeed) {
  // שמאל
  if (leftSpeed > 0) { digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW); analogWrite(PWM_LEFT, leftSpeed); }
  else if (leftSpeed < 0) { digitalWrite(IN1, LOW); digitalWrite(IN2, HIGH); analogWrite(PWM_LEFT, abs(leftSpeed)); }
  else { digitalWrite(IN1, LOW); digitalWrite(IN2, LOW); analogWrite(PWM_LEFT, 0); }
  // ימין
  if (rightSpeed > 0) { digitalWrite(IN3, HIGH); digitalWrite(IN4, LOW); analogWrite(PWM_RIGHT, rightSpeed); }
  else if (rightSpeed < 0) { digitalWrite(IN3, LOW); digitalWrite(IN4, HIGH); analogWrite(PWM_RIGHT, abs(rightSpeed)); }
  else { digitalWrite(IN3, LOW); digitalWrite(IN4, LOW); analogWrite(PWM_RIGHT, 0); }
}

void controlWinchMotor(int winchSpeed) {
  if (winchSpeed > 0) { digitalWrite(IN5, HIGH); digitalWrite(IN6, LOW); analogWrite(PWM_WINCH, winchSpeed); }
  else if (winchSpeed < 0) { digitalWrite(IN5, LOW); digitalWrite(IN6, HIGH); analogWrite(PWM_WINCH, abs(winchSpeed)); }
  else { digitalWrite(IN5, LOW); digitalWrite(IN6, LOW); analogWrite(PWM_WINCH, 0); }
}