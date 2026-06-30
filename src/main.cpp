#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
#include <Servo.h>

// --- הגדרות פינים (מותאם ל-Arduino Mega מומלץ) ---
// רדיו
RF24 radio(7, 8); // CE, CSN
const byte addressToBoat[6] = "00001";
const byte addressToShore[6] = "00002";

// מנועי נסיעה (L298N #1)
const int PWM_LEFT = 2; const int IN1 = 22; const int IN2 = 24;
const int PWM_RIGHT = 3; const int IN3 = 26; const int IN4 = 28;

// מנוע משיכת רשת / כננת (L298N #2)
const int PWM_WINCH = 4; const int IN5 = 30; const int IN6 = 32;

// סרוו למכ"ם
const int RADAR_SERVO_PIN = 9;
Servo radarServo;

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
};

CommandPacket cmd;
TelemetryPacket telemetry;

// ניהול זמנים
unsigned long previousMillis = 0;
const long TELEMETRY_INTERVAL = 100; // 10 הרץ

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
  radarServo.write(90); // מרכז
  
  // אתחול רדיו
  if (!radio.begin()) {
    while (1); // שגיאת רדיו קריטית
  }
  radio.openWritingPipe(addressToShore);
  radio.openReadingPipe(1, addressToBoat);
  radio.setPALevel(RF24_PA_MAX);
  radio.startListening();
}

void loop() {
  // 1. קריאת פקודות מממסר החוף
  if (radio.available()) {
    radio.read(&cmd, sizeof(CommandPacket));
    
    // ביצוע הפקודות
    controlDriveMotors(cmd.leftSpeed, cmd.rightSpeed);
    controlWinchMotor(cmd.winchSpeed);
    radarServo.write(cmd.radarAngle);
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
    
    // שידור הטלמטריה
    radio.stopListening(); 
    radio.write(&telemetry, sizeof(TelemetryPacket));
    radio.startListening(); 
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