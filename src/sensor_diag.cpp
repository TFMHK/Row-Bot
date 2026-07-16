// דיאגנוסטיקת חיישני אולטרה-סוני דרך USB (עוקף RF לגמרי).
// קורא את כל 4 החיישנים ומדפיס רציף ~5Hz על הטורי עם תווית הפין של כל אחד.
// זהה ללוגיקת readAllUltrasonic בקושחה האמיתית -> המיפוי תואם.
// פלאש:  pio run -e diag -t upload   (הסירה מחוברת ב-USB, COM6)
// קריאה: python host/sensor_diag_read.py COM6
#include <Arduino.h>

const int US_TRIG_PIN = 2;          // טריגר משותף
const int US_ECHO_RADAR_PIN = A5;   // אחורה
const int US_ECHO_FRONT_PIN = A1;   // קדימה
const int US_ECHO_LEFT_PIN  = A2;   // שמאל
const int US_ECHO_RIGHT_PIN = A4;   // ימין

const unsigned long US_TIMEOUT_US = 25000UL; // ~4m

// results[0]=radar/back, [1]=front, [2]=left, [3]=right  (999 = אין הד)
void readAllUltrasonic(int results[4]) {
  const int echoPins[4] = {
    US_ECHO_RADAR_PIN, US_ECHO_FRONT_PIN, US_ECHO_LEFT_PIN, US_ECHO_RIGHT_PIN
  };

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

void setup() {
  Serial.begin(115200);
  pinMode(US_TRIG_PIN, OUTPUT);
  digitalWrite(US_TRIG_PIN, LOW);
  pinMode(US_ECHO_RADAR_PIN, INPUT);
  pinMode(US_ECHO_FRONT_PIN, INPUT);
  pinMode(US_ECHO_LEFT_PIN, INPUT);
  pinMode(US_ECHO_RIGHT_PIN, INPUT);
  Serial.println(F("SENSOR DIAG ready. cols: FRONT(A1) RIGHT(A4) BACK(A5) LEFT(A2)"));
}

void loop() {
  int r[4];
  readAllUltrasonic(r);
  // r[0]=back(A5) r[1]=front(A1) r[2]=left(A2) r[3]=right(A4)
  char line[80];
  snprintf(line, sizeof(line),
           "FRONT(A1)=%4d  RIGHT(A4)=%4d  BACK(A5)=%4d  LEFT(A2)=%4d",
           r[1], r[3], r[0], r[2]);
  Serial.println(line);
  delay(180);
}
