#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>

// הגדרת הרדיו (פינים 7 ו-8 יכולים להשתנות לפי החיווט שלכם)
RF24 radio(7, 8); 

// כתובות לתקשורת (אחת לשידור ואחת לקליטה)
const byte addressToBoat[6] = "00001";
const byte addressToShore[6] = "00002";

// מבנה הנתונים שנשלח לסירה (פקודות)
struct CommandPacket {
  int leftSpeed;
  int rightSpeed;
  int winchSpeed;
  int radarAngle;
};

// מבנה הנתונים שמתקבל מהסירה (טלמטריה)
struct TelemetryPacket {
  int usRadar;
  int usFront;
  int usLeft;
  int usRight;
};

CommandPacket cmd = {0, 0, 0, 90}; // ערכי ברירת מחדל
TelemetryPacket telemetry;

void setup() {
  Serial.begin(115200);
  
  if (!radio.begin()) {
    Serial.println("ERROR: Radio hardware is not responding!");
    while (1); // עצירה אם אין רדיו
  }
  
  radio.openWritingPipe(addressToBoat);
  radio.openReadingPipe(1, addressToShore);
  radio.setPALevel(RF24_PA_MAX); // עוצמת שידור מקסימלית
  radio.startListening();
}

void loop() {
  // 1. קריאת פקודות מהמחשב (דרך כבל ה-Serial)
  if (Serial.available() > 0) {
    String inputData = Serial.readStringUntil('\n');
    
    // פירוק הפקודה מהמחשב: מהירות_שמאל,מהירות_ימין,מהירות_כננת,זווית_מכם
    // נשתמש ב-sscanf לפירוק מהיר (או ב-indexOf כמו קודם)
    int parsed = sscanf(inputData.c_str(), "%d,%d,%d,%d", &cmd.leftSpeed, &cmd.rightSpeed, &cmd.winchSpeed, &cmd.radarAngle);
    
    if (parsed == 4) {
      // אם התקבלו כל 4 הערכים נשדר אותם לסירה
      radio.stopListening(); // חייבים לעצור האזנה כדי לשדר
      bool ok = radio.write(&cmd, sizeof(CommandPacket));
      radio.startListening(); // חזרה להאזנה מיד לאחר מכן
      
      if (!ok) {
        Serial.println("ERROR: Radio transmission failed");
      }
    }
  }

  // 2. קבלת טלמטריה מהסירה והעברתה למחשב
  if (radio.available()) {
    radio.read(&telemetry, sizeof(TelemetryPacket));
    
    // שליחה למחשב בפורמט קריא/CSV
    Serial.print(telemetry.usRadar); Serial.print(",");
    Serial.print(telemetry.usFront); Serial.print(",");
    Serial.print(telemetry.usLeft); Serial.print(",");
    Serial.println(telemetry.usRight);
  }
}
