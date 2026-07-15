#include <RH_ASK.h>

// רדיו: speed=2000bps, rxPin=11, txPin=10 (חייב להתאים לקצב של הסירה)
// קצב נמוך => מקלט ASK רגיש יותר => טווח טוב יותר. ה-heartbeat בשרת הוגדל
RH_ASK driver(2000, 11, 10);

struct CommandPacket {
  int leftSpeed;
  int rightSpeed;
  int winchSpeed;
  int radarAngle;
  int mode;           // 0 = ידני, 1 = אוטומטי (צבע טבעת הלדים בסירה)
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

void setup() {
  Serial.begin(115200);

  if (!driver.init()) {
    Serial.println("ERROR: RF driver init failed!");
    while (1);
  }
}

void loop() {
  // 1. קריאת פקודות מהמחשב (דרך כבל ה-Serial)
  if (Serial.available() > 0) {
    String inputData = Serial.readStringUntil('\n');
    int parsed = sscanf(inputData.c_str(), "%d,%d,%d,%d,%d",
                        &cmd.leftSpeed, &cmd.rightSpeed, &cmd.winchSpeed, &cmd.radarAngle, &cmd.mode);
    if (parsed == 5) {
      driver.send((uint8_t*)&cmd, sizeof(CommandPacket));
      driver.waitPacketSent();
    }
  }

  // 2. קבלת טלמטריה מהסירה והעברתה למחשב
  uint8_t buf[sizeof(TelemetryPacket)];
  uint8_t buflen = sizeof(TelemetryPacket);
  if (driver.recv(buf, &buflen)) {
    if (buflen == sizeof(TelemetryPacket)) {
      memcpy(&telemetry, buf, sizeof(TelemetryPacket));
      Serial.print(telemetry.usRadar);  Serial.print(",");
      Serial.print(telemetry.usFront);  Serial.print(",");
      Serial.print(telemetry.usLeft);   Serial.print(",");
      Serial.print(telemetry.usRight);  Serial.print(",");
      Serial.println(telemetry.radarAngle);
    }
  }
}
