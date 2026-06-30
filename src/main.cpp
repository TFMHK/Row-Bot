#include <RH_ASK.h>
#include <Adafruit_NeoPixel.h>

// רדיו: speed=2000bps, rxPin=7, txPin=6
RH_ASK driver(2000, 7, 6);
const int PIXEL_PIN = 5;
const int PIXEL_COUNT = 1;
Adafruit_NeoPixel pixel(PIXEL_COUNT, PIXEL_PIN, NEO_GRB + NEO_KHZ800);

// מנוע בדיקה יחיד דרך H-Bridge
// כיוון: D8/D9, מהירות PWM: D10
const int MOTOR1_IN1 = 8;
const int MOTOR1_IN2 = 9;
const int MOTOR1_PWM = 10;

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

void controlTestMotor(int speed);
void setPixelFromCommand(const CommandPacket& packet);

void setup() {
  pinMode(MOTOR1_IN1, OUTPUT);
  pinMode(MOTOR1_IN2, OUTPUT);
  pinMode(MOTOR1_PWM, OUTPUT);

  pixel.begin();
  pixel.setBrightness(40);
  setPixelFromCommand(cmd);

  if (!driver.init()) {
    while (1);
  }
}

void loop() {
  uint8_t buf[sizeof(CommandPacket)];
  uint8_t buflen = sizeof(CommandPacket);
  if (driver.recv(buf, &buflen) && buflen == sizeof(CommandPacket)) {
    memcpy(&cmd, buf, sizeof(CommandPacket));
    setPixelFromCommand(cmd);
    controlTestMotor(cmd.leftSpeed);

    // בחומרת ASK פשוטה עדיף לענות אחרי קבלה במקום לשדר כל הזמן
    telemetry.usRadar = 999;
    telemetry.usFront = 999;
    telemetry.usLeft = 999;
    telemetry.usRight = 999;
    telemetry.radarAngle = cmd.radarAngle;
    driver.send((uint8_t*)&telemetry, sizeof(TelemetryPacket));
    driver.waitPacketSent();
  }
}

void controlTestMotor(int speed) {
  int pwm = abs(constrain(speed, -255, 255));

  if (speed > 0) {
    digitalWrite(MOTOR1_IN1, HIGH);
    digitalWrite(MOTOR1_IN2, LOW);
    analogWrite(MOTOR1_PWM, pwm);
  } else if (speed < 0) {
    digitalWrite(MOTOR1_IN1, LOW);
    digitalWrite(MOTOR1_IN2, HIGH);
    analogWrite(MOTOR1_PWM, pwm);
  } else {
    digitalWrite(MOTOR1_IN1, LOW);
    digitalWrite(MOTOR1_IN2, LOW);
    analogWrite(MOTOR1_PWM, 0);
  }
}

void setPixelFromCommand(const CommandPacket& packet) {
  int angle = constrain(packet.radarAngle, 0, 180);
  int leftMag = abs(constrain(packet.leftSpeed, -255, 255));
  int rightMag = abs(constrain(packet.rightSpeed, -255, 255));
  int winch = constrain(packet.winchSpeed, -255, 255);

  // Hue לפי זווית מכ"ם, בהירות לפי מהירות מנועי נסיעה, והיסט גוון לפי כננת.
  uint16_t baseHue = (uint16_t)map(angle, 0, 180, 0, 65535);
  int hueOffset = map(winch, -255, 255, -12000, 12000);
  uint16_t hue = (uint16_t)(baseHue + hueOffset);
  uint8_t brightness = (uint8_t)map(max(leftMag, rightMag), 0, 255, 25, 255);

  uint32_t color = pixel.gamma32(pixel.ColorHSV(hue, 255, brightness));
  pixel.setPixelColor(0, color);
  pixel.show();
}