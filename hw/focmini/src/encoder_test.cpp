// AS5600 read-only diagnostic for original ESP32-WROOM. No PWM/motor library
// or motion commands. EN/SLEEP/RESET stay low, even without motor power.
// AS5600 datasheet DS000365: address 0x36; STATUS 0x0B bits 5/4/3 = MD/ML/MH;
// RAW ANGLE 0x0C..0x0D is 12-bit. Degrees = raw * 360/4096.
#include <Arduino.h>
#include <Wire.h>

namespace {
constexpr uint8_t kAddress = 0x36;
bool read_registers(uint8_t reg, uint8_t *bytes, size_t count) {
  Wire.beginTransmission(kAddress);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom(kAddress, count, true) != count) return false;
  for (size_t i = 0; i < count; ++i) bytes[i] = Wire.read();
  return true;
}
}

void setup() {
  // Configure external gate first. Never wake or enable the power stage.
  pinMode(32, OUTPUT); digitalWrite(32, LOW);
  pinMode(14, OUTPUT); digitalWrite(14, LOW);
  pinMode(33, OUTPUT); digitalWrite(33, LOW);
  Serial.begin(115200);
  Wire.begin(21, 22, 100000);
  Wire.setTimeOut(50);
  Serial.println("ENCODER_TEST: motor disabled; rotate shaft manually; no ARM support.");
}

void loop() {
  uint8_t status = 0, angle[2] = {};
  if (!read_registers(0x0B, &status, 1) || !read_registers(0x0C, angle, 2)) {
    Serial.println("ENC,i2c_ok=0");
  } else {
    const unsigned int raw = ((angle[0] & 0x0f) << 8) | angle[1];
    Serial.printf("ENC,i2c_ok=1,magnet_detected=%u,too_weak=%u,too_strong=%u,raw=%u,angle_deg=%.3f\n",
                  (status >> 5) & 1, (status >> 4) & 1, (status >> 3) & 1,
                  raw, raw * (360.0 / 4096.0));
  }
  delay(100);
}
