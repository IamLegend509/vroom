// DFRobot DRI0058 SimpleFOCmini + ESP32 bring-up target.
//
// Purpose: fail-safe open-loop bring-up plus an optional AS5600 closed-loop mode.
// NOT an RL deployment target and contains no exported policy. The simulator
// remains the source for all controller studies.
//
// Units: SimpleFOC velocity targets are rad/s; voltage_limit is V. This wiring
// has no phase-current sensing, so voltage command is not current/torque control.
// The board's 2.5 A/channel rating is not enforceable by this firmware.
// Assumptions: DRI0058 3.3-V logic; optional AS5600 is selected at build time.
// Limits: GPIO mapping is proposed for ESP32 DevKit only; verify all wiring.
// Repro (build only): platformio run -d hw/focmini

#include <Arduino.h>
#include <SimpleFOC.h>

#ifndef FOCMINI_USE_AS5600
#define FOCMINI_USE_AS5600 0
#endif
#if FOCMINI_USE_AS5600
#include <Wire.h>
#endif
namespace focmini {
// IN1/IN2/IN3, EN, RESET, FAULT, SLEEP respectively.
constexpr int kPwmA = 25, kPwmB = 26, kPwmC = 27, kEnable = 32;
constexpr int kReset = 33, kFault = 34, kSleep = 14;
constexpr float kPwmFrequencyHz = 20'000.0f;
#if FOCMINI_USE_AS5600
constexpr int kI2cSda = 21, kI2cScl = 22;
#endif
constexpr float kAlignmentVoltageV = 0.30f;
// Enter the measured supply voltage. Zero deliberately keeps a new checkout off.
constexpr float kBusVoltageV = 9.0f;
constexpr float kVoltageLimitV = 0.50f;
constexpr float kVelocityLimitRadPerSec = 5.0f;

// Set from verified motor data (14 magnet poles means 7 pole pairs).
// Zero deliberately keeps a new checkout off.
constexpr int kMotorPolePairs = 7;
volatile bool fault_latched = false;

void IRAM_ATTR on_fault() { fault_latched = true; }

void disable_power_stage() {
  digitalWrite(kEnable, LOW);
  digitalWrite(kSleep, LOW);
}

bool configuration_is_safe() {
  return kMotorPolePairs > 0 && kAlignmentVoltageV > 0.0f &&
         kBusVoltageV >= 8.0f && kBusVoltageV <= 30.0f &&
         kVoltageLimitV > 0.0f && kVoltageLimitV <= kBusVoltageV;
}
}  // namespace focmini

using namespace focmini;
BLDCMotor motor(kMotorPolePairs);
#if FOCMINI_USE_AS5600
MagneticSensorI2C sensor = MagneticSensorI2C(AS5600_I2C);
#endif
BLDCDriver3PWM driver(kPwmA, kPwmB, kPwmC, kEnable);

void setup() {
  Serial.begin(115200);
  pinMode(kEnable, OUTPUT);
  pinMode(kReset, OUTPUT);
  pinMode(kSleep, OUTPUT);
  pinMode(kFault, INPUT);
  disable_power_stage();
  digitalWrite(kReset, LOW);
  delay(10);
  digitalWrite(kReset, HIGH);
  delay(10);
  attachInterrupt(digitalPinToInterrupt(kFault), on_fault, FALLING);

  if (!configuration_is_safe()) {
    Serial.println("DISARMED: set measured kMotorPolePairs and limits.");
    return;
  }
  if (digitalRead(kFault) == LOW) {
    fault_latched = true;
    Serial.println("DISARMED: driver FAULT is active.");
    return;
  }

  driver.voltage_power_supply = kBusVoltageV;
#if FOCMINI_USE_AS5600
  Wire.begin(kI2cSda, kI2cScl);
  sensor.init(&Wire);
  motor.linkSensor(&sensor);
#endif
  driver.pwm_frequency = kPwmFrequencyHz;
  driver.voltage_limit = kVoltageLimitV;
  if (!driver.init()) {
    Serial.println("DISARMED: PWM driver initialization failed.");
    return;
  }
  motor.linkDriver(&driver);
#if FOCMINI_USE_AS5600
  motor.controller = MotionControlType::velocity;
#else
  motor.controller = MotionControlType::velocity_openloop;
#endif
  motor.voltage_limit = kVoltageLimitV;
  motor.velocity_limit = kVelocityLimitRadPerSec;
  motor.init();

#if FOCMINI_USE_AS5600
  motor.voltage_sensor_align = kAlignmentVoltageV;
#endif
  digitalWrite(kSleep, HIGH);
  digitalWrite(kEnable, HIGH);
#if FOCMINI_USE_AS5600
  if (!motor.initFOC()) {
    disable_power_stage();
    Serial.println("DISARMED: AS5600 FOC alignment failed.");
    return;
  }
  Serial.println("ARMED AS5600 CLOSED LOOP: target 0 rad/s.");
#else
  Serial.println("ARMED OPEN LOOP: target 0 rad/s. Send a target within limit.");
#endif
}

void loop() {
  if (fault_latched) {
    disable_power_stage();
    Serial.println("FAULT LATCHED: cycle power and inspect wiring.");
    while (true) delay(1000);
  }
  if (!configuration_is_safe()) {
    delay(1000);
    return;
  }

#if FOCMINI_USE_AS5600
  motor.loopFOC();
#endif
  if (Serial.available()) {
    const float target_rad_per_sec = Serial.parseFloat();
    if (isfinite(target_rad_per_sec) &&
        fabsf(target_rad_per_sec) <= kVelocityLimitRadPerSec) {
      motor.target = target_rad_per_sec;
    } else {
      Serial.println("Rejected target: finite and within velocity_limit required.");
    }
  }
  motor.move();
}
