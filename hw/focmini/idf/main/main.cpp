// DFRobot DRI0058 + ESP32 DevKit native ESP-IDF bring-up application.
//
// Purpose: conservative sensorless open-loop first spin, with an optional
// AS5600 closed-loop velocity mode. Not an RL deployment target.
// Units: bus/voltage settings are converted from menuconfig mV to V;
// velocity command/limit are rad/s. No phase-current measurement exists, so
// voltage control cannot enforce motor or driver current limits.
// Assumptions: DRI0058 wiring follows ../README.md; supply and pole pairs are
// entered by the operator after measurement/verification. Zero defaults keep
// the power stage disabled. Limits: validate GPIO mapping on the exact board.
// Repro: idf.py set-target esp32; idf.py menuconfig; idf.py build.

#include <cmath>

#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_simplefoc.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

namespace {
constexpr gpio_num_t kPwmA = GPIO_NUM_25;
constexpr gpio_num_t kPwmB = GPIO_NUM_26;
constexpr gpio_num_t kPwmC = GPIO_NUM_27;
constexpr gpio_num_t kEnable = GPIO_NUM_32;
constexpr gpio_num_t kReset = GPIO_NUM_33;
constexpr gpio_num_t kFault = GPIO_NUM_34;
constexpr gpio_num_t kSleep = GPIO_NUM_14;
constexpr gpio_num_t kI2cScl = GPIO_NUM_22;
constexpr gpio_num_t kI2cSda = GPIO_NUM_21;
constexpr float kPwmFrequencyHz = 20000.0f;
constexpr float kAlignmentVoltageV = 0.30f;
constexpr char kTag[] = "focmini";

constexpr float kBusVoltageV = CONFIG_FOCMINI_BUS_MV / 1000.0f;
constexpr float kVoltageLimitV = CONFIG_FOCMINI_VOLTAGE_LIMIT_MV / 1000.0f;
constexpr float kVelocityLimitRadPerSec =
    CONFIG_FOCMINI_VELOCITY_LIMIT_MRAD_S / 1000.0f;

BLDCMotor motor(CONFIG_FOCMINI_POLE_PAIRS);
BLDCDriver3PWM driver(kPwmA, kPwmB, kPwmC);
#if CONFIG_FOCMINI_USE_AS5600
AS5600 sensor(I2C_NUM_0, kI2cScl, kI2cSda);
#endif
float target_rad_per_sec = 0.0f;
Commander command(Serial);

void disable_power_stage() {
    gpio_set_level(kEnable, 0);
    gpio_set_level(kSleep, 0);
}

bool safe_configuration() {
    return CONFIG_FOCMINI_POLE_PAIRS > 0 && kBusVoltageV >= 8.0f &&
           kBusVoltageV <= 30.0f && kVoltageLimitV > 0.0f &&
           kVoltageLimitV <= kBusVoltageV &&
           kVelocityLimitRadPerSec > 0.0f;
}

void on_target(char *input) {
    command.scalar(&target_rad_per_sec, input);
    if (!std::isfinite(target_rad_per_sec) ||
        std::fabs(target_rad_per_sec) > kVelocityLimitRadPerSec) {
        target_rad_per_sec = 0.0f;
        ESP_LOGW(kTag, "Rejected target; command must be finite and within limit");
    }
}

bool initialize_safe_gpio() {
    gpio_config_t outputs{};
    outputs.pin_bit_mask = (1ULL << kEnable) | (1ULL << kReset) | (1ULL << kSleep);
    outputs.mode = GPIO_MODE_OUTPUT;
    outputs.pull_up_en = GPIO_PULLUP_DISABLE;
    outputs.pull_down_en = GPIO_PULLDOWN_DISABLE;
    outputs.intr_type = GPIO_INTR_DISABLE;
    if (gpio_config(&outputs) != ESP_OK) return false;
    disable_power_stage();
    gpio_set_level(kReset, 1);  // RESET is active low; keep it deasserted.

    gpio_config_t input{};
    input.pin_bit_mask = 1ULL << kFault;
    input.mode = GPIO_MODE_INPUT;
    input.pull_up_en = GPIO_PULLUP_DISABLE;  // External 10 kOhm pull-up required.
    input.pull_down_en = GPIO_PULLDOWN_DISABLE;
    input.intr_type = GPIO_INTR_DISABLE;
    return gpio_config(&input) == ESP_OK;
}
}  // namespace

extern "C" void app_main(void) {
    if (!initialize_safe_gpio()) {
        ESP_LOGE(kTag, "GPIO setup failed; leaving power stage disabled");
        return;
    }
    Serial.begin(115200);
    command.add('T', on_target, const_cast<char *>("velocity rad/s"));

    if (!safe_configuration()) {
        ESP_LOGW(kTag, "DISARMED: configure verified pole pairs and measured supply in menuconfig");
        return;
    }
    if (gpio_get_level(kFault) == 0) {
        ESP_LOGE(kTag, "DISARMED: DRI0058 FAULT is active");
        return;
    }

    driver.voltage_power_supply = kBusVoltageV;
    driver.voltage_limit = kVoltageLimitV;
    driver.pwm_frequency = kPwmFrequencyHz;
#if CONFIG_SOC_MCPWM_SUPPORTED
    if (driver.init(0) == 0) {
#else
    if (driver.init({kPwmA, kPwmB, kPwmC}) == 0) {
#endif
        ESP_LOGE(kTag, "PWM driver initialization failed");
        return;
    }
    motor.linkDriver(&driver);
    motor.voltage_limit = kVoltageLimitV;
    motor.velocity_limit = kVelocityLimitRadPerSec;

#if CONFIG_FOCMINI_USE_AS5600
    sensor.init();
    motor.linkSensor(&sensor);
    motor.controller = MotionControlType::velocity;
    motor.voltage_sensor_align = kAlignmentVoltageV;
    motor.PID_velocity.P = 0.2f;
    motor.PID_velocity.I = 2.0f;
    motor.LPF_velocity.Tf = 0.05f;
#else
    motor.controller = MotionControlType::velocity_openloop;
#endif

    motor.init();
    gpio_set_level(kSleep, 1);
    gpio_set_level(kEnable, 1);
#if CONFIG_FOCMINI_USE_AS5600
    if (!motor.initFOC()) {
        disable_power_stage();
        ESP_LOGE(kTag, "DISARMED: AS5600 FOC alignment failed");
        return;
    }
    ESP_LOGI(kTag, "AS5600 closed-loop ready; command with T<rad/s>, e.g. T1");
#else
    ESP_LOGI(kTag, "Open-loop ready; command with T<rad/s>, e.g. T1");
#endif

    while (true) {
        if (gpio_get_level(kFault) == 0) {
            disable_power_stage();
            ESP_LOGE(kTag, "FAULT latched; power stage disabled until reboot");
            while (true) vTaskDelay(pdMS_TO_TICKS(1000));
        }
        command.run();
#if CONFIG_FOCMINI_USE_AS5600
        motor.loopFOC();
#endif
        motor.move(target_rad_per_sec);
        vTaskDelay(pdMS_TO_TICKS(1));
    }
}
