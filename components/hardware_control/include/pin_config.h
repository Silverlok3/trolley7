#ifndef PIN_CONFIG_H
#define PIN_CONFIG_H
#include "driver/ledc.h"
#include "driver/i2c.h"
#include "driver/gpio.h"

// ESC Control Pin (Littlebee 30A ESC) - ESP32-S3 compatible pins
#define ESC_PWM_PIN             ((gpio_num_t)18)    // ESC PWM signal pin (LEDC capable)
#define ESC_ENABLE_PIN          ((gpio_num_t)16)    // ESC enable/arm pin (FIXED: was GPIO19 - USB conflict)

// Hall Sensor Pin (for motor feedback) - ESP32-S3 compatible
#define HALL_PIN                ((gpio_num_t)4)     // Hall sensor input (interrupt capable)
#define HALL_SENSOR_PIN         HALL_PIN            // Alternative name

// MPU6050/IMU Pins (I2C) - ESP32-S3 default I2C pins
#define I2C_SDA_PIN             ((gpio_num_t)8)     // I2C SDA (ESP32-S3 default)
#define I2C_SCL_PIN             ((gpio_num_t)9)     // I2C SCL (ESP32-S3 default)
#define I2C_PORT_NUM            I2C_NUM_0

// Status LED Pins - ESP32-S3 compatible
#define STATUS_LED_PIN          ((gpio_num_t)2)     // Built-in RGB LED data pin on ESP32-S3
#define ERROR_LED_PIN           ((gpio_num_t)15)    // Additional LED for errors

// USB Serial/JTAG pins (ESP32-S3 specific)
// Note: GPIO19 and GPIO20 are used for USB on ESP32-S3
// Avoid using GPIO19-20 if you need USB functionality

// ESC PWM Configuration (Standard servo PWM) - ESP32-S3 Compatible
#define ESC_PWM_FREQ_HZ         50    // 50Hz for standard servo PWM
#define ESC_PWM_RESOLUTION      LEDC_TIMER_14_BIT  // 14-bit resolution (ESP32-S3 maximum)
#define ESC_PWM_CHANNEL         LEDC_CHANNEL_0     // ESC PWM channel
#define ESC_PWM_TIMER           LEDC_TIMER_0

// ESC PWM Pulse Width Constants (in microseconds) - Same as ESP32
#define ESC_MIN_PULSE_US        1000   // Minimum pulse width (full reverse)
#define ESC_NEUTRAL_PULSE_US    1500   // Neutral pulse width (stop)
#define ESC_MAX_PULSE_US        2000   // Maximum pulse width (full forward)
#define ESC_ARM_PULSE_US        1000   // Arming pulse width

// ESC PWM Duty Cycle Constants (calculated for 14-bit resolution at 50Hz)
// Formula: duty = (pulse_width_us / 20000us) * (2^14 - 1)
#define ESC_MIN_DUTY            819    // 1000us pulse (5% duty cycle) - 14-bit
#define ESC_NEUTRAL_DUTY        1229   // 1500us pulse (7.5% duty cycle) - 14-bit
#define ESC_MAX_DUTY            1638   // 2000us pulse (10% duty cycle) - 14-bit
#define ESC_ARM_DUTY            819    // 1000us pulse for arming - 14-bit

// ESC Configuration - Same as ESP32
#define ESC_ARM_TIME_MS         3000   // Time to keep arming signal (3 seconds)
#define ESC_DEADBAND            50     // Deadband around neutral position
#define ESC_MAX_SPEED_CHANGE    100    // Maximum speed change per update

// Direction control for brushless motor
typedef enum {
    ESC_DIRECTION_STOP = 0,
    ESC_DIRECTION_FORWARD,
    ESC_DIRECTION_REVERSE
} esc_direction_t;

// ESP32-S3 Specific Notes:
// - GPIO0: Used for Boot mode selection (avoid if possible)
// - GPIO1-18: General purpose I/O (safe to use)
// - GPIO19-20: USB D-/D+ (avoid if USB needed)
// - GPIO21-48: General purpose I/O (safe to use)
// - Built-in RGB LED on GPIO2 (WS2812)
// - SPI Flash: GPIO26-32 (do not use)
// - PSRAM: GPIO33-37 (do not use if PSRAM enabled)

#endif // PIN_CONFIG_H