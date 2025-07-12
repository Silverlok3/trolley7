#ifndef HARDWARE_CONTROL_H
#define HARDWARE_CONTROL_H

#include "esp_err.h"
#include <stdint.h>
#include <stdbool.h>

// ═══════════════════════════════════════════════════════════════════════════════
// HARDWARE_CONTROL.H - LOW-LEVEL HARDWARE ABSTRACTION LAYER
// ═══════════════════════════════════════════════════════════════════════════════
// 
// SINGLE RESPONSIBILITY: Hardware abstraction only
// - ESC PWM control (init, set_duty, arm, disarm)
// - Hall sensor interrupt handling  
// - GPIO initialization
// - Basic motor speed commands
// - Emergency stop (hardware level)
// 
// NO business logic, NO mode management - Pure hardware interface
// ═══════════════════════════════════════════════════════════════════════════════

// Hardware configuration constants
#define WHEEL_CIRCUMFERENCE_MM      191.6f      // 61mm wheel circumference
#define MM_TO_M                     0.001f      // Conversion factor
#define MAX_SPEED_MS               2.0f         // Hardware speed limit
#define ESC_SPEED_DEADBAND         0.05f        // Minimum ESC response speed
#define ESC_UPDATE_INTERVAL_MS     20           // 50Hz update rate
#define HALL_TIMEOUT_MS            2000         // Hall sensor timeout

// Hardware status structure
typedef struct {
    bool esc_armed;                    // ESC armed status
    bool esc_responding;               // ESC responding to commands
    uint16_t current_esc_duty;         // Current PWM duty cycle
    float current_speed_ms;            // Current speed from Hall sensor
    float target_speed_ms;             // Target speed command
    bool direction_forward;            // Current direction
    uint32_t total_rotations;          // Total Hall pulses since init
    uint64_t last_hall_time;           // Last Hall pulse timestamp
    bool hall_sensor_healthy;          // Hall sensor status
    bool system_initialized;           // Hardware initialization status
} hardware_status_t;

// Hardware error types
typedef enum {
    HW_ERROR_NONE = 0,
    HW_ERROR_ESC_NOT_RESPONDING,
    HW_ERROR_HALL_SENSOR_TIMEOUT,
    HW_ERROR_PWM_INIT_FAILED,
    HW_ERROR_GPIO_INIT_FAILED,
    HW_ERROR_SPEED_OUT_OF_RANGE,
    HW_ERROR_SYSTEM_NOT_INITIALIZED
} hardware_error_t;

// ═══════════════════════════════════════════════════════════════════════════════
// CORE HARDWARE INITIALIZATION
// ═══════════════════════════════════════════════════════════════════════════════

/**
 * @brief Initialize all hardware components
 * @return ESP_OK on success, error code on failure
 */
esp_err_t hardware_init(void);

/**
 * @brief Get current hardware status
 * @return hardware_status_t structure with current state
 */
hardware_status_t hardware_get_status(void);

/**
 * @brief Get last hardware error
 * @return hardware_error_t error code
 */
hardware_error_t hardware_get_last_error(void);

/**
 * @brief Check if hardware is ready for operation
 * @return true if ready, false if not
 */
bool hardware_is_ready(void);

// ═══════════════════════════════════════════════════════════════════════════════
// ESC CONTROL FUNCTIONS
// ═══════════════════════════════════════════════════════════════════════════════

/**
 * @brief Arm the ESC (required before motor operation)
 * @return ESP_OK on success, error code on failure
 */
esp_err_t hardware_esc_arm(void);

/**
 * @brief Disarm the ESC (safe state)
 * @return ESP_OK on success, error code on failure
 */
esp_err_t hardware_esc_disarm(void);

/**
 * @brief Check if ESC is armed
 * @return true if armed, false if disarmed
 */
bool hardware_esc_is_armed(void);

/**
 * @brief Set motor speed and direction
 * @param speed_ms Speed in meters per second (0.0 to MAX_SPEED_MS)
 * @param forward Direction (true = forward, false = reverse)
 * @return ESP_OK on success, error code on failure
 */
esp_err_t hardware_set_motor_speed(float speed_ms, bool forward);

/**
 * @brief Emergency stop - immediate motor halt
 * @return ESP_OK on success, error code on failure
 */
esp_err_t hardware_emergency_stop(void);

/**
 * @brief Get current motor speed from Hall sensor
 * @return Speed in m/s, 0.0 if no movement detected
 */
float hardware_get_current_speed(void);

// ═══════════════════════════════════════════════════════════════════════════════
// HALL SENSOR FUNCTIONS
// ═══════════════════════════════════════════════════════════════════════════════

/**
 * @brief Get total rotation count since initialization
 * @return Number of Hall sensor pulses
 */
uint32_t hardware_get_rotation_count(void);

/**
 * @brief Reset rotation counter
 * @return ESP_OK on success
 */
esp_err_t hardware_reset_rotation_count(void);

/**
 * @brief Get time since last Hall pulse
 * @return Time in microseconds, 0 if never received pulse
 */
uint64_t hardware_get_time_since_last_hall_pulse(void);

/**
 * @brief Check if Hall sensor is responding
 * @return true if healthy, false if timeout or failure
 */
bool hardware_is_hall_sensor_healthy(void);

/**
 * @brief Calculate distance traveled from rotation count
 * @param rotations Number of rotations
 * @return Distance in meters
 */
float hardware_rotations_to_distance(uint32_t rotations);

/**
 * @brief Calculate rotations needed for given distance
 * @param distance_m Distance in meters
 * @return Number of rotations needed
 */
uint32_t hardware_distance_to_rotations(float distance_m);

// ═══════════════════════════════════════════════════════════════════════════════
// GPIO AND SYSTEM FUNCTIONS
// ═══════════════════════════════════════════════════════════════════════════════

/**
 * @brief Set status LED state
 * @param on LED state (true = on, false = off)
 * @return ESP_OK on success
 */
esp_err_t hardware_set_status_led(bool on);

/**
 * @brief Get current position based on rotation count and direction
 * @return Position in meters (can be negative)
 */
float hardware_get_current_position(void);

/**
 * @brief Reset position counter to zero
 * @return ESP_OK on success
 */
esp_err_t hardware_reset_position(void);

/**
 * @brief Update hardware control loop (call regularly)
 * @return ESP_OK on success
 */
esp_err_t hardware_update(void);

// ═══════════════════════════════════════════════════════════════════════════════
// UTILITY FUNCTIONS
// ═══════════════════════════════════════════════════════════════════════════════

/**
 * @brief Convert hardware error to string
 * @param error Hardware error code
 * @return String description of error
 */
const char* hardware_error_to_string(hardware_error_t error);

/**
 * @brief Validate speed parameter
 * @param speed_ms Speed to validate
 * @return true if valid, false if out of range
 */
bool hardware_is_speed_valid(float speed_ms);

/**
 * @brief Get hardware component versions/info
 * @param info_buffer Buffer to write info string
 * @param buffer_size Size of info buffer
 * @return ESP_OK on success
 */
esp_err_t hardware_get_info(char* info_buffer, size_t buffer_size);

// ═══════════════════════════════════════════════════════════════════════════════
// CALLBACK REGISTRATION (for higher-level components)
// ═══════════════════════════════════════════════════════════════════════════════

// Hall sensor pulse callback type
typedef void (*hall_pulse_callback_t)(uint32_t total_rotations, uint64_t timestamp);

// ESC status change callback type  
typedef void (*esc_status_callback_t)(bool armed, bool responding);

/**
 * @brief Register callback for Hall sensor pulses
 * @param callback Function to call on each Hall pulse
 * @return ESP_OK on success
 */
esp_err_t hardware_register_hall_callback(hall_pulse_callback_t callback);

/**
 * @brief Register callback for ESC status changes
 * @param callback Function to call on ESC status change
 * @return ESP_OK on success
 */
esp_err_t hardware_register_esc_callback(esc_status_callback_t callback);

// ═══════════════════════════════════════════════════════════════════════════════
// ADVANCED CONTROL (for mode-specific needs)
// ═══════════════════════════════════════════════════════════════════════════════

/**
 * @brief Set ESC PWM duty cycle directly (advanced users only)
 * @param duty_cycle PWM duty cycle (ESC_MIN_DUTY to ESC_MAX_DUTY)
 * @return ESP_OK on success, error if ESC not armed
 */
esp_err_t hardware_set_esc_duty_direct(uint16_t duty_cycle);

/**
 * @brief Get current ESC PWM duty cycle
 * @return Current duty cycle value
 */
uint16_t hardware_get_esc_duty(void);

/**
 * @brief Enable/disable ESC rate limiting
 * @param enable Enable rate limiting for smooth acceleration
 * @return ESP_OK on success
 */
esp_err_t hardware_set_esc_rate_limiting(bool enable);

#endif // HARDWARE_CONTROL_H