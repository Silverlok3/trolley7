#ifndef MODE_COORDINATOR_H
#define MODE_COORDINATOR_H

#include "esp_err.h"
#include <stdint.h>
#include <stdbool.h>

// UTILITY FUNCTIONS
// ═══════════════════════════════════════════════════════════════════════════════

/**
 * @brief Convert mode enum to string
 * @param mode Trolley operation mode
 * @return String representation of mode
 */
const char* mode_coordinator_mode_to_string(trolley_operation_mode_t mode);

/**
 * @brief Convert mode availability to string
 * @param availability Mode availability state
 * @return String representation of availability
 */
const char* mode_coordinator_availability_to_string(mode_availability_t availability);

/**
 * @brief Convert sensor validation state to string
 * @param state Sensor validation state
 * @return String representation of validation state
 */
const char* mode_coordinator_validation_to_string(sensor_validation_state_t state);

/**
 * @brief Get detailed system status for debugging
 * @param status_buffer Buffer to write status string
 * @param buffer_size Size of status buffer
 * @return ESP_OK on success
 */
esp_err_t mode_coordinator_get_detailed_status(char* status_buffer, size_t buffer_size);

// ═══════════════════════════════════════════════════════════════════════════════
// CONFIGURATION AND CONSTANTS
// ═══════════════════════════════════════════════════════════════════════════════

// Sensor validation timeouts
#define SENSOR_VALIDATION_TIMEOUT_MS    60000     // 1 minute total timeout
#define HALL_VALIDATION_MIN_PULSES      5         // Minimum pulses for validation
#define ACCEL_VALIDATION_MIN_SHAKE_G    0.3f      // Minimum shake for validation

// Mode transition timeouts
#define MODE_START_TIMEOUT_MS           5000      // 5 seconds to start mode
#define MODE_STOP_TIMEOUT_MS            10000     // 10 seconds to stop mode
#define EMERGENCY_STOP_TIMEOUT_MS       2000      // 2 seconds for emergency stop

// Automatic mode requirements
#define AUTO_MODE_MIN_WIRE_LENGTH_M     2.0f      // Minimum wire length for auto mode
#define AUTO_MODE_MAX_SPEED_MS          5.0f      // Maximum speed in automatic mode
#define AUTO_MODE_ACCEL_RATE_MS2        0.5f      // Acceleration rate m/s²

// Safety limits
#define MOTION_VALIDATION_TIMEOUT_MS    2000      // 2 second timeout for motion at 0.3 m/s
#define MAX_SYSTEM_ERRORS               10        // Maximum errors before system lockout
#define ERROR_RESET_TIME_MS             30000     // 30 seconds before error reset

#endif // MODE_COORDINATOR_H
// MODE_COORDINATOR.H - 3-MODE SYSTEM MANAGEMENT AND COORDINATION
// ═══════════════════════════════════════════════════════════════════════════════
// 
// SINGLE RESPONSIBILITY: Mode management and validation
// - system_set_mode() / system_get_mode_status()
// - Mode availability checking
// - Sensor validation requirements
// - Mode transition safety
// - Error handling coordination
// 
// Coordinates between: wire_learning_mode, automatic_mode, manual_mode
// Uses: hardware_control, sensor_health
// ═══════════════════════════════════════════════════════════════════════════════

// ═══════════════════════════════════════════════════════════════════════════════
// 3-MODE SYSTEM DEFINITIONS
// ═══════════════════════════════════════════════════════════════════════════════

/**
 * @brief Trolley operation modes
 */
typedef enum {
    TROLLEY_MODE_NONE = 0,              // No active mode
    TROLLEY_MODE_WIRE_LEARNING,         // Mode 1: Find wire length + optimal speeds
    TROLLEY_MODE_AUTOMATIC,             // Mode 2: Autonomous 5 m/s cycling with coasting
    TROLLEY_MODE_MANUAL                 // Mode 3: Full user control
} trolley_operation_mode_t;

/**
 * @brief Mode availability states
 */
typedef enum {
    MODE_BLOCKED_SENSORS_NOT_VALIDATED = 0,  // Sensors require validation
    MODE_BLOCKED_WIRE_LEARNING_REQUIRED,     // Wire learning must be completed first
    MODE_BLOCKED_SYSTEM_ERROR,               // System error blocking mode
    MODE_AVAILABLE,                          // Mode ready for activation
    MODE_ACTIVE,                             // Mode currently running
    MODE_STOPPING                            // Mode in process of stopping
} mode_availability_t;

/**
 * @brief Sensor validation state
 */
typedef enum {
    SENSOR_VALIDATION_NOT_STARTED = 0,   // Validation not initiated
    SENSOR_VALIDATION_IN_PROGRESS,       // User performing validation steps
    SENSOR_VALIDATION_HALL_PENDING,      // Waiting for Hall confirmation
    SENSOR_VALIDATION_ACCEL_PENDING,     // Waiting for Accelerometer confirmation
    SENSOR_VALIDATION_COMPLETE,          // Both sensors validated
    SENSOR_VALIDATION_FAILED             // Validation failed
} sensor_validation_state_t;

/**
 * @brief Coasting data for automatic mode
 */
typedef struct {
    bool calibrated;                    // Has coasting been measured?
    float coasting_distance_m;          // Distance traveled during coast from 5 m/s
    float coast_start_distance_m;       // Distance from wire end to start coasting
    uint32_t coast_time_ms;             // Time taken to coast to stop
    float decel_rate_ms2;               // Measured deceleration rate
} coasting_data_t;

/**
 * @brief Wire learning results
 */
typedef struct {
    bool complete;                      // Wire learning completed successfully
    float wire_length_m;                // Measured wire length
    float optimal_learning_speed_ms;    // Best speed for learning
    float optimal_cruise_speed_ms;      // Best speed for cruising
    uint32_t forward_rotations;         // Rotations in forward direction
    uint32_t reverse_rotations;         // Rotations in reverse direction
    uint32_t learning_time_ms;          // Time taken for learning
} wire_learning_results_t;

/**
 * @brief System mode status (complete state)
 */
typedef struct {
    // Current mode state
    trolley_operation_mode_t current_mode;
    trolley_operation_mode_t previous_mode;
    uint64_t mode_start_time;
    
    // Mode availability
    mode_availability_t wire_learning_availability;
    mode_availability_t automatic_availability;
    mode_availability_t manual_availability;
    
    // Sensor validation
    sensor_validation_state_t sensor_validation_state;
    bool sensors_validated;
    bool hall_validation_complete;
    bool accel_validation_complete;
    
    // Mode-specific data
    wire_learning_results_t wire_learning;
    coasting_data_t coasting_data;
    
    // Automatic mode data
    uint32_t auto_cycle_count;
    bool auto_cycle_interrupted;
    bool auto_coasting_calibrated;
    
    // Status messages
    char current_mode_status[128];
    char sensor_validation_message[128];
    char error_message[128];
    
    // System health
    bool system_healthy;
    uint32_t error_count;
    uint64_t last_error_time;
} system_mode_status_t;

// ═══════════════════════════════════════════════════════════════════════════════
// CORE MODE COORDINATION API
// ═══════════════════════════════════════════════════════════════════════════════

/**
 * @brief Initialize the mode coordination system
 * @return ESP_OK on success, error code on failure
 */
esp_err_t mode_coordinator_init(void);

/**
 * @brief Get current system mode status
 * @return Complete system mode status structure
 */
system_mode_status_t mode_coordinator_get_status(void);

/**
 * @brief Get current active mode
 * @return Current trolley operation mode
 */
trolley_operation_mode_t mode_coordinator_get_current_mode(void);

/**
 * @brief Check if a specific mode is available
 * @param mode Mode to check
 * @return true if available, false if blocked
 */
bool mode_coordinator_is_mode_available(trolley_operation_mode_t mode);

/**
 * @brief Update mode coordinator (call regularly from main loop)
 * @return ESP_OK on success, error code on system issues
 */
esp_err_t mode_coordinator_update(void);

// ═══════════════════════════════════════════════════════════════════════════════
// SENSOR VALIDATION API
// ═══════════════════════════════════════════════════════════════════════════════

/**
 * @brief Start sensor validation process
 * @return ESP_OK on success
 */
esp_err_t mode_coordinator_start_sensor_validation(void);

/**
 * @brief Confirm Hall sensor validation (user confirms wheel rotation)
 * @return ESP_OK on success, ESP_ERR_INVALID_STATE if not ready
 */
esp_err_t mode_coordinator_confirm_hall_validation(void);

/**
 * @brief Confirm accelerometer validation (user confirms trolley shake)
 * @return ESP_OK on success, ESP_ERR_INVALID_STATE if not ready
 */
esp_err_t mode_coordinator_confirm_accel_validation(void);

/**
 * @brief Reset sensor validation (start over)
 * @return ESP_OK on success
 */
esp_err_t mode_coordinator_reset_sensor_validation(void);

/**
 * @brief Check if sensors are validated
 * @return true if both sensors validated, false otherwise
 */
bool mode_coordinator_are_sensors_validated(void);

/**
 * @brief Get sensor validation message for UI
 * @return Pointer to current validation message string
 */
const char* mode_coordinator_get_sensor_validation_message(void);

// ═══════════════════════════════════════════════════════════════════════════════
// MODE CONTROL API
// ═══════════════════════════════════════════════════════════════════════════════

/**
 * @brief Activate wire learning mode
 * @return ESP_OK on success, error code if blocked or failed
 */
esp_err_t mode_coordinator_activate_wire_learning(void);

/**
 * @brief Activate automatic mode  
 * @return ESP_OK on success, error code if blocked or failed
 */
esp_err_t mode_coordinator_activate_automatic(void);

/**
 * @brief Activate manual mode
 * @return ESP_OK on success, error code if blocked or failed
 */
esp_err_t mode_coordinator_activate_manual(void);

/**
 * @brief Stop current mode
 * @param immediate true for immediate stop, false for graceful stop
 * @return ESP_OK on success
 */
esp_err_t mode_coordinator_stop_current_mode(bool immediate);

/**
 * @brief Emergency stop all modes and return to safe state
 * @return ESP_OK on success
 */
esp_err_t mode_coordinator_emergency_stop(void);

/**
 * @brief Reset entire system (clear all mode data)
 * @return ESP_OK on success
 */
esp_err_t mode_coordinator_reset_system(void);

// ═══════════════════════════════════════════════════════════════════════════════
// MODE DATA MANAGEMENT
// ═══════════════════════════════════════════════════════════════════════════════

/**
 * @brief Set wire learning results (called by wire_learning_mode)
 * @param results Wire learning results structure
 * @return ESP_OK on success
 */
esp_err_t mode_coordinator_set_wire_learning_results(const wire_learning_results_t* results);

/**
 * @brief Get wire learning results
 * @return Pointer to wire learning results, NULL if not available
 */
const wire_learning_results_t* mode_coordinator_get_wire_learning_results(void);

/**
 * @brief Set coasting calibration data (called by automatic_mode)
 * @param coasting_data Coasting calibration results
 * @return ESP_OK on success
 */
esp_err_t mode_coordinator_set_coasting_data(const coasting_data_t* coasting_data);

/**
 * @brief Get coasting calibration data
 * @return Pointer to coasting data, NULL if not available
 */
const coasting_data_t* mode_coordinator_get_coasting_data(void);

/**
 * @brief Update automatic mode cycle count
 * @param cycle_count Current cycle count
 * @return ESP_OK on success
 */
esp_err_t mode_coordinator_update_cycle_count(uint32_t cycle_count);

/**
 * @brief Set automatic mode interruption flag
 * @param interrupted true if user interrupted, false otherwise
 * @return ESP_OK on success
 */
esp_err_t mode_coordinator_set_auto_interrupted(bool interrupted);

// ═══════════════════════════════════════════════════════════════════════════════
// ERROR HANDLING AND SAFETY
// ═══════════════════════════════════════════════════════════════════════════════

/**
 * @brief Report system error to mode coordinator
 * @param error_message Error description
 * @return ESP_OK on success
 */
esp_err_t mode_coordinator_report_error(const char* error_message);

/**
 * @brief Check if system is healthy for mode operation
 * @return true if healthy, false if errors detected
 */
bool mode_coordinator_is_system_healthy(void);

/**
 * @brief Get current error message
 * @return Pointer to current error message, empty string if no error
 */
const char* mode_coordinator_get_error_message(void);

/**
 * @brief Clear current error condition
 * @return ESP_OK on success
 */
esp_err_t mode_coordinator_clear_error(void);

/**
 * @brief Check motion safety before allowing mode operations
 * @return true if safe for motion, false if blocked
 */
bool mode_coordinator_is_motion_safe(void);

// ═══════════════════════════════════════════════════════════════════════════════