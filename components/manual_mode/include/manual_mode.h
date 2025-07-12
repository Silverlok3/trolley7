#ifndef MANUAL_MODE_H
#define MANUAL_MODE_H

#include "esp_err.h"
#include <stdint.h>
#include <stdbool.h>

// ═══════════════════════════════════════════════════════════════════════════════
// MANUAL_MODE.H - MODE 3: MANUAL CONTROL IMPLEMENTATION
// ═══════════════════════════════════════════════════════════════════════════════
// 
// SINGLE RESPONSIBILITY: Manual control mode implementation
// - manual_mode_start() / manual_mode_stop()
// - manual_mode_set_speed() with validation
// - manual_mode_arm_esc() / manual_mode_disarm_esc()
// - User command validation and safety checks
// 
// Uses: hardware_control (for direct motor commands)
// Called by: mode_coordinator
// ═══════════════════════════════════════════════════════════════════════════════

// ═══════════════════════════════════════════════════════════════════════════════
// MANUAL MODE CONFIGURATION
// ═══════════════════════════════════════════════════════════════════════════════

// Speed control parameters
#define MANUAL_MODE_MAX_SPEED_MS        2.0f      // Maximum allowed manual speed
#define MANUAL_MODE_MIN_SPEED_MS        0.05f     // Minimum speed step
#define MANUAL_MODE_SPEED_INCREMENT     0.1f      // Default speed increment
#define MANUAL_MODE_DEFAULT_SPEED_MS    0.5f      // Default speed for forward/backward

// Safety parameters
#define MANUAL_MODE_MAX_ACCEL_MS2       1.0f      // Maximum acceleration rate
#define MANUAL_MODE_EMERGENCY_STOP_MS   100       // Emergency stop response time
#define MANUAL_MODE_COMMAND_TIMEOUT_MS  5000      // Command timeout (safety)
#define MANUAL_MODE_MAX_IMPACT_G        0.8f      // Maximum allowed impact

// ESC control parameters
#define MANUAL_ESC_ARM_TIMEOUT_MS       5000      // ESC arming timeout
#define MANUAL_ESC_RESPONSE_TIMEOUT_MS  2000      // ESC response timeout
#define MANUAL_ESC_DISARM_DELAY_MS      1000      // Delay before disarming

// User command validation
#define MANUAL_MAX_COMMANDS_PER_SEC     10        // Rate limiting for commands
#define MANUAL_COMMAND_HISTORY_SIZE     20        // Command history for analysis

// ═══════════════════════════════════════════════════════════════════════════════
// MANUAL MODE STATE AND DATA STRUCTURES
// ═══════════════════════════════════════════════════════════════════════════════

/**
 * @brief Manual mode state machine
 */
typedef enum {
    MANUAL_MODE_IDLE = 0,               // Not active
    MANUAL_MODE_INITIALIZING,           // Starting up and validating
    MANUAL_MODE_READY,                  // Ready for commands (ESC disarmed)
    MANUAL_MODE_ESC_ARMING,             // Arming ESC process
    MANUAL_MODE_ACTIVE,                 // Active with ESC armed
    MANUAL_MODE_MOVING_FORWARD,         // Currently moving forward
    MANUAL_MODE_MOVING_BACKWARD,        // Currently moving backward
    MANUAL_MODE_STOPPING,               // Stopping motion
    MANUAL_MODE_ESC_DISARMING,          // Disarming ESC process
    MANUAL_MODE_ERROR,                  // Error condition
    MANUAL_MODE_EMERGENCY_STOP          // Emergency stop activated
} manual_mode_state_t;

/**
 * @brief Manual command types
 */
typedef enum {
    MANUAL_CMD_NONE = 0,                // No command
    MANUAL_CMD_SET_SPEED,               // Set speed command
    MANUAL_CMD_FORWARD,                 // Move forward command
    MANUAL_CMD_BACKWARD,                // Move backward command
    MANUAL_CMD_STOP,                    // Stop movement command
    MANUAL_CMD_ARM_ESC,                 // Arm ESC command
    MANUAL_CMD_DISARM_ESC,              // Disarm ESC command
    MANUAL_CMD_EMERGENCY_STOP,          // Emergency stop command
    MANUAL_CMD_INCREASE_SPEED,          // Increase speed command
    MANUAL_CMD_DECREASE_SPEED           // Decrease speed command
} manual_command_type_t;

/**
 * @brief Manual command structure
 */
typedef struct {
    manual_command_type_t type;         // Command type
    float speed_parameter;              // Speed parameter (if applicable)
    bool direction_forward;             // Direction parameter (if applicable)
    uint64_t timestamp;                 // Command timestamp
    bool validated;                     // Command validation status
    char source[32];                    // Command source (web, serial, etc.)
} manual_command_t;

/**
 * @brief Manual mode status data
 */
typedef struct {
    // Current state
    manual_mode_state_t state;
    uint64_t state_start_time;
    uint64_t mode_start_time;
    
    // Current motion parameters
    float current_speed_ms;             // Current actual speed
    float target_speed_ms;              // Current target speed
    bool direction_forward;             // Current direction
    bool motor_active;                  // Motor currently active
    
    // ESC status
    bool esc_armed;                     // ESC armed status
    bool esc_responding;                // ESC responding to commands
    uint64_t esc_arm_time;              // Time when ESC was armed
    
    // Command tracking
    manual_command_t last_command;      // Last executed command
    uint32_t command_count;             // Total commands executed
    uint64_t last_command_time;         // Time of last command
    
    // Safety monitoring
    float max_speed_reached;            // Maximum speed reached in session
    float total_distance_traveled;      // Total distance in session
    uint32_t safety_violations;         // Number of safety violations
    
    // Status and error tracking
    char status_message[128];
    char error_message[128];
    uint32_t error_count;
} manual_mode_status_t;

/**
 * @brief Manual mode session statistics
 */
typedef struct {
    uint32_t total_commands_executed;   // Total commands in session
    uint32_t forward_commands;          // Forward movement commands
    uint32_t backward_commands;         // Backward movement commands
    uint32_t speed_changes;             // Speed change commands
    uint32_t esc_arm_disarm_cycles;     // ESC arm/disarm cycles
    float max_speed_used;               // Maximum speed used
    float total_distance_traveled;      // Total distance traveled
    uint32_t session_duration_ms;       // Total session time
    uint32_t motor_active_time_ms;      // Time motor was active
    float average_speed;                // Average speed when moving
} manual_mode_session_stats_t;

// ═══════════════════════════════════════════════════════════════════════════════
// CORE MANUAL MODE API
// ═══════════════════════════════════════════════════════════════════════════════

/**
 * @brief Initialize manual mode system
 * @return ESP_OK on success, error code on failure
 */
esp_err_t manual_mode_init(void);

/**
 * @brief Start manual mode (enter manual control)
 * @return ESP_OK on success, error code if cannot start
 */
esp_err_t manual_mode_start(void);

/**
 * @brief Stop manual mode (exit manual control)
 * @return ESP_OK on success
 */
esp_err_t manual_mode_stop(void);

/**
 * @brief Update manual mode process (call regularly)
 * @return ESP_OK on success, error code on failure
 */
esp_err_t manual_mode_update(void);

/**
 * @brief Check if manual mode is active
 * @return true if active, false otherwise
 */
bool manual_mode_is_active(void);

/**
 * @brief Get current manual mode status
 * @return Current status data structure
 */
manual_mode_status_t manual_mode_get_status(void);

/**
 * @brief Get session statistics
 * @return Session statistics structure
 */
manual_mode_session_stats_t manual_mode_get_session_stats(void);

// ═══════════════════════════════════════════════════════════════════════════════
// SPEED CONTROL API
// ═══════════════════════════════════════════════════════════════════════════════

/**
 * @brief Set motor speed and direction
 * @param speed_ms Speed in m/s (0.0 to MANUAL_MODE_MAX_SPEED_MS)
 * @param forward Direction (true = forward, false = reverse)
 * @return ESP_OK on success, error code on failure
 */
esp_err_t manual_mode_set_speed(float speed_ms, bool forward);

/**
 * @brief Move forward at default speed
 * @return ESP_OK on success, error code on failure
 */
esp_err_t manual_mode_move_forward(void);

/**
 * @brief Move backward at default speed
 * @return ESP_OK on success, error code on failure
 */
esp_err_t manual_mode_move_backward(void);

/**
 * @brief Stop motor movement
 * @return ESP_OK on success, error code on failure
 */
esp_err_t manual_mode_stop_movement(void);

/**
 * @brief Increase current speed by increment
 * @return ESP_OK on success, error code if at maximum
 */
esp_err_t manual_mode_increase_speed(void);

/**
 * @brief Decrease current speed by increment
 * @return ESP_OK on success, error code if at minimum
 */
esp_err_t manual_mode_decrease_speed(void);

/**
 * @brief Get current target speed
 * @return Current target speed in m/s
 */
float manual_mode_get_current_speed(void);

/**
 * @brief Get current direction
 * @return true if forward, false if reverse
 */
bool manual_mode_get_current_direction(void);

/**
 * @brief Check if motor is currently moving
 * @return true if moving, false if stopped
 */
bool manual_mode_is_moving(void);

// ═══════════════════════════════════════════════════════════════════════════════
// ESC CONTROL API
// ═══════════════════════════════════════════════════════════════════════════════

/**
 * @brief Manually arm ESC
 * @return ESP_OK on success, error code on failure
 */
esp_err_t manual_mode_arm_esc(void);

/**
 * @brief Manually disarm ESC
 * @return ESP_OK on success, error code on failure
 */
esp_err_t manual_mode_disarm_esc(void);

/**
 * @brief Check if ESC is armed
 * @return true if armed, false if disarmed
 */
bool manual_mode_is_esc_armed(void);

/**
 * @brief Check if ESC is responding to commands
 * @return true if responding, false if not responding
 */
bool manual_mode_is_esc_responding(void);

/**
 * @brief Get time since ESC was armed
 * @return Time in milliseconds, 0 if not armed
 */
uint32_t manual_mode_get_esc_armed_time(void);

// ═══════════════════════════════════════════════════════════════════════════════
// COMMAND VALIDATION AND SAFETY API
// ═══════════════════════════════════════════════════════════════════════════════

/**
 * @brief Validate manual command before execution
 * @param command Command to validate
 * @return true if valid and safe, false if invalid or unsafe
 */
bool manual_mode_validate_command(const manual_command_t* command);

/**
 * @brief Execute validated manual command
 * @param command Command to execute
 * @return ESP_OK on success, error code on failure
 */
esp_err_t manual_mode_execute_command(const manual_command_t* command);

/**
 * @brief Check if speed is within safe limits
 * @param speed_ms Speed to check
 * @return true if safe, false if exceeds limits
 */
bool manual_mode_is_speed_safe(float speed_ms);

/**
 * @brief Check command rate limiting (prevent too many commands)
 * @return true if OK to accept command, false if rate limited
 */
bool manual_mode_check_command_rate_limit(void);

/**
 * @brief Emergency stop (immediate halt)
 * @return ESP_OK on success
 */
esp_err_t manual_mode_emergency_stop(void);

/**
 * @brief Check if operation is safe to continue
 * @return true if safe, false if unsafe conditions detected
 */
bool manual_mode_is_operation_safe(void);

// ═══════════════════════════════════════════════════════════════════════════════
// SENSOR MONITORING API
// ═══════════════════════════════════════════════════════════════════════════════

/**
 * @brief Monitor sensor health during manual operation
 * @return true if sensors healthy, false if issues detected
 */
bool manual_mode_monitor_sensor_health(void);

/**
 * @brief Check for impact detection during manual control
 * @return true if impact detected, false otherwise
 */
bool manual_mode_check_impact_detection(void);

/**
 * @brief Monitor Hall sensor responsiveness
 * @return true if Hall sensor responding, false if timeout
 */
bool manual_mode_monitor_hall_sensor(void);

/**
 * @brief Get current position from manual movements
 * @return Current position in meters (relative to start)
 */
float manual_mode_get_current_position(void);

/**
 * @brief Reset position tracking
 * @return ESP_OK on success
 */
esp_err_t manual_mode_reset_position(void);

// ═══════════════════════════════════════════════════════════════════════════════
// USER INTERFACE SUPPORT API
// ═══════════════════════════════════════════════════════════════════════════════

/**
 * @brief Process user command from web interface
 * @param command_char Single character command
 * @param source Source identifier (e.g., "web", "serial")
 * @return ESP_OK on success, error code on failure
 */
esp_err_t manual_mode_process_user_command(char command_char, const char* source);

/**
 * @brief Get available user commands for current state
 * @param commands_buffer Buffer to write available commands
 * @param buffer_size Size of commands buffer
 * @return ESP_OK on success
 */
esp_err_t manual_mode_get_available_commands(char* commands_buffer, size_t buffer_size);

/**
 * @brief Get command help text
 * @param help_buffer Buffer to write help text
 * @param buffer_size Size of help buffer
 * @return ESP_OK on success
 */
esp_err_t manual_mode_get_command_help(char* help_buffer, size_t buffer_size);

/**
 * @brief Create manual command from parameters
 * @param type Command type
 * @param speed Speed parameter (if applicable)
 * @param forward Direction parameter (if applicable)
 * @param source Command source
 * @return Created command structure
 */
manual_command_t manual_mode_create_command(manual_command_type_t type, 
                                           float speed, 
                                           bool forward, 
                                           const char* source);

// ═══════════════════════════════════════════════════════════════════════════════
// UTILITY AND STATUS API
// ═══════════════════════════════════════════════════════════════════════════════

/**
 * @brief Get current status message
 * @return Pointer to status message string
 */
const char* manual_mode_get_status_message(void);

/**
 * @brief Get current error message
 * @return Pointer to error message string, empty if no error
 */
const char* manual_mode_get_error_message(void);

/**
 * @brief Convert manual mode state to string
 * @param state Manual mode state
 * @return String representation of state
 */
const char* manual_mode_state_to_string(manual_mode_state_t state);

/**
 * @brief Convert command type to string
 * @param type Command type
 * @return String representation of command type
 */
const char* manual_mode_command_type_to_string(manual_command_type_t type);

/**
 * @brief Get detailed status for debugging
 * @param status_buffer Buffer to write status string
 * @param buffer_size Size of status buffer
 * @return ESP_OK on success
 */
esp_err_t manual_mode_get_detailed_status(char* status_buffer, size_t buffer_size);

/**
 * @brief Reset manual mode session (clear statistics, return to ready state)
 * @return ESP_OK on success
 */
esp_err_t manual_mode_reset_session(void);

/**
 * @brief Get session duration
 * @return Session duration in milliseconds
 */
uint32_t manual_mode_get_session_duration(void);

/**
 * @brief Export session data for analysis
 * @param data_buffer Buffer to write session data
 * @param buffer_size Size of data buffer
 * @return ESP_OK on success
 */
esp_err_t manual_mode_export_session_data(char* data_buffer, size_t buffer_size);

#endif // MANUAL_MODE_H