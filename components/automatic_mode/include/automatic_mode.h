#ifndef AUTOMATIC_MODE_H
#define AUTOMATIC_MODE_H

#include "esp_err.h"
#include <stdint.h>
#include <stdbool.h>

// ═══════════════════════════════════════════════════════════════════════════════
// AUTOMATIC_MODE.H - MODE 2: AUTONOMOUS 5 M/S CYCLING IMPLEMENTATION
// ═══════════════════════════════════════════════════════════════════════════════
// 
// SINGLE RESPONSIBILITY: Autonomous cycling mode implementation
// - automatic_mode_start() / automatic_mode_stop() / automatic_mode_interrupt()
// - 5 m/s acceleration profiles
// - Coasting distance calculation and implementation
// - Cycle management and counting
// - Auto-arm/disarm ESC
// 
// Uses: hardware_control (for motor commands), mode_coordinator (for wire data)
// Called by: mode_coordinator
// ═══════════════════════════════════════════════════════════════════════════════

// ═══════════════════════════════════════════════════════════════════════════════
// AUTOMATIC MODE CONFIGURATION
// ═══════════════════════════════════════════════════════════════════════════════

// Speed and acceleration parameters
#define AUTO_MODE_MAX_SPEED_MS          5.0f      // Maximum cruising speed
#define AUTO_MODE_START_SPEED_MS        0.1f      // Starting acceleration speed
#define AUTO_MODE_ACCEL_RATE_MS2        0.5f      // Acceleration rate (m/s²)
#define AUTO_MODE_DECEL_RATE_MS2        0.3f      // Deceleration rate (m/s²)
#define AUTO_MODE_SPEED_INCREMENT       0.1f      // Speed increment steps

// Coasting parameters
#define AUTO_COASTING_CALIBRATION_SPEED 5.0f      // Speed for coasting calibration
#define AUTO_COASTING_SAFETY_MARGIN_M   2.0f      // Safety margin before wire end
#define AUTO_COASTING_MAX_DISTANCE_M    50.0f     // Maximum expected coasting distance
#define AUTO_COASTING_MIN_DISTANCE_M    0.5f      // Minimum expected coasting distance

// Cycle management
#define AUTO_MODE_MAX_CYCLES            1000      // Maximum cycles before pause
#define AUTO_MODE_CYCLE_PAUSE_MS        5000      // Pause between cycles
#define AUTO_MODE_DIRECTION_PAUSE_MS    3000      // Pause when changing direction
#define AUTO_MODE_TIMEOUT_MS            300000    // 5 minute timeout per direction

// Safety parameters
#define AUTO_MODE_WIRE_END_APPROACH_MS  1.0f      // Speed when approaching wire end
#define AUTO_MODE_EMERGENCY_DECEL_MS2   2.0f      // Emergency deceleration rate
#define AUTO_MODE_MAX_IMPACT_G          0.5f      // Maximum allowed impact

// ═══════════════════════════════════════════════════════════════════════════════
// AUTOMATIC MODE STATE AND DATA STRUCTURES
// ═══════════════════════════════════════════════════════════════════════════════

/**
 * @brief Automatic mode state machine
 */
typedef enum {
    AUTO_MODE_IDLE = 0,                 // Not active
    AUTO_MODE_INITIALIZING,             // Starting up and validating
    AUTO_MODE_ARMING_ESC,               // Arming ESC automatically
    AUTO_MODE_ACCELERATING,             // Gradual acceleration to cruise speed
    AUTO_MODE_CRUISING,                 // Cruising at maximum speed
    AUTO_MODE_COASTING_CALIBRATION,     // First run: measuring coasting distance
    AUTO_MODE_COASTING,                 // Coasting to wire end
    AUTO_MODE_WIRE_END_APPROACH,        // Approaching wire end at low speed
    AUTO_MODE_DIRECTION_CHANGE,         // Pausing and changing direction
    AUTO_MODE_CYCLE_COMPLETE,           // Completed one full cycle
    AUTO_MODE_STOPPING_GRACEFUL,        // User requested stop (finish current run)
    AUTO_MODE_STOPPING_INTERRUPTED,     // User interrupted (immediate stop)
    AUTO_MODE_ERROR,                    // Error condition
    AUTO_MODE_COMPLETE                  // All cycles complete
} automatic_mode_state_t;

/**
 * @brief Coasting calibration data
 */
typedef struct {
    bool calibrated;                    // Has coasting been measured?
    float calibration_speed_ms;         // Speed used for calibration
    float coasting_distance_m;          // Measured coasting distance
    uint32_t coasting_time_ms;          // Time taken to coast to stop
    float deceleration_rate_ms2;        // Measured deceleration rate
    float coast_start_distance_m;       // Distance from wire end to start coasting
    uint32_t calibration_rotations;     // Rotations during coasting
    bool calibration_successful;        // Calibration completed successfully
} coasting_calibration_t;

/**
 * @brief Cycle data for tracking
 */
typedef struct {
    uint32_t cycle_number;              // Current cycle number
    uint32_t forward_runs;              // Number of forward runs completed
    uint32_t reverse_runs;              // Number of reverse runs completed
    bool current_direction_forward;     // Current movement direction
    uint64_t cycle_start_time;          // Start time of current cycle
    uint64_t run_start_time;            // Start time of current run
    uint32_t run_start_rotations;       // Rotation count at run start
    float max_speed_achieved;           // Maximum speed reached this run
    uint32_t total_distance_m;          // Total distance traveled
} cycle_data_t;

/**
 * @brief Automatic mode progress data
 */
typedef struct {
    // Current state
    automatic_mode_state_t state;
    uint64_t state_start_time;
    uint64_t mode_start_time;
    
    // Speed control
    float current_target_speed;
    float acceleration_rate;
    bool esc_auto_armed;
    
    // Coasting data
    coasting_calibration_t coasting;
    bool coasting_active;
    uint64_t coasting_start_time;
    uint32_t coasting_start_rotations;
    
    // Cycle tracking
    cycle_data_t cycle_data;
    bool user_interrupted;
    bool finishing_current_run;
    
    // Wire end detection
    float wire_length_m;
    float current_position_m;
    float distance_to_wire_end_m;
    
    // Status and error tracking
    char status_message[128];
    char error_message[128];
    uint32_t error_count;
} automatic_mode_progress_t;

/**
 * @brief Automatic mode results (for reporting)
 */
typedef struct {
    uint32_t total_cycles_completed;    // Total successful cycles
    uint32_t total_runs_completed;      // Total individual runs
    uint32_t total_operating_time_ms;   // Total operating time
    float total_distance_traveled_m;    // Total distance covered
    float average_cycle_time_ms;        // Average time per cycle
    float max_speed_achieved_ms;        // Maximum speed reached
    coasting_calibration_t coasting_data; // Final coasting calibration
    bool interrupted_by_user;           // Whether stopped by user intervention
    char completion_reason[64];         // Reason for stopping
} automatic_mode_results_t;

// ═══════════════════════════════════════════════════════════════════════════════
// CORE AUTOMATIC MODE API
// ═══════════════════════════════════════════════════════════════════════════════

/**
 * @brief Initialize automatic mode system
 * @return ESP_OK on success, error code on failure
 */
esp_err_t automatic_mode_init(void);

/**
 * @brief Start automatic cycling mode
 * @return ESP_OK on success, error code if cannot start
 */
esp_err_t automatic_mode_start(void);

/**
 * @brief Stop automatic mode gracefully (finish current run)
 * @return ESP_OK on success
 */
esp_err_t automatic_mode_stop_graceful(void);

/**
 * @brief Interrupt automatic mode immediately
 * @return ESP_OK on success
 */
esp_err_t automatic_mode_interrupt(void);

/**
 * @brief Update automatic mode process (call regularly)
 * @return ESP_OK on success, error code on failure
 */
esp_err_t automatic_mode_update(void);

/**
 * @brief Check if automatic mode is active
 * @return true if mode active, false otherwise
 */
bool automatic_mode_is_active(void);

/**
 * @brief Check if automatic mode is running (not stopping)
 * @return true if actively running, false if stopping or stopped
 */
bool automatic_mode_is_running(void);

/**
 * @brief Get current automatic mode progress
 * @return Current progress data structure
 */
automatic_mode_progress_t automatic_mode_get_progress(void);

/**
 * @brief Get automatic mode results
 * @return Results structure with statistics
 */
automatic_mode_results_t automatic_mode_get_results(void);

// ═══════════════════════════════════════════════════════════════════════════════
// COASTING CALIBRATION API
// ═══════════════════════════════════════════════════════════════════════════════

/**
 * @brief Start coasting calibration process
 * @return ESP_OK on success, error code on failure
 */
esp_err_t automatic_mode_start_coasting_calibration(void);

/**
 * @brief Check if coasting calibration is complete
 * @return true if calibrated, false if not
 */
bool automatic_mode_is_coasting_calibrated(void);

/**
 * @brief Get coasting calibration data
 * @return Coasting calibration structure
 */
coasting_calibration_t automatic_mode_get_coasting_data(void);

/**
 * @brief Calculate coasting start point based on current position
 * @param current_position Current position in meters
 * @param wire_length Total wire length in meters
 * @param direction_forward Current movement direction
 * @return Distance from current position to start coasting
 */
float automatic_mode_calculate_coasting_distance(float current_position, 
                                                float wire_length, 
                                                bool direction_forward);

/**
 * @brief Update coasting calibration with real-time data
 * @param speed Current speed
 * @param position Current position
 * @return ESP_OK on success
 */
esp_err_t automatic_mode_update_coasting_calibration(float speed, float position);

// ═══════════════════════════════════════════════════════════════════════════════
// SPEED CONTROL API
// ═══════════════════════════════════════════════════════════════════════════════

/**
 * @brief Execute gradual acceleration to target speed
 * @param target_speed Target speed in m/s
 * @return ESP_OK on success, error code on failure
 */
esp_err_t automatic_mode_accelerate_to_speed(float target_speed);

/**
 * @brief Execute controlled deceleration
 * @param target_speed Target speed in m/s (can be 0 for stop)
 * @return ESP_OK on success, error code on failure
 */
esp_err_t automatic_mode_decelerate_to_speed(float target_speed);

/**
 * @brief Maintain current cruise speed
 * @return ESP_OK on success, error code on failure
 */
esp_err_t automatic_mode_maintain_cruise_speed(void);

/**
 * @brief Get current target speed
 * @return Current target speed in m/s
 */
float automatic_mode_get_current_target_speed(void);

/**
 * @brief Check if target speed has been reached
 * @param tolerance Speed tolerance in m/s
 * @return true if at target speed, false otherwise
 */
bool automatic_mode_is_at_target_speed(float tolerance);

// ═══════════════════════════════════════════════════════════════════════════════
// CYCLE MANAGEMENT API
// ═══════════════════════════════════════════════════════════════════════════════

/**
 * @brief Get current cycle count
 * @return Number of completed cycles
 */
uint32_t automatic_mode_get_cycle_count(void);

/**
 * @brief Get current run count (individual forward/reverse runs)
 * @return Number of completed runs
 */
uint32_t automatic_mode_get_run_count(void);

/**
 * @brief Check if current run is in forward direction
 * @return true if moving forward, false if reverse
 */
bool automatic_mode_is_direction_forward(void);

/**
 * @brief Start new cycle
 * @return ESP_OK on success
 */
esp_err_t automatic_mode_start_new_cycle(void);

/**
 * @brief Complete current run and prepare for direction change
 * @return ESP_OK on success
 */
esp_err_t automatic_mode_complete_current_run(void);

/**
 * @brief Change direction for next run
 * @return ESP_OK on success
 */
esp_err_t automatic_mode_change_direction(void);

// ═══════════════════════════════════════════════════════════════════════════════
// WIRE END DETECTION AND POSITIONING API
// ═══════════════════════════════════════════════════════════════════════════════

/**
 * @brief Check if approaching wire end
 * @return true if near wire end, false otherwise
 */
bool automatic_mode_is_approaching_wire_end(void);

/**
 * @brief Calculate distance to wire end
 * @param current_position Current position in meters
 * @param direction_forward Movement direction
 * @return Distance to wire end in meters
 */
float automatic_mode_get_distance_to_wire_end(float current_position, bool direction_forward);

/**
 * @brief Check if at wire end (detected impact or other method)
 * @return true if at wire end, false otherwise
 */
bool automatic_mode_is_at_wire_end(void);

/**
 * @brief Handle wire end detection
 * @return ESP_OK on success
 */
esp_err_t automatic_mode_handle_wire_end_reached(void);

/**
 * @brief Reset position tracking for new run
 * @return ESP_OK on success
 */
esp_err_t automatic_mode_reset_position_tracking(void);

// ═══════════════════════════════════════════════════════════════════════════════
// SAFETY AND VALIDATION API
// ═══════════════════════════════════════════════════════════════════════════════

/**
 * @brief Validate prerequisites for automatic mode
 * @return ESP_OK if ready, error code if not ready
 */
esp_err_t automatic_mode_validate_prerequisites(void);

/**
 * @brief Check if operation is safe to continue
 * @return true if safe, false if unsafe conditions detected
 */
bool automatic_mode_is_operation_safe(void);

/**
 * @brief Handle emergency situation during automatic mode
 * @param error_message Description of emergency
 * @return ESP_OK on success
 */
esp_err_t automatic_mode_handle_emergency(const char* error_message);

/**
 * @brief Check sensor health during operation
 * @return true if sensors healthy, false if issues detected
 */
bool automatic_mode_check_sensor_health(void);

/**
 * @brief Monitor for user interruption
 * @return true if user requested stop, false otherwise
 */
bool automatic_mode_check_user_interruption(void);

// ═══════════════════════════════════════════════════════════════════════════════
// ESC MANAGEMENT API
// ═══════════════════════════════════════════════════════════════════════════════

/**
 * @brief Automatically arm ESC for operation
 * @return ESP_OK on success, error code on failure
 */
esp_err_t automatic_mode_auto_arm_esc(void);

/**
 * @brief Automatically disarm ESC at end of operation
 * @return ESP_OK on success
 */
esp_err_t automatic_mode_auto_disarm_esc(void);

/**
 * @brief Check if ESC is properly armed and responding
 * @return true if ESC ready, false if issues
 */
bool automatic_mode_is_esc_ready(void);

/**
 * @brief Monitor ESC health during operation
 * @return true if ESC healthy, false if issues detected
 */
bool automatic_mode_monitor_esc_health(void);

// ═══════════════════════════════════════════════════════════════════════════════
// DATA PERSISTENCE API
// ═══════════════════════════════════════════════════════════════════════════════

/**
 * @brief Save coasting calibration data to non-volatile storage
 * @param coasting_data Coasting data to save
 * @return ESP_OK on success, error code on failure
 */
esp_err_t automatic_mode_save_coasting_data(const coasting_calibration_t* coasting_data);

/**
 * @brief Load coasting calibration data from non-volatile storage
 * @param coasting_data Buffer to load data into
 * @return ESP_OK if data loaded, ESP_ERR_NOT_FOUND if no saved data
 */
esp_err_t automatic_mode_load_coasting_data(coasting_calibration_t* coasting_data);

/**
 * @brief Save automatic mode statistics
 * @param results Results to save
 * @return ESP_OK on success, error code on failure
 */
esp_err_t automatic_mode_save_results(const automatic_mode_results_t* results);

/**
 * @brief Load previous automatic mode statistics
 * @param results Buffer to load results into
 * @return ESP_OK if results loaded, ESP_ERR_NOT_FOUND if no saved results
 */
esp_err_t automatic_mode_load_results(automatic_mode_results_t* results);

/**
 * @brief Clear all saved automatic mode data
 * @return ESP_OK on success
 */
esp_err_t automatic_mode_clear_saved_data(void);

// ═══════════════════════════════════════════════════════════════════════════════
// UTILITY AND STATUS API
// ═══════════════════════════════════════════════════════════════════════════════

/**
 * @brief Get current status message
 * @return Pointer to status message string
 */
const char* automatic_mode_get_status_message(void);

/**
 * @brief Get current error message
 * @return Pointer to error message string, empty if no error
 */
const char* automatic_mode_get_error_message(void);

/**
 * @brief Convert automatic mode state to string
 * @param state Automatic mode state
 * @return String representation of state
 */
const char* automatic_mode_state_to_string(automatic_mode_state_t state);

/**
 * @brief Get operation progress percentage
 * @return Progress percentage (0-100), or -1 if not applicable
 */
int automatic_mode_get_progress_percentage(void);

/**
 * @brief Get estimated time for current operation
 * @return Estimated time in milliseconds, 0 if unknown
 */
uint32_t automatic_mode_get_estimated_time(void);

/**
 * @brief Get detailed performance statistics
 * @param stats_buffer Buffer to write statistics string
 * @param buffer_size Size of statistics buffer
 * @return ESP_OK on success
 */
esp_err_t automatic_mode_get_performance_stats(char* stats_buffer, size_t buffer_size);

/**
 * @brief Reset automatic mode (clear all data and return to idle)
 * @return ESP_OK on success
 */
esp_err_t automatic_mode_reset(void);

/**
 * @brief Get current operating time
 * @return Operating time in milliseconds since mode start
 */
uint32_t automatic_mode_get_operating_time(void);

/**
 * @brief Calculate average speed for current session
 * @return Average speed in m/s, 0.0 if no data
 */
float automatic_mode_get_average_speed(void);

#endif // AUTOMATIC_MODE_H