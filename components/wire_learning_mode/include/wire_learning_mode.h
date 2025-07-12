#ifndef WIRE_LEARNING_MODE_H
#define WIRE_LEARNING_MODE_H

#include "esp_err.h"
#include <stdint.h>
#include <stdbool.h>

// ═══════════════════════════════════════════════════════════════════════════════
// WIRE_LEARNING_MODE.H - MODE 1: WIRE LEARNING IMPLEMENTATION
// ═══════════════════════════════════════════════════════════════════════════════
// 
// SINGLE RESPONSIBILITY: Wire learning mode implementation
// - wire_learning_mode_start() / wire_learning_mode_stop()
// - wire_learning_mode_update()
// - Wire length calculation
// - Optimal speed finding  
// - Wire end detection algorithms
// 
// Uses: hardware_control (for motor commands), sensor_health (for validation)
// Called by: mode_coordinator
// ═══════════════════════════════════════════════════════════════════════════════

// ═══════════════════════════════════════════════════════════════════════════════
// WIRE LEARNING CONFIGURATION
// ═══════════════════════════════════════════════════════════════════════════════

// Learning speed progression
#define WIRE_LEARNING_START_SPEED_MS    0.1f      // Starting speed
#define WIRE_LEARNING_MAX_SPEED_MS      1.0f      // Maximum learning speed
#define WIRE_LEARNING_SPEED_INCREMENT   0.1f      // Speed increment steps
#define WIRE_LEARNING_TIMEOUT_S         60        // Total learning timeout

// Wire validation parameters
#define WIRE_LENGTH_TOLERANCE_PERCENT   5.0f      // Acceptable difference between directions
#define MIN_WIRE_LENGTH_M              2.0f       // Minimum expected wire length
#define MAX_WIRE_LENGTH_M              2000.0f    // Maximum expected wire length

// Wire end detection thresholds
#define WIRE_END_IMPACT_THRESHOLD_G    1.0f       // Impact detection threshold
#define WIRE_END_HALL_TIMEOUT_MS       2000       // Hall timeout for wire end
#define WIRE_END_SPEED_DROP_PERCENT    70.0f      // Speed drop percentage for detection

// Learning validation
#define LEARNING_MIN_HALL_PULSES       10         // Minimum pulses for valid movement
#define LEARNING_HALL_TIMEOUT_MS       3000       // Hall timeout during learning
#define LEARNING_DIRECTION_PAUSE_MS    2000       // Pause between direction changes

// ═══════════════════════════════════════════════════════════════════════════════
// WIRE LEARNING STATE AND DATA STRUCTURES
// ═══════════════════════════════════════════════════════════════════════════════

/**
 * @brief Wire learning state machine
 */
typedef enum {
    WIRE_LEARNING_IDLE = 0,             // Not active
    WIRE_LEARNING_INITIALIZING,         // Starting up and validating
    WIRE_LEARNING_FORWARD_DIRECTION,    // Learning forward direction
    WIRE_LEARNING_DIRECTION_PAUSE,      // Pausing between directions
    WIRE_LEARNING_REVERSE_DIRECTION,    // Learning reverse direction
    WIRE_LEARNING_CALCULATING_RESULTS,  // Processing results
    WIRE_LEARNING_COMPLETE,             // Successfully completed
    WIRE_LEARNING_FAILED,               // Failed or error
    WIRE_LEARNING_STOPPING              // Graceful shutdown
} wire_learning_state_t;

/**
 * @brief Wire end detection method
 */
typedef enum {
    WIRE_END_NONE = 0,                  // No wire end detected
    WIRE_END_IMPACT_DETECTED,           // Detected by accelerometer impact
    WIRE_END_HALL_TIMEOUT,              // Detected by Hall sensor timeout
    WIRE_END_SPEED_DROP,                // Detected by speed drop
    WIRE_END_USER_STOP                  // User-initiated stop
} wire_end_detection_method_t;

/**
 * @brief Wire learning progress data
 */
typedef struct {
    // Current state
    wire_learning_state_t state;
    uint64_t state_start_time;
    uint64_t learning_start_time;
    
    // Current direction data
    bool current_direction_forward;
    uint32_t direction_start_rotations;
    uint64_t direction_start_time;
    float current_learning_speed;
    
    // Forward direction results
    uint32_t forward_rotations;
    float forward_distance_m;
    uint32_t forward_time_ms;
    wire_end_detection_method_t forward_end_method;
    
    // Reverse direction results
    uint32_t reverse_rotations;
    float reverse_distance_m;
    uint32_t reverse_time_ms;
    wire_end_detection_method_t reverse_end_method;
    
    // Final results
    float calculated_wire_length_m;
    float length_difference_percent;
    float optimal_speed_ms;
    bool learning_successful;
    
    // Status and error tracking
    char status_message[128];
    char error_message[128];
    uint32_t error_count;
} wire_learning_progress_t;

/**
 * @brief Wire learning results (final output)
 */
typedef struct {
    bool complete;                      // Learning completed successfully
    float wire_length_m;                // Final measured wire length
    float optimal_learning_speed_ms;    // Best speed found during learning
    float optimal_cruise_speed_ms;      // Recommended cruise speed
    uint32_t forward_rotations;         // Forward direction rotations
    uint32_t reverse_rotations;         // Reverse direction rotations
    uint32_t total_learning_time_ms;    // Total time for learning
    wire_end_detection_method_t primary_detection_method; // Most reliable detection method
    float learning_accuracy_percent;    // Accuracy of learning (100% = perfect match)
} wire_learning_results_t;

// ═══════════════════════════════════════════════════════════════════════════════
// CORE WIRE LEARNING API
// ═══════════════════════════════════════════════════════════════════════════════

/**
 * @brief Initialize wire learning mode system
 * @return ESP_OK on success, error code on failure
 */
esp_err_t wire_learning_mode_init(void);

/**
 * @brief Start wire learning process
 * @return ESP_OK on success, error code if cannot start
 */
esp_err_t wire_learning_mode_start(void);

/**
 * @brief Stop wire learning process
 * @param immediate true for immediate stop, false for graceful stop
 * @return ESP_OK on success
 */
esp_err_t wire_learning_mode_stop(bool immediate);

/**
 * @brief Update wire learning process (call regularly)
 * @return ESP_OK on success, error code on failure
 */
esp_err_t wire_learning_mode_update(void);

/**
 * @brief Check if wire learning is active
 * @return true if learning in progress, false otherwise
 */
bool wire_learning_mode_is_active(void);

/**
 * @brief Check if wire learning is complete
 * @return true if successfully completed, false otherwise
 */
bool wire_learning_mode_is_complete(void);

/**
 * @brief Get current learning progress
 * @return Current progress data structure
 */
wire_learning_progress_t wire_learning_mode_get_progress(void);

/**
 * @brief Get final learning results
 * @return Learning results structure, valid only if complete
 */
wire_learning_results_t wire_learning_mode_get_results(void);

// ═══════════════════════════════════════════════════════════════════════════════
// WIRE END DETECTION API
// ═══════════════════════════════════════════════════════════════════════════════

/**
 * @brief Check for wire end using accelerometer impact
 * @return true if impact detected, false otherwise
 */
bool wire_learning_detect_impact(void);

/**
 * @brief Check for wire end using Hall sensor timeout
 * @return true if Hall timeout detected, false otherwise
 */
bool wire_learning_detect_hall_timeout(void);

/**
 * @brief Check for wire end using speed drop detection
 * @return true if speed drop detected, false otherwise
 */
bool wire_learning_detect_speed_drop(void);

/**
 * @brief Get best wire end detection method for current conditions
 * @return Most reliable detection method
 */
wire_end_detection_method_t wire_learning_get_best_detection_method(void);

/**
 * @brief Reset wire end detection algorithms
 * @return ESP_OK on success
 */
esp_err_t wire_learning_reset_detection(void);

// ═══════════════════════════════════════════════════════════════════════════════
// SPEED OPTIMIZATION API
// ═══════════════════════════════════════════════════════════════════════════════

/**
 * @brief Find optimal learning speed for current wire
 * @param min_speed Minimum speed to test
 * @param max_speed Maximum speed to test
 * @return Optimal speed in m/s, 0.0 if failed
 */
float wire_learning_find_optimal_speed(float min_speed, float max_speed);

/**
 * @brief Test movement at specific speed
 * @param speed Speed to test in m/s
 * @param duration_ms Test duration in milliseconds
 * @return true if movement successful, false if failed
 */
bool wire_learning_test_speed(float speed, uint32_t duration_ms);

/**
 * @brief Calculate recommended cruise speed from learning results
 * @param learning_speed Speed used during learning
 * @return Recommended cruise speed for automatic mode
 */
float wire_learning_calculate_cruise_speed(float learning_speed);

// ═══════════════════════════════════════════════════════════════════════════════
// VALIDATION AND SAFETY API
// ═══════════════════════════════════════════════════════════════════════════════

/**
 * @brief Validate learning prerequisites (sensors, ESC, etc.)
 * @return ESP_OK if ready, error code if not ready
 */
esp_err_t wire_learning_validate_prerequisites(void);

/**
 * @brief Validate wire learning results for consistency
 * @param results Results to validate
 * @return true if results are consistent and valid
 */
bool wire_learning_validate_results(const wire_learning_results_t* results);

/**
 * @brief Check if current position is safe for learning
 * @return true if safe, false if unsafe
 */
bool wire_learning_is_position_safe(void);

/**
 * @brief Emergency stop during learning (called by safety systems)
 * @return ESP_OK on success
 */
esp_err_t wire_learning_emergency_stop(void);

// ═══════════════════════════════════════════════════════════════════════════════
// DATA PERSISTENCE API
// ═══════════════════════════════════════════════════════════════════════════════

/**
 * @brief Save learning results to non-volatile storage
 * @param results Results to save
 * @return ESP_OK on success, error code on failure
 */
esp_err_t wire_learning_save_results(const wire_learning_results_t* results);

/**
 * @brief Load previous learning results from non-volatile storage
 * @param results Buffer to load results into
 * @return ESP_OK if results loaded, ESP_ERR_NOT_FOUND if no saved results
 */
esp_err_t wire_learning_load_results(wire_learning_results_t* results);

/**
 * @brief Clear saved learning results
 * @return ESP_OK on success
 */
esp_err_t wire_learning_clear_saved_results(void);

/**
 * @brief Check if saved results exist
 * @return true if saved results available, false otherwise
 */
bool wire_learning_has_saved_results(void);

// ═══════════════════════════════════════════════════════════════════════════════
// UTILITY AND STATUS API
// ═══════════════════════════════════════════════════════════════════════════════

/**
 * @brief Get current learning status message
 * @return Pointer to status message string
 */
const char* wire_learning_get_status_message(void);

/**
 * @brief Get current error message
 * @return Pointer to error message string, empty if no error
 */
const char* wire_learning_get_error_message(void);

/**
 * @brief Convert wire learning state to string
 * @param state Wire learning state
 * @return String representation of state
 */
const char* wire_learning_state_to_string(wire_learning_state_t state);

/**
 * @brief Convert detection method to string
 * @param method Wire end detection method
 * @return String representation of detection method
 */
const char* wire_learning_detection_method_to_string(wire_end_detection_method_t method);

/**
 * @brief Get learning progress percentage
 * @return Progress percentage (0-100), or -1 if not active
 */
int wire_learning_get_progress_percentage(void);

/**
 * @brief Get estimated time remaining for learning
 * @return Estimated time in milliseconds, 0 if unknown
 */
uint32_t wire_learning_get_estimated_time_remaining(void);

/**
 * @brief Reset wire learning mode (clear all data)
 * @return ESP_OK on success
 */
esp_err_t wire_learning_mode_reset(void);

#endif // WIRE_LEARNING_MODE_H