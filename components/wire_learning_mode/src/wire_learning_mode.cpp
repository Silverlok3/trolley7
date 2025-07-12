// components/wire_learning_mode/src/wire_learning_mode.cpp
// ═══════════════════════════════════════════════════════════════════════════════
// WIRE_LEARNING_MODE.CPP - MODE 1: WIRE LEARNING IMPLEMENTATION
// ═══════════════════════════════════════════════════════════════════════════════
// 
// SINGLE RESPONSIBILITY: Wire learning mode implementation
// - Wire length calculation through forward/reverse runs
// - Optimal speed finding with gradual progression (0.1→1.0 m/s)
// - Wire end detection using multiple methods (impact, timeout, speed drop)
// - Results validation and persistence
// ═══════════════════════════════════════════════════════════════════════════════

#include "wire_learning_mode.h"
#include "hardware_control.h"
#include "sensor_health.h"
#include "mode_coordinator.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "nvs_flash.h"
#include "nvs.h"
#include <cmath>
#include <cstring>

static const char* TAG = "WIRE_LEARNING";

// ═══════════════════════════════════════════════════════════════════════════════
// GLOBAL WIRE LEARNING STATE
// ═══════════════════════════════════════════════════════════════════════════════

static wire_learning_progress_t g_learning_progress = {0};
static wire_learning_results_t g_learning_results = {0};
static bool g_learning_initialized = false;

// Speed progression tracking
static float g_current_test_speed = WIRE_LEARNING_START_SPEED_MS;
static uint32_t g_hall_pulses_at_speed_start = 0;
static uint64_t g_speed_test_start_time = 0;
static bool g_speed_validated = false;

// Coasting calibration tracking
static bool g_coasting_calibration_active = false;
static uint32_t g_coasting_start_rotations = 0;
static uint64_t g_coasting_start_time = 0;
static float g_coasting_start_speed = 0.0f;

// Wire end detection state
static uint32_t g_consecutive_hall_timeouts = 0;
static float g_speed_history[5] = {0};
static int g_speed_history_index = 0;

// ═══════════════════════════════════════════════════════════════════════════════
// SPEED PROGRESSION AND VALIDATION
// ═══════════════════════════════════════════════════════════════════════════════

static esp_err_t start_speed_test(float speed_ms) {
    ESP_LOGI(TAG, "Testing speed: %.1f m/s", speed_ms);
    
    g_current_test_speed = speed_ms;
    g_speed_validated = false;
    g_hall_pulses_at_speed_start = hardware_get_rotation_count();
    g_speed_test_start_time = esp_timer_get_time();
    
    // Set motor speed using hardware control
    esp_err_t result = hardware_set_motor_speed(speed_ms, g_learning_progress.current_direction_forward);
    if (result != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set motor speed");
        return result;
    }
    
    g_learning_progress.current_learning_speed = speed_ms;
    return ESP_OK;
}

static bool validate_current_speed(void) {
    uint64_t elapsed_time = esp_timer_get_time() - g_speed_test_start_time;
    uint32_t current_rotations = hardware_get_rotation_count();
    uint32_t hall_pulses = current_rotations - g_hall_pulses_at_speed_start;
    
    // Check if we have minimum Hall pulses for validation
    if (hall_pulses >= LEARNING_MIN_HALL_PULSES) {
        g_speed_validated = true;
        ESP_LOGI(TAG, "Speed %.1f m/s validated (%lu pulses in %llu ms)", 
                g_current_test_speed, hall_pulses, elapsed_time / 1000);
        return true;
    }
    
    // Check for timeout
    if (elapsed_time > LEARNING_HALL_TIMEOUT_MS * 1000ULL) {
        ESP_LOGW(TAG, "Speed %.1f m/s failed validation (timeout: %lu pulses in %llu ms)", 
                g_current_test_speed, hall_pulses, elapsed_time / 1000);
        return false;
    }
    
    return false; // Still testing
}

static esp_err_t progress_to_next_speed(void) {
    if (g_current_test_speed >= WIRE_LEARNING_MAX_SPEED_MS) {
        ESP_LOGI(TAG, "Maximum learning speed reached: %.1f m/s", g_current_test_speed);
        return ESP_OK;
    }
    
    float next_speed = g_current_test_speed + WIRE_LEARNING_SPEED_INCREMENT;
    if (next_speed > WIRE_LEARNING_MAX_SPEED_MS) {
        next_speed = WIRE_LEARNING_MAX_SPEED_MS;
    }
    
    // Brief pause between speed changes
    vTaskDelay(pdMS_TO_TICKS(500));
    
    return start_speed_test(next_speed);
}

// ═══════════════════════════════════════════════════════════════════════════════
// WIRE END DETECTION ALGORITHMS
// ═══════════════════════════════════════════════════════════════════════════════

bool wire_learning_detect_impact(void) {
    sensor_health_t sensor_status = sensor_health_get_status();
    
    if (sensor_status.total_accel_g > WIRE_END_IMPACT_THRESHOLD_G) {
        ESP_LOGI(TAG, "Wire end detected by impact: %.2f g", sensor_status.total_accel_g);
        return true;
    }
    
    return false;
}

bool wire_learning_detect_hall_timeout(void) {
    uint64_t time_since_last_pulse = hardware_get_time_since_last_hall_pulse();
    
    if (time_since_last_pulse > (WIRE_END_HALL_TIMEOUT_MS * 1000ULL)) {
        g_consecutive_hall_timeouts++;
        
        if (g_consecutive_hall_timeouts >= 3) { // Multiple consecutive timeouts
            ESP_LOGI(TAG, "Wire end detected by Hall timeout: %llu us since last pulse", 
                    time_since_last_pulse);
            g_consecutive_hall_timeouts = 0;
            return true;
        }
    } else {
        g_consecutive_hall_timeouts = 0;
    }
    
    return false;
}

bool wire_learning_detect_speed_drop(void) {
    float current_speed = hardware_get_current_speed();
    
    // Update speed history
    g_speed_history[g_speed_history_index] = current_speed;
    g_speed_history_index = (g_speed_history_index + 1) % 5;
    
    // Calculate average speed
    float avg_speed = 0;
    for (int i = 0; i < 5; i++) {
        avg_speed += g_speed_history[i];
    }
    avg_speed /= 5.0f;
    
    // Check for significant speed drop
    float speed_drop_threshold = g_current_test_speed * (WIRE_END_SPEED_DROP_PERCENT / 100.0f);
    
    if (g_current_test_speed > 0.2f && avg_speed < speed_drop_threshold) {
        ESP_LOGI(TAG, "Wire end detected by speed drop: target %.1f m/s, actual %.1f m/s", 
                g_current_test_speed, avg_speed);
        return true;
    }
    
    return false;
}

wire_end_detection_method_t wire_learning_get_best_detection_method(void) {
    // Priority order: Impact > Speed Drop > Hall Timeout
    if (wire_learning_detect_impact()) {
        return WIRE_END_IMPACT_DETECTED;
    } else if (wire_learning_detect_speed_drop()) {
        return WIRE_END_SPEED_DROP;
    } else if (wire_learning_detect_hall_timeout()) {
        return WIRE_END_HALL_TIMEOUT;
    }
    
    return WIRE_END_NONE;
}

esp_err_t wire_learning_reset_detection(void) {
    g_consecutive_hall_timeouts = 0;
    g_speed_history_index = 0;
    memset(g_speed_history, 0, sizeof(g_speed_history));
    return ESP_OK;
}

// ═══════════════════════════════════════════════════════════════════════════════
// COASTING CALIBRATION FOR AUTOMATIC MODE
// ═══════════════════════════════════════════════════════════════════════════════

static esp_err_t start_coasting_calibration(void) {
    ESP_LOGI(TAG, "Starting coasting calibration at 5.0 m/s");
    
    g_coasting_calibration_active = true;
    g_coasting_start_rotations = hardware_get_rotation_count();
    g_coasting_start_time = esp_timer_get_time();
    g_coasting_start_speed = 5.0f;
    
    // Accelerate to 5 m/s gradually
    return hardware_set_motor_speed(5.0f, g_learning_progress.current_direction_forward);
}

static esp_err_t process_coasting_calibration(void) {
    if (!g_coasting_calibration_active) return ESP_OK;
    
    float current_speed = hardware_get_current_speed();
    
    // Wait until we reach 5 m/s
    if (current_speed < 4.8f) {
        return ESP_OK; // Still accelerating
    }
    
    // Turn off motor and start coasting
    ESP_LOGI(TAG, "Reached %.1f m/s - starting coast measurement", current_speed);
    hardware_set_motor_speed(0.0f, true); // Stop motor
    
    uint64_t coast_start_time = esp_timer_get_time();
    uint32_t coast_start_rotations = hardware_get_rotation_count();
    
    // Wait for trolley to stop (speed < 0.1 m/s)
    while (hardware_get_current_speed() > 0.1f) {
        vTaskDelay(pdMS_TO_TICKS(100));
        
        // Safety timeout
        if ((esp_timer_get_time() - coast_start_time) > 30000000ULL) { // 30 seconds
            ESP_LOGW(TAG, "Coasting calibration timeout");
            break;
        }
    }
    
    uint64_t coast_end_time = esp_timer_get_time();
    uint32_t coast_end_rotations = hardware_get_rotation_count();
    
    // Calculate coasting data
    coasting_data_t coasting_data = {0};
    coasting_data.calibrated = true;
    coasting_data.coasting_distance_m = hardware_rotations_to_distance(coast_end_rotations - coast_start_rotations);
    coasting_data.coast_time_ms = (coast_end_time - coast_start_time) / 1000;
    coasting_data.decel_rate_ms2 = g_coasting_start_speed / (coasting_data.coast_time_ms / 1000.0f);
    coasting_data.coast_start_distance_m = coasting_data.coasting_distance_m + 2.0f; // Safety margin
    
    // Save coasting data to mode coordinator
    mode_coordinator_set_coasting_data(&coasting_data);
    
    ESP_LOGI(TAG, "Coasting calibration complete:");
    ESP_LOGI(TAG, "  Distance: %.2f m", coasting_data.coasting_distance_m);
    ESP_LOGI(TAG, "  Time: %lu ms", coasting_data.coast_time_ms);
    ESP_LOGI(TAG, "  Deceleration: %.2f m/s²", coasting_data.decel_rate_ms2);
    ESP_LOGI(TAG, "  Coast start distance: %.2f m", coasting_data.coast_start_distance_m);
    
    g_coasting_calibration_active = false;
    return ESP_OK;
}

// ═══════════════════════════════════════════════════════════════════════════════
// MAIN WIRE LEARNING STATE MACHINE
// ═══════════════════════════════════════════════════════════════════════════════

static esp_err_t handle_forward_direction(void) {
    // Start with lowest speed if not already testing
    if (!g_speed_validated && g_current_test_speed == 0.0f) {
        start_speed_test(WIRE_LEARNING_START_SPEED_MS);
        return ESP_OK;
    }
    
    // Validate current speed
    if (!g_speed_validated) {
        if (validate_current_speed()) {
            // Speed validated, progress to next speed
            progress_to_next_speed();
        } else {
            // Still validating or failed - check for wire end
            wire_end_detection_method_t detection = wire_learning_get_best_detection_method();
            if (detection != WIRE_END_NONE) {
                // Wire end detected
                g_learning_progress.forward_rotations = hardware_get_rotation_count() - g_learning_progress.direction_start_rotations;
                g_learning_progress.forward_distance_m = hardware_rotations_to_distance(g_learning_progress.forward_rotations);
                g_learning_progress.forward_time_ms = (esp_timer_get_time() - g_learning_progress.direction_start_time) / 1000;
                g_learning_progress.forward_end_method = detection;
                
                ESP_LOGI(TAG, "Forward direction complete: %.2f m (%lu rotations)", 
                        g_learning_progress.forward_distance_m, g_learning_progress.forward_rotations);
                
                // Validate wire length
                if (g_learning_progress.forward_distance_m < MIN_WIRE_LENGTH_M || 
                    g_learning_progress.forward_distance_m > MAX_WIRE_LENGTH_M) {
                    strcpy(g_learning_progress.error_message, "Wire length out of valid range");
                    g_learning_progress.state = WIRE_LEARNING_FAILED;
                    return ESP_ERR_INVALID_SIZE;
                }
                
                // Start coasting calibration if needed
                if (g_current_test_speed >= 4.0f) {
                    start_coasting_calibration();
                    process_coasting_calibration();
                }
                
                // Prepare for reverse direction
                hardware_emergency_stop();
                vTaskDelay(pdMS_TO_TICKS(LEARNING_DIRECTION_PAUSE_MS));
                
                g_learning_progress.state = WIRE_LEARNING_REVERSE_DIRECTION;
                g_learning_progress.current_direction_forward = false;
                g_learning_progress.direction_start_rotations = hardware_get_rotation_count();
                g_learning_progress.direction_start_time = esp_timer_get_time();
                g_current_test_speed = 0.0f;
                g_speed_validated = false;
                wire_learning_reset_detection();
            }
        }
    }
    
    return ESP_OK;
}

static esp_err_t handle_reverse_direction(void) {
    // Same logic as forward direction but in reverse
    if (!g_speed_validated && g_current_test_speed == 0.0f) {
        start_speed_test(WIRE_LEARNING_START_SPEED_MS);
        return ESP_OK;
    }
    
    if (!g_speed_validated) {
        if (validate_current_speed()) {
            progress_to_next_speed();
        } else {
            wire_end_detection_method_t detection = wire_learning_get_best_detection_method();
            if (detection != WIRE_END_NONE) {
                // Reverse direction complete
                g_learning_progress.reverse_rotations = hardware_get_rotation_count() - g_learning_progress.direction_start_rotations;
                g_learning_progress.reverse_distance_m = hardware_rotations_to_distance(g_learning_progress.reverse_rotations);
                g_learning_progress.reverse_time_ms = (esp_timer_get_time() - g_learning_progress.direction_start_time) / 1000;
                g_learning_progress.reverse_end_method = detection;
                
                ESP_LOGI(TAG, "Reverse direction complete: %.2f m (%lu rotations)", 
                        g_learning_progress.reverse_distance_m, g_learning_progress.reverse_rotations);
                
                // Calculate final results
                g_learning_progress.state = WIRE_LEARNING_CALCULATING_RESULTS;
            }
        }
    }
    
    return ESP_OK;
}

static esp_err_t calculate_final_results(void) {
    ESP_LOGI(TAG, "Calculating wire learning results...");
    
    // Calculate average wire length
    g_learning_progress.calculated_wire_length_m = 
        (g_learning_progress.forward_distance_m + g_learning_progress.reverse_distance_m) / 2.0f;
    
    // Calculate accuracy (difference percentage)
    float difference = fabs(g_learning_progress.forward_distance_m - g_learning_progress.reverse_distance_m);
    g_learning_progress.length_difference_percent = (difference / g_learning_progress.calculated_wire_length_m) * 100.0f;
    
    // Determine if learning was successful
    bool success = g_learning_progress.length_difference_percent <= WIRE_LENGTH_TOLERANCE_PERCENT;
    
    if (success) {
        // Create final results
        g_learning_results.complete = true;
        g_learning_results.wire_length_m = g_learning_progress.calculated_wire_length_m;
        g_learning_results.optimal_learning_speed_ms = g_current_test_speed;
        g_learning_results.optimal_cruise_speed_ms = fmin(g_current_test_speed * 1.5f, 5.0f);
        g_learning_results.forward_rotations = g_learning_progress.forward_rotations;
        g_learning_results.reverse_rotations = g_learning_progress.reverse_rotations;
        g_learning_results.total_learning_time_ms = (esp_timer_get_time() - g_learning_progress.learning_start_time) / 1000;
        g_learning_results.primary_detection_method = g_learning_progress.forward_end_method;
        g_learning_results.learning_accuracy_percent = 100.0f - g_learning_progress.length_difference_percent;
        
        // Save results to mode coordinator
        mode_coordinator_set_wire_learning_results(&g_learning_results);
        
        g_learning_progress.learning_successful = true;
        g_learning_progress.state = WIRE_LEARNING_COMPLETE;
        
        strcpy(g_learning_progress.status_message, "Wire learning completed successfully");
        
        ESP_LOGI(TAG, "=== WIRE LEARNING SUCCESSFUL ===");
        ESP_LOGI(TAG, "Wire Length: %.2f m", g_learning_results.wire_length_m);
        ESP_LOGI(TAG, "Optimal Speed: %.1f m/s", g_learning_results.optimal_learning_speed_ms);
        ESP_LOGI(TAG, "Cruise Speed: %.1f m/s", g_learning_results.optimal_cruise_speed_ms);
        ESP_LOGI(TAG, "Accuracy: %.1f%%", g_learning_results.learning_accuracy_percent);
        
    } else {
        g_learning_progress.learning_successful = false;
        g_learning_progress.state = WIRE_LEARNING_FAILED;
        
        snprintf(g_learning_progress.error_message, sizeof(g_learning_progress.error_message),
                "Wire learning failed: %.1f%% difference > %.1f%% tolerance",
                g_learning_progress.length_difference_percent, WIRE_LENGTH_TOLERANCE_PERCENT);
        
        ESP_LOGE(TAG, "Wire learning failed: Forward %.2f m, Reverse %.2f m (%.1f%% difference)",
                g_learning_progress.forward_distance_m, g_learning_progress.reverse_distance_m,
                g_learning_progress.length_difference_percent);
    }
    
    // Stop motor
    hardware_emergency_stop();
    
    return success ? ESP_OK : ESP_FAIL;
}

// ═══════════════════════════════════════════════════════════════════════════════
// PUBLIC API IMPLEMENTATION
// ═══════════════════════════════════════════════════════════════════════════════

esp_err_t wire_learning_mode_init(void) {
    ESP_LOGI(TAG, "Initializing wire learning mode...");
    
    // Reset learning state
    memset(&g_learning_progress, 0, sizeof(g_learning_progress));
    memset(&g_learning_results, 0, sizeof(g_learning_results));
    
    g_learning_progress.state = WIRE_LEARNING_IDLE;
    strcpy(g_learning_progress.status_message, "Wire learning ready");
    strcpy(g_learning_progress.error_message, "");
    
    g_learning_initialized = true;
    
    ESP_LOGI(TAG, "Wire learning mode initialized");
    return ESP_OK;
}

esp_err_t wire_learning_mode_start(void) {
    if (!g_learning_initialized) {
        return ESP_ERR_INVALID_STATE;
    }
    
    ESP_LOGI(TAG, "Starting wire learning mode...");
    
    // Validate prerequisites
    esp_err_t result = wire_learning_validate_prerequisites();
    if (result != ESP_OK) {
        return result;
    }
    
    // Reset learning state
    memset(&g_learning_progress, 0, sizeof(g_learning_progress));
    g_learning_progress.state = WIRE_LEARNING_INITIALIZING;
    g_learning_progress.learning_start_time = esp_timer_get_time();
    g_learning_progress.current_direction_forward = true;
    
    strcpy(g_learning_progress.status_message, "Initializing wire learning...");
    
    // Auto-arm ESC
    if (!hardware_esc_is_armed()) {
        ESP_LOGI(TAG, "Auto-arming ESC for wire learning");
        result = hardware_esc_arm();
        if (result != ESP_OK) {
            strcpy(g_learning_progress.error_message, "Failed to arm ESC");
            g_learning_progress.state = WIRE_LEARNING_FAILED;
            return result;
        }
    }
    
    // Reset hardware tracking
    hardware_reset_position();
    hardware_reset_rotation_count();
    
    // Start forward direction learning
    g_learning_progress.state = WIRE_LEARNING_FORWARD_DIRECTION;
    g_learning_progress.direction_start_time = esp_timer_get_time();
    g_learning_progress.direction_start_rotations = hardware_get_rotation_count();
    
    g_current_test_speed = 0.0f;
    g_speed_validated = false;
    wire_learning_reset_detection();
    
    strcpy(g_learning_progress.status_message, "Learning forward direction...");
    
    ESP_LOGI(TAG, "Wire learning started - forward direction");
    return ESP_OK;
}

esp_err_t wire_learning_mode_stop(bool immediate) {
    ESP_LOGI(TAG, "Stopping wire learning mode (%s)", immediate ? "immediate" : "graceful");
    
    // Stop motor immediately
    hardware_emergency_stop();
    
    if (immediate) {
        g_learning_progress.state = WIRE_LEARNING_IDLE;
        strcpy(g_learning_progress.status_message, "Wire learning stopped by user");
    } else {
        g_learning_progress.state = WIRE_LEARNING_STOPPING;
        strcpy(g_learning_progress.status_message, "Wire learning stopping gracefully...");
    }
    
    g_coasting_calibration_active = false;
    
    return ESP_OK;
}

esp_err_t wire_learning_mode_update(void) {
    if (!g_learning_initialized) return ESP_ERR_INVALID_STATE;
    
    // Check for timeout
    if (g_learning_progress.state > WIRE_LEARNING_IDLE && 
        g_learning_progress.state < WIRE_LEARNING_COMPLETE) {
        uint64_t elapsed_time = esp_timer_get_time() - g_learning_progress.learning_start_time;
        if (elapsed_time > WIRE_LEARNING_TIMEOUT_S * 1000000ULL) {
            strcpy(g_learning_progress.error_message, "Wire learning timeout");
            g_learning_progress.state = WIRE_LEARNING_FAILED;
            hardware_emergency_stop();
            return ESP_ERR_TIMEOUT;
        }
    }
    
    switch (g_learning_progress.state) {
        case WIRE_LEARNING_IDLE:
            // Nothing to do
            break;
            
        case WIRE_LEARNING_INITIALIZING:
            // Transition to forward direction
            g_learning_progress.state = WIRE_LEARNING_FORWARD_DIRECTION;
            break;
            
        case WIRE_LEARNING_FORWARD_DIRECTION:
            handle_forward_direction();
            break;
            
        case WIRE_LEARNING_DIRECTION_PAUSE:
            // Pause handled in forward direction handler
            break;
            
        case WIRE_LEARNING_REVERSE_DIRECTION:
            handle_reverse_direction();
            break;
            
        case WIRE_LEARNING_CALCULATING_RESULTS:
            calculate_final_results();
            break;
            
        case WIRE_LEARNING_COMPLETE:
        case WIRE_LEARNING_FAILED:
        case WIRE_LEARNING_STOPPING:
            // Terminal states - no action needed
            break;
    }
    
    return ESP_OK;
}

bool wire_learning_mode_is_active(void) {
    return g_learning_progress.state > WIRE_LEARNING_IDLE && 
           g_learning_progress.state < WIRE_LEARNING_COMPLETE;
}

bool wire_learning_mode_is_complete(void) {
    return g_learning_progress.state == WIRE_LEARNING_COMPLETE && 
           g_learning_progress.learning_successful;
}

wire_learning_progress_t wire_learning_mode_get_progress(void) {
    return g_learning_progress;
}

wire_learning_results_t wire_learning_mode_get_results(void) {
    return g_learning_results;
}

// ═══════════════════════════════════════════════════════════════════════════════
// VALIDATION AND UTILITY FUNCTIONS
// ═══════════════════════════════════════════════════════════════════════════════

esp_err_t wire_learning_validate_prerequisites(void) {
    // Check if sensors are validated
    if (!mode_coordinator_are_sensors_validated()) {
        ESP_LOGE(TAG, "Sensors not validated - cannot start wire learning");
        return ESP_ERR_INVALID_STATE;
    }
    
    // Check hardware status
    hardware_status_t hw_status = hardware_get_status();
    if (!hw_status.system_initialized || !hw_status.hall_sensor_healthy) {
        ESP_LOGE(TAG, "Hardware not ready for wire learning");
        return ESP_ERR_INVALID_STATE;
    }
    
    // Check sensor health
    sensor_health_t sensor_status = sensor_health_get_status();
    if (!sensor_status.system_ready) {
        ESP_LOGE(TAG, "Sensors not ready for wire learning");
        return ESP_ERR_INVALID_STATE;
    }
    
    return ESP_OK;
}

bool wire_learning_validate_results(const wire_learning_results_t* results) {
    if (results == NULL) return false;
    
    // Check basic validity
    if (!results->complete) return false;
    if (results->wire_length_m < MIN_WIRE_LENGTH_M || results->wire_length_m > MAX_WIRE_LENGTH_M) return false;
    if (results->learning_accuracy_percent < 90.0f) return false;
    
    return true;
}

bool wire_learning_is_position_safe(void) {
    // Check for safe conditions during learning
    sensor_health_t sensor_status = sensor_health_get_status();
    
    // Check impact threshold
    if (sensor_status.total_accel_g > WIRE_END_IMPACT_THRESHOLD_G * 2.0f) {
        return false;
    }
    
    // Check Hall sensor health
    if (!hardware_is_hall_sensor_healthy()) {
        return false;
    }
    
    return true;
}

esp_err_t wire_learning_emergency_stop(void) {
    ESP_LOGW(TAG, "Emergency stop activated during wire learning");
    
    hardware_emergency_stop();
    g_learning_progress.state = WIRE_LEARNING_FAILED;
    strcpy(g_learning_progress.error_message, "Emergency stop activated");
    g_coasting_calibration_active = false;
    
    return ESP_OK;
}

const char* wire_learning_get_status_message(void) {
    return g_learning_progress.status_message;
}

const char* wire_learning_get_error_message(void) {
    return g_learning_progress.error_message;
}

const char* wire_learning_state_to_string(wire_learning_state_t state) {
    switch (state) {
        case WIRE_LEARNING_IDLE: return "Idle";
        case WIRE_LEARNING_INITIALIZING: return "Initializing";
        case WIRE_LEARNING_FORWARD_DIRECTION: return "Forward Direction";
        case WIRE_LEARNING_DIRECTION_PAUSE: return "Direction Pause";
        case WIRE_LEARNING_REVERSE_DIRECTION: return "Reverse Direction";
        case WIRE_LEARNING_CALCULATING_RESULTS: return "Calculating Results";
        case WIRE_LEARNING_COMPLETE: return "Complete";
        case WIRE_LEARNING_FAILED: return "Failed";
        case WIRE_LEARNING_STOPPING: return "Stopping";
        default: return "Unknown";
    }
}

const char* wire_learning_detection_method_to_string(wire_end_detection_method_t method) {
    switch (method) {
        case WIRE_END_NONE: return "None";
        case WIRE_END_IMPACT_DETECTED: return "Impact";
        case WIRE_END_HALL_TIMEOUT: return "Hall Timeout";
        case WIRE_END_SPEED_DROP: return "Speed Drop";
        case WIRE_END_USER_STOP: return "User Stop";
        default: return "Unknown";
    }
}

int wire_learning_get_progress_percentage(void) {
    switch (g_learning_progress.state) {
        case WIRE_LEARNING_IDLE: return 0;
        case WIRE_LEARNING_INITIALIZING: return 5;
        case WIRE_LEARNING_FORWARD_DIRECTION: return 35;
        case WIRE_LEARNING_DIRECTION_PAUSE: return 50;
        case WIRE_LEARNING_REVERSE_DIRECTION: return 85;
        case WIRE_LEARNING_CALCULATING_RESULTS: return 95;
        case WIRE_LEARNING_COMPLETE: return 100;
        case WIRE_LEARNING_FAILED: return -1;
        case WIRE_LEARNING_STOPPING: return -1;
        default: return 0;
    }
}

uint32_t wire_learning_get_estimated_time_remaining(void) {
    if (g_learning_progress.state <= WIRE_LEARNING_IDLE || 
        g_learning_progress.state >= WIRE_LEARNING_COMPLETE) {
        return 0;
    }
    
    uint64_t elapsed_time = esp_timer_get_time() - g_learning_progress.learning_start_time;
    uint32_t elapsed_seconds = elapsed_time / 1000000;
    
    // Rough estimation based on typical learning time
    uint32_t estimated_total = WIRE_LEARNING_TIMEOUT_S / 2; // Assume half of timeout
    if (elapsed_seconds >= estimated_total) {
        return 0;
    }
    
    return (estimated_total - elapsed_seconds) * 1000;
}

esp_err_t wire_learning_mode_reset(void) {
    ESP_LOGI(TAG, "Resetting wire learning mode");
    
    // Stop any active learning
    wire_learning_mode_stop(true);
    
    // Reset all state
    memset(&g_learning_progress, 0, sizeof(g_learning_progress));
    memset(&g_learning_results, 0, sizeof(g_learning_results));
    
    g_learning_progress.state = WIRE_LEARNING_IDLE;
    strcpy(g_learning_progress.status_message, "Wire learning reset");
    
    g_current_test_speed = 0.0f;
    g_speed_validated = false;
    g_coasting_calibration_active = false;
    wire_learning_reset_detection();
    
    return ESP_OK;
}