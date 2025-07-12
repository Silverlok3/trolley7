// components/automatic_mode/src/automatic_mode.cpp
// ═══════════════════════════════════════════════════════════════════════════════
// AUTOMATIC_MODE.CPP - MODE 2: AUTONOMOUS 5 M/S CYCLING IMPLEMENTATION
// ═══════════════════════════════════════════════════════════════════════════════
// 
// SINGLE RESPONSIBILITY: Autonomous cycling mode implementation
// - Gradual acceleration to 5 m/s with coasting to wire end
// - Cycle management with user interruption support
// - Auto ESC arming/disarming and coasting calibration
// - Uses wire learning data for wire length and coasting distance
// ═══════════════════════════════════════════════════════════════════════════════

#include "automatic_mode.h"
#include "hardware_control.h"
#include "sensor_health.h"
#include "mode_coordinator.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "nvs_flash.h"
#include "nvs.h"
#include <cmath>
#include <cstring>

static const char* TAG = "AUTOMATIC_MODE";

// ═══════════════════════════════════════════════════════════════════════════════
// GLOBAL AUTOMATIC MODE STATE
// ═══════════════════════════════════════════════════════════════════════════════

static automatic_mode_progress_t g_auto_progress = {0};
static automatic_mode_results_t g_auto_results = {0};
static bool g_auto_initialized = false;

// Speed control state
static float g_current_acceleration_target = 0.0f;
static uint64_t g_acceleration_start_time = 0;
static uint32_t g_acceleration_start_rotations = 0;

// Coasting state
static bool g_coasting_in_progress = false;
static uint64_t g_coasting_start_time = 0;
static uint32_t g_coasting_start_rotations = 0;
static float g_coasting_start_speed = 0.0f;

// User interruption tracking
static bool g_user_interruption_requested = false;
static uint64_t g_interruption_request_time = 0;

// ═══════════════════════════════════════════════════════════════════════════════
// COASTING CALIBRATION AND MANAGEMENT
// ═══════════════════════════════════════════════════════════════════════════════

esp_err_t automatic_mode_start_coasting_calibration(void) {
    if (g_auto_progress.coasting.calibrated) {
        ESP_LOGI(TAG, "Coasting already calibrated - skipping");
        return ESP_OK;
    }
    
    ESP_LOGI(TAG, "Starting coasting calibration at %.1f m/s", AUTO_COASTING_CALIBRATION_SPEED);
    
    g_auto_progress.state = AUTO_MODE_COASTING_CALIBRATION;
    g_auto_progress.state_start_time = esp_timer_get_time();
    
    // Accelerate to calibration speed
    esp_err_t result = automatic_mode_accelerate_to_speed(AUTO_COASTING_CALIBRATION_SPEED);
    if (result != ESP_OK) {
        ESP_LOGE(TAG, "Failed to start acceleration for coasting calibration");
        return result;
    }
    
    strcpy(g_auto_progress.status_message, "Coasting calibration - accelerating to 5.0 m/s");
    return ESP_OK;
}

esp_err_t automatic_mode_update_coasting_calibration(float speed, float position) {
    static bool calibration_motor_stopped = false;
    
    // Wait until we reach calibration speed
    if (speed < AUTO_COASTING_CALIBRATION_SPEED - 0.2f) {
        return ESP_OK; // Still accelerating
    }
    
    // Stop motor and start measuring coasting
    if (!calibration_motor_stopped) {
        ESP_LOGI(TAG, "Reached %.1f m/s - stopping motor for coasting measurement", speed);
        
        hardware_set_motor_speed(0.0f, g_auto_progress.cycle_data.current_direction_forward);
        
        g_coasting_start_time = esp_timer_get_time();
        g_coasting_start_rotations = hardware_get_rotation_count();
        g_coasting_start_speed = speed;
        calibration_motor_stopped = true;
        
        strcpy(g_auto_progress.status_message, "Measuring coasting distance...");
        return ESP_OK;
    }
    
    // Monitor coasting until stopped
    if (speed > COAST_DETECTION_SPEED_MS) {
        return ESP_OK; // Still coasting
    }
    
    // Coasting complete - calculate results
    uint64_t coast_end_time = esp_timer_get_time();
    uint32_t coast_end_rotations = hardware_get_rotation_count();
    
    g_auto_progress.coasting.calibrated = true;
    g_auto_progress.coasting.calibration_speed_ms = g_coasting_start_speed;
    g_auto_progress.coasting.coasting_distance_m = hardware_rotations_to_distance(
        coast_end_rotations - g_coasting_start_rotations);
    g_auto_progress.coasting.coasting_time_ms = (coast_end_time - g_coasting_start_time) / 1000;
    g_auto_progress.coasting.deceleration_rate_ms2 = g_coasting_start_speed / 
        (g_auto_progress.coasting.coasting_time_ms / 1000.0f);
    g_auto_progress.coasting.coast_start_distance_m = g_auto_progress.coasting.coasting_distance_m + 
        AUTO_COASTING_SAFETY_MARGIN_M;
    g_auto_progress.coasting.calibration_rotations = coast_end_rotations - g_coasting_start_rotations;
    g_auto_progress.coasting.calibration_successful = true;
    
    // Validate coasting data
    if (g_auto_progress.coasting.coasting_distance_m < AUTO_COASTING_MIN_DISTANCE_M ||
        g_auto_progress.coasting.coasting_distance_m > AUTO_COASTING_MAX_DISTANCE_M) {
        ESP_LOGW(TAG, "Coasting distance out of expected range: %.2f m", 
                g_auto_progress.coasting.coasting_distance_m);
        g_auto_progress.coasting.calibration_successful = false;
        return ESP_ERR_INVALID_SIZE;
    }
    
    // Save coasting data
    coasting_data_t coasting_data = {
        .calibrated = true,
        .coasting_distance_m = g_auto_progress.coasting.coasting_distance_m,
        .coast_start_distance_m = g_auto_progress.coasting.coast_start_distance_m,
        .coast_time_ms = g_auto_progress.coasting.coasting_time_ms,
        .decel_rate_ms2 = g_auto_progress.coasting.deceleration_rate_ms2
    };
    
    mode_coordinator_set_coasting_data(&coasting_data);
    
    ESP_LOGI(TAG, "=== COASTING CALIBRATION COMPLETE ===");
    ESP_LOGI(TAG, "Coasting Distance: %.2f m", g_auto_progress.coasting.coasting_distance_m);
    ESP_LOGI(TAG, "Coasting Time: %lu ms", g_auto_progress.coasting.coasting_time_ms);
    ESP_LOGI(TAG, "Deceleration Rate: %.2f m/s²", g_auto_progress.coasting.deceleration_rate_ms2);
    ESP_LOGI(TAG, "Coast Start Distance: %.2f m from wire end", g_auto_progress.coasting.coast_start_distance_m);
    
    // Reset for normal operation
    calibration_motor_stopped = false;
    strcpy(g_auto_progress.status_message, "Coasting calibration complete");
    
    return ESP_OK;
}

bool automatic_mode_is_coasting_calibrated(void) {
    return g_auto_progress.coasting.calibrated;
}

coasting_calibration_t automatic_mode_get_coasting_data(void) {
    return g_auto_progress.coasting;
}

float automatic_mode_calculate_coasting_distance(float current_position, 
                                                float wire_length, 
                                                bool direction_forward) {
    if (!g_auto_progress.coasting.calibrated) {
        return AUTO_COASTING_SAFETY_MARGIN_M; // Default safety margin
    }
    
    float distance_to_wire_end;
    if (direction_forward) {
        distance_to_wire_end = wire_length - current_position;
    } else {
        distance_to_wire_end = current_position;
    }
    
    return distance_to_wire_end - g_auto_progress.coasting.coast_start_distance_m;
}

// ═══════════════════════════════════════════════════════════════════════════════
// SPEED CONTROL IMPLEMENTATION
// ═══════════════════════════════════════════════════════════════════════════════

esp_err_t automatic_mode_accelerate_to_speed(float target_speed) {
    if (target_speed > AUTO_MODE_MAX_SPEED_MS) {
        ESP_LOGW(TAG, "Target speed %.1f m/s exceeds maximum %.1f m/s", 
                target_speed, AUTO_MODE_MAX_SPEED_MS);
        target_speed = AUTO_MODE_MAX_SPEED_MS;
    }
    
    ESP_LOGI(TAG, "Starting gradual acceleration to %.1f m/s", target_speed);
    
    g_current_acceleration_target = target_speed;
    g_acceleration_start_time = esp_timer_get_time();
    g_acceleration_start_rotations = hardware_get_rotation_count();
    
    // Start with minimum speed
    esp_err_t result = hardware_set_motor_speed(AUTO_MODE_START_SPEED_MS, 
                                               g_auto_progress.cycle_data.current_direction_forward);
    if (result != ESP_OK) {
        ESP_LOGE(TAG, "Failed to start acceleration");
        return result;
    }
    
    g_auto_progress.current_target_speed = AUTO_MODE_START_SPEED_MS;
    g_auto_progress.acceleration_rate = AUTO_MODE_ACCEL_RATE_MS2;
    
    return ESP_OK;
}

esp_err_t automatic_mode_decelerate_to_speed(float target_speed) {
    ESP_LOGI(TAG, "Starting deceleration to %.1f m/s", target_speed);
    
    float current_speed = hardware_get_current_speed();
    if (current_speed <= target_speed) {
        ESP_LOGW(TAG, "Already at or below target speed");
        return ESP_OK;
    }
    
    // Calculate deceleration time
    float speed_difference = current_speed - target_speed;
    float decel_time_s = speed_difference / AUTO_MODE_DECEL_RATE_MS2;
    
    // Gradual deceleration
    uint32_t steps = (uint32_t)(decel_time_s * 10); // 10 steps per second
    float speed_step = speed_difference / steps;
    
    for (uint32_t i = 0; i < steps; i++) {
        float new_speed = current_speed - (speed_step * (i + 1));
        hardware_set_motor_speed(new_speed, g_auto_progress.cycle_data.current_direction_forward);
        vTaskDelay(pdMS_TO_TICKS(100)); // 10 Hz update rate
        
        // Check for safety conditions
        if (!automatic_mode_is_operation_safe()) {
            ESP_LOGW(TAG, "Safety check failed during deceleration");
            automatic_mode_handle_emergency("Safety failure during deceleration");
            return ESP_ERR_INVALID_STATE;
        }
    }
    
    // Set final target speed
    hardware_set_motor_speed(target_speed, g_auto_progress.cycle_data.current_direction_forward);
    g_auto_progress.current_target_speed = target_speed;
    
    ESP_LOGI(TAG, "Deceleration to %.1f m/s complete", target_speed);
    return ESP_OK;
}

esp_err_t automatic_mode_maintain_cruise_speed(void) {
    float current_speed = hardware_get_current_speed();
    float target_speed = AUTO_MODE_MAX_SPEED_MS;
    
    // Simple speed maintenance with tolerance
    if (fabs(current_speed - target_speed) > 0.5f) {
        ESP_LOGD(TAG, "Adjusting cruise speed: %.1f → %.1f m/s", current_speed, target_speed);
        hardware_set_motor_speed(target_speed, g_auto_progress.cycle_data.current_direction_forward);
    }
    
    g_auto_progress.current_target_speed = target_speed;
    return ESP_OK;
}

float automatic_mode_get_current_target_speed(void) {
    return g_auto_progress.current_target_speed;
}

bool automatic_mode_is_at_target_speed(float tolerance) {
    float current_speed = hardware_get_current_speed();
    return fabs(current_speed - g_current_acceleration_target) <= tolerance;
}

// ═══════════════════════════════════════════════════════════════════════════════
// CYCLE MANAGEMENT IMPLEMENTATION
// ═══════════════════════════════════════════════════════════════════════════════

esp_err_t automatic_mode_start_new_cycle(void) {
    ESP_LOGI(TAG, "Starting new automatic cycle %lu", g_auto_progress.cycle_data.cycle_number + 1);
    
    g_auto_progress.cycle_data.cycle_number++;
    g_auto_progress.cycle_data.cycle_start_time = esp_timer_get_time();
    g_auto_progress.cycle_data.current_direction_forward = true;
    g_auto_progress.cycle_data.run_start_time = esp_timer_get_time();
    g_auto_progress.cycle_data.run_start_rotations = hardware_get_rotation_count();
    g_auto_progress.cycle_data.max_speed_achieved = 0.0f;
    
    // Reset position tracking for new cycle
    automatic_mode_reset_position_tracking();
    
    strcpy(g_auto_progress.status_message, "Starting new cycle - accelerating forward");
    return ESP_OK;
}

esp_err_t automatic_mode_complete_current_run(void) {
    uint64_t run_end_time = esp_timer_get_time();
    uint32_t run_end_rotations = hardware_get_rotation_count();
    
    // Calculate run statistics
    uint32_t run_duration = (run_end_time - g_auto_progress.cycle_data.run_start_time) / 1000;
    uint32_t run_rotations = run_end_rotations - g_auto_progress.cycle_data.run_start_rotations;
    float run_distance = hardware_rotations_to_distance(run_rotations);
    
    if (g_auto_progress.cycle_data.current_direction_forward) {
        g_auto_progress.cycle_data.forward_runs++;
        ESP_LOGI(TAG, "Forward run complete: %.2f m in %lu ms", run_distance, run_duration);
    } else {
        g_auto_progress.cycle_data.reverse_runs++;
        ESP_LOGI(TAG, "Reverse run complete: %.2f m in %lu ms", run_distance, run_duration);
    }
    
    g_auto_progress.cycle_data.total_distance_m += (uint32_t)run_distance;
    
    return ESP_OK;
}

esp_err_t automatic_mode_change_direction(void) {
    ESP_LOGI(TAG, "Changing direction - pausing for %d ms", AUTO_MODE_DIRECTION_PAUSE_MS);
    
    // Stop motor
    hardware_set_motor_speed(0.0f, true);
    g_auto_progress.state = AUTO_MODE_DIRECTION_CHANGE;
    
    // Direction pause
    strcpy(g_auto_progress.status_message, "Direction change - pausing");
    vTaskDelay(pdMS_TO_TICKS(AUTO_MODE_DIRECTION_PAUSE_MS));
    
    // Change direction
    g_auto_progress.cycle_data.current_direction_forward = !g_auto_progress.cycle_data.current_direction_forward;
    g_auto_progress.cycle_data.run_start_time = esp_timer_get_time();
    g_auto_progress.cycle_data.run_start_rotations = hardware_get_rotation_count();
    
    const char* direction_str = g_auto_progress.cycle_data.current_direction_forward ? "forward" : "reverse";
    ESP_LOGI(TAG, "Direction changed to %s", direction_str);
    
    snprintf(g_auto_progress.status_message, sizeof(g_auto_progress.status_message),
            "Direction changed - accelerating %s", direction_str);
    
    return ESP_OK;
}

uint32_t automatic_mode_get_cycle_count(void) {
    return g_auto_progress.cycle_data.cycle_number;
}

uint32_t automatic_mode_get_run_count(void) {
    return g_auto_progress.cycle_data.forward_runs + g_auto_progress.cycle_data.reverse_runs;
}

bool automatic_mode_is_direction_forward(void) {
    return g_auto_progress.cycle_data.current_direction_forward;
}

// ═══════════════════════════════════════════════════════════════════════════════
// WIRE END DETECTION AND POSITIONING
// ═══════════════════════════════════════════════════════════════════════════════

bool automatic_mode_is_approaching_wire_end(void) {
    if (g_auto_progress.wire_length_m <= 0) {
        return false; // No wire length data
    }
    
    float distance_to_end = automatic_mode_get_distance_to_wire_end(
        g_auto_progress.current_position_m, 
        g_auto_progress.cycle_data.current_direction_forward);
    
    // Check if we need to start coasting
    if (g_auto_progress.coasting.calibrated) {
        return distance_to_end <= g_auto_progress.coasting.coast_start_distance_m;
    } else {
        return distance_to_end <= AUTO_COASTING_SAFETY_MARGIN_M;
    }
}

float automatic_mode_get_distance_to_wire_end(float current_position, bool direction_forward) {
    if (g_auto_progress.wire_length_m <= 0) {
        return 0.0f;
    }
    
    if (direction_forward) {
        return g_auto_progress.wire_length_m - current_position;
    } else {
        return current_position;
    }
}

bool automatic_mode_is_at_wire_end(void) {
    // Check multiple detection methods
    sensor_health_t sensor_status = sensor_health_get_status();
    
    // Impact detection
    if (sensor_status.total_accel_g > AUTO_MODE_MAX_IMPACT_G) {
        ESP_LOGI(TAG, "Wire end detected by impact: %.2f g", sensor_status.total_accel_g);
        return true;
    }
    
    // Hall sensor timeout
    if (hardware_get_time_since_last_hall_pulse() > 2000000ULL) { // 2 seconds
        ESP_LOGI(TAG, "Wire end detected by Hall timeout");
        return true;
    }
    
    // Speed drop detection
    float current_speed = hardware_get_current_speed();
    if (g_auto_progress.current_target_speed > 0.5f && current_speed < 0.2f) {
        ESP_LOGI(TAG, "Wire end detected by speed drop: %.1f m/s", current_speed);
        return true;
    }
    
    return false;
}

esp_err_t automatic_mode_handle_wire_end_reached(void) {
    ESP_LOGI(TAG, "Wire end reached - completing current run");
    
    // Stop motor immediately
    hardware_set_motor_speed(0.0f, g_auto_progress.cycle_data.current_direction_forward);
    
    // Complete current run
    automatic_mode_complete_current_run();
    
    // Check if user requested interruption
    if (g_user_interruption_requested) {
        ESP_LOGI(TAG, "User interruption - stopping at wire end");
        g_auto_progress.finishing_current_run = false;
        g_auto_progress.user_interrupted = true;
        g_auto_progress.state = AUTO_MODE_COMPLETE;
        strcpy(g_auto_progress.status_message, "Stopped at wire end per user request");
        return ESP_OK;
    }
    
    // Check if maximum cycles reached
    if (g_auto_progress.cycle_data.cycle_number >= AUTO_MODE_MAX_CYCLES) {
        ESP_LOGI(TAG, "Maximum cycles reached - stopping");
        g_auto_progress.state = AUTO_MODE_COMPLETE;
        strcpy(g_auto_progress.status_message, "Maximum cycles completed");
        return ESP_OK;
    }
    
    // Continue with direction change
    automatic_mode_change_direction();
    
    // If completed both directions, start new cycle
    if (g_auto_progress.cycle_data.forward_runs > 0 && g_auto_progress.cycle_data.reverse_runs > 0 &&
        g_auto_progress.cycle_data.forward_runs == g_auto_progress.cycle_data.reverse_runs) {
        g_auto_progress.state = AUTO_MODE_CYCLE_COMPLETE;
        strcpy(g_auto_progress.status_message, "Cycle complete - pausing before next cycle");
        vTaskDelay(pdMS_TO_TICKS(AUTO_MODE_CYCLE_PAUSE_MS));
        automatic_mode_start_new_cycle();
    }
    
    // Start acceleration for next run
    g_auto_progress.state = AUTO_MODE_ACCELERATING;
    automatic_mode_accelerate_to_speed(AUTO_MODE_MAX_SPEED_MS);
    
    return ESP_OK;
}

esp_err_t automatic_mode_reset_position_tracking(void) {
    g_auto_progress.current_position_m = 0.0f;
    g_auto_progress.distance_to_wire_end_m = g_auto_progress.wire_length_m;
    hardware_reset_position();
    return ESP_OK;
}

// ═══════════════════════════════════════════════════════════════════════════════
// MAIN STATE MACHINE IMPLEMENTATION
// ═══════════════════════════════════════════════════════════════════════════════

static esp_err_t handle_acceleration_state(void) {
    float current_speed = hardware_get_current_speed();
    uint64_t elapsed_time = esp_timer_get_time() - g_acceleration_start_time;
    
    // Update max speed achieved
    if (current_speed > g_auto_progress.cycle_data.max_speed_achieved) {
        g_auto_progress.cycle_data.max_speed_achieved = current_speed;
    }
    
    // Gradual acceleration with 0.1 m/s increments
    if (g_auto_progress.current_target_speed < g_current_acceleration_target) {
        // Check if enough time passed for next increment (every 1 second)
        if (elapsed_time > 1000000ULL) { // 1 second
            float next_speed = g_auto_progress.current_target_speed + AUTO_MODE_SPEED_INCREMENT;
            if (next_speed > g_current_acceleration_target) {
                next_speed = g_current_acceleration_target;
            }
            
            ESP_LOGI(TAG, "Acceleration step: %.1f → %.1f m/s", 
                    g_auto_progress.current_target_speed, next_speed);
            
            hardware_set_motor_speed(next_speed, g_auto_progress.cycle_data.current_direction_forward);
            g_auto_progress.current_target_speed = next_speed;
            g_acceleration_start_time = esp_timer_get_time();
        }
    }
    
    // Check if target speed reached
    if (automatic_mode_is_at_target_speed(0.2f)) {
        ESP_LOGI(TAG, "Target speed %.1f m/s reached - switching to cruise", g_current_acceleration_target);
        g_auto_progress.state = AUTO_MODE_CRUISING;
        strcpy(g_auto_progress.status_message, "Cruising at maximum speed");
    }
    
    return ESP_OK;
}

static esp_err_t handle_cruising_state(void) {
    // Maintain cruise speed
    automatic_mode_maintain_cruise_speed();
    
    // Update position tracking
    g_auto_progress.current_position_m = hardware_get_current_position();
    g_auto_progress.distance_to_wire_end_m = automatic_mode_get_distance_to_wire_end(
        g_auto_progress.current_position_m, g_auto_progress.cycle_data.current_direction_forward);
    
    // Check if approaching wire end for coasting
    if (automatic_mode_is_approaching_wire_end()) {
        ESP_LOGI(TAG, "Approaching wire end - starting coasting (%.2f m remaining)", 
                g_auto_progress.distance_to_wire_end_m);
        
        // Stop motor for coasting
        hardware_set_motor_speed(0.0f, g_auto_progress.cycle_data.current_direction_forward);
        g_auto_progress.state = AUTO_MODE_COASTING;
        g_coasting_in_progress = true;
        g_coasting_start_time = esp_timer_get_time();
        g_coasting_start_rotations = hardware_get_rotation_count();
        g_coasting_start_speed = hardware_get_current_speed();
        
        strcpy(g_auto_progress.status_message, "Coasting to wire end");
    }
    
    return ESP_OK;
}

static esp_err_t handle_coasting_state(void) {
    float current_speed = hardware_get_current_speed();
    
    // Check if wire end reached
    if (automatic_mode_is_at_wire_end() || current_speed < COAST_DETECTION_SPEED_MS) {
        ESP_LOGI(TAG, "Wire end reached via coasting - speed: %.2f m/s", current_speed);
        
        g_coasting_in_progress = false;
        g_auto_progress.state = AUTO_MODE_WIRE_END_APPROACH;
        
        // Final approach at low speed
        hardware_set_motor_speed(AUTO_MODE_WIRE_END_APPROACH_MS, 
                                g_auto_progress.cycle_data.current_direction_forward);
        strcpy(g_auto_progress.status_message, "Final approach to wire end");
        
        // Brief approach time
        vTaskDelay(pdMS_TO_TICKS(500));
        
        // Handle wire end reached
        automatic_mode_handle_wire_end_reached();
    }
    
    return ESP_OK;
}

// ═══════════════════════════════════════════════════════════════════════════════
// PUBLIC API IMPLEMENTATION
// ═══════════════════════════════════════════════════════════════════════════════

esp_err_t automatic_mode_init(void) {
    ESP_LOGI(TAG, "Initializing automatic mode...");
    
    // Reset automatic mode state
    memset(&g_auto_progress, 0, sizeof(g_auto_progress));
    memset(&g_auto_results, 0, sizeof(g_auto_results));
    
    g_auto_progress.state = AUTO_MODE_IDLE;
    strcpy(g_auto_progress.status_message, "Automatic mode ready");
    strcpy(g_auto_progress.error_message, "");
    
    // Reset control variables
    g_current_acceleration_target = 0.0f;
    g_coasting_in_progress = false;
    g_user_interruption_requested = false;
    
    g_auto_initialized = true;
    
    ESP_LOGI(TAG, "Automatic mode initialized");
    return ESP_OK;
}

esp_err_t automatic_mode_start(void) {
    if (!g_auto_initialized) {
        return ESP_ERR_INVALID_STATE;
    }
    
    ESP_LOGI(TAG, "Starting automatic mode...");
    
    // Validate prerequisites
    esp_err_t result = automatic_mode_validate_prerequisites();
    if (result != ESP_OK) {
        return result;
    }
    
    // Get wire learning data
    const wire_learning_results_t* wire_data = mode_coordinator_get_wire_learning_results();
    if (wire_data == NULL || !wire_data->complete) {
        ESP_LOGE(TAG, "Wire learning data not available");
        strcpy(g_auto_progress.error_message, "Wire learning required before automatic mode");
        return ESP_ERR_INVALID_STATE;
    }
    
    // Initialize automatic mode state
    g_auto_progress.state = AUTO_MODE_INITIALIZING;
    g_auto_progress.mode_start_time = esp_timer_get_time();
    g_auto_progress.wire_length_m = wire_data->wire_length_m;
    g_auto_progress.user_interrupted = false;
    g_auto_progress.finishing_current_run = false;
    g_user_interruption_requested = false;
    
    // Try to load existing coasting data
    const coasting_data_t* coasting_data = mode_coordinator_get_coasting_data();
    if (coasting_data != NULL && coasting_data->calibrated) {
        g_auto_progress.coasting.calibrated = true;
        g_auto_progress.coasting.coasting_distance_m = coasting_data->coasting_distance_m;
        g_auto_progress.coasting.coast_start_distance_m = coasting_data->coast_start_distance_m;
        g_auto_progress.coasting.coasting_time_ms = coasting_data->coast_time_ms;
        g_auto_progress.coasting.deceleration_rate_ms2 = coasting_data->decel_rate_ms2;
        ESP_LOGI(TAG, "Loaded existing coasting data: %.2f m", coasting_data->coasting_distance_m);
    }
    
    // Auto-arm ESC
    g_auto_progress.state = AUTO_MODE_ARMING_ESC;
    strcpy(g_auto_progress.status_message, "Auto-arming ESC...");
    
    result = automatic_mode_auto_arm_esc();
    if (result != ESP_OK) {
        strcpy(g_auto_progress.error_message, "Failed to auto-arm ESC");
        g_auto_progress.state = AUTO_MODE_ERROR;
        return result;
    }
    
    // Start coasting calibration if needed
    if (!g_auto_progress.coasting.calibrated) {
        ESP_LOGI(TAG, "Coasting not calibrated - starting calibration");
        result = automatic_mode_start_coasting_calibration();
        if (result != ESP_OK) {
            strcpy(g_auto_progress.error_message, "Failed to start coasting calibration");
            g_auto_progress.state = AUTO_MODE_ERROR;
            return result;
        }
    } else {
        // Start first cycle
        automatic_mode_start_new_cycle();
        g_auto_progress.state = AUTO_MODE_ACCELERATING;
        automatic_mode_accelerate_to_speed(AUTO_MODE_MAX_SPEED_MS);
    }
    
    ESP_LOGI(TAG, "Automatic mode started successfully");
    ESP_LOGI(TAG, "Wire length: %.2f m, Coasting: %s", 
            g_auto_progress.wire_length_m,
            g_auto_progress.coasting.calibrated ? "calibrated" : "will calibrate");
    
    return ESP_OK;
}

esp_err_t automatic_mode_stop_graceful(void) {
    ESP_LOGI(TAG, "Graceful stop requested - will finish current run");
    
    g_user_interruption_requested = true;
    g_interruption_request_time = esp_timer_get_time();
    g_auto_progress.finishing_current_run = true;
    
    strcpy(g_auto_progress.status_message, "Stopping gracefully - finishing current run");
    
    return ESP_OK;
}

esp_err_t automatic_mode_interrupt(void) {
    ESP_LOGI(TAG, "Immediate interruption requested");
    
    // Stop motor immediately
    hardware_emergency_stop();
    
    // Update state
    g_auto_progress.state = AUTO_MODE_STOPPING_INTERRUPTED;
    g_auto_progress.user_interrupted = true;
    g_auto_progress.finishing_current_run = false;
    g_user_interruption_requested = true;
    
    strcpy(g_auto_progress.status_message, "Interrupted by user - stopping immediately");
    
    // Auto-disarm ESC
    automatic_mode_auto_disarm_esc();
    
    return ESP_OK;
}

esp_err_t automatic_mode_update(void) {
    if (!g_auto_initialized || g_auto_progress.state == AUTO_MODE_IDLE) {
        return ESP_OK;
    }
    
    // Update current position
    g_auto_progress.current_position_m = hardware_get_current_position();
    
    // Check for timeout
    uint64_t elapsed_time = esp_timer_get_time() - g_auto_progress.mode_start_time;
    if (elapsed_time > AUTO_MODE_TIMEOUT_MS * 1000ULL) {
        ESP_LOGW(TAG, "Automatic mode timeout after %d minutes", AUTO_MODE_TIMEOUT_MS / 60000);
        automatic_mode_handle_emergency("Automatic mode timeout");
        return ESP_ERR_TIMEOUT;
    }
    
    // Main state machine
    switch (g_auto_progress.state) {
        case AUTO_MODE_IDLE:
            // Nothing to do
            break;
            
        case AUTO_MODE_INITIALIZING:
            // Transition handled in start function
            break;
            
        case AUTO_MODE_ARMING_ESC:
            // ESC arming handled in start function
            break;
            
        case AUTO_MODE_ACCELERATING:
            handle_acceleration_state();
            break;
            
        case AUTO_MODE_CRUISING:
            handle_cruising_state();
            break;
            
        case AUTO_MODE_COASTING_CALIBRATION:
            {
                float current_speed = hardware_get_current_speed();
                automatic_mode_update_coasting_calibration(current_speed, g_auto_progress.current_position_m);
                
                // Check if calibration complete
                if (g_auto_progress.coasting.calibrated) {
                    ESP_LOGI(TAG, "Coasting calibration complete - starting first cycle");
                    automatic_mode_start_new_cycle();
                    g_auto_progress.state = AUTO_MODE_ACCELERATING;
                    automatic_mode_accelerate_to_speed(AUTO_MODE_MAX_SPEED_MS);
                }
            }
            break;
            
        case AUTO_MODE_COASTING:
            handle_coasting_state();
            break;
            
        case AUTO_MODE_WIRE_END_APPROACH:
            // Check if wire end reached
            if (automatic_mode_is_at_wire_end()) {
                automatic_mode_handle_wire_end_reached();
            }
            break;
            
        case AUTO_MODE_DIRECTION_CHANGE:
            // Direction change handled in wire end function
            break;
            
        case AUTO_MODE_CYCLE_COMPLETE:
            // Cycle completion handled in wire end function
            break;
            
        case AUTO_MODE_STOPPING_GRACEFUL:
        case AUTO_MODE_STOPPING_INTERRUPTED:
        case AUTO_MODE_ERROR:
        case AUTO_MODE_COMPLETE:
            // Terminal states - no action needed
            break;
    }
    
    // Monitor safety
    if (!automatic_mode_is_operation_safe()) {
        ESP_LOGW(TAG, "Safety check failed - stopping automatic mode");
        automatic_mode_handle_emergency("Safety check failed");
        return ESP_ERR_INVALID_STATE;
    }
    
    return ESP_OK;
}

bool automatic_mode_is_active(void) {
    return g_auto_progress.state > AUTO_MODE_IDLE && 
           g_auto_progress.state < AUTO_MODE_COMPLETE;
}

bool automatic_mode_is_running(void) {
    return automatic_mode_is_active() && 
           g_auto_progress.state != AUTO_MODE_STOPPING_GRACEFUL &&
           g_auto_progress.state != AUTO_MODE_STOPPING_INTERRUPTED;
}

automatic_mode_progress_t automatic_mode_get_progress(void) {
    return g_auto_progress;
}

automatic_mode_results_t automatic_mode_get_results(void) {
    // Update results with current data
    g_auto_results.total_cycles_completed = g_auto_progress.cycle_data.cycle_number;
    g_auto_results.total_runs_completed = automatic_mode_get_run_count();
    g_auto_results.total_operating_time_ms = automatic_mode_get_operating_time();
    g_auto_results.total_distance_traveled_m = g_auto_progress.cycle_data.total_distance_m;
    g_auto_results.max_speed_achieved_ms = g_auto_progress.cycle_data.max_speed_achieved;
    g_auto_results.coasting_data = g_auto_progress.coasting;
    g_auto_results.interrupted_by_user = g_auto_progress.user_interrupted;
    
    if (g_auto_results.total_cycles_completed > 0) {
        g_auto_results.average_cycle_time_ms = g_auto_results.total_operating_time_ms / g_auto_results.total_cycles_completed;
    }
    
    // Set completion reason
    if (g_auto_progress.user_interrupted) {
        strcpy(g_auto_results.completion_reason, "User interruption");
    } else if (g_auto_progress.state == AUTO_MODE_ERROR) {
        strcpy(g_auto_results.completion_reason, "Error condition");
    } else if (g_auto_results.total_cycles_completed >= AUTO_MODE_MAX_CYCLES) {
        strcpy(g_auto_results.completion_reason, "Maximum cycles reached");
    } else {
        strcpy(g_auto_results.completion_reason, "In progress");
    }
    
    return g_auto_results;
}

// ═══════════════════════════════════════════════════════════════════════════════
// SAFETY AND VALIDATION IMPLEMENTATION
// ═══════════════════════════════════════════════════════════════════════════════

esp_err_t automatic_mode_validate_prerequisites(void) {
    // Check sensor validation
    if (!mode_coordinator_are_sensors_validated()) {
        ESP_LOGE(TAG, "Sensors not validated - cannot start automatic mode");
        return ESP_ERR_INVALID_STATE;
    }
    
    // Check wire learning completion
    const wire_learning_results_t* wire_data = mode_coordinator_get_wire_learning_results();
    if (wire_data == NULL || !wire_data->complete) {
        ESP_LOGE(TAG, "Wire learning not complete - cannot start automatic mode");
        return ESP_ERR_INVALID_STATE;
    }
    
    // Validate wire length
    if (wire_data->wire_length_m < AUTO_MODE_MIN_WIRE_LENGTH_M) {
        ESP_LOGE(TAG, "Wire too short for automatic mode: %.2f m < %.2f m", 
                wire_data->wire_length_m, AUTO_MODE_MIN_WIRE_LENGTH_M);
        return ESP_ERR_INVALID_SIZE;
    }
    
    // Check hardware status
    hardware_status_t hw_status = hardware_get_status();
    if (!hw_status.system_initialized || !hw_status.hall_sensor_healthy) {
        ESP_LOGE(TAG, "Hardware not ready for automatic mode");
        return ESP_ERR_INVALID_STATE;
    }
    
    // Check sensor health
    sensor_health_t sensor_status = sensor_health_get_status();
    if (!sensor_status.system_ready) {
        ESP_LOGE(TAG, "Sensors not ready for automatic mode");
        return ESP_ERR_INVALID_STATE;
    }
    
    return ESP_OK;
}

bool automatic_mode_is_operation_safe(void) {
    // Check system health
    if (!mode_coordinator_is_motion_safe()) {
        return false;
    }
    
    // Check sensor health
    if (!automatic_mode_check_sensor_health()) {
        return false;
    }
    
    // Check ESC health
    if (!automatic_mode_monitor_esc_health()) {
        return false;
    }
    
    return true;
}

esp_err_t automatic_mode_handle_emergency(const char* error_message) {
    ESP_LOGE(TAG, "EMERGENCY: %s", error_message);
    
    // Stop motor immediately
    hardware_emergency_stop();
    
    // Update state
    g_auto_progress.state = AUTO_MODE_ERROR;
    strncpy(g_auto_progress.error_message, error_message, sizeof(g_auto_progress.error_message) - 1);
    g_auto_progress.error_message[sizeof(g_auto_progress.error_message) - 1] = '\0';
    
    strcpy(g_auto_progress.status_message, "EMERGENCY STOP - Automatic mode halted");
    
    // Auto-disarm ESC
    automatic_mode_auto_disarm_esc();
    
    // Report to mode coordinator
    mode_coordinator_report_error(error_message);
    
    return ESP_OK;
}

bool automatic_mode_check_sensor_health(void) {
    sensor_health_t sensor_status = sensor_health_get_status();
    
    // Check if sensors are still validated
    if (!mode_coordinator_are_sensors_validated()) {
        ESP_LOGW(TAG, "Sensors no longer validated");
        return false;
    }
    
    // Check for excessive impact
    if (sensor_status.total_accel_g > AUTO_MODE_MAX_IMPACT_G) {
        ESP_LOGW(TAG, "Excessive impact detected: %.2f g", sensor_status.total_accel_g);
        return false;
    }
    
    // Check Hall sensor during movement
    if (g_auto_progress.current_target_speed > 0.3f && 
        hardware_get_time_since_last_hall_pulse() > 2000000ULL) {
        ESP_LOGW(TAG, "Hall sensor not responding during movement");
        return false;
    }
    
    return true;
}

bool automatic_mode_check_user_interruption(void) {
    return g_user_interruption_requested;
}

// ═══════════════════════════════════════════════════════════════════════════════
// ESC MANAGEMENT IMPLEMENTATION
// ═══════════════════════════════════════════════════════════════════════════════

esp_err_t automatic_mode_auto_arm_esc(void) {
    if (hardware_esc_is_armed()) {
        ESP_LOGI(TAG, "ESC already armed");
        g_auto_progress.esc_auto_armed = true;
        return ESP_OK;
    }
    
    ESP_LOGI(TAG, "Auto-arming ESC for automatic mode");
    
    esp_err_t result = hardware_esc_arm();
    if (result == ESP_OK) {
        g_auto_progress.esc_auto_armed = true;
        ESP_LOGI(TAG, "ESC auto-armed successfully");
    } else {
        ESP_LOGE(TAG, "Failed to auto-arm ESC");
    }
    
    return result;
}

esp_err_t automatic_mode_auto_disarm_esc(void) {
    ESP_LOGI(TAG, "Auto-disarming ESC");
    
    esp_err_t result = hardware_esc_disarm();
    g_auto_progress.esc_auto_armed = false;
    
    return result;
}

bool automatic_mode_is_esc_ready(void) {
    return hardware_esc_is_armed() && g_auto_progress.esc_auto_armed;
}

bool automatic_mode_monitor_esc_health(void) {
    hardware_status_t hw_status = hardware_get_status();
    
    if (!hw_status.esc_armed || !hw_status.esc_responding) {
        ESP_LOGW(TAG, "ESC health check failed: armed=%s, responding=%s",
                hw_status.esc_armed ? "yes" : "no",
                hw_status.esc_responding ? "yes" : "no");
        return false;
    }
    
    return true;
}

// ═══════════════════════════════════════════════════════════════════════════════
// UTILITY FUNCTIONS IMPLEMENTATION
// ═══════════════════════════════════════════════════════════════════════════════

const char* automatic_mode_get_status_message(void) {
    return g_auto_progress.status_message;
}

const char* automatic_mode_get_error_message(void) {
    return g_auto_progress.error_message;
}

const char* automatic_mode_state_to_string(automatic_mode_state_t state) {
    switch (state) {
        case AUTO_MODE_IDLE: return "Idle";
        case AUTO_MODE_INITIALIZING: return "Initializing";
        case AUTO_MODE_ARMING_ESC: return "Arming ESC";
        case AUTO_MODE_ACCELERATING: return "Accelerating";
        case AUTO_MODE_CRUISING: return "Cruising";
        case AUTO_MODE_COASTING_CALIBRATION: return "Coasting Calibration";
        case AUTO_MODE_COASTING: return "Coasting";
        case AUTO_MODE_WIRE_END_APPROACH: return "Wire End Approach";
        case AUTO_MODE_DIRECTION_CHANGE: return "Direction Change";
        case AUTO_MODE_CYCLE_COMPLETE: return "Cycle Complete";
        case AUTO_MODE_STOPPING_GRACEFUL: return "Stopping Gracefully";
        case AUTO_MODE_STOPPING_INTERRUPTED: return "Stopping Interrupted";
        case AUTO_MODE_ERROR: return "Error";
        case AUTO_MODE_COMPLETE: return "Complete";
        default: return "Unknown";
    }
}

int automatic_mode_get_progress_percentage(void) {
    switch (g_auto_progress.state) {
        case AUTO_MODE_IDLE: return 0;
        case AUTO_MODE_INITIALIZING: return 5;
        case AUTO_MODE_ARMING_ESC: return 10;
        case AUTO_MODE_COASTING_CALIBRATION: return 20;
        case AUTO_MODE_ACCELERATING: return 30 + (g_auto_progress.current_target_speed / AUTO_MODE_MAX_SPEED_MS) * 20;
        case AUTO_MODE_CRUISING: return 55;
        case AUTO_MODE_COASTING: return 70;
        case AUTO_MODE_WIRE_END_APPROACH: return 85;
        case AUTO_MODE_DIRECTION_CHANGE: return 90;
        case AUTO_MODE_CYCLE_COMPLETE: return 95;
        case AUTO_MODE_COMPLETE: return 100;
        case AUTO_MODE_ERROR: return -1;
        default: return 0;
    }
}

uint32_t automatic_mode_get_estimated_time(void) {
    if (g_auto_progress.state <= AUTO_MODE_IDLE || g_auto_progress.state >= AUTO_MODE_COMPLETE) {
        return 0;
    }
    
    // Rough estimation based on wire length and speed
    if (g_auto_progress.wire_length_m > 0) {
        float time_per_run = (g_auto_progress.wire_length_m / AUTO_MODE_MAX_SPEED_MS) + 10; // +10s for accel/decel
        return (uint32_t)(time_per_run * 1000 * 2); // Forward + reverse
    }
    
    return 60000; // Default 1 minute estimate
}

esp_err_t automatic_mode_get_performance_stats(char* stats_buffer, size_t buffer_size) {
    if (stats_buffer == NULL) return ESP_ERR_INVALID_ARG;
    
    automatic_mode_results_t results = automatic_mode_get_results();
    
    snprintf(stats_buffer, buffer_size,
        "=== AUTOMATIC MODE PERFORMANCE ===\n"
        "Cycles: %lu completed\n"
        "Runs: %lu total (%s)\n"
        "Operating Time: %.1f minutes\n"
        "Distance: %.1f m total\n"
        "Max Speed: %.1f m/s\n"
        "Avg Cycle Time: %.1f minutes\n"
        "Coasting Distance: %.2f m\n"
        "Wire Length: %.2f m\n"
        "Status: %s\n"
        "Completion: %s\n",
        results.total_cycles_completed,
        results.total_runs_completed,
        g_auto_progress.cycle_data.current_direction_forward ? "forward" : "reverse",
        results.total_operating_time_ms / 60000.0f,
        results.total_distance_traveled_m,
        results.max_speed_achieved_ms,
        results.average_cycle_time_ms / 60000.0f,
        results.coasting_data.coasting_distance_m,
        g_auto_progress.wire_length_m,
        automatic_mode_state_to_string(g_auto_progress.state),
        results.completion_reason);
    
    return ESP_OK;
}

esp_err_t automatic_mode_reset(void) {
    ESP_LOGI(TAG, "Resetting automatic mode");
    
    // Stop any active operation
    if (automatic_mode_is_active()) {
        automatic_mode_interrupt();
    }
    
    // Reset all state
    memset(&g_auto_progress, 0, sizeof(g_auto_progress));
    memset(&g_auto_results, 0, sizeof(g_auto_results));
    
    g_auto_progress.state = AUTO_MODE_IDLE;
    strcpy(g_auto_progress.status_message, "Automatic mode reset");
    
    // Reset control variables
    g_current_acceleration_target = 0.0f;
    g_coasting_in_progress = false;
    g_user_interruption_requested = false;
    
    return ESP_OK;
}

uint32_t automatic_mode_get_operating_time(void) {
    if (g_auto_progress.mode_start_time == 0) {
        return 0;
    }
    
    return (esp_timer_get_time() - g_auto_progress.mode_start_time) / 1000;
}

float automatic_mode_get_average_speed(void) {
    if (g_auto_progress.cycle_data.total_distance_m == 0 || automatic_mode_get_operating_time() == 0) {
        return 0.0f;
    }
    
    float operating_time_s = automatic_mode_get_operating_time() / 1000.0f;
    return g_auto_progress.cycle_data.total_distance_m / operating_time_s;
}