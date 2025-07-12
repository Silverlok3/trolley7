// components/manual_mode/src/manual_mode.cpp - COMPLETE VERSION
// ═══════════════════════════════════════════════════════════════════════════════
// MANUAL_MODE.CPP - MODE 3: MANUAL CONTROL IMPLEMENTATION
// ═══════════════════════════════════════════════════════════════════════════════

#include "manual_mode.h"
#include "hardware_control.h"
#include "sensor_health.h"
#include "mode_coordinator.h"
#include "esp_log.h"
#include "esp_timer.h"
#include <cstring>
#include <cmath>

static const char* TAG = "MANUAL_MODE";

// FIXED: Add missing constant definition
#define MOTION_VALIDATION_TIMEOUT_MS    2000      // 2 second timeout for motion validation

// ═══════════════════════════════════════════════════════════════════════════════
// GLOBAL MANUAL MODE STATE - FIXED: Proper struct initialization
// ═══════════════════════════════════════════════════════════════════════════════

static manual_mode_status_t g_manual_status = {
    .state = MANUAL_MODE_IDLE,
    .state_start_time = 0,
    .mode_start_time = 0,
    .current_speed_ms = 0.0f,
    .target_speed_ms = 0.0f,
    .direction_forward = true,
    .motor_active = false,
    .esc_armed = false,
    .esc_responding = false,
    .esc_arm_time = 0,
    .last_command = {MANUAL_CMD_NONE, 0.0f, true, 0, false, {0}},
    .command_count = 0,
    .last_command_time = 0,
    .max_speed_reached = 0.0f,
    .total_distance_traveled = 0.0f,
    .safety_violations = 0,
    .status_message = {0},
    .error_message = {0},
    .error_count = 0
};

static manual_mode_session_stats_t g_session_stats = {
    .total_commands_executed = 0,
    .forward_commands = 0,
    .backward_commands = 0,
    .speed_changes = 0,
    .esc_arm_disarm_cycles = 0,
    .max_speed_used = 0.0f,
    .total_distance_traveled_m = 0.0f,
    .session_duration_ms = 0,
    .motor_active_time_ms = 0,
    .average_speed = 0.0f
};

static bool g_manual_initialized = false;

// Command rate limiting
static uint64_t g_command_times[MANUAL_COMMAND_HISTORY_SIZE] = {0};
static int g_command_history_index = 0;
static uint32_t g_commands_this_second = 0;
static uint64_t g_last_second_start = 0;

// Safety monitoring
static uint64_t g_last_motion_validation_time = 0;
static uint32_t g_motion_validation_failures = 0;

// ═══════════════════════════════════════════════════════════════════════════════
// COMMAND VALIDATION AND RATE LIMITING
// ═══════════════════════════════════════════════════════════════════════════════

bool manual_mode_check_command_rate_limit(void) {
    uint64_t current_time = esp_timer_get_time();
    
    // Reset counter every second
    if (current_time - g_last_second_start > 1000000ULL) {
        g_commands_this_second = 0;
        g_last_second_start = current_time;
    }
    
    // Check rate limit
    if (g_commands_this_second >= MANUAL_MAX_COMMANDS_PER_SEC) {
        ESP_LOGW(TAG, "Command rate limit exceeded: %lu commands this second", g_commands_this_second);
        return false;
    }
    
    // Update command history
    g_command_times[g_command_history_index] = current_time;
    g_command_history_index = (g_command_history_index + 1) % MANUAL_COMMAND_HISTORY_SIZE;
    g_commands_this_second++;
    
    return true;
}

bool manual_mode_validate_command(const manual_command_t* command) {
    if (command == NULL) {
        return false;
    }
    
    // Check rate limiting
    if (!manual_mode_check_command_rate_limit()) {
        return false;
    }
    
    // Check if manual mode is active
    if (!manual_mode_is_active()) {
        ESP_LOGW(TAG, "Manual mode not active - command rejected");
        return false;
    }
    
    // Validate speed parameter
    if (command->type == MANUAL_CMD_SET_SPEED || 
        command->type == MANUAL_CMD_FORWARD || 
        command->type == MANUAL_CMD_BACKWARD) {
        if (!manual_mode_is_speed_safe(command->speed_parameter)) {
            ESP_LOGW(TAG, "Unsafe speed parameter: %.2f m/s", command->speed_parameter);
            return false;
        }
    }
    
    // Check ESC requirements for motor commands
    if (command->type == MANUAL_CMD_SET_SPEED || 
        command->type == MANUAL_CMD_FORWARD || 
        command->type == MANUAL_CMD_BACKWARD) {
        if (!g_manual_status.esc_armed) {
            ESP_LOGW(TAG, "ESC not armed - motor command rejected");
            return false;
        }
    }
    
    // Check sensor requirements
    if (!manual_mode_monitor_sensor_health()) {
        ESP_LOGW(TAG, "Sensor health check failed - command rejected");
        return false;
    }
    
    return true;
}

bool manual_mode_is_speed_safe(float speed_ms) {
    if (speed_ms < 0.0f || speed_ms > MANUAL_MODE_MAX_SPEED_MS) {
        return false;
    }
    
    // Check if speed increment is reasonable
    float current_speed = g_manual_status.current_speed_ms;
    float speed_change = fabs(speed_ms - current_speed);
    
    if (speed_change > 1.0f) { // More than 1 m/s change at once
        ESP_LOGW(TAG, "Large speed change detected: %.2f to %.2f m/s", current_speed, speed_ms);
        return false;
    }
    
    return true;
}

// ═══════════════════════════════════════════════════════════════════════════════
// COMMAND EXECUTION
// ═══════════════════════════════════════════════════════════════════════════════

esp_err_t manual_mode_execute_command(const manual_command_t* command) {
    if (!manual_mode_validate_command(command)) {
        return ESP_ERR_INVALID_ARG;
    }
    
    ESP_LOGI(TAG, "Executing command: %s from %s", 
            manual_mode_command_type_to_string(command->type), command->source);
    
    esp_err_t result = ESP_OK;
    
    switch (command->type) {
        case MANUAL_CMD_SET_SPEED:
            result = manual_mode_set_speed(command->speed_parameter, command->direction_forward);
            if (result == ESP_OK) {
                g_session_stats.speed_changes++;
            }
            break;
            
        case MANUAL_CMD_FORWARD:
            result = manual_mode_move_forward();
            if (result == ESP_OK) {
                g_session_stats.forward_commands++;
            }
            break;
            
        case MANUAL_CMD_BACKWARD:
            result = manual_mode_move_backward();
            if (result == ESP_OK) {
                g_session_stats.backward_commands++;
            }
            break;
            
        case MANUAL_CMD_STOP:
            result = manual_mode_stop_movement();
            break;
            
        case MANUAL_CMD_ARM_ESC:
            result = manual_mode_arm_esc();
            if (result == ESP_OK) {
                g_session_stats.esc_arm_disarm_cycles++;
            }
            break;
            
        case MANUAL_CMD_DISARM_ESC:
            result = manual_mode_disarm_esc();
            break;
            
        case MANUAL_CMD_EMERGENCY_STOP:
            result = manual_mode_emergency_stop();
            break;
            
        case MANUAL_CMD_INCREASE_SPEED:
            result = manual_mode_increase_speed();
            if (result == ESP_OK) {
                g_session_stats.speed_changes++;
            }
            break;
            
        case MANUAL_CMD_DECREASE_SPEED:
            result = manual_mode_decrease_speed();
            if (result == ESP_OK) {
                g_session_stats.speed_changes++;
            }
            break;
            
        default:
            ESP_LOGW(TAG, "Unknown command type: %d", command->type);
            result = ESP_ERR_INVALID_ARG;
            break;
    }
    
    // Update command tracking
    if (result == ESP_OK) {
        memcpy(&g_manual_status.last_command, command, sizeof(manual_command_t));
        g_manual_status.command_count++;
        g_manual_status.last_command_time = esp_timer_get_time();
        g_session_stats.total_commands_executed++;
    } else {
        g_manual_status.error_count++;
    }
    
    return result;
}

manual_command_t manual_mode_create_command(manual_command_type_t type, 
                                           float speed, 
                                           bool forward, 
                                           const char* source) {
    manual_command_t command = {
        .type = type,
        .speed_parameter = speed,
        .direction_forward = forward,
        .timestamp = esp_timer_get_time(),
        .validated = false,
        .source = {0}
    };
    
    if (source != NULL) {
        strncpy(command.source, source, sizeof(command.source) - 1);
        command.source[sizeof(command.source) - 1] = '\0';
    } else {
        strcpy(command.source, "unknown");
    }
    
    return command;
}

// ═══════════════════════════════════════════════════════════════════════════════
// SPEED CONTROL IMPLEMENTATION
// ═══════════════════════════════════════════════════════════════════════════════

esp_err_t manual_mode_set_speed(float speed_ms, bool forward) {
    if (!manual_mode_is_active()) {
        return ESP_ERR_INVALID_STATE;
    }
    
    if (!manual_mode_is_speed_safe(speed_ms)) {
        return ESP_ERR_INVALID_ARG;
    }
    
    if (!g_manual_status.esc_armed && speed_ms > 0.0f) {
        ESP_LOGW(TAG, "Cannot set speed - ESC not armed");
        return ESP_ERR_INVALID_STATE;
    }
    
    ESP_LOGI(TAG, "Setting speed: %.2f m/s %s", speed_ms, forward ? "forward" : "reverse");
    
    // Update status first
    g_manual_status.target_speed_ms = speed_ms;
    g_manual_status.direction_forward = forward;
    g_manual_status.motor_active = (speed_ms > MANUAL_MODE_MIN_SPEED_MS);
    
    // Update session stats
    if (speed_ms > g_session_stats.max_speed_used) {
        g_session_stats.max_speed_used = speed_ms;
    }
    
    if (speed_ms > g_manual_status.max_speed_reached) {
        g_manual_status.max_speed_reached = speed_ms;
    }
    
    // Apply speed to hardware
    esp_err_t result = hardware_set_motor_speed(speed_ms, forward);
    if (result != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set hardware speed");
        return result;
    }
    
    // Update state
    if (speed_ms > MANUAL_MODE_MIN_SPEED_MS) {
        if (forward) {
            g_manual_status.state = MANUAL_MODE_MOVING_FORWARD;
        } else {
            g_manual_status.state = MANUAL_MODE_MOVING_BACKWARD;
        }
        strcpy(g_manual_status.status_message, "Motor active - manual control");
    } else {
        g_manual_status.state = MANUAL_MODE_ACTIVE;
        strcpy(g_manual_status.status_message, "Motor stopped - ready for commands");
    }
    
    return ESP_OK;
}

esp_err_t manual_mode_move_forward(void) {
    return manual_mode_set_speed(MANUAL_MODE_DEFAULT_SPEED_MS, true);
}

esp_err_t manual_mode_move_backward(void) {
    return manual_mode_set_speed(MANUAL_MODE_DEFAULT_SPEED_MS, false);
}

esp_err_t manual_mode_stop_movement(void) {
    ESP_LOGI(TAG, "Stopping movement");
    
    esp_err_t result = manual_mode_set_speed(0.0f, g_manual_status.direction_forward);
    if (result == ESP_OK) {
        g_manual_status.state = MANUAL_MODE_STOPPING;
        strcpy(g_manual_status.status_message, "Stopping motor...");
        
        // Brief delay for smooth stop
        vTaskDelay(pdMS_TO_TICKS(200));
        
        g_manual_status.state = MANUAL_MODE_ACTIVE;
        strcpy(g_manual_status.status_message, "Motor stopped - ready for commands");
    }
    
    return result;
}

esp_err_t manual_mode_increase_speed(void) {
    float new_speed = g_manual_status.target_speed_ms + MANUAL_MODE_SPEED_INCREMENT;
    
    if (new_speed > MANUAL_MODE_MAX_SPEED_MS) {
        ESP_LOGW(TAG, "Cannot increase speed - already at maximum");
        return ESP_ERR_INVALID_ARG;
    }
    
    return manual_mode_set_speed(new_speed, g_manual_status.direction_forward);
}

esp_err_t manual_mode_decrease_speed(void) {
    float new_speed = g_manual_status.target_speed_ms - MANUAL_MODE_SPEED_INCREMENT;
    
    if (new_speed < 0.0f) {
        new_speed = 0.0f;
    }
    
    return manual_mode_set_speed(new_speed, g_manual_status.direction_forward);
}

float manual_mode_get_current_speed(void) {
    return g_manual_status.target_speed_ms;
}

bool manual_mode_get_current_direction(void) {
    return g_manual_status.direction_forward;
}

bool manual_mode_is_moving(void) {
    return g_manual_status.motor_active;
}

// ═══════════════════════════════════════════════════════════════════════════════
// ESC CONTROL IMPLEMENTATION
// ═══════════════════════════════════════════════════════════════════════════════

esp_err_t manual_mode_arm_esc(void) {
    if (!manual_mode_is_active()) {
        return ESP_ERR_INVALID_STATE;
    }
    
    if (g_manual_status.esc_armed) {
        ESP_LOGW(TAG, "ESC already armed");
        return ESP_OK;
    }
    
    ESP_LOGI(TAG, "Arming ESC in manual mode");
    
    g_manual_status.state = MANUAL_MODE_ESC_ARMING;
    strcpy(g_manual_status.status_message, "Arming ESC...");
    
    esp_err_t result = hardware_esc_arm();
    if (result == ESP_OK) {
        g_manual_status.esc_armed = true;
        g_manual_status.esc_responding = true;
        g_manual_status.esc_arm_time = esp_timer_get_time();
        g_manual_status.state = MANUAL_MODE_ACTIVE;
        strcpy(g_manual_status.status_message, "ESC armed - ready for motor commands");
        
        ESP_LOGI(TAG, "ESC armed successfully in manual mode");
    } else {
        g_manual_status.state = MANUAL_MODE_ERROR;
        strcpy(g_manual_status.error_message, "Failed to arm ESC");
        ESP_LOGE(TAG, "Failed to arm ESC in manual mode");
    }
    
    return result;
}

esp_err_t manual_mode_disarm_esc(void) {
    ESP_LOGI(TAG, "Disarming ESC in manual mode");
    
    // Stop motor first
    manual_mode_stop_movement();
    
    g_manual_status.state = MANUAL_MODE_ESC_DISARMING;
    strcpy(g_manual_status.status_message, "Disarming ESC...");
    
    esp_err_t result = hardware_esc_disarm();
    
    g_manual_status.esc_armed = false;
    g_manual_status.esc_responding = false;
    g_manual_status.motor_active = false;
    g_manual_status.target_speed_ms = 0.0f;
    g_manual_status.state = MANUAL_MODE_READY;
    strcpy(g_manual_status.status_message, "ESC disarmed - ready for arming");
    
    ESP_LOGI(TAG, "ESC disarmed in manual mode");
    return result;
}

bool manual_mode_is_esc_armed(void) {
    return g_manual_status.esc_armed;
}

bool manual_mode_is_esc_responding(void) {
    return g_manual_status.esc_responding && hardware_esc_is_armed();
}

uint32_t manual_mode_get_esc_armed_time(void) {
    if (!g_manual_status.esc_armed) {
        return 0;
    }
    
    return (esp_timer_get_time() - g_manual_status.esc_arm_time) / 1000;
}

// ═══════════════════════════════════════════════════════════════════════════════
// SAFETY AND MONITORING
// ═══════════════════════════════════════════════════════════════════════════════

bool manual_mode_monitor_sensor_health(void) {
    sensor_health_t sensor_status = sensor_health_get_status();
    
    // Check if sensors are still validated
    if (!mode_coordinator_are_sensors_validated()) {
        ESP_LOGW(TAG, "Sensors no longer validated");
        return false;
    }
    
    // Check for excessive impact
    if (manual_mode_check_impact_detection()) {
        ESP_LOGW(TAG, "Excessive impact detected");
        return false;
    }
    
    // Check Hall sensor during movement
    if (g_manual_status.motor_active && !manual_mode_monitor_hall_sensor()) {
        ESP_LOGW(TAG, "Hall sensor not responding during movement");
        return false;
    }
    
    return true;
}

bool manual_mode_check_impact_detection(void) {
    sensor_health_t sensor_status = sensor_health_get_status();
    
    if (sensor_status.total_accel_g > MANUAL_MODE_MAX_IMPACT_G) {
        ESP_LOGW(TAG, "Impact detected: %.2f g > %.2f g threshold", 
                sensor_status.total_accel_g, MANUAL_MODE_MAX_IMPACT_G);
        
        // Auto-stop on high impact
        manual_mode_emergency_stop();
        return true;
    }
    
    return false;
}

bool manual_mode_monitor_hall_sensor(void) {
    uint64_t current_time = esp_timer_get_time();
    
    // Only check during movement
    if (!g_manual_status.motor_active) {
        return true;
    }
    
    // Check if we're getting expected Hall pulses during movement
    uint64_t time_since_validation = current_time - g_last_motion_validation_time;
    
    if (time_since_validation > MOTION_VALIDATION_TIMEOUT_MS * 1000ULL) {
        // Check if we got Hall pulses in the last period
        if (hardware_get_time_since_last_hall_pulse() > MOTION_VALIDATION_TIMEOUT_MS * 1000ULL) {
            g_motion_validation_failures++;
            
            if (g_motion_validation_failures >= 3) {
                ESP_LOGW(TAG, "Hall sensor validation failed - no movement detected");
                manual_mode_emergency_stop();
                return false;
            }
        } else {
            g_motion_validation_failures = 0;
        }
        
        g_last_motion_validation_time = current_time;
    }
    
    return true;
}

bool manual_mode_is_operation_safe(void) {
    // Check system health
    if (!mode_coordinator_is_motion_safe()) {
        return false;
    }
    
    // Check sensor health
    if (!manual_mode_monitor_sensor_health()) {
        return false;
    }
    
    // Check ESC health if armed
    if (g_manual_status.esc_armed && !manual_mode_is_esc_responding()) {
        ESP_LOGW(TAG, "ESC not responding");
        return false;
    }
    
    return true;
}

esp_err_t manual_mode_emergency_stop(void) {
    ESP_LOGW(TAG, "EMERGENCY STOP activated in manual mode");
    
    // Stop hardware immediately
    hardware_emergency_stop();
    
    // Update state
    g_manual_status.state = MANUAL_MODE_EMERGENCY_STOP;
    g_manual_status.motor_active = false;
    g_manual_status.target_speed_ms = 0.0f;
    g_manual_status.current_speed_ms = 0.0f;
    strcpy(g_manual_status.status_message, "EMERGENCY STOP - All motion halted");
    
    return ESP_OK;
}

// ═══════════════════════════════════════════════════════════════════════════════
// POSITION TRACKING
// ═══════════════════════════════════════════════════════════════════════════════

float manual_mode_get_current_position(void) {
    return hardware_get_current_position();
}

esp_err_t manual_mode_reset_position(void) {
    ESP_LOGI(TAG, "Resetting position tracking in manual mode");
    
    esp_err_t result = hardware_reset_position();
    if (result == ESP_OK) {
        g_manual_status.total_distance_traveled = 0.0f;
        g_session_stats.total_distance_traveled_m = 0.0f;
    }
    
    return result;
}

// ═══════════════════════════════════════════════════════════════════════════════
// PUBLIC API IMPLEMENTATION
// ═══════════════════════════════════════════════════════════════════════════════

esp_err_t manual_mode_init(void) {
    ESP_LOGI(TAG, "Initializing manual mode...");
    
    // Reset manual mode state - already properly initialized above
    g_manual_status.state = MANUAL_MODE_IDLE;
    strcpy(g_manual_status.status_message, "Manual mode ready");
    strcpy(g_manual_status.error_message, "");
    
    // Reset rate limiting
    memset(g_command_times, 0, sizeof(g_command_times));
    g_command_history_index = 0;
    g_commands_this_second = 0;
    g_last_second_start = 0;
    
    // Reset safety monitoring
    g_last_motion_validation_time = 0;
    g_motion_validation_failures = 0;
    
    g_manual_initialized = true;
    
    ESP_LOGI(TAG, "Manual mode initialized");
    return ESP_OK;
}

esp_err_t manual_mode_start(void) {
    if (!g_manual_initialized) {
        return ESP_ERR_INVALID_STATE;
    }
    
    ESP_LOGI(TAG, "Starting manual mode...");
    
    // Check prerequisites
    if (!mode_coordinator_are_sensors_validated()) {
        ESP_LOGE(TAG, "Cannot start manual mode - sensors not validated");
        return ESP_ERR_INVALID_STATE;
    }
    
    // Reset session data
    memset(&g_session_stats, 0, sizeof(g_session_stats));
    g_session_stats.session_duration_ms = 0;
    
    // Update state
    g_manual_status.state = MANUAL_MODE_READY;
    g_manual_status.mode_start_time = esp_timer_get_time();
    g_manual_status.esc_armed = false;
    g_manual_status.motor_active = false;
    g_manual_status.target_speed_ms = 0.0f;
    g_manual_status.current_speed_ms = 0.0f;
    g_manual_status.direction_forward = true;
    g_manual_status.command_count = 0;
    g_manual_status.error_count = 0;
    
    strcpy(g_manual_status.status_message, "Manual mode active - ARM ESC to enable motor control");
    strcpy(g_manual_status.error_message, "");
    
    // Reset position tracking
    hardware_reset_position();
    
    ESP_LOGI(TAG, "Manual mode started successfully");
    return ESP_OK;
}

esp_err_t manual_mode_stop(void) {
    ESP_LOGI(TAG, "Stopping manual mode...");
    
    // Stop movement and disarm ESC
    manual_mode_stop_movement();
    if (g_manual_status.esc_armed) {
        manual_mode_disarm_esc();
    }
    
    // Calculate final session stats
    g_session_stats.session_duration_ms = (esp_timer_get_time() - g_manual_status.mode_start_time) / 1000;
    g_session_stats.total_commands_executed = g_manual_status.command_count;
    
    if (g_session_stats.motor_active_time_ms > 0) {
        g_session_stats.average_speed = g_session_stats.total_distance_traveled_m / 
                                       (g_session_stats.motor_active_time_ms / 1000.0f);
    }
    
    // Update state
    g_manual_status.state = MANUAL_MODE_IDLE;
    strcpy(g_manual_status.status_message, "Manual mode stopped");
    
    ESP_LOGI(TAG, "Manual mode stopped - Session: %lu commands, %.2f m traveled", 
            g_session_stats.total_commands_executed, g_session_stats.total_distance_traveled_m);
    
    return ESP_OK;
}

esp_err_t manual_mode_update(void) {
    if (!g_manual_initialized || g_manual_status.state == MANUAL_MODE_IDLE) {
        return ESP_OK;
    }
    
    // Update current speed from hardware
    g_manual_status.current_speed_ms = hardware_get_current_speed();
    
    // Update ESC status
    g_manual_status.esc_responding = hardware_esc_is_armed();
    
    // Update distance tracking
    static float last_position = 0.0f;
    float current_position = hardware_get_current_position();
    float distance_increment = fabs(current_position - last_position);
    g_manual_status.total_distance_traveled += distance_increment;
    g_session_stats.total_distance_traveled_m += distance_increment;
    last_position = current_position;
    
    // Update motor active time
    static uint64_t last_update_time = 0;
    uint64_t current_time = esp_timer_get_time();
    if (last_update_time > 0 && g_manual_status.motor_active) {
        uint32_t time_increment = (current_time - last_update_time) / 1000;
        g_session_stats.motor_active_time_ms += time_increment;
    }
    last_update_time = current_time;
    
    // Monitor safety
    if (!manual_mode_is_operation_safe()) {
        ESP_LOGW(TAG, "Safety check failed - stopping manual mode");
        manual_mode_emergency_stop();
        return ESP_ERR_INVALID_STATE;
    }
    
    return ESP_OK;
}

bool manual_mode_is_active(void) {
    return g_manual_status.state > MANUAL_MODE_IDLE && 
           g_manual_status.state < MANUAL_MODE_EMERGENCY_STOP;
}

manual_mode_status_t manual_mode_get_status(void) {
    return g_manual_status;
}

manual_mode_session_stats_t manual_mode_get_session_stats(void) {
    return g_session_stats;
}

// ═══════════════════════════════════════════════════════════════════════════════
// USER INTERFACE SUPPORT
// ═══════════════════════════════════════════════════════════════════════════════

esp_err_t manual_mode_process_user_command(char command_char, const char* source) {
    manual_command_t command = {0};
    
    switch (command_char) {
        case 'A':
        case 'a':
            command = manual_mode_create_command(MANUAL_CMD_ARM_ESC, 0.0f, true, source);
            break;
        case 'D':
        case 'd':
            command = manual_mode_create_command(MANUAL_CMD_DISARM_ESC, 0.0f, true, source);
            break;
        case 'F':
        case 'f':
            command = manual_mode_create_command(MANUAL_CMD_FORWARD, MANUAL_MODE_DEFAULT_SPEED_MS, true, source);
            break;
        case 'B':
        case 'b':
            command = manual_mode_create_command(MANUAL_CMD_BACKWARD, MANUAL_MODE_DEFAULT_SPEED_MS, false, source);
            break;
        case 'S':
        case 's':
            command = manual_mode_create_command(MANUAL_CMD_STOP, 0.0f, true, source);
            break;
        case '+':
            command = manual_mode_create_command(MANUAL_CMD_INCREASE_SPEED, 0.0f, true, source);
            break;
        case '-':
            command = manual_mode_create_command(MANUAL_CMD_DECREASE_SPEED, 0.0f, true, source);
            break;
        case 'E':
        case 'e':
            command = manual_mode_create_command(MANUAL_CMD_EMERGENCY_STOP, 0.0f, true, source);
            break;
        default:
            ESP_LOGW(TAG, "Unknown command character: '%c'", command_char);
            return ESP_ERR_INVALID_ARG;
    }
    
    return manual_mode_execute_command(&command);
}

esp_err_t manual_mode_get_available_commands(char* commands_buffer, size_t buffer_size) {
    if (commands_buffer == NULL) return ESP_ERR_INVALID_ARG;
    
    if (g_manual_status.state == MANUAL_MODE_READY) {
        snprintf(commands_buffer, buffer_size, "No commands available in current state");
    }
    
    return ESP_OK;
}

esp_err_t manual_mode_get_command_help(char* help_buffer, size_t buffer_size) {
    if (help_buffer == NULL) return ESP_ERR_INVALID_ARG;
    
    snprintf(help_buffer, buffer_size,
        "MANUAL MODE COMMANDS:\n"
        "A = Arm ESC (required before movement)\n"
        "D = Disarm ESC (safe state)\n"
        "F = Move Forward (%.1f m/s)\n"
        "B = Move Backward (%.1f m/s)\n"
        "S = Stop movement\n"
        "+ = Increase speed (+%.1f m/s)\n"
        "- = Decrease speed (-%.1f m/s)\n"
        "E = Emergency stop (immediate)\n"
        "Max Speed: %.1f m/s\n"
        "Rate Limit: %d commands/second",
        MANUAL_MODE_DEFAULT_SPEED_MS,
        MANUAL_MODE_DEFAULT_SPEED_MS,
        MANUAL_MODE_SPEED_INCREMENT,
        MANUAL_MODE_SPEED_INCREMENT,
        MANUAL_MODE_MAX_SPEED_MS,
        MANUAL_MAX_COMMANDS_PER_SEC);
    
    return ESP_OK;
}

// ═══════════════════════════════════════════════════════════════════════════════
// UTILITY FUNCTIONS
// ═══════════════════════════════════════════════════════════════════════════════

const char* manual_mode_get_status_message(void) {
    return g_manual_status.status_message;
}

const char* manual_mode_get_error_message(void) {
    return g_manual_status.error_message;
}

const char* manual_mode_state_to_string(manual_mode_state_t state) {
    switch (state) {
        case MANUAL_MODE_IDLE: return "Idle";
        case MANUAL_MODE_INITIALIZING: return "Initializing";
        case MANUAL_MODE_READY: return "Ready";
        case MANUAL_MODE_ESC_ARMING: return "ESC Arming";
        case MANUAL_MODE_ACTIVE: return "Active";
        case MANUAL_MODE_MOVING_FORWARD: return "Moving Forward";
        case MANUAL_MODE_MOVING_BACKWARD: return "Moving Backward";
        case MANUAL_MODE_STOPPING: return "Stopping";
        case MANUAL_MODE_ESC_DISARMING: return "ESC Disarming";
        case MANUAL_MODE_ERROR: return "Error";
        case MANUAL_MODE_EMERGENCY_STOP: return "Emergency Stop";
        default: return "Unknown";
    }
}

const char* manual_mode_command_type_to_string(manual_command_type_t type) {
    switch (type) {
        case MANUAL_CMD_NONE: return "None";
        case MANUAL_CMD_SET_SPEED: return "Set Speed";
        case MANUAL_CMD_FORWARD: return "Forward";
        case MANUAL_CMD_BACKWARD: return "Backward";
        case MANUAL_CMD_STOP: return "Stop";
        case MANUAL_CMD_ARM_ESC: return "Arm ESC";
        case MANUAL_CMD_DISARM_ESC: return "Disarm ESC";
        case MANUAL_CMD_EMERGENCY_STOP: return "Emergency Stop";
        case MANUAL_CMD_INCREASE_SPEED: return "Increase Speed";
        case MANUAL_CMD_DECREASE_SPEED: return "Decrease Speed";
        default: return "Unknown";
    }
}

esp_err_t manual_mode_get_detailed_status(char* status_buffer, size_t buffer_size) {
    if (status_buffer == NULL) return ESP_ERR_INVALID_ARG;
    
    snprintf(status_buffer, buffer_size,
        "=== MANUAL MODE STATUS ===\n"
        "State: %s\n"
        "ESC Armed: %s\n"
        "Motor Active: %s\n"
        "Speed: %.2f m/s %s\n"
        "Position: %.2f m\n"
        "Commands: %lu total, %lu errors\n"
        "Session Time: %lu ms\n"
        "Distance: %.2f m\n"
        "Max Speed: %.2f m/s\n"
        "Status: %s\n",
        manual_mode_state_to_string(g_manual_status.state),
        g_manual_status.esc_armed ? "Yes" : "No",
        g_manual_status.motor_active ? "Yes" : "No",
        g_manual_status.current_speed_ms,
        g_manual_status.direction_forward ? "forward" : "reverse",
        manual_mode_get_current_position(),
        g_manual_status.command_count,
        g_manual_status.error_count,
        manual_mode_get_session_duration(),
        g_manual_status.total_distance_traveled,
        g_manual_status.max_speed_reached,
        g_manual_status.status_message);
    
    return ESP_OK;
}

esp_err_t manual_mode_reset_session(void) {
    ESP_LOGI(TAG, "Resetting manual mode session");
    
    // Stop current activity
    if (g_manual_status.motor_active) {
        manual_mode_stop_movement();
    }
    
    // Reset session statistics
    memset(&g_session_stats, 0, sizeof(g_session_stats));
    
    // Reset status tracking
    g_manual_status.command_count = 0;
    g_manual_status.error_count = 0;
    g_manual_status.max_speed_reached = 0.0f;
    g_manual_status.total_distance_traveled = 0.0f;
    g_manual_status.mode_start_time = esp_timer_get_time();
    
    // Reset position
    manual_mode_reset_position();
    
    // Clear error message
    strcpy(g_manual_status.error_message, "");
    
    if (g_manual_status.esc_armed) {
        strcpy(g_manual_status.status_message, "Session reset - ESC armed and ready");
    } else {
        strcpy(g_manual_status.status_message, "Session reset - ARM ESC to enable motor control");
    }
    
    ESP_LOGI(TAG, "Manual mode session reset complete");
    return ESP_OK;
}

uint32_t manual_mode_get_session_duration(void) {
    if (g_manual_status.mode_start_time == 0) {
        return 0;
    }
    
    return (esp_timer_get_time() - g_manual_status.mode_start_time) / 1000;
}

esp_err_t manual_mode_export_session_data(char* data_buffer, size_t buffer_size) {
    if (data_buffer == NULL) return ESP_ERR_INVALID_ARG;
    
    manual_mode_session_stats_t stats = manual_mode_get_session_stats();
    
    snprintf(data_buffer, buffer_size,
        "MANUAL_MODE_SESSION_DATA\n"
        "session_duration_ms=%lu\n"
        "total_commands=%lu\n"
        "forward_commands=%lu\n"
        "backward_commands=%lu\n"
        "speed_changes=%lu\n"
        "esc_cycles=%lu\n"
        "max_speed_ms=%.2f\n"
        "total_distance_m=%.2f\n"
        "motor_active_time_ms=%lu\n"
        "average_speed_ms=%.2f\n"
        "error_count=%lu\n"
        "max_speed_reached=%.2f\n",
        stats.session_duration_ms,
        stats.total_commands_executed,
        stats.forward_commands,
        stats.backward_commands,
        stats.speed_changes,
        stats.esc_arm_disarm_cycles,
        stats.max_speed_used,
        stats.total_distance_traveled_m,
        stats.motor_active_time_ms,
        stats.average_speed,
        g_manual_status.error_count,
        g_manual_status.max_speed_reached);
    
    return ESP_OK;
}buffer, buffer_size, "A=Arm ESC");
    } else if (g_manual_status.state == MANUAL_MODE_ACTIVE) {
        snprintf(commands_buffer, buffer_size, "F=Forward, B=Backward, S=Stop, +=Faster, -=Slower, D=Disarm, E=Emergency");
    } else {
        snprintf(commands_// components/manual_mode/src/manual_mode.cpp


// ═══════════════