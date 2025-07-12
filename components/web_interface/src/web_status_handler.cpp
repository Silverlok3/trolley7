// components/web_interface/src/web_status_handler.cpp
// ═══════════════════════════════════════════════════════════════════════════════
// WEB_STATUS_HANDLER.CPP - STATUS JSON GENERATION
// ═══════════════════════════════════════════════════════════════════════════════
// 
// SINGLE RESPONSIBILITY: Generate JSON status responses
// - Collect status from all components
// - Format as clean JSON for web interface
// - Handle mode-specific status data
// - Provide real-time sensor updates
// ═══════════════════════════════════════════════════════════════════════════════

#include "web_interface.h"
#include "mode_coordinator.h"
#include "hardware_control.h"
#include "sensor_health.h"
#include "wire_learning_mode.h"
#include "automatic_mode.h"
#include "manual_mode.h"
#include "esp_log.h"
#include <cstring>

static const char* TAG = "WEB_STATUS";

// ═══════════════════════════════════════════════════════════════════════════════
// STATUS JSON GENERATION
// ═══════════════════════════════════════════════════════════════════════════════

esp_err_t web_generate_status_json(char* json_buffer, size_t buffer_size) {
    if (json_buffer == NULL) return ESP_ERR_INVALID_ARG;
    
    // Get status from all components
    system_mode_status_t mode_status = mode_coordinator_get_status();
    hardware_status_t hw_status = hardware_get_status();
    sensor_health_t sensor_status = sensor_health_get_status();
    
    // Get mode-specific status
    wire_learning_progress_t wire_progress = {0};
    automatic_mode_progress_t auto_progress = {0};
    manual_mode_status_t manual_status = {0};
    
    if (wire_learning_mode_is_active()) {
        wire_progress = wire_learning_mode_get_progress();
    }
    
    if (automatic_mode_is_active()) {
        auto_progress = automatic_mode_get_progress();
    }
    
    if (manual_mode_is_active()) {
        manual_status = manual_mode_get_status();
    }
    
    // Generate comprehensive JSON status
    snprintf(json_buffer, buffer_size,
        "{"
        "\"system_healthy\": %s,"
        "\"current_mode\": \"%s\","
        "\"current_mode_status\": \"%s\","
        "\"error_message\": \"%s\","
        
        // Sensor Validation Status
        "\"sensors_validated\": %s,"
        "\"sensor_validation_state\": \"%s\","
        "\"sensor_validation_message\": \"%s\","
        "\"hall_validation_complete\": %s,"
        "\"accel_validation_complete\": %s,"
        
        // Mode Availability
        "\"wire_learning_availability\": \"%s\","
        "\"automatic_availability\": \"%s\","
        "\"manual_availability\": \"%s\","
        
        // Hall Sensor Data
        "\"hall_status\": \"%s\","
        "\"hall_pulses\": %lu,"
        "\"wheel_rpm\": %.1f,"
        "\"wheel_speed\": %.2f,"
        "\"wheel_rotation_detected\": %s,"
        
        // Accelerometer Data
        "\"accel_status\": \"%s\","
        "\"accel_total\": %.2f,"
        "\"last_impact\": %.2f,"
        "\"impact_threshold\": %.1f,"
        "\"trolley_shake_detected\": %s,"
        
        // Hardware Status
        "\"esc_armed\": %s,"
        "\"position_m\": %.2f,"
        "\"current_speed_ms\": %.2f,"
        "\"target_speed_ms\": %.2f,"
        "\"direction_forward\": %s,"
        "\"rotations\": %lu,"
        
        // Wire Learning Status
        "\"wire_learning_complete\": %s,"
        "\"wire_length_m\": %.2f,"
        "\"wire_learning_state\": \"%s\","
        "\"wire_learning_progress\": %d,"
        
        // Automatic Mode Status
        "\"auto_cycle_count\": %lu,"
        "\"auto_cycle_interrupted\": %s,"
        "\"auto_coasting_calibrated\": %s,"
        "\"automatic_state\": \"%s\","
        "\"automatic_progress\": %d,"
        
        // Manual Mode Status
        "\"manual_speed\": %.2f,"
        "\"manual_direction_forward\": %s,"
        "\"manual_esc_armed\": %s,"
        "\"manual_motor_active\": %s,"
        "\"manual_state\": \"%s\""
        "}",
        
        // System Status
        mode_status.system_healthy ? "true" : "false",
        mode_coordinator_mode_to_string(mode_status.current_mode),
        mode_status.current_mode_status,
        mode_status.error_message,
        
        // Sensor Validation
        mode_status.sensors_validated ? "true" : "false",
        mode_coordinator_validation_to_string(mode_status.sensor_validation_state),
        mode_status.sensor_validation_message,
        mode_status.hall_validation_complete ? "true" : "false",
        mode_status.accel_validation_complete ? "true" : "false",
        
        // Mode Availability
        mode_coordinator_availability_to_string(mode_status.wire_learning_availability),
        mode_coordinator_availability_to_string(mode_status.automatic_availability),
        mode_coordinator_availability_to_string(mode_status.manual_availability),
        
        // Hall Sensor Data
        sensor_status.hall_status == SENSOR_STATUS_HEALTHY ? "healthy" :
        sensor_status.hall_status == SENSOR_STATUS_FAILED ? "failed" :
        sensor_status.hall_status == SENSOR_STATUS_TIMEOUT ? "timeout" : "testing",
        sensor_status.hall_pulse_count,
        sensor_status.current_rpm,
        sensor_status.wheel_speed_ms,
        sensor_status.wheel_rotation_detected ? "true" : "false",
        
        // Accelerometer Data
        sensor_status.accel_status == SENSOR_STATUS_HEALTHY ? "healthy" :
        sensor_status.accel_status == SENSOR_STATUS_FAILED ? "failed" :
        sensor_status.accel_status == SENSOR_STATUS_TIMEOUT ? "timeout" : "testing",
        sensor_status.total_accel_g,
        sensor_status.last_impact_g,
        0.5f, // Default impact threshold
        sensor_status.trolley_shake_detected ? "true" : "false",
        
        // Hardware Status
        hw_status.esc_armed ? "true" : "false",
        hw_status.current_position_m,
        hw_status.current_speed_ms,
        hw_status.target_speed_ms,
        hw_status.direction_forward ? "true" : "false",
        hw_status.total_rotations,
        
        // Wire Learning Status
        mode_status.wire_learning.complete ? "true" : "false",
        mode_status.wire_learning.wire_length_m,
        wire_learning_state_to_string(wire_progress.state),
        wire_learning_mode_is_active() ? wire_learning_get_progress_percentage() : 
            (mode_status.wire_learning.complete ? 100 : 0),
        
        // Automatic Mode Status
        mode_status.auto_cycle_count,
        mode_status.auto_cycle_interrupted ? "true" : "false",
        mode_status.auto_coasting_calibrated ? "true" : "false",
        automatic_mode_state_to_string(auto_progress.state),
        automatic_mode_is_active() ? automatic_mode_get_progress_percentage() : 0,
        
        // Manual Mode Status
        manual_status.current_speed_ms,
        manual_status.direction_forward ? "true" : "false",
        manual_status.esc_armed ? "true" : "false",
        manual_status.motor_active ? "true" : "false",
        manual_mode_state_to_string(manual_status.state)
    );
    
    return ESP_OK;
}

// ═══════════════════════════════════════════════════════════════════════════════
// UTILITY FUNCTIONS
// ═══════════════════════════════════════════════════════════════════════════════

esp_err_t web_generate_simple_status_json(char* json_buffer, size_t buffer_size) {
    if (json_buffer == NULL) return ESP_ERR_INVALID_ARG;
    
    // Lightweight status for frequent polling
    system_mode_status_t mode_status = mode_coordinator_get_status();
    hardware_status_t hw_status = hardware_get_status();
    sensor_health_t sensor_status = sensor_health_get_status();
    
    snprintf(json_buffer, buffer_size,
        "{"
        "\"mode\": \"%s\","
        "\"healthy\": %s,"
        "\"speed\": %.2f,"
        "\"hall_pulses\": %lu,"
        "\"accel_g\": %.2f,"
        "\"esc_armed\": %s"
        "}",
        mode_coordinator_mode_to_string(mode_status.current_mode),
        mode_status.system_healthy ? "true" : "false",
        hw_status.current_speed_ms,
        sensor_status.hall_pulse_count,
        sensor_status.total_accel_g,
        hw_status.esc_armed ? "true" : "false"
    );
    
    return ESP_OK;
}

esp_err_t web_generate_sensor_status_json(char* json_buffer, size_t buffer_size) {
    if (json_buffer == NULL) return ESP_ERR_INVALID_ARG;
    
    // Sensor-specific status for validation UI
    sensor_health_t sensor_status = sensor_health_get_status();
    system_mode_status_t mode_status = mode_coordinator_get_status();
    
    snprintf(json_buffer, buffer_size,
        "{"
        "\"validation_state\": \"%s\","
        "\"validation_message\": \"%s\","
        "\"hall_healthy\": %s,"
        "\"hall_pulses\": %lu,"
        "\"wheel_rotating\": %s,"
        "\"accel_healthy\": %s,"
        "\"accel_total\": %.2f,"
        "\"trolley_shaking\": %s,"
        "\"sensors_validated\": %s"
        "}",
        mode_coordinator_validation_to_string(mode_status.sensor_validation_state),
        mode_status.sensor_validation_message,
        sensor_status.hall_status == SENSOR_STATUS_HEALTHY ? "true" : "false",
        sensor_status.hall_pulse_count,
        sensor_status.wheel_rotation_detected ? "true" : "false",
        sensor_status.accel_status == SENSOR_STATUS_HEALTHY ? "true" : "false",
        sensor_status.total_accel_g,
        sensor_status.trolley_shake_detected ? "true" : "false",
        mode_status.sensors_validated ? "true" : "false"
    );
    
    return ESP_OK;
}

esp_err_t web_generate_mode_status_json(char* json_buffer, size_t buffer_size) {
    if (json_buffer == NULL) return ESP_ERR_INVALID_ARG;
    
    // Mode-specific status for mode selection UI
    system_mode_status_t mode_status = mode_coordinator_get_status();
    
    snprintf(json_buffer, buffer_size,
        "{"
        "\"current_mode\": \"%s\","
        "\"wire_learning_availability\": \"%s\","
        "\"automatic_availability\": \"%s\","
        "\"manual_availability\": \"%s\","
        "\"wire_learning_complete\": %s,"
        "\"wire_length_m\": %.2f,"
        "\"auto_cycle_count\": %lu"
        "}",
        mode_coordinator_mode_to_string(mode_status.current_mode),
        mode_coordinator_availability_to_string(mode_status.wire_learning_availability),
        mode_coordinator_availability_to_string(mode_status.automatic_availability),
        mode_coordinator_availability_to_string(mode_status.manual_availability),
        mode_status.wire_learning.complete ? "true" : "false",
        mode_status.wire_learning.wire_length_m,
        mode_status.auto_cycle_count
    );
    
    return ESP_OK;
}