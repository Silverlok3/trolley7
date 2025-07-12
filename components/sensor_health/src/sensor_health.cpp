// components/sensor_health/src/sensor_health.cpp - FIXED VERSION
#include "sensor_health.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <cmath>
#include <cstring>

static const char* TAG = "SENSOR_HEALTH";

// Global sensor health data - FIXED: Proper struct initialization
static sensor_health_t g_sensor_health = {
    .hall_status = SENSOR_STATUS_UNKNOWN,
    .hall_pulse_count = 0,
    .last_hall_pulse_time = 0,
    .current_rpm = 0.0f,
    .wheel_speed_ms = 0.0f,
    .wheel_rotation_detected = false,
    .accel_status = SENSOR_STATUS_UNKNOWN,
    .accel_x_g = 0.0f,
    .accel_y_g = 0.0f,
    .accel_z_g = 0.0f,
    .total_accel_g = 0.0f,
    .last_impact_g = 0.0f,
    .last_impact_time = 0,
    .trolley_shake_detected = false,
    .init_state = INIT_STATE_START,
    .status_message = {0},
    .error_message = {0},
    .init_start_time = 0,
    .sensors_validated = false,
    .system_ready = false
};

static MPU_t* g_mpu_sensor = nullptr;
static uint32_t g_last_hall_count = 0;
static uint64_t g_last_hall_time = 0;
static bool g_validation_active = false;

// Initialize sensor health monitoring
esp_err_t sensor_health_init(MPU_t* mpu_sensor) {
    ESP_LOGI(TAG, "Initializing sensor health monitoring system...");
    ESP_LOGI(TAG, "Wheel: 61mm diameter, 191.6mm circumference");
    
    g_mpu_sensor = mpu_sensor;
    
    // Initialize sensor health structure with proper enum values
    g_sensor_health.hall_status = SENSOR_STATUS_UNKNOWN;
    g_sensor_health.accel_status = SENSOR_STATUS_UNKNOWN;
    g_sensor_health.init_state = INIT_STATE_START;
    g_sensor_health.init_start_time = esp_timer_get_time();
    
    strcpy(g_sensor_health.status_message, "System initializing...");
    strcpy(g_sensor_health.error_message, "");
    
    ESP_LOGI(TAG, "Sensor health monitoring initialized");
    return ESP_OK;
}

// Main sensor health update function (call regularly)
void sensor_health_update(void) {
    uint64_t current_time = esp_timer_get_time();
    uint64_t elapsed_time = current_time - g_sensor_health.init_start_time;
    
    switch (g_sensor_health.init_state) {
        case INIT_STATE_START:
            g_sensor_health.init_state = INIT_STATE_WAIT_WHEEL_ROTATION;
            g_sensor_health.hall_status = SENSOR_STATUS_TESTING;
            strcpy(g_sensor_health.status_message, "ROTATE THE WHEEL - Testing Hall sensor...");
            ESP_LOGI(TAG, "=== ROTATE THE WHEEL ===");
            g_validation_active = true;
            break;
            
        case INIT_STATE_WAIT_WHEEL_ROTATION:
            if (g_sensor_health.wheel_rotation_detected) {
                g_sensor_health.hall_status = SENSOR_STATUS_HEALTHY;
                g_sensor_health.init_state = INIT_STATE_WAIT_TROLLEY_SHAKE;
                g_sensor_health.accel_status = SENSOR_STATUS_TESTING;
                strcpy(g_sensor_health.status_message, "SHAKE THE TROLLEY - Testing accelerometer...");
                ESP_LOGI(TAG, "Hall sensor OK. === SHAKE THE TROLLEY ===");
            } else if (elapsed_time > HALL_VALIDATION_TIMEOUT_MS * 1000ULL) {
                g_sensor_health.hall_status = SENSOR_STATUS_TIMEOUT;
                g_sensor_health.init_state = INIT_STATE_FAILED;
                strcpy(g_sensor_health.error_message, "No wheel rotation detected - check/replace Hall sensor");
                ESP_LOGE(TAG, "Hall sensor validation FAILED - timeout");
            }
            break;
            
        case INIT_STATE_WAIT_TROLLEY_SHAKE:
            // Update accelerometer readings
            if (g_mpu_sensor) {
                mpud::raw_axes_t accelRaw, gyroRaw;
                if (g_mpu_sensor->motion(&accelRaw, &gyroRaw) == ESP_OK) {
                    // Convert to g-force (assuming ±8g range, 4096 LSB/g for MPU6050)
                    float x_g = accelRaw.x / 4096.0f;
                    float y_g = accelRaw.y / 4096.0f;
                    float z_g = accelRaw.z / 4096.0f;
                    sensor_health_process_accel_data(x_g, y_g, z_g);
                }
            }
            
            if (g_sensor_health.trolley_shake_detected) {
                g_sensor_health.accel_status = SENSOR_STATUS_HEALTHY;
                g_sensor_health.init_state = INIT_STATE_SENSORS_READY;
                strcpy(g_sensor_health.status_message, "Both sensors validated - System ready!");
                ESP_LOGI(TAG, "Accelerometer OK. Both sensors validated!");
            } else if (elapsed_time > ACCEL_VALIDATION_TIMEOUT_MS * 1000ULL) {
                g_sensor_health.accel_status = SENSOR_STATUS_TIMEOUT;
                g_sensor_health.init_state = INIT_STATE_FAILED;
                strcpy(g_sensor_health.error_message, "Fix/replace accelerometer or its connections");
                ESP_LOGE(TAG, "Accelerometer validation FAILED - timeout");
            }
            break;
            
        case INIT_STATE_SENSORS_READY:
            g_sensor_health.sensors_validated = true;
            g_sensor_health.system_ready = true;
            g_sensor_health.init_state = INIT_STATE_SYSTEM_READY;
            g_validation_active = false;
            strcpy(g_sensor_health.status_message, "System operational - All sensors healthy");
            break;
            
        case INIT_STATE_SYSTEM_READY:
            // Continuous health monitoring
            sensor_health_validate_hall_sensor();
            if (g_mpu_sensor) {
                mpud::raw_axes_t accelRaw, gyroRaw;
                if (g_mpu_sensor->motion(&accelRaw, &gyroRaw) == ESP_OK) {
                    float x_g = accelRaw.x / 4096.0f;
                    float y_g = accelRaw.y / 4096.0f;
                    float z_g = accelRaw.z / 4096.0f;
                    sensor_health_process_accel_data(x_g, y_g, z_g);
                }
            }
            break;
            
        case INIT_STATE_FAILED:
            // System blocked - sensors failed validation
            g_sensor_health.system_ready = false;
            break;
    }
}

// Hall sensor pulse detected callback
void sensor_health_hall_pulse_detected(void) {
    uint64_t current_time = esp_timer_get_time();
    g_sensor_health.hall_pulse_count++;
    g_sensor_health.last_hall_pulse_time = current_time;
    
    // Calculate RPM and wheel speed
    if (g_last_hall_time > 0) {
        uint64_t time_diff_us = current_time - g_last_hall_time;
        if (time_diff_us > 0) {
            // Calculate RPM (assuming 1 pulse per revolution)
            float time_diff_s = time_diff_us / 1000000.0f;
            g_sensor_health.current_rpm = 60.0f / time_diff_s;
            
            // Calculate wheel speed using 61mm wheel circumference
            g_sensor_health.wheel_speed_ms = (WHEEL_CIRCUMFERENCE_MM * 0.001f) * 
                                           (g_sensor_health.current_rpm / 60.0f);
        }
    }
    
    g_last_hall_time = current_time;
    
    // Mark wheel rotation as detected during validation
    if (g_validation_active) {
        g_sensor_health.wheel_rotation_detected = true;
        ESP_LOGI(TAG, "Wheel rotation detected! RPM: %.1f, Speed: %.2f m/s", 
                g_sensor_health.current_rpm, g_sensor_health.wheel_speed_ms);
    }
}

// Process accelerometer data
void sensor_health_process_accel_data(float x_g, float y_g, float z_g) {
    g_sensor_health.accel_x_g = x_g;
    g_sensor_health.accel_y_g = y_g;
    g_sensor_health.accel_z_g = z_g;
    
    // Calculate total acceleration magnitude (subtract gravity)
    float total_accel = sqrt(x_g * x_g + y_g * y_g + z_g * z_g);
    g_sensor_health.total_accel_g = total_accel;
    
    // Detect shake during validation (above threshold)
    if (g_validation_active && total_accel > MINIMUM_SHAKE_THRESHOLD_G) {
        g_sensor_health.trolley_shake_detected = true;
        ESP_LOGI(TAG, "Trolley shake detected! Accel: %.2f g", total_accel);
    }
    
    // Detect impacts during normal operation
    if (!g_validation_active && total_accel > IMPACT_THRESHOLD_G) {
        g_sensor_health.last_impact_g = total_accel;
        g_sensor_health.last_impact_time = esp_timer_get_time();
        ESP_LOGI(TAG, "Impact detected: %.2f g", total_accel);
    }
}

// Validate Hall sensor health
bool sensor_health_validate_hall_sensor(void) {
    uint64_t current_time = esp_timer_get_time();
    
    if (g_sensor_health.last_hall_pulse_time > 0) {
        uint64_t time_since_last_pulse = current_time - g_sensor_health.last_hall_pulse_time;
        
        if (time_since_last_pulse > HALL_PULSE_TIMEOUT_MS * 1000ULL) {
            // No recent pulses - might be stopped or failed
            g_sensor_health.current_rpm = 0.0f;
            g_sensor_health.wheel_speed_ms = 0.0f;
        }
        
        return true; // Has received pulses
    }
    
    return false; // Never received pulses
}

// Check if system is ready for commands
bool sensor_health_check_command_safety(void) {
    if (!g_sensor_health.system_ready) {
        ESP_LOGW(TAG, "Command blocked - sensors not validated");
        return false;
    }
    
    if (!sensor_health_validate_hall_sensor()) {
        ESP_LOGW(TAG, "Command blocked - Hall sensor not responding");
        return false;
    }
    
    return true;
}

// Get sensor health status
sensor_health_t sensor_health_get_status(void) {
    return g_sensor_health;
}

// Check if system is ready
bool sensor_health_is_system_ready(void) {
    return g_sensor_health.system_ready;
}

// Get initialization message
const char* sensor_health_get_init_message(void) {
    if (strlen(g_sensor_health.error_message) > 0) {
        return g_sensor_health.error_message;
    }
    return g_sensor_health.status_message;
}

// Get last impact G-force
float sensor_health_get_last_impact(void) {
    return g_sensor_health.last_impact_g;
}

// Reset validation (for testing)
void sensor_health_reset_validation(void) {
    // Reset validation state but keep enum values proper
    g_sensor_health.init_state = INIT_STATE_START;
    g_sensor_health.hall_status = SENSOR_STATUS_UNKNOWN;
    g_sensor_health.accel_status = SENSOR_STATUS_UNKNOWN;
    g_sensor_health.init_start_time = esp_timer_get_time();
    g_sensor_health.sensors_validated = false;
    g_sensor_health.system_ready = false;
    g_sensor_health.wheel_rotation_detected = false;
    g_sensor_health.trolley_shake_detected = false;
    g_sensor_health.hall_pulse_count = 0;
    g_sensor_health.current_rpm = 0.0f;
    g_sensor_health.wheel_speed_ms = 0.0f;
    g_sensor_health.total_accel_g = 0.0f;
    g_sensor_health.last_impact_g = 0.0f;
    
    strcpy(g_sensor_health.status_message, "");
    strcpy(g_sensor_health.error_message, "");
    
    g_validation_active = false;
    ESP_LOGI(TAG, "Sensor validation reset");
}