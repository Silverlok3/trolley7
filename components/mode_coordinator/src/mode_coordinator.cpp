// components/mode_coordinator/src/mode_coordinator.cpp
// ═══════════════════════════════════════════════════════════════════════════════
// MODE_COORDINATOR.CPP - 3-MODE SYSTEM MANAGEMENT AND COORDINATION
// ═══════════════════════════════════════════════════════════════════════════════
// 
// SINGLE RESPONSIBILITY: Mode management and validation
// - system_set_mode() / system_get_mode_status()
// - Mode availability checking and sensor validation requirements
// - Mode transition safety and error handling coordination
// - Data sharing between wire_learning_mode, automatic_mode, manual_mode
// ═══════════════════════════════════════════════════════════════════════════════

#include "mode_coordinator.h"
#include "hardware_control.h"
#include "sensor_health.h"
#include "wire_learning_mode.h"
#include "automatic_mode.h"
#include "manual_mode.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "nvs_flash.h"
#include "nvs.h"
#include <cstring>
#include <cmath>

static const char* TAG = "MODE_COORDINATOR";

// ═══════════════════════════════════════════════════════════════════════════════
// GLOBAL MODE COORDINATOR STATE
// ═══════════════════════════════════════════════════════════════════════════════

static system_mode_status_t g_mode_status = {0};
static bool g_coordinator_initialized = false;

// Sensor validation tracking
static uint64_t g_sensor_validation_start_time = 0;
static bool g_hall_validation_user_confirmed = false;
static bool g_accel_validation_user_confirmed = false;

// Mode data storage
static wire_learning_results_t g_wire_learning_data = {0};
static coasting_data_t g_coasting_data = {0};

// Error tracking
static uint32_t g_error_count = 0;
static uint64_t g_last_error_reset_time = 0;

// ═══════════════════════════════════════════════════════════════════════════════
// SENSOR VALIDATION FUNCTIONS
// ═══════════════════════════════════════════════════════════════════════════════

static void update_sensor_validation_state(void) {
    sensor_health_t sensor_status = sensor_health_get_status();
    
    switch (g_mode_status.sensor_validation_state) {
        case SENSOR_VALIDATION_NOT_STARTED:
            strcpy(g_mode_status.sensor_validation_message, 
                  "SENSOR VALIDATION REQUIRED: Step 1: ROTATE THE WHEEL manually");
            break;
            
        case SENSOR_VALIDATION_IN_PROGRESS:
            if (sensor_status.wheel_rotation_detected && !g_hall_validation_user_confirmed) {
                g_mode_status.sensor_validation_state = SENSOR_VALIDATION_HALL_PENDING;
                strcpy(g_mode_status.sensor_validation_message,
                      "HALL SENSOR OK! Press 'Confirm Hall Sensor' button to continue");
            }
            break;
            
        case SENSOR_VALIDATION_HALL_PENDING:
            if (g_hall_validation_user_confirmed) {
                g_mode_status.sensor_validation_state = SENSOR_VALIDATION_ACCEL_PENDING;
                g_mode_status.hall_validation_complete = true;
                strcpy(g_mode_status.sensor_validation_message,
                      "Step 2: SHAKE THE TROLLEY to test accelerometer");
            }
            break;
            
        case SENSOR_VALIDATION_ACCEL_PENDING:
            if (sensor_status.trolley_shake_detected && !g_accel_validation_user_confirmed) {
                strcpy(g_mode_status.sensor_validation_message,
                      "ACCELEROMETER OK! Press 'Confirm Accelerometer' button to complete");
            } else if (g_accel_validation_user_confirmed) {
                g_mode_status.sensor_validation_state = SENSOR_VALIDATION_COMPLETE;
                g_mode_status.accel_validation_complete = true;
                g_mode_status.sensors_validated = true;
                strcpy(g_mode_status.sensor_validation_message,
                      "✅ ALL SENSORS VALIDATED - Modes now available");
            }
            break;
            
        case SENSOR_VALIDATION_COMPLETE:
            strcpy(g_mode_status.sensor_validation_message,
                  "✅ Sensors validated and ready for operation");
            break;
            
        case SENSOR_VALIDATION_FAILED:
            strcpy(g_mode_status.sensor_validation_message,
                  "❌ Sensor validation FAILED - Check connections and retry");
            break;
    }
    
    // Check for validation timeout
    if (g_mode_status.sensor_validation_state > SENSOR_VALIDATION_NOT_STARTED &&
        g_mode_status.sensor_validation_state < SENSOR_VALIDATION_COMPLETE) {
        uint64_t elapsed_time = esp_timer_get_time() - g_sensor_validation_start_time;
        if (elapsed_time > SENSOR_VALIDATION_TIMEOUT_MS * 1000ULL) {
            g_mode_status.sensor_validation_state = SENSOR_VALIDATION_FAILED;
            strcpy(g_mode_status.sensor_validation_message,
                  "❌ Validation timeout - Restart validation process");
        }
    }
}

static void update_mode_availability(void) {
    // Wire Learning Mode availability
    if (!g_mode_status.sensors_validated) {
        g_mode_status.wire_learning_availability = MODE_BLOCKED_SENSORS_NOT_VALIDATED;
    } else if (wire_learning_mode_is_active()) {
        g_mode_status.wire_learning_availability = MODE_ACTIVE;
    } else if (!g_mode_status.system_healthy) {
        g_mode_status.wire_learning_availability = MODE_BLOCKED_SYSTEM_ERROR;
    } else {
        g_mode_status.wire_learning_availability = MODE_AVAILABLE;
    }
    
    // Automatic Mode availability
    if (!g_mode_status.sensors_validated) {
        g_mode_status.automatic_availability = MODE_BLOCKED_SENSORS_NOT_VALIDATED;
    } else if (!g_wire_learning_data.complete) {
        g_mode_status.automatic_availability = MODE_BLOCKED_WIRE_LEARNING_REQUIRED;
    } else if (automatic_mode_is_active()) {
        g_mode_status.automatic_availability = MODE_ACTIVE;
    } else if (!g_mode_status.system_healthy) {
        g_mode_status.automatic_availability = MODE_BLOCKED_SYSTEM_ERROR;
    } else {
        g_mode_status.automatic_availability = MODE_AVAILABLE;
    }
    
    // Manual Mode availability
    if (!g_mode_status.sensors_validated) {
        g_mode_status.manual_availability = MODE_BLOCKED_SENSORS_NOT_VALIDATED;
    } else if (manual_mode_is_active()) {
        g_mode_status.manual_availability = MODE_ACTIVE;
    } else if (!g_mode_status.system_healthy) {
        g_mode_status.manual_availability = MODE_BLOCKED_SYSTEM_ERROR;
    } else {
        g_mode_status.manual_availability = MODE_AVAILABLE;
    }
}

static void update_current_mode_status(void) {
    if (wire_learning_mode_is_active()) {
        g_mode_status.current_mode = TROLLEY_MODE_WIRE_LEARNING;
        wire_learning_progress_t progress = wire_learning_mode_get_progress();
        snprintf(g_mode_status.current_mode_status, sizeof(g_mode_status.current_mode_status),
                "Wire Learning: %s - %.1f m/s", 
                wire_learning_state_to_string(progress.state), 
                progress.current_learning_speed);
    } else if (automatic_mode_is_active()) {
        g_mode_status.current_mode = TROLLEY_MODE_AUTOMATIC;
        automatic_mode_progress_t progress = automatic_mode_get_progress();
        snprintf(g_mode_status.current_mode_status, sizeof(g_mode_status.current_mode_status),
                "Automatic: Cycle %lu, %s", 
                progress.cycle_data.cycle_number,
                automatic_mode_state_to_string(progress.state));
    } else if (manual_mode_is_active()) {
        g_mode_status.current_mode = TROLLEY_MODE_MANUAL;
        manual_mode_status_t manual_status = manual_mode_get_status();
        snprintf(g_mode_status.current_mode_status, sizeof(g_mode_status.current_mode_status),
                "Manual: %.1f m/s %s", 
                manual_status.current_speed_ms,
                manual_status.direction_forward ? "forward" : "reverse");
    } else {
        g_mode_status.current_mode = TROLLEY_MODE_NONE;
        strcpy(g_mode_status.current_mode_status, "No active mode");
    }
}

static esp_err_t load_persistent_data(void) {
    nvs_handle_t nvs_handle;
    esp_err_t err = nvs_open("trolley_modes", NVS_READONLY, &nvs_handle);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "Could not open NVS for reading");
        return err;
    }
    
    // Load wire learning results
    size_t required_size = sizeof(wire_learning_results_t);
    err = nvs_get_blob(nvs_handle, "wire_learning", &g_wire_learning_data, &required_size);
    if (err == ESP_OK) {
        ESP_LOGI(TAG, "Loaded wire learning data: %.2f m wire length", 
                g_wire_learning_data.wire_length_m);
    }
    
    // Load coasting data
    required_size = sizeof(coasting_data_t);
    err = nvs_get_blob(nvs_handle, "coasting", &g_coasting_data, &required_size);
    if (err == ESP_OK) {
        ESP_LOGI(TAG, "Loaded coasting data: %.2f m coasting distance", 
                g_coasting_data.coasting_distance_m);
    }
    
    nvs_close(nvs_handle);
    return ESP_OK;
}

static esp_err_t save_persistent_data(void) {
    nvs_handle_t nvs_handle;
    esp_err_t err = nvs_open("trolley_modes", NVS_READWRITE, &nvs_handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Could not open NVS for writing");
        return err;
    }
    
    // Save wire learning results if complete
    if (g_wire_learning_data.complete) {
        err = nvs_set_blob(nvs_handle, "wire_learning", &g_wire_learning_data, 
                          sizeof(wire_learning_results_t));
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "Failed to save wire learning data");
        }
    }
    
    // Save coasting data if calibrated
    if (g_coasting_data.calibrated) {
        err = nvs_set_blob(nvs_handle, "coasting", &g_coasting_data, 
                          sizeof(coasting_data_t));
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "Failed to save coasting data");
        }
    }
    
    err = nvs_commit(nvs_handle);
    nvs_close(nvs_handle);
    
    return err;
}

// ═══════════════════════════════════════════════════════════════════════════════
// PUBLIC API IMPLEMENTATION
// ═══════════════════════════════════════════════════════════════════════════════

esp_err_t mode_coordinator_init(void) {
    ESP_LOGI(TAG, "Initializing 3-mode coordination system...");
    
    // Reset mode status
    memset(&g_mode_status, 0, sizeof(g_mode_status));
    g_mode_status.current_mode = TROLLEY_MODE_NONE;
    g_mode_status.sensor_validation_state = SENSOR_VALIDATION_NOT_STARTED;
    g_mode_status.system_healthy = true;
    
    // Load any previously saved mode data
    load_persistent_data();
    
    // Initialize mode status messages
    strcpy(g_mode_status.current_mode_status, "System initializing...");
    strcpy(g_mode_status.sensor_validation_message, 
          "SENSOR VALIDATION REQUIRED: Step 1: ROTATE THE WHEEL manually");
    strcpy(g_mode_status.error_message, "");
    
    g_coordinator_initialized = true;
    
    ESP_LOGI(TAG, "Mode coordinator initialized successfully");
    ESP_LOGI(TAG, "Available modes: Wire Learning → Automatic → Manual");
    ESP_LOGI(TAG, "Sensor validation required before mode activation");
    
    return ESP_OK;
}

system_mode_status_t mode_coordinator_get_status(void) {
    return g_mode_status;
}

trolley_operation_mode_t mode_coordinator_get_current_mode(void) {
    return g_mode_status.current_mode;
}

bool mode_coordinator_is_mode_available(trolley_operation_mode_t mode) {
    switch (mode) {
        case TROLLEY_MODE_WIRE_LEARNING:
            return g_mode_status.wire_learning_availability == MODE_AVAILABLE;
        case TROLLEY_MODE_AUTOMATIC:
            return g_mode_status.automatic_availability == MODE_AVAILABLE;
        case TROLLEY_MODE_MANUAL:
            return g_mode_status.manual_availability == MODE_AVAILABLE;
        default:
            return false;
    }
}

esp_err_t mode_coordinator_update(void) {
    if (!g_coordinator_initialized) return ESP_ERR_INVALID_STATE;
    
    // Update sensor validation state
    update_sensor_validation_state();
    
    // Update mode availability
    update_mode_availability();
    
    // Update current mode status
    update_current_mode_status();
    
    // Check system health
    hardware_status_t hw_status = hardware_get_status();
    sensor_health_t sensor_status = sensor_health_get_status();
    
    g_mode_status.system_healthy = hw_status.system_initialized && 
                                   sensor_status.system_ready &&
                                   (g_error_count < MAX_SYSTEM_ERRORS);
    
    // Auto-reset errors after timeout
    uint64_t current_time = esp_timer_get_time();
    if (g_error_count > 0 && 
        (current_time - g_last_error_reset_time) > (ERROR_RESET_TIME_MS * 1000ULL)) {
        g_error_count = 0;
        strcpy(g_mode_status.error_message, "");
        ESP_LOGI(TAG, "Error count reset after timeout");
    }
    
    return ESP_OK;
}

// ═══════════════════════════════════════════════════════════════════════════════
// SENSOR VALIDATION API IMPLEMENTATION
// ═══════════════════════════════════════════════════════════════════════════════

esp_err_t mode_coordinator_start_sensor_validation(void) {
    ESP_LOGI(TAG, "Starting sensor validation process");
    
    g_mode_status.sensor_validation_state = SENSOR_VALIDATION_IN_PROGRESS;
    g_sensor_validation_start_time = esp_timer_get_time();
    g_hall_validation_user_confirmed = false;
    g_accel_validation_user_confirmed = false;
    g_mode_status.sensors_validated = false;
    g_mode_status.hall_validation_complete = false;
    g_mode_status.accel_validation_complete = false;
    
    strcpy(g_mode_status.sensor_validation_message,
          "Step 1: ROTATE THE WHEEL manually to test Hall sensor");
    
    return ESP_OK;
}

esp_err_t mode_coordinator_confirm_hall_validation(void) {
    if (g_mode_status.sensor_validation_state != SENSOR_VALIDATION_HALL_PENDING) {
        ESP_LOGW(TAG, "Hall validation not ready for confirmation");
        return ESP_ERR_INVALID_STATE;
    }
    
    g_hall_validation_user_confirmed = true;
    ESP_LOGI(TAG, "Hall sensor validation confirmed by user");
    
    return ESP_OK;
}

esp_err_t mode_coordinator_confirm_accel_validation(void) {
    if (g_mode_status.sensor_validation_state != SENSOR_VALIDATION_ACCEL_PENDING) {
        ESP_LOGW(TAG, "Accelerometer validation not ready for confirmation");
        return ESP_ERR_INVALID_STATE;
    }
    
    g_accel_validation_user_confirmed = true;
    ESP_LOGI(TAG, "Accelerometer validation confirmed by user");
    
    return ESP_OK;
}

esp_err_t mode_coordinator_reset_sensor_validation(void) {
    ESP_LOGI(TAG, "Resetting sensor validation");
    
    g_mode_status.sensor_validation_state = SENSOR_VALIDATION_NOT_STARTED;
    g_hall_validation_user_confirmed = false;
    g_accel_validation_user_confirmed = false;
    g_mode_status.sensors_validated = false;
    g_mode_status.hall_validation_complete = false;
    g_mode_status.accel_validation_complete = false;
    
    // Reset sensor health system
    sensor_health_reset_validation();
    
    return ESP_OK;
}

bool mode_coordinator_are_sensors_validated(void) {
    return g_mode_status.sensors_validated;
}

const char* mode_coordinator_get_sensor_validation_message(void) {
    return g_mode_status.sensor_validation_message;
}

// ═══════════════════════════════════════════════════════════════════════════════
// MODE CONTROL API IMPLEMENTATION
// ═══════════════════════════════════════════════════════════════════════════════

esp_err_t mode_coordinator_activate_wire_learning(void) {
    if (!mode_coordinator_is_mode_available(TROLLEY_MODE_WIRE_LEARNING)) {
        ESP_LOGW(TAG, "Wire learning mode not available");
        return ESP_ERR_INVALID_STATE;
    }
    
    // Stop any active mode first
    mode_coordinator_stop_current_mode(true);
    
    ESP_LOGI(TAG, "Activating wire learning mode");
    
    esp_err_t result = wire_learning_mode_start();
    if (result == ESP_OK) {
        g_mode_status.mode_start_time = esp_timer_get_time();
    }
    
    return result;
}

esp_err_t mode_coordinator_activate_automatic(void) {
    if (!mode_coordinator_is_mode_available(TROLLEY_MODE_AUTOMATIC)) {
        ESP_LOGW(TAG, "Automatic mode not available");
        return ESP_ERR_INVALID_STATE;
    }
    
    // Stop any active mode first
    mode_coordinator_stop_current_mode(true);
    
    ESP_LOGI(TAG, "Activating automatic mode");
    
    esp_err_t result = automatic_mode_start();
    if (result == ESP_OK) {
        g_mode_status.mode_start_time = esp_timer_get_time();
    }
    
    return result;
}

esp_err_t mode_coordinator_activate_manual(void) {
    if (!mode_coordinator_is_mode_available(TROLLEY_MODE_MANUAL)) {
        ESP_LOGW(TAG, "Manual mode not available");
        return ESP_ERR_INVALID_STATE;
    }
    
    // Stop any active mode first
    mode_coordinator_stop_current_mode(true);
    
    ESP_LOGI(TAG, "Activating manual mode");
    
    esp_err_t result = manual_mode_start();
    if (result == ESP_OK) {
        g_mode_status.mode_start_time = esp_timer_get_time();
    }
    
    return result;
}

esp_err_t mode_coordinator_stop_current_mode(bool immediate) {
    ESP_LOGI(TAG, "Stopping current mode (%s)", immediate ? "immediate" : "graceful");
    
    switch (g_mode_status.current_mode) {
        case TROLLEY_MODE_WIRE_LEARNING:
            wire_learning_mode_stop(immediate);
            break;
        case TROLLEY_MODE_AUTOMATIC:
            if (immediate) {
                automatic_mode_interrupt();
            } else {
                automatic_mode_stop_graceful();
            }
            break;
        case TROLLEY_MODE_MANUAL:
            manual_mode_stop();
            break;
        case TROLLEY_MODE_NONE:
            // Already stopped
            break;
    }
    
    g_mode_status.previous_mode = g_mode_status.current_mode;
    g_mode_status.current_mode = TROLLEY_MODE_NONE;
    
    return ESP_OK;
}

esp_err_t mode_coordinator_emergency_stop(void) {
    ESP_LOGW(TAG, "EMERGENCY STOP activated - stopping all modes");
    
    // Stop hardware immediately
    hardware_emergency_stop();
    
    // Stop all modes immediately
    wire_learning_mode_stop(true);
    automatic_mode_interrupt();
    manual_mode_stop();
    
    g_mode_status.current_mode = TROLLEY_MODE_NONE;
    strcpy(g_mode_status.error_message, "Emergency stop activated");
    
    return ESP_OK;
}

esp_err_t mode_coordinator_reset_system(void) {
    ESP_LOGI(TAG, "Resetting entire 3-mode system");
    
    // Emergency stop first
    mode_coordinator_emergency_stop();
    
    // Reset all mode data
    memset(&g_wire_learning_data, 0, sizeof(g_wire_learning_data));
    memset(&g_coasting_data, 0, sizeof(g_coasting_data));
    
    // Reset sensor validation
    mode_coordinator_reset_sensor_validation();
    
    // Reset error tracking
    g_error_count = 0;
    strcpy(g_mode_status.error_message, "");
    
    // Reset hardware
    hardware_reset_position();
    
    ESP_LOGI(TAG, "System reset complete");
    return ESP_OK;
}

// ═══════════════════════════════════════════════════════════════════════════════
// MODE DATA MANAGEMENT IMPLEMENTATION
// ═══════════════════════════════════════════════════════════════════════════════

esp_err_t mode_coordinator_set_wire_learning_results(const wire_learning_results_t* results) {
    if (results == NULL) return ESP_ERR_INVALID_ARG;
    
    memcpy(&g_wire_learning_data, results, sizeof(wire_learning_results_t));
    
    // Update mode status
    g_mode_status.wire_learning.complete = results->complete;
    g_mode_status.wire_learning.wire_length_m = results->wire_length_m;
    g_mode_status.wire_learning.optimal_learning_speed_ms = results->optimal_learning_speed_ms;
    g_mode_status.wire_learning.optimal_cruise_speed_ms = results->optimal_cruise_speed_ms;
    g_mode_status.wire_learning.forward_rotations = results->forward_rotations;
    g_mode_status.wire_learning.reverse_rotations = results->reverse_rotations;
    g_mode_status.wire_learning.learning_time_ms = results->total_learning_time_ms;
    
    // Save to persistent storage
    save_persistent_data();
    
    ESP_LOGI(TAG, "Wire learning results set: %.2f m wire length", results->wire_length_m);
    return ESP_OK;
}

const wire_learning_results_t* mode_coordinator_get_wire_learning_results(void) {
    return g_wire_learning_data.complete ? &g_wire_learning_data : NULL;
}

esp_err_t mode_coordinator_set_coasting_data(const coasting_data_t* coasting_data) {
    if (coasting_data == NULL) return ESP_ERR_INVALID_ARG;
    
    memcpy(&g_coasting_data, coasting_data, sizeof(coasting_data_t));
    
    // Update mode status
    g_mode_status.coasting_data.calibrated = coasting_data->calibrated;
    g_mode_status.coasting_data.coasting_distance_m = coasting_data->coasting_distance_m;
    g_mode_status.coasting_data.coast_start_distance_m = coasting_data->coast_start_distance_m;
    g_mode_status.coasting_data.coast_time_ms = coasting_data->coast_time_ms;
    g_mode_status.coasting_data.decel_rate_ms2 = coasting_data->decel_rate_ms2;
    
    // Save to persistent storage
    save_persistent_data();
    
    ESP_LOGI(TAG, "Coasting data set: %.2f m coasting distance", coasting_data->coasting_distance_m);
    return ESP_OK;
}

const coasting_data_t* mode_coordinator_get_coasting_data(void) {
    return g_coasting_data.calibrated ? &g_coasting_data : NULL;
}

esp_err_t mode_coordinator_update_cycle_count(uint32_t cycle_count) {
    g_mode_status.auto_cycle_count = cycle_count;
    return ESP_OK;
}

esp_err_t mode_coordinator_set_auto_interrupted(bool interrupted) {
    g_mode_status.auto_cycle_interrupted = interrupted;
    return ESP_OK;
}

// ═══════════════════════════════════════════════════════════════════════════════
// ERROR HANDLING AND SAFETY IMPLEMENTATION
// ═══════════════════════════════════════════════════════════════════════════════

esp_err_t mode_coordinator_report_error(const char* error_message) {
    if (error_message == NULL) return ESP_ERR_INVALID_ARG;
    
    g_error_count++;
    g_mode_status.last_error_time = esp_timer_get_time();
    g_last_error_reset_time = g_mode_status.last_error_time;
    
    strncpy(g_mode_status.error_message, error_message, sizeof(g_mode_status.error_message) - 1);
    g_mode_status.error_message[sizeof(g_mode_status.error_message) - 1] = '\0';
    
    ESP_LOGW(TAG, "System error reported: %s (count: %lu)", error_message, g_error_count);
    
    // Emergency stop if too many errors
    if (g_error_count >= MAX_SYSTEM_ERRORS) {
        ESP_LOGE(TAG, "Maximum error count reached - activating emergency stop");
        mode_coordinator_emergency_stop();
        g_mode_status.system_healthy = false;
    }
    
    return ESP_OK;
}

bool mode_coordinator_is_system_healthy(void) {
    return g_mode_status.system_healthy;
}

const char* mode_coordinator_get_error_message(void) {
    return g_mode_status.error_message;
}

esp_err_t mode_coordinator_clear_error(void) {
    g_error_count = 0;
    strcpy(g_mode_status.error_message, "");
    g_mode_status.system_healthy = true;
    
    ESP_LOGI(TAG, "Error condition cleared");
    return ESP_OK;
}

bool mode_coordinator_is_motion_safe(void) {
    // Check sensor validation
    if (!g_mode_status.sensors_validated) {
        return false;
    }
    
    // Check system health
    if (!g_mode_status.system_healthy) {
        return false;
    }
    
    // Check hardware status
    hardware_status_t hw_status = hardware_get_status();
    if (!hw_status.system_initialized || !hw_status.hall_sensor_healthy) {
        return false;
    }
    
    // Check sensor health
    sensor_health_t sensor_status = sensor_health_get_status();
    if (!sensor_status.system_ready) {
        return false;
    }
    
    return true;
}

// ═══════════════════════════════════════════════════════════════════════════════
// UTILITY FUNCTIONS IMPLEMENTATION
// ═══════════════════════════════════════════════════════════════════════════════

const char* mode_coordinator_mode_to_string(trolley_operation_mode_t mode) {
    switch (mode) {
        case TROLLEY_MODE_NONE: return "None";
        case TROLLEY_MODE_WIRE_LEARNING: return "Wire Learning";
        case TROLLEY_MODE_AUTOMATIC: return "Automatic";
        case TROLLEY_MODE_MANUAL: return "Manual";
        default: return "Unknown";
    }
}

const char* mode_coordinator_availability_to_string(mode_availability_t availability) {
    switch (availability) {
        case MODE_BLOCKED_SENSORS_NOT_VALIDATED: return "Sensors not validated";
        case MODE_BLOCKED_WIRE_LEARNING_REQUIRED: return "Wire learning required";
        case MODE_BLOCKED_SYSTEM_ERROR: return "System error";
        case MODE_AVAILABLE: return "Available";
        case MODE_ACTIVE: return "Active";
        case MODE_STOPPING: return "Stopping";
        default: return "Unknown";
    }
}

const char* mode_coordinator_validation_to_string(sensor_validation_state_t state) {
    switch (state) {
        case SENSOR_VALIDATION_NOT_STARTED: return "Not started";
        case SENSOR_VALIDATION_IN_PROGRESS: return "In progress";
        case SENSOR_VALIDATION_HALL_PENDING: return "Hall pending confirmation";
        case SENSOR_VALIDATION_ACCEL_PENDING: return "Accel pending confirmation";
        case SENSOR_VALIDATION_COMPLETE: return "Complete";
        case SENSOR_VALIDATION_FAILED: return "Failed";
        default: return "Unknown";
    }
}

esp_err_t mode_coordinator_get_detailed_status(char* status_buffer, size_t buffer_size) {
    if (status_buffer == NULL) return ESP_ERR_INVALID_ARG;
    
    snprintf(status_buffer, buffer_size,
        "=== 3-MODE SYSTEM STATUS ===\n"
        "Current Mode: %s\n"
        "Sensor Validation: %s\n"
        "Wire Learning: %s\n"
        "Automatic: %s\n"
        "Manual: %s\n"
        "System Health: %s\n"
        "Error Count: %lu\n"
        "Wire Length: %.2f m\n"
        "Coasting Distance: %.2f m\n"
        "Current Status: %s\n",
        mode_coordinator_mode_to_string(g_mode_status.current_mode),
        mode_coordinator_validation_to_string(g_mode_status.sensor_validation_state),
        mode_coordinator_availability_to_string(g_mode_status.wire_learning_availability),
        mode_coordinator_availability_to_string(g_mode_status.automatic_availability),
        mode_coordinator_availability_to_string(g_mode_status.manual_availability),
        g_mode_status.system_healthy ? "Healthy" : "Error",
        g_error_count,
        g_wire_learning_data.wire_length_m,
        g_coasting_data.coasting_distance_m,
        g_mode_status.current_mode_status);
    
    return ESP_OK;
}