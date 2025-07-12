// components/hardware_control/src/hardware_control.cpp - FIXED VERSION
#include "hardware_control.h"
#include "pin_config.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "driver/gpio.h"
#include "driver/ledc.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include <cstring>
#include <cmath>

static const char* TAG = "HARDWARE_CONTROL";

// ═══════════════════════════════════════════════════════════════════════════════
// GLOBAL HARDWARE STATE - FIXED: Proper struct initialization
// ═══════════════════════════════════════════════════════════════════════════════

static hardware_status_t g_hw_status = {
    .esc_armed = false,
    .esc_responding = false,
    .current_esc_duty = ESC_NEUTRAL_DUTY,
    .current_speed_ms = 0.0f,
    .target_speed_ms = 0.0f,
    .direction_forward = true,
    .total_rotations = 0,
    .last_hall_time = 0,
    .hall_sensor_healthy = false,
    .system_initialized = false
};

static QueueHandle_t g_hall_event_queue = NULL;
static TaskHandle_t g_esc_update_task_handle = NULL;
static hall_pulse_callback_t g_hall_callback = NULL;
static esc_status_callback_t g_esc_callback = NULL;
static hardware_error_t g_last_error = HW_ERROR_NONE;

// Position tracking
static float g_current_position_m = 0.0f;
static uint32_t g_rotation_count_offset = 0;

// Rate limiting
static bool g_rate_limiting_enabled = true;
static uint16_t g_last_esc_duty = ESC_NEUTRAL_DUTY;

// ═══════════════════════════════════════════════════════════════════════════════
// HALL SENSOR INTERRUPT AND PROCESSING
// ═══════════════════════════════════════════════════════════════════════════════

static void IRAM_ATTR hall_sensor_isr_handler(void* arg) {
    uint64_t current_time = esp_timer_get_time();
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    
    // Send timestamp to processing queue
    xQueueSendFromISR(g_hall_event_queue, &current_time, &xHigherPriorityTaskWoken);
    
    if (xHigherPriorityTaskWoken) {
        portYIELD_FROM_ISR();
    }
}

static void hall_processing_task(void* pvParameter) {
    uint64_t hall_time;
    uint64_t last_time = 0;
    
    ESP_LOGI(TAG, "ESC armed successfully");
    return ESP_OK;
}

esp_err_t hardware_esc_disarm(void) {
    ESP_LOGI(TAG, "Disarming ESC...");
    
    // Stop motor immediately
    g_hw_status.target_speed_ms = 0.0f;
    ledc_set_duty(LEDC_LOW_SPEED_MODE, ESC_PWM_CHANNEL, ESC_NEUTRAL_DUTY);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, ESC_PWM_CHANNEL);
    
    g_hw_status.esc_armed = false;
    g_hw_status.current_esc_duty = ESC_NEUTRAL_DUTY;
    
    // Turn off status LED
    gpio_set_level(STATUS_LED_PIN, 0);
    
    // Notify callback
    if (g_esc_callback) {
        g_esc_callback(false, true);
    }
    
    ESP_LOGI(TAG, "ESC disarmed");
    return ESP_OK;
}

bool hardware_esc_is_armed(void) {
    return g_hw_status.esc_armed;
}

esp_err_t hardware_set_motor_speed(float speed_ms, bool forward) {
    if (!g_hw_status.system_initialized) {
        g_last_error = HW_ERROR_SYSTEM_NOT_INITIALIZED;
        return ESP_ERR_INVALID_STATE;
    }
    
    if (!hardware_is_speed_valid(speed_ms)) {
        g_last_error = HW_ERROR_SPEED_OUT_OF_RANGE;
        return ESP_ERR_INVALID_ARG;
    }
    
    if (!g_hw_status.esc_armed) {
        g_last_error = HW_ERROR_ESC_NOT_RESPONDING;
        return ESP_ERR_INVALID_STATE;
    }
    
    g_hw_status.target_speed_ms = speed_ms;
    g_hw_status.direction_forward = forward;
    
    ESP_LOGD(TAG, "Motor speed set: %.2f m/s %s", speed_ms, forward ? "forward" : "reverse");
    return ESP_OK;
}

esp_err_t hardware_emergency_stop(void) {
    ESP_LOGW(TAG, "EMERGENCY STOP activated");
    
    g_hw_status.target_speed_ms = 0.0f;
    g_hw_status.current_esc_duty = ESC_NEUTRAL_DUTY;
    
    ledc_set_duty(LEDC_LOW_SPEED_MODE, ESC_PWM_CHANNEL, ESC_NEUTRAL_DUTY);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, ESC_PWM_CHANNEL);
    
    gpio_set_level(STATUS_LED_PIN, 0);
    
    return ESP_OK;
}

float hardware_get_current_speed(void) {
    return g_hw_status.current_speed_ms;
}

uint32_t hardware_get_rotation_count(void) {
    return g_hw_status.total_rotations - g_rotation_count_offset;
}

esp_err_t hardware_reset_rotation_count(void) {
    g_rotation_count_offset = g_hw_status.total_rotations;
    return ESP_OK;
}

uint64_t hardware_get_time_since_last_hall_pulse(void) {
    if (g_hw_status.last_hall_time == 0) return 0;
    return esp_timer_get_time() - g_hw_status.last_hall_time;
}

bool hardware_is_hall_sensor_healthy(void) {
    return g_hw_status.hall_sensor_healthy;
}

float hardware_rotations_to_distance(uint32_t rotations) {
    return rotations * WHEEL_CIRCUMFERENCE_MM * MM_TO_M;
}

uint32_t hardware_distance_to_rotations(float distance_m) {
    return (uint32_t)(distance_m / (WHEEL_CIRCUMFERENCE_MM * MM_TO_M));
}

esp_err_t hardware_set_status_led(bool on) {
    gpio_set_level(STATUS_LED_PIN, on ? 1 : 0);
    return ESP_OK;
}

float hardware_get_current_position(void) {
    return g_current_position_m;
}

esp_err_t hardware_reset_position(void) {
    g_current_position_m = 0.0f;
    hardware_reset_rotation_count();
    return ESP_OK;
}

esp_err_t hardware_update(void) {
    // Hardware update is handled by background tasks
    // This function can be used for additional periodic checks
    return ESP_OK;
}

hardware_status_t hardware_get_status(void) {
    return g_hw_status;
}

hardware_error_t hardware_get_last_error(void) {
    return g_last_error;
}

bool hardware_is_ready(void) {
    return g_hw_status.system_initialized && 
           g_hw_status.hall_sensor_healthy && 
           g_hw_status.esc_responding;
}

const char* hardware_error_to_string(hardware_error_t error) {
    switch (error) {
        case HW_ERROR_NONE: return "No error";
        case HW_ERROR_ESC_NOT_RESPONDING: return "ESC not responding";
        case HW_ERROR_HALL_SENSOR_TIMEOUT: return "Hall sensor timeout";
        case HW_ERROR_PWM_INIT_FAILED: return "PWM initialization failed";
        case HW_ERROR_GPIO_INIT_FAILED: return "GPIO initialization failed";
        case HW_ERROR_SPEED_OUT_OF_RANGE: return "Speed out of range";
        case HW_ERROR_SYSTEM_NOT_INITIALIZED: return "System not initialized";
        default: return "Unknown error";
    }
}

bool hardware_is_speed_valid(float speed_ms) {
    return (speed_ms >= 0.0f && speed_ms <= MAX_SPEED_MS);
}

esp_err_t hardware_get_info(char* info_buffer, size_t buffer_size) {
    snprintf(info_buffer, buffer_size,
        "Hardware Control System\n"
        "ESC: GPIO%d (50Hz PWM)\n"
        "Hall: GPIO%d (Interrupt)\n"
        "LED: GPIO%d\n"
        "Wheel: %.1fmm circumference\n"
        "Max Speed: %.1f m/s",
        (int)ESC_PWM_PIN, (int)HALL_SENSOR_PIN, (int)STATUS_LED_PIN,
        WHEEL_CIRCUMFERENCE_MM, MAX_SPEED_MS);
    return ESP_OK;
}

esp_err_t hardware_register_hall_callback(hall_pulse_callback_t callback) {
    g_hall_callback = callback;
    return ESP_OK;
}

esp_err_t hardware_register_esc_callback(esc_status_callback_t callback) {
    g_esc_callback = callback;
    return ESP_OK;
}

esp_err_t hardware_set_esc_duty_direct(uint16_t duty_cycle) {
    if (!g_hw_status.esc_armed) {
        return ESP_ERR_INVALID_STATE;
    }
    
    if (duty_cycle < ESC_MIN_DUTY || duty_cycle > ESC_MAX_DUTY) {
        return ESP_ERR_INVALID_ARG;
    }
    
    ledc_set_duty(LEDC_LOW_SPEED_MODE, ESC_PWM_CHANNEL, duty_cycle);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, ESC_PWM_CHANNEL);
    g_hw_status.current_esc_duty = duty_cycle;
    
    return ESP_OK;
}

uint16_t hardware_get_esc_duty(void) {
    return g_hw_status.current_esc_duty;
}

esp_err_t hardware_set_esc_rate_limiting(bool enable) {
    g_rate_limiting_enabled = enable;
    return ESP_OK;
}, "Hall sensor processing task started");
    
    while (1) {
        if (xQueueReceive(g_hall_event_queue, &hall_time, portMAX_DELAY)) {
            // Increment rotation counter
            g_hw_status.total_rotations++;
            
            // Calculate speed from time between pulses
            if (last_time > 0) {
                uint64_t time_diff_us = hall_time - last_time;
                if (time_diff_us > 0) {
                    float time_diff_s = time_diff_us / 1000000.0f;
                    float speed_ms = (WHEEL_CIRCUMFERENCE_MM * MM_TO_M) / time_diff_s;
                    // Apply smoothing filter
                    g_hw_status.current_speed_ms = g_hw_status.current_speed_ms * 0.7f + speed_ms * 0.3f;
                }
            }
            
            // Update position based on direction
            float distance_increment = WHEEL_CIRCUMFERENCE_MM * MM_TO_M;
            if (g_hw_status.direction_forward) {
                g_current_position_m += distance_increment;
            } else {
                g_current_position_m -= distance_increment;
            }
            
            // Update timing
            last_time = hall_time;
            g_hw_status.last_hall_time = hall_time;
            g_hw_status.hall_sensor_healthy = true;
            
            // Call registered callback
            if (g_hall_callback) {
                g_hall_callback(g_hw_status.total_rotations, hall_time);
            }
        }
    }
}

// ═══════════════════════════════════════════════════════════════════════════════
// ESC PWM INITIALIZATION AND CONTROL - FIXED: Proper LEDC configuration
// ═══════════════════════════════════════════════════════════════════════════════

static esp_err_t init_esc_pwm(void) {
    ESP_LOGI(TAG, "Initializing ESC PWM system...");
    
    // Wait for power stabilization
    vTaskDelay(pdMS_TO_TICKS(2000));
    
    // Configure LEDC timer - FIXED: Correct field order and all required fields
    ledc_timer_config_t ledc_timer = {
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .duty_resolution = ESC_PWM_RESOLUTION,    // Must come before freq_hz
        .timer_num = ESC_PWM_TIMER,
        .freq_hz = ESC_PWM_FREQ_HZ,
        .clk_cfg = LEDC_AUTO_CLK,
        .deconfigure = false
    };
    ESP_ERROR_CHECK(ledc_timer_config(&ledc_timer));
    
    // Configure LEDC channel - FIXED: Proper initialization
    ledc_channel_config_t esc_channel = {
        .gpio_num = ESC_PWM_PIN,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .channel = ESC_PWM_CHANNEL,
        .intr_type = LEDC_INTR_DISABLE,
        .timer_sel = ESC_PWM_TIMER,
        .duty = ESC_NEUTRAL_DUTY,
        .hpoint = 0,
        .flags = {0}  // Initialize flags properly
    };
    ESP_ERROR_CHECK(ledc_channel_config(&esc_channel));
    
    g_hw_status.current_esc_duty = ESC_NEUTRAL_DUTY;
    g_last_esc_duty = ESC_NEUTRAL_DUTY;
    
    ESP_LOGI(TAG, "ESC PWM initialized successfully");
    return ESP_OK;
}

static esp_err_t init_gpio_pins(void) {
    ESP_LOGI(TAG, "Initializing GPIO pins...");
    
    // Configure output pins
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << (int)ESC_ENABLE_PIN) | (1ULL << (int)STATUS_LED_PIN),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    ESP_ERROR_CHECK(gpio_config(&io_conf));
    
    // Set initial states
    gpio_set_level(ESC_ENABLE_PIN, 1);
    gpio_set_level(STATUS_LED_PIN, 0);
    
    // Configure Hall sensor input
    io_conf.pin_bit_mask = (1ULL << (int)HALL_SENSOR_PIN);
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pull_up_en = GPIO_PULLUP_ENABLE;
    io_conf.intr_type = GPIO_INTR_POSEDGE;
    ESP_ERROR_CHECK(gpio_config(&io_conf));
    
    ESP_LOGI(TAG, "GPIO pins initialized successfully");
    return ESP_OK;
}

static uint16_t speed_to_esc_duty(float speed_ms, bool forward) {
    if (speed_ms > MAX_SPEED_MS) speed_ms = MAX_SPEED_MS;
    if (speed_ms < 0.0f) speed_ms = 0.0f;
    
    if (speed_ms < ESC_SPEED_DEADBAND) {
        return ESC_NEUTRAL_DUTY;
    }
    
    uint16_t duty;
    if (forward) {
        float duty_range = ESC_MAX_DUTY - ESC_NEUTRAL_DUTY;
        duty = ESC_NEUTRAL_DUTY + (uint16_t)((speed_ms / MAX_SPEED_MS) * duty_range);
    } else {
        float duty_range = ESC_NEUTRAL_DUTY - ESC_MIN_DUTY;
        duty = ESC_NEUTRAL_DUTY - (uint16_t)((speed_ms / MAX_SPEED_MS) * duty_range);
    }
    
    return duty;
}

static void esc_update_task(void* pvParameter) {
    TickType_t xLastWakeTime = xTaskGetTickCount();
    
    ESP_LOGI(TAG, "ESC update task started");
    
    while (1) {
        if (g_hw_status.system_initialized && g_hw_status.esc_armed) {
            uint16_t target_duty = speed_to_esc_duty(g_hw_status.target_speed_ms, g_hw_status.direction_forward);
            
            // Apply rate limiting if enabled
            if (g_rate_limiting_enabled) {
                int16_t duty_diff = target_duty - g_hw_status.current_esc_duty;
                if (abs(duty_diff) > ESC_MAX_SPEED_CHANGE) {
                    if (duty_diff > 0) {
                        g_hw_status.current_esc_duty += ESC_MAX_SPEED_CHANGE;
                    } else {
                        g_hw_status.current_esc_duty -= ESC_MAX_SPEED_CHANGE;
                    }
                } else {
                    g_hw_status.current_esc_duty = target_duty;
                }
            } else {
                g_hw_status.current_esc_duty = target_duty;
            }
            
            // Update ESC PWM output
            ledc_set_duty(LEDC_LOW_SPEED_MODE, ESC_PWM_CHANNEL, g_hw_status.current_esc_duty);
            ledc_update_duty(LEDC_LOW_SPEED_MODE, ESC_PWM_CHANNEL);
            
            g_last_esc_duty = g_hw_status.current_esc_duty;
            
            // Update status LED
            gpio_set_level(STATUS_LED_PIN, (g_hw_status.target_speed_ms > ESC_SPEED_DEADBAND) ? 1 : 0);
        }
        
        // Check ESC health
        g_hw_status.esc_responding = (g_hw_status.current_esc_duty == g_last_esc_duty);
        
        // Check Hall sensor health
        uint64_t current_time = esp_timer_get_time();
        if (g_hw_status.last_hall_time > 0 && 
            (current_time - g_hw_status.last_hall_time) > (HALL_TIMEOUT_MS * 1000ULL)) {
            g_hw_status.current_speed_ms = 0.0f;
            if (g_hw_status.target_speed_ms > ESC_SPEED_DEADBAND) {
                g_hw_status.hall_sensor_healthy = false;
            }
        }
        
        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(ESC_UPDATE_INTERVAL_MS));
    }
}

// ═══════════════════════════════════════════════════════════════════════════════
// PUBLIC API IMPLEMENTATION
// ═══════════════════════════════════════════════════════════════════════════════

esp_err_t hardware_init(void) {
    ESP_LOGI(TAG, "Initializing hardware control system...");
    
    // Reset hardware status to known state
    g_hw_status.esc_armed = false;
    g_hw_status.esc_responding = false;
    g_hw_status.current_esc_duty = ESC_NEUTRAL_DUTY;
    g_hw_status.current_speed_ms = 0.0f;
    g_hw_status.target_speed_ms = 0.0f;
    g_hw_status.direction_forward = true;
    g_hw_status.total_rotations = 0;
    g_hw_status.last_hall_time = 0;
    g_hw_status.hall_sensor_healthy = false;
    g_hw_status.system_initialized = false;
    
    // Create Hall sensor event queue
    g_hall_event_queue = xQueueCreate(10, sizeof(uint64_t));
    if (g_hall_event_queue == NULL) {
        ESP_LOGE(TAG, "Failed to create Hall sensor queue");
        g_last_error = HW_ERROR_GPIO_INIT_FAILED;
        return ESP_ERR_NO_MEM;
    }
    
    // Initialize GPIO pins
    if (init_gpio_pins() != ESP_OK) {
        g_last_error = HW_ERROR_GPIO_INIT_FAILED;
        return ESP_FAIL;
    }
    
    // Initialize ESC PWM
    if (init_esc_pwm() != ESP_OK) {
        g_last_error = HW_ERROR_PWM_INIT_FAILED;
        return ESP_FAIL;
    }
    
    // Install Hall sensor interrupt
    ESP_ERROR_CHECK(gpio_install_isr_service(0));
    ESP_ERROR_CHECK(gpio_isr_handler_add(HALL_SENSOR_PIN, hall_sensor_isr_handler, 
                                        (void*) HALL_SENSOR_PIN));
    
    // Create tasks
    xTaskCreate(hall_processing_task, "hall_proc", 4096, NULL, 10, NULL);
    xTaskCreate(esc_update_task, "esc_update", 4096, NULL, 8, &g_esc_update_task_handle);
    
    g_hw_status.system_initialized = true;
    ESP_LOGI(TAG, "Hardware control system initialized successfully");
    
    return ESP_OK;
}

esp_err_t hardware_esc_arm(void) {
    if (!g_hw_status.system_initialized) {
        g_last_error = HW_ERROR_SYSTEM_NOT_INITIALIZED;
        return ESP_ERR_INVALID_STATE;
    }
    
    ESP_LOGI(TAG, "Arming ESC...");
    
    // Step 1: Neutral position
    ledc_set_duty(LEDC_LOW_SPEED_MODE, ESC_PWM_CHANNEL, ESC_NEUTRAL_DUTY);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, ESC_PWM_CHANNEL);
    vTaskDelay(pdMS_TO_TICKS(1000));
    
    // Step 2: Arming signal
    ledc_set_duty(LEDC_LOW_SPEED_MODE, ESC_PWM_CHANNEL, ESC_ARM_DUTY);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, ESC_PWM_CHANNEL);
    vTaskDelay(pdMS_TO_TICKS(3000));
    
    // Step 3: Return to neutral
    ledc_set_duty(LEDC_LOW_SPEED_MODE, ESC_PWM_CHANNEL, ESC_NEUTRAL_DUTY);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, ESC_PWM_CHANNEL);
    
    g_hw_status.esc_armed = true;
    g_hw_status.current_esc_duty = ESC_NEUTRAL_DUTY;
    
    // Notify callback
    if (g_esc_callback) {
        g_esc_callback(true, true);
    }
    
    ESP_LOGI(TAG, "ESC armed successfully");
    return ESP_OK;
}

esp_err_t hardware_esc_disarm(void) {
    ESP_LOGI(TAG, "Disarming ESC...");
    
    // Stop motor immediately
    g_hw_status.target_speed_ms = 0.0f;
    ledc_set_duty(LEDC_LOW_SPEED_MODE, ESC_PWM_CHANNEL, ESC_NEUTRAL_DUTY);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, ESC_PWM_CHANNEL);
    
    g_hw_status.esc_armed = false;
    g_hw_status.current_esc_duty = ESC_NEUTRAL_DUTY;
    
    // Turn off status LED
    gpio_set_level(STATUS_LED_PIN, 0);
    
    // Notify callback
    if (g_esc_callback) {
        g_esc_callback(false, true);
    }
    
    ESP_LOGI(TAG, "ESC disarmed");
    return ESP_OK;
}

bool hardware_esc_is_armed(void) {
    return g_hw_status.esc_armed;
}

esp_err_t hardware_set_motor_speed(float speed_ms, bool forward) {
    if (!g_hw_status.system_initialized) {
        g_last_error = HW_ERROR_SYSTEM_NOT_INITIALIZED;
        return ESP_ERR_INVALID_STATE;
    }
    
    if (!hardware_is_speed_valid(speed_ms)) {
        g_last_error = HW_ERROR_SPEED_OUT_OF_RANGE;
        return ESP_ERR_INVALID_ARG;
    }
    
    if (!g_hw_status.esc_armed) {
        g_last_error = HW_ERROR_ESC_NOT_RESPONDING;
        return ESP_ERR_INVALID_STATE;
    }
    
    g_hw_status.target_speed_ms = speed_ms;
    g_hw_status.direction_forward = forward;
    
    ESP_LOGD(TAG, "Motor speed set: %.2f m/s %s", speed_ms, forward ? "forward" : "reverse");
    return ESP_OK;
}

esp_err_t hardware_emergency_stop(void) {
    ESP_LOGW(TAG, "EMERGENCY STOP activated");
    
    g_hw_status.target_speed_ms = 0.0f;
    g_hw_status.current_esc_duty = ESC_NEUTRAL_DUTY;
    
    ledc_set_duty(LEDC_LOW_SPEED_MODE, ESC_PWM_CHANNEL, ESC_NEUTRAL_DUTY);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, ESC_PWM_CHANNEL);
    
    gpio_set_level(STATUS_LED_PIN, 0);
    
    return ESP_OK;
}

float hardware_get_current_speed(void) {
    return g_hw_status.current_speed_ms;
}

uint32_t hardware_get_rotation_count(void) {
    return g_hw_status.total_rotations - g_rotation_count_offset;
}

esp_err_t hardware_reset_rotation_count(void) {
    g_rotation_count_offset = g_hw_status.total_rotations;
    return ESP_OK;
}

uint64_t hardware_get_time_since_last_hall_pulse(void) {
    if (g_hw_status.last_hall_time == 0) return 0;
    return esp_timer_get_time() - g_hw_status.last_hall_time;
}

bool hardware_is_hall_sensor_healthy(void) {
    return g_hw_status.hall_sensor_healthy;
}

float hardware_rotations_to_distance(uint32_t rotations) {
    return rotations * WHEEL_CIRCUMFERENCE_MM * MM_TO_M;
}

uint32_t hardware_distance_to_rotations(float distance_m) {
    return (uint32_t)(distance_m / (WHEEL_CIRCUMFERENCE_MM * MM_TO_M));
}

esp_err_t hardware_set_status_led(bool on) {
    gpio_set_level(STATUS_LED_PIN, on ? 1 : 0);
    return ESP_OK;
}

float hardware_get_current_position(void) {
    return g_current_position_m;
}

esp_err_t hardware_reset_position(void) {
    g_current_position_m = 0.0f;
    hardware_reset_rotation_count();
    return ESP_OK;
}

esp_err_t hardware_update(void) {
    // Hardware update is handled by background tasks
    // This function can be used for additional periodic checks
    return ESP_OK;
}

hardware_status_t hardware_get_status(void) {
    return g_hw_status;
}

hardware_error_t hardware_get_last_error(void) {
    return g_last_error;
}

bool hardware_is_ready(void) {
    return g_hw_status.system_initialized && 
           g_hw_status.hall_sensor_healthy && 
           g_hw_status.esc_responding;
}

const char* hardware_error_to_string(hardware_error_t error) {
    switch (error) {
        case HW_ERROR_NONE: return "No error";
        case HW_ERROR_ESC_NOT_RESPONDING: return "ESC not responding";
        case HW_ERROR_HALL_SENSOR_TIMEOUT: return "Hall sensor timeout";
        case HW_ERROR_PWM_INIT_FAILED: return "PWM initialization failed";
        case HW_ERROR_GPIO_INIT_FAILED: return "GPIO initialization failed";
        case HW_ERROR_SPEED_OUT_OF_RANGE: return "Speed out of range";
        case HW_ERROR_SYSTEM_NOT_INITIALIZED: return "System not initialized";
        default: return "Unknown error";
    }
}

bool hardware_is_speed_valid(float speed_ms) {
    return (speed_ms >= 0.0f && speed_ms <= MAX_SPEED_MS);
}

esp_err_t hardware_get_info(char* info_buffer, size_t buffer_size) {
    snprintf(info_buffer, buffer_size,
        "Hardware Control System\n"
        "ESC: GPIO%d (50Hz PWM)\n"
        "Hall: GPIO%d (Interrupt)\n"
        "LED: GPIO%d\n"
        "Wheel: %.1fmm circumference\n"
        "Max Speed: %.1f m/s",
        (int)ESC_PWM_PIN, (int)HALL_SENSOR_PIN, (int)STATUS_LED_PIN,
        WHEEL_CIRCUMFERENCE_MM, MAX_SPEED_MS);
    return ESP_OK;
}

esp_err_t hardware_register_hall_callback(hall_pulse_callback_t callback) {
    g_hall_callback = callback;
    return ESP_OK;
}

esp_err_t hardware_register_esc_callback(esc_status_callback_t callback) {
    g_esc_callback = callback;
    return ESP_OK;
}

esp_err_t hardware_set_esc_duty_direct(uint16_t duty_cycle) {
    if (!g_hw_status.esc_armed) {
        return ESP_ERR_INVALID_STATE;
    }
    
    if (duty_cycle < ESC_MIN_DUTY || duty_cycle > ESC_MAX_DUTY) {
        return ESP_ERR_INVALID_ARG;
    }
    
    ledc_set_duty(LEDC_LOW_SPEED_MODE, ESC_PWM_CHANNEL, duty_cycle);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, ESC_PWM_CHANNEL);
    g_hw_status.current_esc_duty = duty_cycle;
    
    return ESP_OK;
}

uint16_t hardware_get_esc_duty(void) {
    return g_hw_status.current_esc_duty;
}

esp_err_t hardware_set_esc_rate_limiting(bool enable) {
    g_rate_limiting_enabled = enable;
    return ESP_OK;
}