// components/sensor_health/include/sensor_health.h
#ifndef SENSOR_HEALTH_H
#define SENSOR_HEALTH_H

#include "esp_err.h"
#include "MPU.hpp"
#include <stdint.h>
#include <stdbool.h>

// Sensor health status
typedef enum {
    SENSOR_STATUS_UNKNOWN = 0,
    SENSOR_STATUS_TESTING,
    SENSOR_STATUS_HEALTHY,
    SENSOR_STATUS_FAILED,
    SENSOR_STATUS_TIMEOUT
} sensor_status_t;

// System initialization state
typedef enum {
    INIT_STATE_START = 0,
    INIT_STATE_WAIT_WHEEL_ROTATION,
    INIT_STATE_WAIT_TROLLEY_SHAKE,
    INIT_STATE_SENSORS_READY,
    INIT_STATE_SYSTEM_READY,
    INIT_STATE_FAILED
} init_state_t;

// Sensor health data structure
typedef struct {
    // Hall sensor data
    sensor_status_t hall_status;
    uint32_t hall_pulse_count;
    uint64_t last_hall_pulse_time;
    float current_rpm;
    float wheel_speed_ms;
    bool wheel_rotation_detected;
    
    // Accelerometer data
    sensor_status_t accel_status;
    float accel_x_g;
    float accel_y_g;
    float accel_z_g;
    float total_accel_g;
    float last_impact_g;
    uint64_t last_impact_time;
    bool trolley_shake_detected;
    
    // System state
    init_state_t init_state;
    char status_message[128];
    char error_message[128];
    uint64_t init_start_time;
    bool sensors_validated;
    bool system_ready;
} sensor_health_t;

// Configuration constants - CORRECTED for 61mm wheel
#define WHEEL_DIAMETER_MM               61.0f     // Actual wheel diameter
#define WHEEL_CIRCUMFERENCE_MM          191.6f    // π × 61mm ≈ 191.6mm
#define HALL_VALIDATION_TIMEOUT_MS      60000     // 1 minute timeout
#define ACCEL_VALIDATION_TIMEOUT_MS     60000     // 1 minute timeout
#define MINIMUM_SHAKE_THRESHOLD_G       0.3f      // Minimum shake detection
#define IMPACT_THRESHOLD_G              0.1f      // Impact detection threshold
#define HALL_PULSE_TIMEOUT_MS           5000      // Hall pulse timeout
#define ACCEL_SAMPLE_RATE_MS            50        // Accelerometer sampling rate

// Function declarations
esp_err_t sensor_health_init(MPU_t* mpu_sensor);
void sensor_health_update(void);
sensor_health_t sensor_health_get_status(void);
bool sensor_health_is_system_ready(void);
bool sensor_health_validate_hall_sensor(void);
bool sensor_health_validate_accelerometer(void);
void sensor_health_reset_validation(void);
const char* sensor_health_get_init_message(void);
bool sensor_health_check_command_safety(void);

// Hall sensor specific functions
void sensor_health_hall_pulse_detected(void);
float sensor_health_calculate_wheel_speed(void);
bool sensor_health_is_hall_healthy(void);

// Accelerometer specific functions
void sensor_health_process_accel_data(float x_g, float y_g, float z_g);
bool sensor_health_is_accel_healthy(void);
float sensor_health_get_last_impact(void);

#endif // SENSOR_HEALTH_H