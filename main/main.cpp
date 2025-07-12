// main/main.cpp - COMPLETE 3-MODE SYSTEM INTEGRATION (~100 LINES)
// ═══════════════════════════════════════════════════════════════════════════════
// 
// SINGLE RESPONSIBILITY: Application initialization and coordination
// - System-wide component initialization  
// - Task creation for background operations
// - WiFi setup and web interface startup
// - Error handling and system monitoring
// 
// All business logic now handled by dedicated components:
// - hardware_control: Low-level hardware abstraction
// - mode_coordinator: 3-mode system management
// - wire_learning_mode, automatic_mode, manual_mode: Mode implementations  
// - web_interface: Complete web UI with real-time updates
// - sensor_health: Sensor validation workflow
// ═══════════════════════════════════════════════════════════════════════════════

#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "driver/uart.h"
#include "driver/i2c.h"

// New modular component includes
#include "hardware_control.h"
#include "mode_coordinator.h"
#include "wire_learning_mode.h"
#include "automatic_mode.h"
#include "manual_mode.h"
#include "web_interface.h"
#include "sensor_health.h"
#include "MPU.hpp"
#include "pin_config.h"

static const char* TAG = "MAIN";

// Global objects
static MPU_t mpu;
static bool system_ready = false;

// ═══════════════════════════════════════════════════════════════════════════════
// SYSTEM INITIALIZATION
// ═══════════════════════════════════════════════════════════════════════════════

/**
 * @brief Initialize MPU6050 sensor with I2C
 */
static bool init_mpu6050(void) {
    ESP_LOGI(TAG, "Initializing MPU6050 sensor...");
    
    // Initialize I2C for ESP32-S3
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_SDA_PIN,        // GPIO8 for ESP32-S3
        .scl_io_num = I2C_SCL_PIN,        // GPIO9 for ESP32-S3
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master = { .clk_speed = 400000 },
        .clk_flags = 0
    };
    
    ESP_ERROR_CHECK(i2c_param_config(I2C_PORT_NUM, &conf));
    ESP_ERROR_CHECK(i2c_driver_install(I2C_PORT_NUM, conf.mode, 0, 0, 0));
    
    // Initialize MPU6050
    if (mpu.initialize() != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize MPU6050");
        return false;
    }
    
    // Configure MPU6050 settings
    mpu.setAccelFullScale(mpud::ACCEL_FS_8G);
    mpu.setGyroFullScale(mpud::GYRO_FS_500DPS);
    mpu.setDigitalLowPassFilter(mpud::DLPF_42HZ);
    mpu.setSampleRate(100);
    
    ESP_LOGI(TAG, "MPU6050 initialized successfully");
    return true;
}

/**
 * @brief Initialize all system components in correct order
 */
static esp_err_t init_system_components(void) {
    ESP_LOGI(TAG, "=== INITIALIZING 3-MODE TROLLEY SYSTEM ===");
    
    // Step 1: Initialize MPU6050 first (required by sensor_health)
    if (!init_mpu6050()) {
        ESP_LOGE(TAG, "MPU6050 initialization failed - system cannot proceed");
        return ESP_FAIL;
    }
    
    // Step 2: Initialize hardware control (ESC, Hall, GPIO)
    ESP_LOGI(TAG, "Initializing hardware control layer...");
    ESP_ERROR_CHECK(hardware_init());
    
    // Step 3: Initialize sensor health monitoring  
    ESP_LOGI(TAG, "Initializing sensor health monitoring...");
    ESP_ERROR_CHECK(sensor_health_init(&mpu));
    
    // Step 4: Initialize individual mode components
    ESP_LOGI(TAG, "Initializing mode components...");
    ESP_ERROR_CHECK(wire_learning_mode_init());
    ESP_ERROR_CHECK(automatic_mode_init());
    ESP_ERROR_CHECK(manual_mode_init());
    
    // Step 5: Initialize mode coordinator (3-mode system management)
    ESP_LOGI(TAG, "Initializing 3-mode coordinator...");
    ESP_ERROR_CHECK(mode_coordinator_init());
    
    // Step 6: Initialize web interface
    ESP_LOGI(TAG, "Initializing web interface...");
    ESP_ERROR_CHECK(web_interface_init(NULL)); // Use default config
    
    ESP_LOGI(TAG, "=== ALL COMPONENTS INITIALIZED SUCCESSFULLY ===");
    return ESP_OK;
}

// ═══════════════════════════════════════════════════════════════════════════════
// BACKGROUND TASKS
// ═══════════════════════════════════════════════════════════════════════════════

/**
 * @brief Main system update task - coordinates all components
 */
static void system_update_task(void* pvParameter) {
    TickType_t xLastWakeTime = xTaskGetTickCount();
    
    ESP_LOGI(TAG, "System update task started");
    
    while (1) {
        // Update all components in correct order
        hardware_update();                // Update hardware status
        sensor_health_update();           // Process sensor validation
        mode_coordinator_update();        // Update mode management
        
        // Update active mode components
        wire_learning_mode_update();      // Update wire learning if active
        automatic_mode_update();          // Update automatic mode if active  
        manual_mode_update();             // Update manual mode if active
        
        // Update web interface
        web_interface_update();           // Handle web maintenance
        
        // Run every 50ms for responsive system operation
        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(50));
    }
}

/**
 * @brief System monitoring and health check task
 */
static void system_monitor_task(void* pvParameter) {
    TickType_t xLastWakeTime = xTaskGetTickCount();
    uint32_t heartbeat_counter = 0;
    
    ESP_LOGI(TAG, "System monitor task started");
    
    while (1) {
        // Log system status every 30 seconds
        if (++heartbeat_counter % 600 == 0) { // 600 * 50ms = 30 seconds
            system_mode_status_t mode_status = mode_coordinator_get_status();
            hardware_status_t hw_status = hardware_get_status();
            web_server_stats_t web_stats = web_interface_get_stats();
            
            ESP_LOGI(TAG, "=== SYSTEM HEARTBEAT ===");
            ESP_LOGI(TAG, "Mode: %s, Sensors: %s, ESC: %s", 
                    mode_coordinator_mode_to_string(mode_status.current_mode),
                    mode_status.sensors_validated ? "Validated" : "Not Validated",
                    hw_status.esc_armed ? "Armed" : "Disarmed");
            ESP_LOGI(TAG, "Web: %lu requests, %d clients, %zu KB free", 
                    web_stats.total_requests, web_wifi_get_client_count(),
                    esp_get_free_heap_size() / 1024);
        }
        
        // Check system health
        if (!mode_coordinator_is_system_healthy()) {
            ESP_LOGW(TAG, "System health issue: %s", mode_coordinator_get_error_message());
        }
        
        // Monitor memory usage
        size_t free_heap = esp_get_free_heap_size();
        if (free_heap < 50000) { // Less than 50KB free
            ESP_LOGW(TAG, "Low memory warning: %zu bytes free", free_heap);
        }
        
        // Run every 50ms
        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(50));
    }
}

/**
 * @brief Serial command interface for debugging (minimal - web is primary interface)
 */
static void serial_command_task(void* pvParameter) {
    char input_char;
    
    ESP_LOGI(TAG, "Serial debug interface started");
    printf("\n╔══════════════════════════════════════════════════════════════╗\n");
    printf("║              ESP32-S3 TROLLEY - 3-MODE SYSTEM               ║\n");
    printf("║                                                              ║\n");
    printf("║  Hardware: ESP32-S3 + Eco II 2807 + Littlebee 30A ESC      ║\n");  
    printf("║  Wheel: 61mm diameter (191.6mm circumference)               ║\n");
    printf("║  WiFi: ESP32S3_TROLLEY_3MODE → http://192.168.4.1           ║\n");
    printf("║                                                              ║\n");
    printf("║  Modes: Wire Learning → Automatic (5 m/s) → Manual          ║\n");
    printf("║  Safety: Sensor validation required before operation        ║\n");
    printf("║                                                              ║\n");
    printf("║  Debug Commands: T=Status, R=Reset, E=Emergency, H=Help     ║\n");
    printf("║  Full Control: Use web interface at 192.168.4.1             ║\n");
    printf("╚══════════════════════════════════════════════════════════════╝\n\n");
    
    while (1) {
        // Read single character from UART
        int chars_read = uart_read_bytes(UART_NUM_0, &input_char, 1, pdMS_TO_TICKS(1000));
        
        if (chars_read > 0) {
            printf("Debug Command: '%c'\n", input_char);
            
            // Process minimal debug commands
            char response[256];
            esp_err_t result = web_process_command(input_char, "debug_serial", response, sizeof(response));
            
            printf("Response: %s\n", response);
            if (result != ESP_OK) {
                printf("Note: Use web interface for full system control\n");
            }
            printf("\n");
        }
    }
}

// ═══════════════════════════════════════════════════════════════════════════════
// MAIN APPLICATION ENTRY POINT
// ═══════════════════════════════════════════════════════════════════════════════

extern "C" void app_main(void) {
    ESP_LOGI(TAG, "╔══════════════════════════════════════════════════════════════╗");
    ESP_LOGI(TAG, "║            ESP32-S3 TROLLEY SYSTEM STARTING                  ║");
    ESP_LOGI(TAG, "║                                                              ║");
    ESP_LOGI(TAG, "║  Architecture: Modular 3-Mode System                        ║");
    ESP_LOGI(TAG, "║  Chip: %s                                            ║", CONFIG_IDF_TARGET);
    ESP_LOGI(TAG, "║  System: Component-based with mode coordinator              ║");
    ESP_LOGI(TAG, "╚══════════════════════════════════════════════════════════════╝");
    
    // Initialize NVS (required for WiFi and data persistence)
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);
    
    // Initialize UART for debug interface
    uart_config_t uart_config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .rx_flow_ctrl_thresh = 122,
        .source_clk = UART_SCLK_DEFAULT
    };
    ESP_ERROR_CHECK(uart_param_config(UART_NUM_0, &uart_config));
    ESP_ERROR_CHECK(uart_driver_install(UART_NUM_0, 1024, 0, 0, NULL, 0));
    
    // Initialize all system components
    if (init_system_components() != ESP_OK) {
        ESP_LOGE(TAG, "FATAL: System component initialization failed");
        ESP_LOGE(TAG, "System cannot proceed - restarting in 5 seconds...");
        vTaskDelay(pdMS_TO_TICKS(5000));
        esp_restart();
    }
    
    // Start WiFi Access Point and web server
    ESP_LOGI(TAG, "Starting WiFi Access Point and web server...");
    ESP_ERROR_CHECK(web_wifi_init_ap("ESP32S3_TROLLEY_3MODE", "")); // Open network
    ESP_ERROR_CHECK(web_interface_start());
    
    // Create system background tasks
    ESP_LOGI(TAG, "Creating system tasks...");
    xTaskCreatePinnedToCore(system_update_task, "sys_update", 4096, NULL, 8, NULL, 1);
    xTaskCreatePinnedToCore(system_monitor_task, "sys_monitor", 3072, NULL, 5, NULL, 0);
    xTaskCreatePinnedToCore(serial_command_task, "serial_debug", 3072, NULL, 3, NULL, 0);
    
    // System initialization complete
    system_ready = true;
    
    ESP_LOGI(TAG, "╔══════════════════════════════════════════════════════════════╗");
    ESP_LOGI(TAG, "║                SYSTEM READY FOR OPERATION                   ║");
    ESP_LOGI(TAG, "║                                                              ║");
    ESP_LOGI(TAG, "║  🌐 Web Interface: http://192.168.4.1                       ║");
    ESP_LOGI(TAG, "║  📱 WiFi Network: ESP32S3_TROLLEY_3MODE (Open)              ║");
    ESP_LOGI(TAG, "║                                                              ║");
    ESP_LOGI(TAG, "║  🛡️  IMPORTANT: Sensor validation required!                 ║");
    ESP_LOGI(TAG, "║     Step 1: ROTATE THE WHEEL manually                       ║");
    ESP_LOGI(TAG, "║     Step 2: SHAKE THE TROLLEY                               ║");
    ESP_LOGI(TAG, "║     Step 3: Confirm sensors via web interface               ║");
    ESP_LOGI(TAG, "║                                                              ║");
    ESP_LOGI(TAG, "║  🔄 Mode Sequence: Wire Learning → Automatic → Manual       ║");
    ESP_LOGI(TAG, "║  ⚡ Speed Range: 0.1-1.0 m/s (learning), up to 5 m/s (auto) ║");
    ESP_LOGI(TAG, "║  🛑 Emergency stop available in all modes                   ║");
    ESP_LOGI(TAG, "╚══════════════════════════════════════════════════════════════╝");
    
    // Main initialization complete - system runs via background tasks
    ESP_LOGI(TAG, "Main initialization complete - system operational");
    
    // Keep main task alive for system health monitoring
    uint32_t health_check_counter = 0;
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(10000)); // Sleep 10 seconds between checks
        
        health_check_counter++;
        
        // Periodic system health validation (every 5 minutes)
        if (health_check_counter % 30 == 0) { // 30 * 10s = 5 minutes
            if (!system_ready || !mode_coordinator_is_system_healthy()) {
                ESP_LOGE(TAG, "System health check failed - attempting recovery");
                
                // Attempt graceful recovery
                mode_coordinator_emergency_stop();
                
                // If recovery fails, restart system
                if (!mode_coordinator_is_system_healthy()) {
                    ESP_LOGE(TAG, "Recovery failed - restarting system in 5 seconds");
                    vTaskDelay(pdMS_TO_TICKS(5000));
                    esp_restart();
                }
            }
        }
        
        // Log periodic status for long-term monitoring
        if (health_check_counter % 180 == 0) { // 180 * 10s = 30 minutes
            ESP_LOGI(TAG, "Long-term status: Uptime %lu minutes, Free heap: %zu KB", 
                    (esp_timer_get_time() / 1000000) / 60, esp_get_free_heap_size() / 1024);
        }
    }
}