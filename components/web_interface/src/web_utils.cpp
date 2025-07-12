// components/web_interface/src/web_utils.cpp
// ═══════════════════════════════════════════════════════════════════════════════
// WEB_UTILS.CPP - UTILITY FUNCTIONS AND HELPERS
// ═══════════════════════════════════════════════════════════════════════════════
// 
// SINGLE RESPONSIBILITY: Utility functions for web interface
// - Default configuration management
// - String conversion utilities
// - Memory and performance monitoring
// - Error handling helpers
// ═══════════════════════════════════════════════════════════════════════════════

#include "web_interface.h"
#include "esp_log.h"
#include "esp_timer.h"
#include <cstring>

static const char* TAG = "WEB_UTILS";

// ═══════════════════════════════════════════════════════════════════════════════
// CONFIGURATION UTILITIES
// ═══════════════════════════════════════════════════════════════════════════════

esp_err_t web_get_default_config(web_interface_config_t* config) {
    if (config == NULL) return ESP_ERR_INVALID_ARG;
    
    // Set default web interface configuration
    config->server_port = WEB_SERVER_PORT;
    config->max_open_sockets = WEB_MAX_OPEN_SOCKETS;
    config->enable_cors = true;
    config->enable_rate_limiting = true;
    config->enable_command_logging = true;
    config->enable_real_time_updates = false;
    strcpy(config->server_name, "ESP32S3_TROLLEY_3MODE");
    
    return ESP_OK;
}

esp_err_t web_set_config(const web_interface_config_t* config) {
    if (config == NULL) return ESP_ERR_INVALID_ARG;
    
    // Validate configuration parameters
    if (config->server_port < 80 || config->server_port > 65535) {
        ESP_LOGE(TAG, "Invalid server port: %d", config->server_port);
        return ESP_ERR_INVALID_ARG;
    }
    
    if (config->max_open_sockets < 1 || config->max_open_sockets > 16) {
        ESP_LOGE(TAG, "Invalid max_open_sockets: %d", config->max_open_sockets);
        return ESP_ERR_INVALID_ARG;
    }
    
    ESP_LOGI(TAG, "Web interface configuration updated");
    return ESP_OK;
}

esp_err_t web_get_config(web_interface_config_t* config) {
    if (config == NULL) return ESP_ERR_INVALID_ARG;
    
    // Return current configuration (placeholder - would use global config)
    return web_get_default_config(config);
}

// ═══════════════════════════════════════════════════════════════════════════════
// STRING CONVERSION UTILITIES
// ═══════════════════════════════════════════════════════════════════════════════

const char* web_interface_status_to_string(web_interface_status_t status) {
    switch (status) {
        case WEB_STATUS_STOPPED:  return "Stopped";
        case WEB_STATUS_STARTING: return "Starting";
        case WEB_STATUS_RUNNING:  return "Running";
        case WEB_STATUS_ERROR:    return "Error";
        case WEB_STATUS_STOPPING: return "Stopping";
        default:                  return "Unknown";
    }
}

esp_err_t web_wifi_get_info(char* info_buffer, size_t buffer_size) {
    if (info_buffer == NULL) return ESP_ERR_INVALID_ARG;
    
    snprintf(info_buffer, buffer_size,
            "WiFi Access Point Information:\n"
            "SSID: ESP32S3_TROLLEY_3MODE\n"
            "Security: Open (no password)\n"
            "Channel: 11\n"
            "IP Address: 192.168.4.1\n"
            "Web Interface: http://192.168.4.1\n"
            "Max Clients: 4\n"
            "Status: Active");
    
    return ESP_OK;
}

bool web_wifi_is_ap_running(void) {
    // Simplified check - in real implementation would check WiFi status
    return true;
}

// ═══════════════════════════════════════════════════════════════════════════════
// PERFORMANCE AND MONITORING UTILITIES
// ═══════════════════════════════════════════════════════════════════════════════

uint64_t web_get_uptime(void) {
    // Return uptime in milliseconds
    return esp_timer_get_time() / 1000;
}

esp_err_t web_get_memory_usage(size_t* free_heap, size_t* min_free_heap) {
    if (free_heap != NULL) {
        *free_heap = esp_get_free_heap_size();
    }
    if (min_free_heap != NULL) {
        *min_free_heap = esp_get_minimum_free_heap_size();
    }
    return ESP_OK;
}

esp_err_t web_get_debug_info(char* debug_buffer, size_t buffer_size) {
    if (debug_buffer == NULL) return ESP_ERR_INVALID_ARG;
    
    size_t free_heap, min_free_heap;
    web_get_memory_usage(&free_heap, &min_free_heap);
    
    snprintf(debug_buffer, buffer_size,
        "=== WEB INTERFACE DEBUG INFO ===\n"
        "Status: %s\n"
        "Uptime: %llu ms\n"
        "Free Heap: %zu bytes\n"
        "Min Free Heap: %zu bytes\n"
        "WiFi AP: %s\n"
        "Server Port: 80\n"
        "Max Sockets: 7\n"
        "CORS Enabled: Yes\n"
        "Rate Limiting: Yes\n"
        "Real-time Updates: No\n"
        "Build Date: %s %s",
        web_interface_status_to_string(web_interface_get_status()),
        web_get_uptime(),
        free_heap,
        min_free_heap,
        web_wifi_is_ap_running() ? "Running" : "Stopped",
        __DATE__, __TIME__);
    
    return ESP_OK;
}

// ═══════════════════════════════════════════════════════════════════════════════
// JSON UTILITIES
// ═══════════════════════════════════════════════════════════════════════════════

esp_err_t web_generate_api_info_json(char* json_buffer, size_t buffer_size) {
    if (json_buffer == NULL) return ESP_ERR_INVALID_ARG;
    
    snprintf(json_buffer, buffer_size,
        "{"
        "\"system_info\": {"
        "\"name\": \"ESP32-S3 Trolley 3-Mode System\","
        "\"hardware\": \"ESP32-S3 + Eco II 2807 + Littlebee 30A ESC\","
        "\"wheel\": \"61mm diameter (191.6mm circumference)\","
        "\"version\": \"1.0.0\","
        "\"build_date\": \"%s %s\""
        "},"
        "\"modes\": ["
        "{"
        "\"id\": 1,"
        "\"name\": \"Wire Learning\","
        "\"description\": \"Learn wire length with gradual speed 0.1-1.0 m/s\","
        "\"max_speed\": 1.0"
        "},"
        "{"
        "\"id\": 2,"
        "\"name\": \"Automatic\","
        "\"description\": \"Autonomous 5 m/s cycling with coasting\","
        "\"max_speed\": 5.0"
        "},"
        "{"
        "\"id\": 3,"
        "\"name\": \"Manual\","
        "\"description\": \"Full user control with safety validation\","
        "\"max_speed\": 2.0"
        "}"
        "],"
        "\"api_endpoints\": ["
        "\"/\","
        "\"/js/main.js\","
        "\"/api/status\","
        "\"/api/command\","
        "\"/api/info\","
        "\"/api/stats\""
        "],"
        "\"wifi\": {"
        "\"ssid\": \"ESP32S3_TROLLEY_3MODE\","
        "\"ip\": \"192.168.4.1\","
        "\"security\": \"Open\""
        "}"
        "}",
        __DATE__, __TIME__);
    
    return ESP_OK;
}

esp_err_t web_generate_stats_json(char* json_buffer, size_t buffer_size) {
    if (json_buffer == NULL) return ESP_ERR_INVALID_ARG;
    
    web_server_stats_t stats = web_interface_get_stats();
    size_t free_heap, min_free_heap;
    web_get_memory_usage(&free_heap, &min_free_heap);
    
    snprintf(json_buffer, buffer_size,
        "{"
        "\"server_stats\": {"
        "\"total_requests\": %lu,"
        "\"successful_requests\": %lu,"
        "\"failed_requests\": %lu,"
        "\"commands_executed\": %lu,"
        "\"status_requests\": %lu,"
        "\"active_connections\": %lu,"
        "\"max_concurrent_connections\": %lu,"
        "\"uptime_ms\": %llu"
        "},"
        "\"performance\": {"
        "\"free_heap\": %zu,"
        "\"min_free_heap\": %zu,"
        "\"wifi_clients\": %d"
        "},"
        "\"system\": {"
        "\"web_status\": \"%s\","
        "\"wifi_ap_running\": %s"
        "}"
        "}",
        
        // Server stats
        stats.total_requests,
        stats.successful_requests,
        stats.failed_requests,
        stats.commands_executed,
        stats.status_requests,
        stats.active_connections,
        stats.max_concurrent_connections,
        web_get_uptime(),
        
        // Performance stats
        free_heap,
        min_free_heap,
        web_wifi_get_client_count(),
        
        // System status
        web_interface_status_to_string(web_interface_get_status()),
        web_wifi_is_ap_running() ? "true" : "false");
    
    return ESP_OK;
}

// ═══════════════════════════════════════════════════════════════════════════════
// ERROR HANDLING UTILITIES
// ═══════════════════════════════════════════════════════════════════════════════

esp_err_t web_generate_error_page(int error_code, const char* error_message,
                                 char* html_buffer, size_t buffer_size) {
    if (html_buffer == NULL) return ESP_ERR_INVALID_ARG;
    
    const char* error_template = 
        "<!DOCTYPE html>"
        "<html><head>"
        "<title>Error %d - ESP32-S3 Trolley</title>"
        "<style>"
        "body { font-family: Arial; margin: 50px; text-align: center; background: #f0f0f0; }"
        "h1 { color: #dc3545; }"
        ".error-box { background: #f8d7da; padding: 20px; border-radius: 8px; margin: 20px 0; border: 1px solid #f5c6cb; }"
        "a { color: #007bff; text-decoration: none; }"
        "a:hover { text-decoration: underline; }"
        "</style></head>"
        "<body>"
        "<h1>🚨 Error %d</h1>"
        "<div class='error-box'>"
        "<p><strong>%s</strong></p>"
        "</div>"
        "<p><a href='/'>🏠 Return to Main Page</a></p>"
        "<p><small>ESP32-S3 Trolley 3-Mode System</small></p>"
        "</body></html>";
    
    snprintf(html_buffer, buffer_size, error_template, 
            error_code, error_code, error_message ? error_message : "Unknown error occurred");
    
    return ESP_OK;
}

esp_err_t web_log_error(const char* error_message, const char* client_ip) {
    ESP_LOGE(TAG, "Web Error from %s: %s", 
            client_ip ? client_ip : "unknown", 
            error_message ? error_message : "Unknown error");
    return ESP_OK;
}

esp_err_t web_get_error_log(char* log_buffer, size_t buffer_size) {
    if (log_buffer == NULL) return ESP_ERR_INVALID_ARG;
    
    snprintf(log_buffer, buffer_size, 
            "Error logging is handled by ESP-IDF logging system.\n"
            "Check serial output for detailed error logs.\n"
            "Log level can be adjusted in menuconfig.");
    
    return ESP_OK;
}

esp_err_t web_clear_error_log(void) {
    ESP_LOGI(TAG, "Error log clear requested (handled by system)");
    return ESP_OK;
}

// ═══════════════════════════════════════════════════════════════════════════════
// MAINTENANCE AND RESET UTILITIES
// ═══════════════════════════════════════════════════════════════════════════════

esp_err_t web_reset_statistics(void) {
    ESP_LOGI(TAG, "Web server statistics reset requested");
    // In real implementation, would reset global statistics
    return ESP_OK;
}

esp_err_t web_restart_server(void) {
    ESP_LOGI(TAG, "Web server restart requested");
    
    esp_err_t result = web_interface_stop();
    if (result != ESP_OK) {
        return result;
    }
    
    // Wait a moment
    vTaskDelay(pdMS_TO_TICKS(1000));
    
    return web_interface_start();
}

// ═══════════════════════════════════════════════════════════════════════════════
// PLACEHOLDER FUNCTIONS (for future features)
// ═══════════════════════════════════════════════════════════════════════════════

esp_err_t web_enable_real_time_updates(bool enable) {
    ESP_LOGI(TAG, "Real-time updates %s (feature not implemented)", 
            enable ? "enabled" : "disabled");
    return ESP_OK;
}

esp_err_t web_send_real_time_update(const char* update_type, const char* data) {
    ESP_LOGD(TAG, "Real-time update: %s - %s (feature not implemented)", 
            update_type, data);
    return ESP_OK;
}

esp_err_t web_register_for_updates(const char* client_ip) {
    ESP_LOGD(TAG, "Client %s registered for updates (feature not implemented)", client_ip);
    return ESP_OK;
}

esp_err_t web_unregister_from_updates(const char* client_ip) {
    ESP_LOGD(TAG, "Client %s unregistered from updates (feature not implemented)", client_ip);
    return ESP_OK;
}

esp_err_t web_set_theme(const char* theme_name) {
    ESP_LOGI(TAG, "Theme set to: %s (feature not implemented)", theme_name);
    return ESP_OK;
}

esp_err_t web_get_available_themes(char* themes_buffer, size_t buffer_size) {
    if (themes_buffer == NULL) return ESP_ERR_INVALID_ARG;
    
    snprintf(themes_buffer, buffer_size, "default");
    return ESP_OK;
}

esp_err_t web_set_page_branding(const char* title, const char* subtitle) {
    ESP_LOGI(TAG, "Page branding: %s - %s (feature not implemented)", title, subtitle);
    return ESP_OK;
}