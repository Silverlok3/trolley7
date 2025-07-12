#ifndef WEB_INTERFACE_H
#define WEB_INTERFACE_H

#include "esp_err.h"
#include "esp_http_server.h"
#include <stdint.h>
#include <stdbool.h>

// ═══════════════════════════════════════════════════════════════════════════════
// WEB_INTERFACE.H - WEB USER INTERFACE COMPONENT
// ═══════════════════════════════════════════════════════════════════════════════
// 
// SINGLE RESPONSIBILITY: Web UI and HTTP handling
// - HTTP server setup and management
// - Web page generation and serving
// - Command routing to mode_coordinator
// - Status JSON generation for real-time updates
// - WebSocket support for live data (optional future)
// 
// Uses: mode_coordinator (for commands), all mode components (for status)
// Called by: main.cpp
// ═══════════════════════════════════════════════════════════════════════════════

// ═══════════════════════════════════════════════════════════════════════════════
// WEB INTERFACE CONFIGURATION
// ═══════════════════════════════════════════════════════════════════════════════

// HTTP server configuration
#define WEB_SERVER_PORT                 80        // HTTP server port
#define WEB_MAX_OPEN_SOCKETS           7         // Maximum concurrent connections
#define WEB_RESPONSE_TIMEOUT_MS        5000      // Response timeout
#define WEB_REQUEST_TIMEOUT_MS         10000     // Request timeout

// Content configuration
#define WEB_HTML_BUFFER_SIZE           8192      // HTML content buffer size
#define WEB_JSON_BUFFER_SIZE           2048      // JSON response buffer size
#define WEB_COMMAND_BUFFER_SIZE        256       // Command buffer size
#define WEB_STATUS_UPDATE_INTERVAL_MS  1000      // Status update interval

// Security and rate limiting
#define WEB_MAX_COMMANDS_PER_MINUTE    60        // Command rate limiting
#define WEB_MAX_CONCURRENT_COMMANDS    3         // Concurrent command limit
#define WEB_COMMAND_TIMEOUT_MS         5000      // Individual command timeout

// UI update configuration
#define WEB_REAL_TIME_UPDATE_MS        500       // Real-time data update rate
#define WEB_STATUS_CACHE_TIME_MS       1000      // Status caching time
#define WEB_ERROR_DISPLAY_TIME_MS      5000      // Error message display time

// ═══════════════════════════════════════════════════════════════════════════════
// WEB INTERFACE DATA STRUCTURES
// ═══════════════════════════════════════════════════════════════════════════════

/**
 * @brief Web interface status
 */
typedef enum {
    WEB_STATUS_STOPPED = 0,             // Web server not running
    WEB_STATUS_STARTING,                // Starting up
    WEB_STATUS_RUNNING,                 // Running normally
    WEB_STATUS_ERROR,                   // Error condition
    WEB_STATUS_STOPPING                 // Shutting down
} web_interface_status_t;

/**
 * @brief Client connection info
 */
typedef struct {
    uint32_t client_id;                 // Unique client identifier
    char ip_address[16];                // Client IP address
    uint64_t connect_time;              // Connection timestamp
    uint32_t requests_sent;             // Number of requests from this client
    uint64_t last_request_time;         // Last request timestamp
    bool rate_limited;                  // Rate limiting status
} web_client_info_t;

/**
 * @brief Web server statistics
 */
typedef struct {
    uint32_t total_requests;            // Total HTTP requests handled
    uint32_t successful_requests;       // Successful requests
    uint32_t failed_requests;           // Failed requests
    uint32_t commands_executed;         // Commands executed via web
    uint32_t status_requests;           // Status page requests
    uint32_t active_connections;        // Current active connections
    uint32_t max_concurrent_connections; // Peak concurrent connections
    uint64_t server_start_time;         // Server start timestamp
    uint64_t last_request_time;         // Last request timestamp
    char last_client_ip[16];            // IP of last client
} web_server_stats_t;

/**
 * @brief Web interface configuration
 */
typedef struct {
    uint16_t server_port;               // HTTP server port
    uint8_t max_open_sockets;           // Maximum open sockets
    bool enable_cors;                   // Enable CORS headers
    bool enable_rate_limiting;          // Enable request rate limiting
    bool enable_command_logging;        // Log all commands
    bool enable_real_time_updates;      // Enable real-time status updates
    char server_name[32];               // Server identification name
} web_interface_config_t;

// ═══════════════════════════════════════════════════════════════════════════════
// CORE WEB INTERFACE API
// ═══════════════════════════════════════════════════════════════════════════════

/**
 * @brief Initialize web interface system
 * @param config Web interface configuration (NULL for defaults)
 * @return ESP_OK on success, error code on failure
 */
esp_err_t web_interface_init(const web_interface_config_t* config);

/**
 * @brief Start web server
 * @return ESP_OK on success, error code on failure
 */
esp_err_t web_interface_start(void);

/**
 * @brief Stop web server
 * @return ESP_OK on success, error code on failure
 */
esp_err_t web_interface_stop(void);

/**
 * @brief Get current web interface status
 * @return Current status
 */
web_interface_status_t web_interface_get_status(void);

/**
 * @brief Check if web server is running
 * @return true if running, false otherwise
 */
bool web_interface_is_running(void);

/**
 * @brief Get web server statistics
 * @return Server statistics structure
 */
web_server_stats_t web_interface_get_stats(void);

/**
 * @brief Update web interface (call regularly for maintenance)
 * @return ESP_OK on success
 */
esp_err_t web_interface_update(void);

// ═══════════════════════════════════════════════════════════════════════════════
// HTTP HANDLER FUNCTIONS
// ═══════════════════════════════════════════════════════════════════════════════

/**
 * @brief Root page handler (main 3-mode interface)
 * @param req HTTP request
 * @return ESP_OK on success, error code on failure
 */
esp_err_t web_handler_root(httpd_req_t *req);

/**
 * @brief Status JSON handler (real-time system status)
 * @param req HTTP request
 * @return ESP_OK on success, error code on failure
 */
esp_err_t web_handler_status(httpd_req_t *req);

/**
 * @brief Command handler (process user commands)
 * @param req HTTP request
 * @return ESP_OK on success, error code on failure
 */
esp_err_t web_handler_command(httpd_req_t *req);

/**
 * @brief CORS options handler (preflight requests)
 * @param req HTTP request
 * @return ESP_OK on success, error code on failure
 */
esp_err_t web_handler_options(httpd_req_t *req);

/**
 * @brief API info handler (system information)
 * @param req HTTP request
 * @return ESP_OK on success, error code on failure
 */
esp_err_t web_handler_api_info(httpd_req_t *req);

/**
 * @brief Performance stats handler (detailed statistics)
 * @param req HTTP request
 * @return ESP_OK on success, error code on failure
 */
esp_err_t web_handler_stats(httpd_req_t *req);

// ═══════════════════════════════════════════════════════════════════════════════
// CONTENT GENERATION API
// ═══════════════════════════════════════════════════════════════════════════════

/**
 * @brief Generate main HTML page content
 * @param html_buffer Buffer to write HTML content
 * @param buffer_size Size of HTML buffer
 * @return ESP_OK on success, error code on failure
 */
esp_err_t web_generate_main_page(char* html_buffer, size_t buffer_size);

/**
 * @brief Generate system status JSON
 * @param json_buffer Buffer to write JSON content
 * @param buffer_size Size of JSON buffer
 * @return ESP_OK on success, error code on failure
 */
esp_err_t web_generate_status_json(char* json_buffer, size_t buffer_size);

/**
 * @brief Generate command response JSON
 * @param success Command execution success status
 * @param message Response message
 * @param json_buffer Buffer to write JSON response
 * @param buffer_size Size of JSON buffer
 * @return ESP_OK on success, error code on failure
 */
esp_err_t web_generate_command_response(bool success, const char* message, 
                                       char* json_buffer, size_t buffer_size);

/**
 * @brief Generate performance statistics JSON
 * @param json_buffer Buffer to write JSON content
 * @param buffer_size Size of JSON buffer
 * @return ESP_OK on success, error code on failure
 */
esp_err_t web_generate_stats_json(char* json_buffer, size_t buffer_size);

/**
 * @brief Generate API information JSON
 * @param json_buffer Buffer to write JSON content
 * @param buffer_size Size of JSON buffer
 * @return ESP_OK on success, error code on failure
 */
esp_err_t web_generate_api_info_json(char* json_buffer, size_t buffer_size);

// ═══════════════════════════════════════════════════════════════════════════════
// COMMAND PROCESSING API
// ═══════════════════════════════════════════════════════════════════════════════

/**
 * @brief Process web command and route to appropriate mode
 * @param command_char Single character command
 * @param client_ip Client IP address for logging
 * @param response_buffer Buffer for response message
 * @param buffer_size Size of response buffer
 * @return ESP_OK on success, error code on failure
 */
esp_err_t web_process_command(char command_char, const char* client_ip,
                             char* response_buffer, size_t buffer_size);

/**
 * @brief Validate web command before processing
 * @param command_char Command to validate
 * @param client_ip Client IP for rate limiting
 * @return true if valid and allowed, false otherwise
 */
bool web_validate_command(char command_char, const char* client_ip);

/**
 * @brief Log command execution for security/debugging
 * @param command_char Executed command
 * @param client_ip Client IP address
 * @param success Execution success status
 * @param response_message Response message
 * @return ESP_OK on success
 */
esp_err_t web_log_command(char command_char, const char* client_ip, 
                         bool success, const char* response_message);

/**
 * @brief Get command help text for web interface
 * @param help_buffer Buffer to write help text
 * @param buffer_size Size of help buffer
 * @return ESP_OK on success
 */
esp_err_t web_get_command_help(char* help_buffer, size_t buffer_size);

// ═══════════════════════════════════════════════════════════════════════════════
// SECURITY AND RATE LIMITING API
// ═══════════════════════════════════════════════════════════════════════════════

/**
 * @brief Check if client is rate limited
 * @param client_ip Client IP address
 * @return true if rate limited, false if allowed
 */
bool web_is_client_rate_limited(const char* client_ip);

/**
 * @brief Update rate limiting for client
 * @param client_ip Client IP address
 * @return ESP_OK on success
 */
esp_err_t web_update_rate_limiting(const char* client_ip);

/**
 * @brief Get client information
 * @param client_ip Client IP address
 * @param client_info Buffer to write client info
 * @return ESP_OK if client found, ESP_ERR_NOT_FOUND otherwise
 */
esp_err_t web_get_client_info(const char* client_ip, web_client_info_t* client_info);

/**
 * @brief Clear rate limiting data (reset counters)
 * @return ESP_OK on success
 */
esp_err_t web_clear_rate_limiting(void);

/**
 * @brief Block client IP address temporarily
 * @param client_ip Client IP to block
 * @param duration_ms Block duration in milliseconds
 * @return ESP_OK on success
 */
esp_err_t web_block_client(const char* client_ip, uint32_t duration_ms);

// ═══════════════════════════════════════════════════════════════════════════════
// WIFI INTEGRATION API
// ═══════════════════════════════════════════════════════════════════════════════

/**
 * @brief Initialize WiFi Access Point for web interface
 * @param ssid WiFi network name
 * @param password WiFi password (empty string for open network)
 * @return ESP_OK on success, error code on failure
 */
esp_err_t web_wifi_init_ap(const char* ssid, const char* password);

/**
 * @brief Get WiFi connection information
 * @param info_buffer Buffer to write WiFi info
 * @param buffer_size Size of info buffer
 * @return ESP_OK on success
 */
esp_err_t web_wifi_get_info(char* info_buffer, size_t buffer_size);

/**
 * @brief Check WiFi Access Point status
 * @return true if AP running, false otherwise
 */
bool web_wifi_is_ap_running(void);

/**
 * @brief Get connected client count
 * @return Number of connected WiFi clients
 */
uint8_t web_wifi_get_client_count(void);

// ═══════════════════════════════════════════════════════════════════════════════
// REAL-TIME UPDATES API (Future WebSocket Support)
// ═══════════════════════════════════════════════════════════════════════════════

/**
 * @brief Enable real-time status updates for clients
 * @param enable Enable/disable real-time updates
 * @return ESP_OK on success
 */
esp_err_t web_enable_real_time_updates(bool enable);

/**
 * @brief Send real-time update to all connected clients
 * @param update_type Type of update (status, mode_change, error, etc.)
 * @param data Update data in JSON format
 * @return ESP_OK on success
 */
esp_err_t web_send_real_time_update(const char* update_type, const char* data);

/**
 * @brief Register for real-time update notifications
 * @param client_ip Client IP to register
 * @return ESP_OK on success
 */
esp_err_t web_register_for_updates(const char* client_ip);

/**
 * @brief Unregister from real-time update notifications
 * @param client_ip Client IP to unregister
 * @return ESP_OK on success
 */
esp_err_t web_unregister_from_updates(const char* client_ip);

// ═══════════════════════════════════════════════════════════════════════════════
// ERROR HANDLING AND DEBUGGING API
// ═══════════════════════════════════════════════════════════════════════════════

/**
 * @brief Generate error page HTML
 * @param error_code HTTP error code
 * @param error_message Error description
 * @param html_buffer Buffer to write error page HTML
 * @param buffer_size Size of HTML buffer
 * @return ESP_OK on success
 */
esp_err_t web_generate_error_page(int error_code, const char* error_message,
                                 char* html_buffer, size_t buffer_size);

/**
 * @brief Log web interface error
 * @param error_message Error description
 * @param client_ip Client IP (if applicable)
 * @return ESP_OK on success
 */
esp_err_t web_log_error(const char* error_message, const char* client_ip);

/**
 * @brief Get web interface error log
 * @param log_buffer Buffer to write error log
 * @param buffer_size Size of log buffer
 * @return ESP_OK on success
 */
esp_err_t web_get_error_log(char* log_buffer, size_t buffer_size);

/**
 * @brief Clear error log
 * @return ESP_OK on success
 */
esp_err_t web_clear_error_log(void);

/**
 * @brief Get debug information for troubleshooting
 * @param debug_buffer Buffer to write debug info
 * @param buffer_size Size of debug buffer
 * @return ESP_OK on success
 */
esp_err_t web_get_debug_info(char* debug_buffer, size_t buffer_size);

// ═══════════════════════════════════════════════════════════════════════════════
// UTILITY AND CONFIGURATION API
// ═══════════════════════════════════════════════════════════════════════════════

/**
 * @brief Get default web interface configuration
 * @param config Buffer to write default configuration
 * @return ESP_OK on success
 */
esp_err_t web_get_default_config(web_interface_config_t* config);

/**
 * @brief Set web interface configuration
 * @param config New configuration
 * @return ESP_OK on success, error code if invalid
 */
esp_err_t web_set_config(const web_interface_config_t* config);

/**
 * @brief Get current web interface configuration
 * @param config Buffer to write current configuration
 * @return ESP_OK on success
 */
esp_err_t web_get_config(web_interface_config_t* config);

/**
 * @brief Convert web interface status to string
 * @param status Web interface status
 * @return String representation of status
 */
const char* web_interface_status_to_string(web_interface_status_t status);

/**
 * @brief Get web server uptime
 * @return Uptime in milliseconds
 */
uint64_t web_get_uptime(void);

/**
 * @brief Get server memory usage statistics
 * @param free_heap Pointer to store free heap size
 * @param min_free_heap Pointer to store minimum free heap
 * @return ESP_OK on success
 */
esp_err_t web_get_memory_usage(size_t* free_heap, size_t* min_free_heap);

/**
 * @brief Reset web interface statistics
 * @return ESP_OK on success
 */
esp_err_t web_reset_statistics(void);

/**
 * @brief Restart web server (stop and start)
 * @return ESP_OK on success, error code on failure
 */
esp_err_t web_restart_server(void);

// ═══════════════════════════════════════════════════════════════════════════════
// CONTENT TEMPLATES AND THEMES
// ═══════════════════════════════════════════════════════════════════════════════

/**
 * @brief Set web interface theme (CSS styling)
 * @param theme_name Theme identifier ("default", "dark", "minimal")
 * @return ESP_OK on success, ESP_ERR_NOT_FOUND if theme not found
 */
esp_err_t web_set_theme(const char* theme_name);

/**
 * @brief Get available themes
 * @param themes_buffer Buffer to write theme list
 * @param buffer_size Size of themes buffer
 * @return ESP_OK on success
 */
esp_err_t web_get_available_themes(char* themes_buffer, size_t buffer_size);

/**
 * @brief Customize page title and branding
 * @param title Page title
 * @param subtitle Page subtitle
 * @return ESP_OK on success
 */
esp_err_t web_set_page_branding(const char* title, const char* subtitle);

#endif // WEB_INTERFACE_H