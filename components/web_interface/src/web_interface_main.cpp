// components/web_interface/src/web_interface_main.cpp
// ═══════════════════════════════════════════════════════════════════════════════
// WEB_INTERFACE_MAIN.CPP - CORE WEB SERVER AND ROUTING
// ═══════════════════════════════════════════════════════════════════════════════
// 
// SINGLE RESPONSIBILITY: HTTP server management and request routing
// - HTTP server setup and configuration
// - Request routing to appropriate handlers
// - Static HTML page serving (simple approach)
// - Basic authentication and rate limiting
// ═══════════════════════════════════════════════════════════════════════════════

#include "web_interface.h"
#include "mode_coordinator.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "esp_http_server.h"
#include <cstring>

static const char* TAG = "WEB_MAIN";

// ═══════════════════════════════════════════════════════════════════════════════
// GLOBAL STATE
// ═══════════════════════════════════════════════════════════════════════════════

static httpd_handle_t g_server_handle = NULL;
static web_interface_status_t g_web_status = WEB_STATUS_STOPPED;
static web_server_stats_t g_server_stats = {0};
static bool g_web_initialized = false;

// Simple rate limiting
static uint32_t g_request_count = 0;
static uint64_t g_last_request_reset = 0;

// ═══════════════════════════════════════════════════════════════════════════════
// STATIC HTML CONTENT (MUCH SIMPLER APPROACH)
// ═══════════════════════════════════════════════════════════════════════════════

// Main page HTML - Static content with JavaScript for dynamic updates
static const char* HTML_MAIN_PAGE = R"(
<!DOCTYPE html>
<html>
<head>
    <title>ESP32-S3 Trolley - 3-Mode System</title>
    <meta name="viewport" content="width=device-width, initial-scale=1">
    <style>
        body { font-family: Arial; margin: 20px; background: #f0f0f0; }
        h1 { color: #333; text-align: center; }
        .status-panel { background: white; padding: 15px; border-radius: 8px; margin: 10px 0; box-shadow: 0 2px 4px rgba(0,0,0,0.1); }
        .mode-grid { display: grid; grid-template-columns: 1fr 1fr 1fr; gap: 15px; margin: 15px 0; }
        .mode-card { background: #f8f9fa; padding: 15px; border-radius: 6px; text-align: center; }
        .mode-available { border-left: 4px solid #28a745; background: #d4edda; }
        .mode-active { border-left: 4px solid #17a2b8; background: #d1ecf1; }
        .mode-blocked { border-left: 4px solid #dc3545; background: #f8d7da; }
        .sensor-grid { display: grid; grid-template-columns: 1fr 1fr; gap: 15px; }
        .sensor-card { background: #f8f9fa; padding: 12px; border-radius: 6px; }
        .sensor-healthy { border-left: 4px solid #28a745; }
        .sensor-warning { border-left: 4px solid #ffc107; background: #fff3cd; }
        .sensor-error { border-left: 4px solid #dc3545; background: #f8d7da; }
        .value { font-size: 1.2em; font-weight: bold; color: #007bff; }
        .error-msg { color: #dc3545; font-weight: bold; background: #f8d7da; padding: 10px; border-radius: 5px; }
        .success-msg { color: #155724; font-weight: bold; background: #d4edda; padding: 10px; border-radius: 5px; }
        .warning-msg { color: #856404; font-weight: bold; background: #fff3cd; padding: 10px; border-radius: 5px; }
        button { padding: 12px 20px; margin: 8px; font-size: 14px; border: none; border-radius: 5px; cursor: pointer; }
        .btn-primary { background-color: #007bff; color: white; }
        .btn-success { background-color: #28a745; color: white; }
        .btn-danger { background-color: #dc3545; color: white; }
        .btn-warning { background-color: #ffc107; color: black; }
        .btn-secondary { background-color: #6c757d; color: white; }
        .btn:disabled { opacity: 0.6; cursor: not-allowed; }
        .real-time { font-family: monospace; background: #000; color: #0f0; padding: 8px; border-radius: 4px; }
        .chip-info { background: #e3f2fd; padding: 10px; border-radius: 5px; margin: 10px 0; font-size: 14px; }
    </style>
</head>
<body>
    <h1>🚃 ESP32-S3 Trolley - 3-Mode System</h1>
    
    <div class="chip-info">
        <strong>Hardware:</strong> ESP32-S3 | <strong>Motor:</strong> Eco II 2807 + Littlebee 30A ESC<br>
        <strong>Wheel:</strong> 61mm diameter (191.6mm circumference) | <strong>Sensors:</strong> Hall + MPU6050<br>
        <strong>System:</strong> Wire Learning → Automatic (5 m/s) → Manual Control
    </div>
    
    <!-- System Status -->
    <div class="status-panel">
        <h2>🛡️ System Status</h2>
        <div id="system-status">Loading system status...</div>
    </div>
    
    <!-- Sensor Validation -->
    <div class="status-panel">
        <h2>📋 Sensor Validation</h2>
        <div id="sensor-validation">Loading sensor validation status...</div>
        <button class="btn btn-primary" onclick="sendCommand('V')">Start Sensor Validation</button>
        <button class="btn btn-success" id="confirm-hall-btn" onclick="sendCommand('H')" disabled>Confirm Hall Sensor</button>
        <button class="btn btn-success" id="confirm-accel-btn" onclick="sendCommand('C')" disabled>Confirm Accelerometer</button>
    </div>
    
    <!-- Three Modes -->
    <div class="mode-grid">
        <div class="mode-card" id="wire-learning-card">
            <h3>🔍 Mode 1: Wire Learning</h3>
            <div>Status: <span id="wire-learning-status">Loading...</span></div>
            <div>Speed: 0.1→1.0 m/s gradual</div>
            <div>Detection: Impact + Timeout + Speed</div>
            <button class="btn btn-warning" id="wire-learning-btn" onclick="sendCommand('W')" disabled>Start Wire Learning</button>
        </div>
        
        <div class="mode-card" id="automatic-card">
            <h3>🚀 Mode 2: Automatic</h3>
            <div>Status: <span id="automatic-status">Loading...</span></div>
            <div>Speed: 5 m/s + Coasting</div>
            <div>Cycles: <span id="cycle-count">0</span></div>
            <button class="btn btn-primary" id="automatic-btn" onclick="sendCommand('U')" disabled>Start Automatic</button>
            <button class="btn btn-secondary" id="interrupt-btn" onclick="sendCommand('I')" disabled>Interrupt</button>
        </div>
        
        <div class="mode-card" id="manual-card">
            <h3>🎮 Mode 3: Manual</h3>
            <div>Status: <span id="manual-status">Loading...</span></div>
            <div>Speed: <span id="manual-speed">0.0 m/s</span></div>
            <div>Direction: <span id="manual-direction">Forward</span></div>
            <button class="btn btn-success" id="manual-btn" onclick="sendCommand('M')" disabled>Activate Manual</button>
        </div>
    </div>
    
    <!-- Sensor Status -->
    <div class="sensor-grid">
        <div class="sensor-card" id="hall-sensor">
            <h3>🔄 Hall Sensor</h3>
            <div>Status: <span id="hall-sensor-status">Unknown</span></div>
            <div>Pulse Count: <span class="value" id="hall-pulses">0</span></div>
            <div>Wheel RPM: <span class="value" id="wheel-rpm">0.0</span></div>
            <div>Speed: <span class="value" id="wheel-speed">0.00 m/s</span></div>
            <div class="real-time" id="hall-real-time">●●●</div>
        </div>
        
        <div class="sensor-card" id="accel-sensor">
            <h3>📊 MPU6050 Accelerometer</h3>
            <div>Status: <span id="accel-sensor-status">Unknown</span></div>
            <div>Total: <span class="value" id="accel-total">0.00g</span></div>
            <div>Last Impact: <span class="value" id="last-impact">0.00g</span></div>
            <div>Threshold: <span class="value" id="impact-threshold">0.5g</span></div>
            <div id="impact-status" class="real-time">SAFE</div>
        </div>
    </div>
    
    <!-- Manual Controls (shown only when manual mode active) -->
    <div class="status-panel" id="manual-controls" style="display:none">
        <h2>🎮 Manual Control Commands</h2>
        <button class="btn btn-success" onclick="sendCommand('A')">ARM ESC</button>
        <button class="btn btn-danger" onclick="sendCommand('D')">DISARM ESC</button><br>
        <button class="btn btn-primary" onclick="sendCommand('F')">FORWARD</button>
        <button class="btn btn-primary" onclick="sendCommand('B')">BACKWARD</button>
        <button class="btn btn-secondary" onclick="sendCommand('S')">STOP</button><br>
        <button class="btn btn-warning" onclick="sendCommand('+')"">FASTER (+)</button>
        <button class="btn btn-warning" onclick="sendCommand('-')">SLOWER (-)</button>
    </div>
    
    <!-- System Commands -->
    <div class="status-panel">
        <h2>🔧 System Commands</h2>
        <button class="btn btn-secondary" onclick="sendCommand('T')">REFRESH STATUS</button>
        <button class="btn btn-secondary" onclick="sendCommand('Q')">STOP CURRENT MODE</button>
        <button class="btn btn-danger" onclick="sendCommand('E')">🚨 EMERGENCY STOP</button>
        <button class="btn btn-secondary" onclick="sendCommand('R')">RESET SYSTEM</button>
    </div>
    
    <script src="/js/main.js"></script>
</body>
</html>
)";

// JavaScript file content
static const char* JS_MAIN_CONTENT = R"(
// Main JavaScript for 3-Mode Trolley Interface

function sendCommand(cmd) {
    console.log('Sending command:', cmd);
    fetch('/api/command', {
        method: 'POST',
        body: cmd,
        headers: {'Content-Type': 'text/plain'}
    })
    .then(response => response.json())
    .then(data => {
        if (data.success) {
            showMessage(data.message, 'success');
        } else {
            showMessage('Command failed: ' + data.message, 'error');
        }
        updateStatus();
    })
    .catch(error => {
        console.error('Error:', error);
        showMessage('Communication error: ' + error, 'error');
    });
}

function showMessage(msg, type) {
    const statusDiv = document.getElementById('system-status');
    const className = type === 'success' ? 'success-msg' : type === 'error' ? 'error-msg' : 'warning-msg';
    statusDiv.innerHTML = '<div class="' + className + '">' + msg + '</div>';
}

function updateStatus() {
    fetch('/api/status')
    .then(response => response.json())
    .then(data => {
        updateSystemStatus(data);
        updateSensorStatus(data);
        updateModeStatus(data);
        updateButtons(data);
    })
    .catch(error => {
        console.error('Status update error:', error);
        document.getElementById('system-status').innerHTML = '<div class="error-msg">Communication Error</div>';
    });
}

function updateSystemStatus(data) {
    const systemDiv = document.getElementById('system-status');
    if (data.system_healthy) {
        systemDiv.innerHTML = '<div class="success-msg">✅ System Healthy - ' + data.current_mode_status + '</div>';
    } else {
        systemDiv.innerHTML = '<div class="error-msg">❌ System Error: ' + data.error_message + '</div>';
    }

    const validationDiv = document.getElementById('sensor-validation');
    if (data.sensors_validated) {
        validationDiv.innerHTML = '<div class="success-msg">✅ ' + data.sensor_validation_message + '</div>';
    } else {
        validationDiv.innerHTML = '<div class="warning-msg">⚠️ ' + data.sensor_validation_message + '</div>';
    }
}

function updateSensorStatus(data) {
    // Hall sensor
    const hallCard = document.getElementById('hall-sensor');
    const hallStatus = data.hall_status || 'unknown';
    hallCard.className = 'sensor-card ' + (hallStatus === 'healthy' ? 'sensor-healthy' : hallStatus === 'failed' ? 'sensor-error' : 'sensor-warning');
    document.getElementById('hall-sensor-status').textContent = hallStatus;
    document.getElementById('hall-pulses').textContent = data.hall_pulses || 0;
    document.getElementById('wheel-rpm').textContent = (data.wheel_rpm || 0).toFixed(1);
    document.getElementById('wheel-speed').textContent = (data.wheel_speed || 0).toFixed(2);
    document.getElementById('hall-real-time').textContent = data.wheel_rotation_detected ? '🟢 ROTATING' : '🔴 STOPPED';

    // Accelerometer
    const accelCard = document.getElementById('accel-sensor');
    const accelStatus = data.accel_status || 'unknown';
    accelCard.className = 'sensor-card ' + (accelStatus === 'healthy' ? 'sensor-healthy' : accelStatus === 'failed' ? 'sensor-error' : 'sensor-warning');
    document.getElementById('accel-sensor-status').textContent = accelStatus;
    document.getElementById('accel-total').textContent = (data.accel_total || 0).toFixed(2) + 'g';
    document.getElementById('last-impact').textContent = (data.last_impact || 0).toFixed(2) + 'g';
    document.getElementById('impact-threshold').textContent = (data.impact_threshold || 0.5).toFixed(1) + 'g';
    const impactLevel = data.accel_total || 0;
    document.getElementById('impact-status').textContent = impactLevel > (data.impact_threshold || 0.5) ? '⚠️ IMPACT' : 'SAFE';
}

function updateModeStatus(data) {
    // Wire Learning
    const wireCard = document.getElementById('wire-learning-card');
    const wireAvail = data.wire_learning_availability || 'blocked';
    wireCard.className = 'mode-card ' + (wireAvail === 'available' ? 'mode-available' : wireAvail === 'active' ? 'mode-active' : 'mode-blocked');
    document.getElementById('wire-learning-status').textContent = wireAvail;

    // Automatic
    const autoCard = document.getElementById('automatic-card');
    const autoAvail = data.automatic_availability || 'blocked';
    autoCard.className = 'mode-card ' + (autoAvail === 'available' ? 'mode-available' : autoAvail === 'active' ? 'mode-active' : 'mode-blocked');
    document.getElementById('automatic-status').textContent = autoAvail;
    document.getElementById('cycle-count').textContent = data.auto_cycle_count || 0;

    // Manual
    const manualCard = document.getElementById('manual-card');
    const manualAvail = data.manual_availability || 'blocked';
    manualCard.className = 'mode-card ' + (manualAvail === 'available' ? 'mode-available' : manualAvail === 'active' ? 'mode-active' : 'mode-blocked');
    document.getElementById('manual-status').textContent = manualAvail;
    document.getElementById('manual-speed').textContent = (data.manual_speed || 0).toFixed(1) + ' m/s';
    document.getElementById('manual-direction').textContent = data.manual_direction_forward ? 'Forward' : 'Reverse';

    // Show/hide manual controls
    const manualControls = document.getElementById('manual-controls');
    manualControls.style.display = (data.current_mode === 'Manual') ? 'block' : 'none';
}

function updateButtons(data) {
    const sensorsValidated = data.sensors_validated || false;
    const wireComplete = data.wire_learning_complete || false;
    const currentMode = data.current_mode || 'None';

    // Sensor validation buttons
    document.getElementById('confirm-hall-btn').disabled = data.sensor_validation_state !== 'hall_pending';
    document.getElementById('confirm-accel-btn').disabled = data.sensor_validation_state !== 'accel_pending';

    // Mode buttons
    document.getElementById('wire-learning-btn').disabled = !sensorsValidated || currentMode !== 'None';
    document.getElementById('automatic-btn').disabled = !sensorsValidated || !wireComplete || currentMode !== 'None';
    document.getElementById('manual-btn').disabled = !sensorsValidated || currentMode !== 'None';
    document.getElementById('interrupt-btn').disabled = currentMode !== 'Automatic';
}

// Auto-update every 1 second
updateStatus();
setInterval(updateStatus, 1000);
)";

// ═══════════════════════════════════════════════════════════════════════════════
// HTTP HANDLERS
// ═══════════════════════════════════════════════════════════════════════════════

esp_err_t web_handler_root(httpd_req_t *req) {
    g_server_stats.total_requests++;
    
    httpd_resp_set_type(req, "text/html");
    httpd_resp_set_hdr(req, "Cache-Control", "no-cache");
    httpd_resp_send(req, HTML_MAIN_PAGE, strlen(HTML_MAIN_PAGE));
    
    g_server_stats.successful_requests++;
    return ESP_OK;
}

esp_err_t web_handler_js_main(httpd_req_t *req) {
    httpd_resp_set_type(req, "application/javascript");
    httpd_resp_set_hdr(req, "Cache-Control", "max-age=3600"); // Cache for 1 hour
    httpd_resp_send(req, JS_MAIN_CONTENT, strlen(JS_MAIN_CONTENT));
    return ESP_OK;
}

esp_err_t web_handler_api_status(httpd_req_t *req) {
    g_server_stats.total_requests++;
    g_server_stats.status_requests++;
    
    // Generate status JSON - delegated to status handler
    char json_buffer[2048];
    esp_err_t result = web_generate_status_json(json_buffer, sizeof(json_buffer));
    
    if (result != ESP_OK) {
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Status generation failed");
        g_server_stats.failed_requests++;
        return result;
    }
    
    httpd_resp_set_type(req, "application/json");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    httpd_resp_set_hdr(req, "Cache-Control", "no-cache");
    httpd_resp_send(req, json_buffer, strlen(json_buffer));
    
    g_server_stats.successful_requests++;
    return ESP_OK;
}

esp_err_t web_handler_api_command(httpd_req_t *req) {
    g_server_stats.total_requests++;
    
    // Simple rate limiting
    uint64_t current_time = esp_timer_get_time();
    if (current_time - g_last_request_reset > 60000000ULL) { // Reset every minute
        g_request_count = 0;
        g_last_request_reset = current_time;
    }
    
    if (g_request_count > 60) { // Max 60 commands per minute
        httpd_resp_send_err(req, HTTPD_429_TOO_MANY_REQUESTS, "Rate limit exceeded");
        g_server_stats.failed_requests++;
        return ESP_FAIL;
    }
    g_request_count++;
    
    // Read command
    char command_buffer[16];
    int ret = httpd_req_recv(req, command_buffer, sizeof(command_buffer) - 1);
    
    if (ret <= 0) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Bad request");
        g_server_stats.failed_requests++;
        return ESP_FAIL;
    }
    
    command_buffer[ret] = '\0';
    
    // Process command - delegated to command handler
    char response_message[256];
    esp_err_t result = web_process_command(command_buffer[0], "web_client", 
                                          response_message, sizeof(response_message));
    
    // Generate JSON response
    char json_response[512];
    snprintf(json_response, sizeof(json_response),
        "{"
        "\"success\": %s,"
        "\"message\": \"%s\","
        "\"timestamp\": %llu"
        "}",
        result == ESP_OK ? "true" : "false",
        response_message,
        esp_timer_get_time() / 1000);
    
    httpd_resp_set_type(req, "application/json");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    httpd_resp_send(req, json_response, strlen(json_response));
    
    if (result == ESP_OK) {
        g_server_stats.successful_requests++;
    } else {
        g_server_stats.failed_requests++;
    }
    
    return ESP_OK;
}

esp_err_t web_handler_options(httpd_req_t *req) {
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Methods", "POST, GET, OPTIONS");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Headers", "Content-Type");
    httpd_resp_send(req, NULL, 0);
    return ESP_OK;
}

// ═══════════════════════════════════════════════════════════════════════════════
// PUBLIC API IMPLEMENTATION
// ═══════════════════════════════════════════════════════════════════════════════

esp_err_t web_interface_init(const web_interface_config_t* config) {
    ESP_LOGI(TAG, "Initializing web interface...");
    
    memset(&g_server_stats, 0, sizeof(g_server_stats));
    g_server_stats.server_start_time = esp_timer_get_time();
    
    g_web_status = WEB_STATUS_STOPPED;
    g_web_initialized = true;
    
    ESP_LOGI(TAG, "Web interface initialized");
    return ESP_OK;
}

esp_err_t web_interface_start(void) {
    if (!g_web_initialized) {
        return ESP_ERR_INVALID_STATE;
    }
    
    ESP_LOGI(TAG, "Starting web server...");
    g_web_status = WEB_STATUS_STARTING;
    
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.server_port = 80;
    config.max_open_sockets = 7;
    config.stack_size = 8192;
    config.task_priority = 5;
    
    esp_err_t result = httpd_start(&g_server_handle, &config);
    if (result != ESP_OK) {
        ESP_LOGE(TAG, "Failed to start HTTP server: %s", esp_err_to_name(result));
        g_web_status = WEB_STATUS_ERROR;
        return result;
    }
    
    // Register handlers
    httpd_uri_t uri_handlers[] = {
        {.uri = "/",              .method = HTTP_GET,  .handler = web_handler_root,        .user_ctx = NULL},
        {.uri = "/js/main.js",    .method = HTTP_GET,  .handler = web_handler_js_main,     .user_ctx = NULL},
        {.uri = "/api/status",    .method = HTTP_GET,  .handler = web_handler_api_status,  .user_ctx = NULL},
        {.uri = "/api/command",   .method = HTTP_POST, .handler = web_handler_api_command, .user_ctx = NULL},
        {.uri = "/*",             .method = HTTP_OPTIONS, .handler = web_handler_options, .user_ctx = NULL}
    };
    
    for (int i = 0; i < sizeof(uri_handlers) / sizeof(uri_handlers[0]); i++) {
        httpd_register_uri_handler(g_server_handle, &uri_handlers[i]);
    }
    
    g_web_status = WEB_STATUS_RUNNING;
    ESP_LOGI(TAG, "Web server started successfully on port 80");
    
    return ESP_OK;
}

esp_err_t web_interface_stop(void) {
    ESP_LOGI(TAG, "Stopping web server...");
    g_web_status = WEB_STATUS_STOPPING;
    
    if (g_server_handle != NULL) {
        esp_err_t result = httpd_stop(g_server_handle);
        g_server_handle = NULL;
        g_web_status = (result == ESP_OK) ? WEB_STATUS_STOPPED : WEB_STATUS_ERROR;
        return result;
    }
    
    g_web_status = WEB_STATUS_STOPPED;
    return ESP_OK;
}

web_interface_status_t web_interface_get_status(void) {
    return g_web_status;
}

bool web_interface_is_running(void) {
    return g_web_status == WEB_STATUS_RUNNING;
}

web_server_stats_t web_interface_get_stats(void) {
    return g_server_stats;
}

esp_err_t web_interface_update(void) {
    // Minimal maintenance
    return ESP_OK;
}

// WiFi AP setup
esp_err_t web_wifi_init_ap(const char* ssid, const char* password) {
    ESP_LOGI(TAG, "Initializing WiFi Access Point...");
    
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    
    esp_netif_create_default_wifi_ap();
    
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    
    wifi_config_t wifi_config = {0};
    strncpy((char*)wifi_config.ap.ssid, ssid, sizeof(wifi_config.ap.ssid) - 1);
    wifi_config.ap.ssid_len = strlen(ssid);
    
    if (password != NULL && strlen(password) > 0) {
        strncpy((char*)wifi_config.ap.password, password, sizeof(wifi_config.ap.password) - 1);
        wifi_config.ap.authmode = WIFI_AUTH_WPA2_PSK;
    } else {
        wifi_config.ap.authmode = WIFI_AUTH_OPEN;
    }
    
    wifi_config.ap.max_connection = 4;
    wifi_config.ap.channel = 11;
    wifi_config.ap.beacon_interval = 100;
    
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_AP));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());
    
    ESP_LOGI(TAG, "WiFi AP: SSID='%s', IP: 192.168.4.1", ssid);
    return ESP_OK;
}

uint8_t web_wifi_get_client_count(void) {
    return g_server_stats.active_connections;
}