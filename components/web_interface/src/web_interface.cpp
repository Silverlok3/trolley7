g_client_info[index].rate_limited = false;
        } else {
            return ESP_ERR_NO_MEM; // Too many clients
        }
    }
    
    web_client_info_t* client = &g_client_info[index];
    client->requests_sent++;
    client->last_request_time = esp_timer_get_time();
    
    // Check rate limit
    if (client->requests_sent > WEB_MAX_COMMANDS_PER_MINUTE) {
        client->rate_limited = true;
        ESP_LOGW(TAG, "Rate limiting client %s: %lu requests", client_ip, client->requests_sent);
    }
    
    return ESP_OK;
}

esp_err_t web_get_client_info(const char* client_ip, web_client_info_t* client_info) {
    if (client_ip == NULL || client_info == NULL) return ESP_ERR_INVALID_ARG;
    
    int index = find_client_index(client_ip);
    if (index < 0) {
        return ESP_ERR_NOT_FOUND;
    }
    
    memcpy(client_info, &g_client_info[index], sizeof(web_client_info_t));
    return ESP_OK;
}

esp_err_t web_clear_rate_limiting(void) {
    memset(g_client_info, 0, sizeof(g_client_info));
    g_client_count = 0;
    ESP_LOGI(TAG, "Rate limiting data cleared");
    return ESP_OK;
}

// ═══════════════════════════════════════════════════════════════════════════════
// COMMAND PROCESSING
// ═══════════════════════════════════════════════════════════════════════════════

esp_err_t web_process_command(char command_char, const char* client_ip,
                             char* response_buffer, size_t buffer_size) {
    if (response_buffer == NULL) return ESP_ERR_INVALID_ARG;
    
    // Validate command first
    if (!web_validate_command(command_char, client_ip)) {
        snprintf(response_buffer, buffer_size, "Command rejected: rate limited or invalid");
        return ESP_ERR_INVALID_ARG;
    }
    
    esp_err_t result = ESP_OK;
    command_char = toupper(command_char);
    
    ESP_LOGI(TAG, "Processing command '%c' from %s", command_char, client_ip ? client_ip : "unknown");
    
    switch (command_char) {
        // Mode activation commands
        case 'W': // Wire Learning Mode
            result = mode_coordinator_activate_wire_learning();
            if (result == ESP_OK) {
                snprintf(response_buffer, buffer_size, "Wire learning mode activated");
            } else {
                snprintf(response_buffer, buffer_size, "Failed to activate wire learning: %s", 
                        mode_coordinator_get_error_message());
            }
            break;
            
        case 'U': // Automatic Mode
            result = mode_coordinator_activate_automatic();
            if (result == ESP_OK) {
                snprintf(response_buffer, buffer_size, "Automatic mode activated - 5 m/s cycling started");
            } else {
                snprintf(response_buffer, buffer_size, "Failed to activate automatic mode: %s", 
                        mode_coordinator_get_error_message());
            }
            break;
            
        case 'M': // Manual Mode
            result = mode_coordinator_activate_manual();
            if (result == ESP_OK) {
                snprintf(response_buffer, buffer_size, "Manual mode activated - ARM ESC to enable motor control");
            } else {
                snprintf(response_buffer, buffer_size, "Failed to activate manual mode: %s", 
                        mode_coordinator_get_error_message());
            }
            break;
            
        // Sensor validation commands
        case 'H': // Confirm Hall sensor
            result = mode_coordinator_confirm_hall_validation();
            if (result == ESP_OK) {
                snprintf(response_buffer, buffer_size, "Hall sensor validation confirmed - proceed to shake trolley");
            } else {
                snprintf(response_buffer, buffer_size, "Hall sensor validation not ready for confirmation");
            }
            break;
            
        case 'C': // Confirm Accelerometer
            result = mode_coordinator_confirm_accel_validation();
            if (result == ESP_OK) {
                snprintf(response_buffer, buffer_size, "Accelerometer validation confirmed - all sensors validated!");
            } else {
                snprintf(response_buffer, buffer_size, "Accelerometer validation not ready for confirmation");
            }
            break;
            
        case 'V': // Start sensor validation
            result = mode_coordinator_start_sensor_validation();
            if (result == ESP_OK) {
                snprintf(response_buffer, buffer_size, "Sensor validation started - ROTATE THE WHEEL manually");
            } else {
                snprintf(response_buffer, buffer_size, "Failed to start sensor validation");
            }
            break;
            
        // Manual mode commands (when manual mode is active)
        case 'A': // Arm ESC
            if (manual_mode_is_active()) {
                result = manual_mode_arm_esc();
                if (result == ESP_OK) {
                    snprintf(response_buffer, buffer_size, "ESC armed - motor control enabled");
                } else {
                    snprintf(response_buffer, buffer_size, "Failed to arm ESC");
                }
            } else {
                snprintf(response_buffer, buffer_size, "Manual mode not active");
                result = ESP_ERR_INVALID_STATE;
            }
            break;
            
        case 'D': // Disarm ESC
            if (manual_mode_is_active()) {
                result = manual_mode_disarm_esc();
                snprintf(response_buffer, buffer_size, "ESC disarmed - motor control disabled");
            } else {
                snprintf(response_buffer, buffer_size, "Manual mode not active");
                result = ESP_ERR_INVALID_STATE;
            }
            break;
            
        case 'F': // Forward
            if (manual_mode_is_active()) {
                result = manual_mode_move_forward();
                if (result == ESP_OK) {
                    snprintf(response_buffer, buffer_size, "Moving forward at %.1f m/s", 
                            manual_mode_get_current_speed());
                } else {
                    snprintf(response_buffer, buffer_size, "Failed to move forward - check ESC status");
                }
            } else {
                snprintf(response_buffer, buffer_size, "Manual mode not active");
                result = ESP_ERR_INVALID_STATE;
            }
            break;
            
        case 'B': // Backward
            if (manual_mode_is_active()) {
                result = manual_mode_move_backward();
                if (result == ESP_OK) {
                    snprintf(response_buffer, buffer_size, "Moving backward at %.1f m/s", 
                            manual_mode_get_current_speed());
                } else {
                    snprintf(response_buffer, buffer_size, "Failed to move backward - check ESC status");
                }
            } else {
                snprintf(response_buffer, buffer_size, "Manual mode not active");
                result = ESP_ERR_INVALID_STATE;
            }
            break;
            
        case 'S': // Stop movement
            if (manual_mode_is_active()) {
                result = manual_mode_stop_movement();
                snprintf(response_buffer, buffer_size, "Motor stopped");
            } else {
                snprintf(response_buffer, buffer_size, "Manual mode not active");
                result = ESP_ERR_INVALID_STATE;
            }
            break;
            
        case '+': // Increase speed
            if (manual_mode_is_active()) {
                result = manual_mode_increase_speed();
                if (result == ESP_OK) {
                    snprintf(response_buffer, buffer_size, "Speed increased to %.1f m/s", 
                            manual_mode_get_current_speed());
                } else {
                    snprintf(response_buffer, buffer_size, "Cannot increase speed - at maximum or ESC not armed");
                }
            } else {
                snprintf(response_buffer, buffer_size, "Manual mode not active");
                result = ESP_ERR_INVALID_STATE;
            }
            break;
            
        case '-': // Decrease speed
            if (manual_mode_is_active()) {
                result = manual_mode_decrease_speed();
                if (result == ESP_OK) {
                    snprintf(response_buffer, buffer_size, "Speed decreased to %.1f m/s", 
                            manual_mode_get_current_speed());
                } else {
                    snprintf(response_buffer, buffer_size, "Cannot decrease speed");
                }
            } else {
                snprintf(response_buffer, buffer_size, "Manual mode not active");
                result = ESP_ERR_INVALID_STATE;
            }
            break;
            
        // Mode control commands
        case 'Q': // Stop current mode
            result = mode_coordinator_stop_current_mode(false);
            snprintf(response_buffer, buffer_size, "Current mode stopped gracefully");
            break;
            
        case 'I': // Interrupt current mode
            if (automatic_mode_is_active()) {
                result = automatic_mode_interrupt();
                snprintf(response_buffer, buffer_size, "Automatic mode interrupted - will stop at wire end");
            } else {
                result = mode_coordinator_stop_current_mode(true);
                snprintf(response_buffer, buffer_size, "Current mode stopped immediately");
            }
            break;
            
        // Emergency and system commands
        case 'E': // Emergency stop
            result = mode_coordinator_emergency_stop();
            snprintf(response_buffer, buffer_size, "EMERGENCY STOP - All modes stopped, motor halted");
            break;
            
        case 'R': // Reset system
            result = mode_coordinator_reset_system();
            snprintf(response_buffer, buffer_size, "System reset complete - sensor validation required");
            break;
            
        case 'T': // Status
            {
                system_mode_status_t status = mode_coordinator_get_status();
                snprintf(response_buffer, buffer_size, 
                        "Mode: %s, Sensors: %s, Status: %s", 
                        mode_coordinator_mode_to_string(status.current_mode),
                        status.sensors_validated ? "Validated" : "Not Validated",
                        status.current_mode_status);
            }
            break;
            
        default:
            snprintf(response_buffer, buffer_size, "Unknown command '%c' - use web interface for available commands", command_char);
            result = ESP_ERR_INVALID_ARG;
            break;
    }
    
    // Log command execution
    if (g_web_config.enable_command_logging) {
        web_log_command(command_char, client_ip, (result == ESP_OK), response_buffer);
    }
    
    // Update statistics
    if (result == ESP_OK) {
        g_server_stats.commands_executed++;
    }
    
    return result;
}

bool web_validate_command(char command_char, const char* client_ip) {
    // Check rate limiting
    if (web_is_client_rate_limited(client_ip)) {
        ESP_LOGW(TAG, "Command from %s rejected - rate limited", client_ip);
        return false;
    }
    
    // Update rate limiting
    web_update_rate_limiting(client_ip);
    
    // Validate command character
    const char* valid_commands = "WUMHCVADFSB+-QIETRLK";
    if (strchr(valid_commands, toupper(command_char)) == NULL) {
        ESP_LOGW(TAG, "Invalid command character: '%c'", command_char);
        return false;
    }
    
    return true;
}

esp_err_t web_log_command(char command_char, const char* client_ip, 
                         bool success, const char* response_message) {
    const char* status_str = success ? "SUCCESS" : "FAILED";
    ESP_LOGI(TAG, "CMD[%c] %s from %s: %s", command_char, status_str, 
            client_ip ? client_ip : "unknown", response_message);
    return ESP_OK;
}

// ═══════════════════════════════════════════════════════════════════════════════
// HTML CONTENT GENERATION
// ═══════════════════════════════════════════════════════════════════════════════

esp_err_t web_generate_main_page(char* html_buffer, size_t buffer_size) {
    if (html_buffer == NULL) return ESP_ERR_INVALID_ARG;
    
    const char* html_template = 
        "<!DOCTYPE html>"
        "<html><head>"
        "<title>ESP32-S3 Trolley - 3-Mode System</title>"
        "<meta name='viewport' content='width=device-width, initial-scale=1'>"
        "<style>"
        "body { font-family: Arial; margin: 20px; background: #f0f0f0; }"
        "h1 { color: #333; text-align: center; }"
        "h2 { color: #666; margin-top: 20px; }"
        ".status-panel { background: white; padding: 15px; border-radius: 8px; margin: 10px 0; box-shadow: 0 2px 4px rgba(0,0,0,0.1); }"
        ".mode-grid { display: grid; grid-template-columns: 1fr 1fr 1fr; gap: 15px; margin: 15px 0; }"
        ".mode-card { background: #f8f9fa; padding: 15px; border-radius: 6px; border-left: 4px solid #007bff; text-align: center; }"
        ".mode-available { border-left-color: #28a745; background: #d4edda; }"
        ".mode-active { border-left-color: #17a2b8; background: #d1ecf1; }"
        ".mode-blocked { border-left-color: #dc3545; background: #f8d7da; }"
        ".sensor-grid { display: grid; grid-template-columns: 1fr 1fr; gap: 15px; margin: 15px 0; }"
        ".sensor-card { background: #f8f9fa; padding: 12px; border-radius: 6px; border-left: 4px solid #007bff; }"
        ".sensor-healthy { border-left-color: #28a745; }"
        ".sensor-warning { border-left-color: #ffc107; background: #fff3cd; }"
        ".sensor-error { border-left-color: #dc3545; background: #f8d7da; }"
        ".value { font-size: 1.2em; font-weight: bold; color: #007bff; }"
        ".error-msg { color: #dc3545; font-weight: bold; background: #f8d7da; padding: 10px; border-radius: 5px; }"
        ".success-msg { color: #155724; font-weight: bold; background: #d4edda; padding: 10px; border-radius: 5px; }"
        ".warning-msg { color: #856404; font-weight: bold; background: #fff3cd; padding: 10px; border-radius: 5px; }"
        "button { padding: 12px 20px; margin: 8px; font-size: 14px; border: none; border-radius: 5px; cursor: pointer; }"
        ".btn-primary { background-color: #007bff; color: white; }"
        ".btn-success { background-color: #28a745; color: white; }"
        ".btn-danger { background-color: #dc3545; color: white; }"
        ".btn-warning { background-color: #ffc107; color: black; }"
        ".btn-secondary { background-color: #6c757d; color: white; }"
        ".btn-safe { background-color: #17a2b8; color: white; }"
        ".btn:disabled { opacity: 0.6; cursor: not-allowed; }"
        ".real-time { font-family: monospace; background: #000; color: #0f0; padding: 8px; border-radius: 4px; }"
        ".chip-info { background: #e3f2fd; padding: 10px; border-radius: 5px; margin: 10px 0; font-size: 14px; }"
        "</style></head>"
        "<body>"
        "<h1>🚃 ESP32-S3 Trolley - 3-Mode System</h1>"
        
        "<div class='chip-info'>"
        "<strong>Hardware:</strong> ESP32-S3 | <strong>Motor:</strong> Eco II 2807 + Littlebee 30A ESC<br>"
        "<strong>Wheel:</strong> 61mm diameter (191.6mm circumference) | <strong>Sensors:</strong> Hall + MPU6050<br>"
        "<strong>System:</strong> Wire Learning → Automatic (5 m/s) → Manual Control"
        "</div>"
        
        "<div class='status-panel'>"
        "<h2>🛡️ System Status</h2>"
        "<div id='system-status'>Loading system status...</div>"
        "</div>"
        
        "<div class='status-panel'>"
        "<h2>📋 Sensor Validation</h2>"
        "<div id='sensor-validation'>Loading sensor validation status...</div>"
        "<button class='btn btn-primary' onclick=\"sendCommand('V')\">Start Sensor Validation</button>"
        "<button class='btn btn-success' id='confirm-hall-btn' onclick=\"sendCommand('H')\" disabled>Confirm Hall Sensor</button>"
        "<button class='btn btn-success' id='confirm-accel-btn' onclick=\"sendCommand('C')\" disabled>Confirm Accelerometer</button>"
        "</div>"
        
        "<div class='mode-grid'>"
        "<div class='mode-card' id='wire-learning-card'>"
        "<h3>🔍 Mode 1: Wire Learning</h3>"
        "<div>Status: <span id='wire-learning-status'>Loading...</span></div>"
        "<div>Speed: 0.1→1.0 m/s gradual</div>"
        "<div>Detection: Impact + Timeout + Speed</div>"
        "<button class='btn btn-warning' id='wire-learning-btn' onclick=\"sendCommand('W')\" disabled>Start Wire Learning</button>"
        "</div>"
        
        "<div class='mode-card' id='automatic-card'>"
        "<h3>🚀 Mode 2: Automatic</h3>"
        "<div>Status: <span id='automatic-status'>Loading...</span></div>"
        "<div>Speed: 5 m/s + Coasting</div>"
        "<div>Cycles: <span id='cycle-count'>0</span></div>"
        "<button class='btn btn-primary' id='automatic-btn' onclick=\"sendCommand('U')\" disabled>Start Automatic</button>"
        "<button class='btn btn-secondary' id='interrupt-btn' onclick=\"sendCommand('I')\" disabled>Interrupt</button>"
        "</div>"
        
        "<div class='mode-card' id='manual-card'>"
        "<h3>🎮 Mode 3: Manual</h3>"
        "<div>Status: <span id='manual-status'>Loading...</span></div>"
        "<div>Speed: <span id='manual-speed'>0.0 m/s</span></div>"
        "<div>Direction: <span id='manual-direction'>Forward</span></div>"
        "<button class='btn btn-success' id='manual-btn' onclick=\"sendCommand('M')\" disabled>Activate Manual</button>"
        "</div>"
        "</div>"
        
        "<div class='sensor-grid'>"
        "<div class='sensor-card' id='hall-sensor'>"
        "<h3>🔄 Hall Sensor</h3>"
        "<div>Status: <span id='hall-sensor-status'>Unknown</span></div>"
        "<div>Pulse Count: <span class='value' id='hall-pulses'>0</span></div>"
        "<div>Wheel RPM: <span class='value' id='wheel-rpm'>0.0</span></div>"
        "<div>Speed: <span class='value' id='wheel-speed'>0.00 m/s</span></div>"
        "<div class='real-time' id='hall-real-time'>●●●</div>"
        "</div>"
        
        "<div class='sensor-card' id='accel-sensor'>"
        "<h3>📊 MPU6050 Accelerometer</h3>"
        "<div>Status: <span id='accel-sensor-status'>Unknown</span></div>"
        "<div>Total: <span class='value' id='accel-total'>0.00g</span></div>"
        "<div>Last Impact: <span class='value' id='last-impact'>0.00g</span></div>"
        "<div>Threshold: <span class='value' id='impact-threshold'>0.5g</span></div>"
        "<div id='impact-status' class='real-time'>SAFE</div>"
        "</div>"
        "</div>"
        
        "<div class='status-panel' id='manual-controls' style='display:none'>"
        "<h2>🎮 Manual Control Commands</h2>"
        "<button class='btn btn-success' onclick=\"sendCommand('A')\">ARM ESC</button>"
        "<button class='btn btn-danger' onclick=\"sendCommand('D')\">DISARM ESC</button><br>"
        "<button class='btn btn-primary' onclick=\"sendCommand('F')\">FORWARD</button>"
        "<button class='btn btn-primary' onclick=\"sendCommand('B')\">BACKWARD</button>"
        "<button class='btn btn-secondary' onclick=\"sendCommand('S')\">STOP</button><br>"
        "<button class='btn btn-warning' onclick=\"sendCommand('+')\">FASTER (+)</button>"
        "<button class='btn btn-warning' onclick=\"sendCommand('-')\">SLOWER (-)</button>"
        "</div>"
        
        "<div class='status-panel'>"
        "<h2>🔧 System Commands</h2>"
        "<button class='btn btn-secondary' onclick=\"sendCommand('T')\">REFRESH STATUS</button>"
        "<button class='btn btn-safe' onclick=\"sendCommand('Q')\">STOP CURRENT MODE</button>"
        "<button class='btn btn-danger' onclick=\"sendCommand('E')\">🚨 EMERGENCY STOP</button>"
        "<button class='btn btn-secondary' onclick=\"sendCommand('R')\">RESET SYSTEM</button>"
        "</div>"
        
        "<script>"
        "function sendCommand(cmd) {"
        "  console.log('Sending command:', cmd);"
        "  fetch('/command', {"
        "    method: 'POST',"
        "    body: cmd,"
        "    headers: {'Content-Type': 'text/plain'}"
        "  })"
        "  .then(response => response.json())"
        "  .then(data => {"
        "    if (data.success) {"
        "      showMessage(data.message, 'success');"
        "    } else {"
        "      showMessage('Command failed: ' + data.message, 'error');"
        "    }"
        "    updateStatus();"
        "  })"
        "  .catch(error => {"
        "    console.error('Error:', error);"
        "    showMessage('Communication error: ' + error, 'error');"
        "  });"
        "}"
        
        "function showMessage(msg, type) {"
        "  const statusDiv = document.getElementById('system-status');"
        "  const className = type === 'success' ? 'success-msg' : type === 'error' ? 'error-msg' : 'warning-msg';"
        "  statusDiv.innerHTML = '<div class=\"' + className + '\">' + msg + '</div>';"
        "}"
        
        "function updateStatus() {"
        "  fetch('/status')"
        "  .then(response => response.json())"
        "  .then(data => {"
        "    updateSystemStatus(data);"
        "    updateSensorStatus(data);"
        "    updateModeStatus(data);"
        "    updateButtons(data);"
        "  })"
        "  .catch(error => {"
        "    console.error('Status update error:', error);"
        "    document.getElementById('system-status').innerHTML = '<div class=\"error-msg\">Communication Error</div>';"
        "  });"
        "}"
        
        "function updateSystemStatus(data) {"
        "  const systemDiv = document.getElementById('system-status');"
        "  if (data.system_healthy) {"
        "    systemDiv.innerHTML = '<div class=\"success-msg\">✅ System Healthy - ' + data.current_mode_status + '</div>';"
        "  } else {"
        "    systemDiv.innerHTML = '<div class=\"error-msg\">❌ System Error: ' + data.error_message + '</div>';"
        "  }"
        
        "  const validationDiv = document.getElementById('sensor-validation');"
        "  if (data.sensors_validated) {"
        "    validationDiv.innerHTML = '<div class=\"success-msg\">✅ ' + data.sensor_validation_message + '</div>';"
        "  } else {"
        "    validationDiv.innerHTML = '<div class=\"warning-msg\">⚠️ ' + data.sensor_validation_message + '</div>';"
        "  }"
        "}"
        
        "function updateSensorStatus(data) {"
        "  // Hall sensor"
        "  const hallCard = document.getElementById('hall-sensor');"
        "  const hallStatus = data.hall_status || 'unknown';"
        "  hallCard.className = 'sensor-card ' + (hallStatus === 'healthy' ? 'sensor-healthy' : hallStatus === 'failed' ? 'sensor-error' : 'sensor-warning');"
        "  document.getElementById('hall-sensor-status').textContent = hallStatus;"
        "  document.getElementById('hall-pulses').textContent = data.hall_pulses || 0;"
        "  document.getElementById('wheel-rpm').textContent = (data.wheel_rpm || 0).toFixed(1);"
        "  document.getElementById('wheel-speed').textContent = (data.wheel_speed || 0).toFixed(2);"
        "  document.getElementById('hall-real-time').textContent = data.wheel_rotation_detected ? '🟢 ROTATING' : '🔴 STOPPED';"
        
        "  // Accelerometer"
        "  const accelCard = document.getElementById('accel-sensor');"
        "  const accelStatus = data.accel_status || 'unknown';"
        "  accelCard.className = 'sensor-card ' + (accelStatus === 'healthy' ? 'sensor-healthy' : accelStatus === 'failed' ? 'sensor-error' : 'sensor-warning');"
        "  document.getElementById('accel-sensor-status').textContent = accelStatus;"
        "  document.getElementById('accel-total').textContent = (data.accel_total || 0).toFixed(2) + 'g';"
        "  document.getElementById('last-impact').textContent = (data.last_impact || 0).toFixed(2) + 'g';"
        "  document.getElementById('impact-threshold').textContent = (data.impact_threshold || 0.5).toFixed(1) + 'g';"
        "  const impactLevel = data.accel_total || 0;"
        "  document.getElementById('impact-status').textContent = impactLevel > (data.impact_threshold || 0.5) ? '⚠️ IMPACT' : 'SAFE';"
        "}"
        
        "function updateModeStatus(data) {"
        "  // Wire Learning"
        "  const wireCard = document.getElementById('wire-learning-card');"
        "  const wireAvail = data.wire_learning_availability || 'blocked';"
        "  wireCard.className = 'mode-card ' + (wireAvail === 'available' ? 'mode-available' : wireAvail === 'active' ? 'mode-active' : 'mode-blocked');"
        "  document.getElementById('wire-learning-status').textContent = wireAvail;"
        
        "  // Automatic"
        "  const autoCard = document.getElementById('automatic-card');"
        "  const autoAvail = data.automatic_availability || 'blocked';"
        "  autoCard.className = 'mode-card ' + (autoAvail === 'available' ? 'mode-available' : autoAvail === 'active' ? 'mode-active' : 'mode-blocked');"
        "  document.getElementById('automatic-status').textContent = autoAvail;"
        "  document.getElementById('cycle-count').textContent = data.auto_cycle_count || 0;"
        
        "  // Manual"
        "  const manualCard = document.getElementById('manual-card');"
        "  const manualAvail = data.manual_availability || 'blocked';"
        "  manualCard.className = 'mode-card ' + (manualAvail === 'available' ? 'mode-available' : manualAvail === 'active' ? 'mode-active' : 'mode-blocked');"
        "  document.getElementById('manual-status').textContent = manualAvail;"
        "  document.getElementById('manual-speed').textContent = (data.manual_speed || 0).toFixed(1) + ' m/s';"
        "  document.getElementById('manual-direction').textContent = data.manual_direction_forward ? 'Forward' : 'Reverse';"
        
        "  // Show/hide manual controls"
        "  const manualControls = document.getElementById('manual-controls');"
        "  manualControls.style.display = (data.current_mode === 'Manual') ? 'block' : 'none';"
        "}"
        
        "function updateButtons(data) {"
        "  const sensorsValidated = data.sensors_validated || false;"
        "  const wireComplete = data.wire_learning_complete || false;"
        "  const currentMode = data.current_mode || 'None';"
        
        "  // Sensor validation buttons"
        "  document.getElementById('confirm-hall-btn').disabled = data.sensor_validation_state !== 'hall_pending';"
        "  document.getElementById('confirm-accel-btn').disabled = data.sensor_validation_state !== 'accel_pending';"
        
        "  // Mode buttons"
        "  document.getElementById('wire-learning-btn').disabled = !sensorsValidated || currentMode !== 'None';"
        "  document.getElementById('automatic-btn').disabled = !sensorsValidated || !wireComplete || currentMode !== 'None';"
        "  document.getElementById('manual-btn').disabled = !sensorsValidated || currentMode !== 'None';"
        "  document.getElementById('interrupt-btn').disabled = currentMode !== 'Automatic';"
        "}"
        
        "// Auto-update every 1 second"
        "updateStatus();"
        "setInterval(updateStatus, 1000);"
        "</script>"
        "</body></html>";
    
    strncpy(html_buffer, html_template, buffer_size - 1);
    html_buffer[buffer_size - 1] = '\0';
    
    return ESP_OK;
}

esp_err_t web_generate_status_json(char* json_buffer, size_t buffer_size) {
    if (json_buffer == NULL) return ESP_ERR_INVALID_ARG;
    
    // Get status from all components
    system_mode_status_t mode_status = mode_coordinator_get_status();
    hardware_status_t hw_status = hardware_get_status();
    sensor_health_t sensor_status = sensor_health_get_status();
    
    // Get mode-specific status
    wire_learning_progress_t wire_progress = wire_learning_mode_get_progress();
    automatic_mode_progress_t auto_progress = automatic_mode_get_progress();
    manual_mode_status_t manual_status = manual_mode_get_status();
    
    snprintf(json_buffer, buffer_size,
        "{"
        "\"system_healthy\": %s,"
        "\"current_mode\": \"%s\","
        "\"current_mode_status\": \"%s\","
        "\"error_message\": \"%s\","
        
        "\"sensors_validated\": %s,"
        "\"sensor_validation_state\": \"%s\","
        "\"sensor_validation_message\": \"%s\","
        "\"hall_validation_complete\": %s,"
        "\"accel_validation_complete\": %s,"
        
        "\"wire_learning_availability\": \"%s\","
        "\"automatic_availability\": \"%s\","
        "\"manual_availability\": \"%s\","
        
        "\"hall_status\": \"%s\","
        "\"hall_pulses\": %lu,"
        "\"wheel_rpm\": %.1f,"
        "\"wheel_speed\": %.2f,"
        "\"wheel_rotation_detected\": %s,"
        
        "\"accel_status\": \"%s\","
        "\"accel_total\": %.2f,"
        "\"last_impact\": %.2f,"
        "\"impact_threshold\": %.1f,"
        "\"trolley_shake_detected\": %s,"
        
        "\"esc_armed\": %s,"
        "\"position_m\": %.2f,"
        "\"current_speed_ms\": %.2f,"
        "\"target_speed_ms\": %.2f,"
        "\"direction_forward\": %s,"
        "\"rotations\": %lu,"
        
        "\"wire_learning_complete\": %s,"
        "\"wire_length_m\": %.2f,"
        "\"wire_learning_state\": \"%s\","
        "\"wire_learning_progress\": %d,"
        
        "\"auto_cycle_count\": %lu,"
        "\"auto_cycle_interrupted\": %s,"
        "\"auto_coasting_calibrated\": %s,"
        "\"automatic_state\": \"%s\","
        "\"automatic_progress\": %d,"
        
        "\"manual_speed\": %.2f,"
        "\"manual_direction_forward\": %s,"
        "\"manual_esc_armed\": %s,"
        "\"manual_motor_active\": %s,"
        "\"manual_state\": \"%s\""
        "}",
        
        // System status
        mode_status.system_healthy ? "true" : "false",
        mode_coordinator_mode_to_string(mode_status.current_mode),
        mode_status.current_mode_status,
        mode_status.error_message,
        
        // Sensor validation
        mode_status.sensors_validated ? "true" : "false",
        mode_coordinator_validation_to_string(mode_status.sensor_validation_state),
        mode_status.sensor_validation_message,
        mode_status.hall_validation_complete ? "true" : "false",
        mode_status.accel_validation_complete ? "true" : "false",
        
        // Mode availability
        mode_coordinator_availability_to_string(mode_status.wire_learning_availability),
        mode_coordinator_availability_to_string(mode_status.automatic_availability),
        mode_coordinator_availability_to_string(mode_status.manual_availability),
        
        // Hall sensor data
        sensor_status.hall_status == SENSOR_STATUS_HEALTHY ? "healthy" :
        sensor_status.hall_status == SENSOR_STATUS_FAILED ? "failed" :
        sensor_status.hall_status == SENSOR_STATUS_TIMEOUT ? "timeout" : "testing",
        sensor_status.hall_pulse_count,
        sensor_status.current_rpm,
        sensor_status.wheel_speed_ms,
        sensor_status.wheel_rotation_detected ? "true" : "false",
        
        // Accelerometer data
        sensor_status.accel_status == SENSOR_STATUS_HEALTHY ? "healthy" :
        sensor_status.accel_status == SENSOR_STATUS_FAILED ? "failed" :
        sensor_status.accel_status == SENSOR_STATUS_TIMEOUT ? "timeout" : "testing",
        sensor_status.total_accel_g,
        sensor_status.last_impact_g,
        0.5f, // Default impact threshold
        sensor_status.trolley_shake_detected ? "true" : "false",
        
        // Hardware status
        hw_status.esc_armed ? "true" : "false",
        hw_status.current_position_m,
        hw_status.current_speed_ms,
        hw_status.target_speed_ms,
        hw_status.direction_forward ? "true" : "false",
        hw_status.total_rotations,
        
        // Wire learning status
        mode_status.wire_learning.complete ? "true" : "false",
        mode_status.wire_learning.wire_length_m,
        wire_learning_state_to_string(wire_progress.state),
        wire_learning_mode_is_active() ? wire_learning_get_progress_percentage() : 
            (mode_status.wire_learning.complete ? 100 : 0),
        
        // Automatic mode status
        mode_status.auto_cycle_count,
        mode_status.auto_cycle_interrupted ? "true" : "false",
        mode_status.auto_coasting_calibrated ? "true" : "false",
        automatic_mode_state_to_string(auto_progress.state),
        automatic_mode_is_active() ? automatic_mode_get_progress_percentage() : 0,
        
        // Manual mode status
        manual_status.current_speed_ms,
        manual_status.direction_forward ? "true" : "false",
        manual_status.esc_armed ? "true" : "false",
        manual_status.motor_active ? "true" : "false",
        manual_mode_state_to_string(manual_status.state)
    );
    
    return ESP_OK;
}

esp_err_t web_generate_command_response(bool success, const char* message, 
                                       char* json_buffer, size_t buffer_size) {
    if (json_buffer == NULL) return ESP_ERR_INVALID_ARG;
    
    const char* safe_message = message ? message : "No message";
    
    snprintf(json_buffer, buffer_size,
        "{"
        "\"success\": %s,"
        "\"message\": \"%s\","
        "\"timestamp\": %llu"
        "}",
        success ? "true" : "false",
        safe_message,
        esp_timer_get_time() / 1000);
    
    return ESP_OK;
}

// ═══════════════════════════════════════════════════════════════════════════════
// HTTP HANDLERS IMPLEMENTATION
// ═══════════════════════════════════════════════════════════════════════════════

esp_err_t web_handler_root(httpd_req_t *req) {
    g_server_stats.total_requests++;
    g_server_stats.last_request_time = esp_timer_get_time();
    
    // Get client IP
    char client_ip[16];
    httpd_req_get_hdr_value_str(req, "Host", client_ip, sizeof(client_ip));
    strncpy(g_server_stats.last_client_ip, client_ip, sizeof(g_server_stats.last_client_ip) - 1);
    
    // Generate HTML content
    char* html_buffer = malloc(WEB_HTML_BUFFER_SIZE);
    if (html_buffer == NULL) {
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Memory allocation failed");
        g_server_stats.failed_requests++;
        return ESP_ERR_NO_MEM;
    }
    
    esp_err_t result = web_generate_main_page(html_buffer, WEB_HTML_BUFFER_SIZE);
    if (result != ESP_OK) {
        free(html_buffer);
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Content generation failed");
        g_server_stats.failed_requests++;
        return result;
    }
    
    // Set headers
    httpd_resp_set_type(req, "text/html");
    httpd_resp_set_hdr(req, "Cache-Control", "no-cache");
    
    // Send response
    httpd_resp_send(req, html_buffer, strlen(html_buffer));
    free(html_buffer);
    
    g_server_stats.successful_requests++;
    return ESP_OK;
}

esp_err_t web_handler_status(httpd_req_t *req) {
    g_server_stats.total_requests++;
    g_server_stats.status_requests++;
    
    char json_buffer[WEB_JSON_BUFFER_SIZE];
    esp_err_t result = web_generate_status_json(json_buffer, sizeof(json_buffer));
    
    if (result != ESP_OK) {
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Status generation failed");
        g_server_stats.failed_requests++;
        return result;
    }
    
    // Set headers
    httpd_resp_set_type(req, "application/json");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    httpd_resp_set_hdr(req, "Cache-Control", "no-cache");
    
    // Send JSON response
    httpd_resp_send(req, json_buffer, strlen(json_buffer));
    g_server_stats.successful_requests++;
    
    return ESP_OK;
}

esp_err_t web_handler_command(httpd_req_t *req) {
    g_server_stats.total_requests++;
    
    // Get client IP
    char client_ip[16] = "unknown";
    httpd_req_get_hdr_value_str(req, "X-Forwarded-For", client_ip, sizeof(client_ip));
    
    // Read command from request body
    char command_buffer[16];
    int ret = httpd_req_recv(req, command_buffer, sizeof(command_buffer) - 1);
    
    if (ret <= 0) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Bad request");
        g_server_stats.failed_requests++;
        return ESP_FAIL;
    }
    
    command_buffer[ret] = '\0';
    
    // Process command
    char response_message[256];
    esp_err_t result = web_process_command(command_buffer[0], client_ip, 
                                          response_message, sizeof(response_message));
    
    // Generate JSON response
    char json_response[512];
    web_generate_command_response((result == ESP_OK), response_message, 
                                 json_response, sizeof(json_response));
    
    // Set headers
    httpd_resp_set_type(req, "application/json");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Methods", "POST, GET, OPTIONS");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Headers", "Content-Type");
    
    // Send response
    httpd_resp_send(req, json_response, strlen(json_response));
    
    if (result == ESP_OK) {
        g_server_stats.successful_requests++;
    } else {
        g_server_stats.failed_requests++;
    }
    
    return ESP_OK;
}

esp_err_t web_handler_options(httpd_req_t *req) {
    // CORS preflight response
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Methods", "POST, GET, OPTIONS");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Headers", "Content-Type");
    httpd_resp_send(req, NULL, 0);
    return ESP_OK;
}

esp_err_t web_handler_api_info(httpd_req_t *req) {
    char json_buffer[512];
    
    snprintf(json_buffer, sizeof(json_buffer),
        "{"
        "\"system\": \"ESP32-S3 Trolley 3-Mode System\","
        "\"hardware\": \"ESP32-S3 + Eco II 2807 + Littlebee 30A ESC\","
        "\"modes\": [\"Wire Learning\", \"Automatic\", \"Manual\"],"
        "\"version\": \"1.0.0\","
        "\"uptime_ms\": %llu,"
        "\"memory_free\": %u,"
        "\"wifi_clients\": %d"
        "}",
        web_get_uptime(),
        esp_get_free_heap_size(),
        g_server_stats.active_connections);
    
    httpd_resp_set_type(req, "application/json");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    httpd_resp_send(req, json_buffer, strlen(json_buffer));
    
    return ESP_OK;
}

esp_err_t web_handler_stats(httpd_req_t *req) {
    char json_buffer[1024];
    
    snprintf(json_buffer, sizeof(json_buffer),
        "{"
        "\"server_stats\": {"
        "\"total_requests\": %lu,"
        "\"successful_requests\": %lu,"
        "\"failed_requests\": %lu,"
        "\"commands_executed\": %lu,"
        "\"status_requests\": %lu,"
        "\"active_connections\": %lu,"
        "\"max_concurrent_connections\": %lu,"
        "\"server_start_time\": %llu,"
        "\"last_request_time\": %llu,"
        "\"last_client_ip\": \"%s\""
        "},"
        "\"system_stats\": {"
        "\"uptime_ms\": %llu,"
        "\"free_heap\": %u,"
        "\"min_free_heap\": %u,"
        "\"wifi_clients\": %d"
        "}"
        "}",
        
        // Server stats
        g_server_stats.total_requests,
        g_server_stats.successful_requests,
        g_server_stats.failed_requests,
        g_server_stats.commands_executed,
        g_server_stats.status_requests,
        g_server_stats.active_connections,
        g_server_stats.max_concurrent_connections,
        g_server_stats.server_start_time,
        g_server_stats.last_request_time,
        g_server_stats.last_client_ip,
        
        // System stats
        web_get_uptime(),
        esp_get_free_heap_size(),
        esp_get_minimum_free_heap_size(),
        g_server_stats.active_connections);
    
    httpd_resp_set_type(req, "application/json");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    httpd_resp_send(req, json_buffer, strlen(json_buffer));
    
    return ESP_OK;
}

// ═══════════════════════════════════════════════════════════════════════════════
// PUBLIC API IMPLEMENTATION
// ═══════════════════════════════════════════════════════════════════════════════

esp_err_t web_interface_init(const web_interface_config_t* config) {
    ESP_LOGI(TAG, "Initializing web interface...");
    
    // Set default configuration if none provided
    if (config != NULL) {
        memcpy(&g_web_config, config, sizeof(web_interface_config_t));
    } else {
        web_get_default_config(&g_web_config);
    }
    
    // Reset statistics
    memset(&g_server_stats, 0, sizeof(g_server_stats));
    g_server_stats.server_start_time = esp_timer_get_time();
    
    // Reset client tracking
    memset(g_client_info, 0, sizeof(g_client_info));
    g_client_count = 0;
    
    g_web_status = WEB_STATUS_STOPPED;
    g_web_initialized = true;
    
    ESP_LOGI(TAG, "Web interface initialized - Port: %d, Rate limiting: %s", 
            g_web_config.server_port, g_web_config.enable_rate_limiting ? "enabled" : "disabled");
    
    return ESP_OK;
}

esp_err_t web_interface_start(void) {
    if (!g_web_initialized) {
        return ESP_ERR_INVALID_STATE;
    }
    
    ESP_LOGI(TAG, "Starting web server...");
    g_web_status = WEB_STATUS_STARTING;
    
    // Configure HTTP server
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.server_port = g_web_config.server_port;
    config.max_open_sockets = g_web_config.max_open_sockets;
    config.stack_size = 8192;
    config.task_priority = 5;
    
    // Start server
    esp_err_t result = httpd_start(&g_server_handle, &config);
    if (result != ESP_OK) {
        ESP_LOGE(TAG, "Failed to start HTTP server: %s", esp_err_to_name(result));
        g_web_status = WEB_STATUS_ERROR;
        return result;
    }
    
    // Register URI handlers
    httpd_uri_t root_uri = {
        .uri = "/",
        .method = HTTP_GET,
        .handler = web_handler_root,
        .user_ctx = NULL
    };
    httpd_register_uri_handler(g_server_handle, &root_uri);
    
    httpd_uri_t status_uri = {
        .uri = "/status",
        .method = HTTP_GET,
        .handler = web_handler_status,
        .user_ctx = NULL
    };
    httpd_register_uri_handler(g_server_handle, &status_uri);
    
    httpd_uri_t command_uri = {
        .uri = "/command",
        .method = HTTP_POST,
        .handler = web_handler_command,
        .user_ctx = NULL
    };
    httpd_register_uri_handler(g_server_handle, &command_uri);
    
    httpd_uri_t options_uri = {
        .uri = "/*",
        .method = HTTP_OPTIONS,
        .handler = web_handler_options,
        .user_ctx = NULL
    };
    httpd_register_uri_handler(g_server_handle, &options_uri);
    
    httpd_uri_t api_info_uri = {
        .uri = "/api/info",
        .method = HTTP_GET,
        .handler = web_handler_api_info,
        .user_ctx = NULL
    };
    httpd_register_uri_handler(g_server_handle, &api_info_uri);
    
    httpd_uri_t stats_uri = {
        .uri = "/api/stats",
        .method = HTTP_GET,
        .handler = web_handler_stats,
        .user_ctx = NULL
    };
    httpd_register_uri_handler(g_server_handle, &stats_uri);
    
    g_web_status = WEB_STATUS_RUNNING;
    ESP_LOGI(TAG, "Web server started successfully on port %d", config.server_port);
    ESP_LOGI(TAG, "Available endpoints: /, /status, /command, /api/info, /api/stats");
    
    return ESP_OK;
}

esp_err_t web_interface_stop(void) {
    ESP_LOGI(TAG, "Stopping web server...");
    g_web_status = WEB_STATUS_STOPPING;
    
    if (g_server_handle != NULL) {
        esp_err_t result = httpd_stop(g_server_handle);
        g_server_handle = NULL;
        
        if (result == ESP_OK) {
            g_web_status = WEB_STATUS_STOPPED;
            ESP_LOGI(TAG, "Web server stopped successfully");
        } else {
            g_web_status = WEB_STATUS_ERROR;
            ESP_LOGE(TAG, "Failed to stop web server: %s", esp_err_to_name(result));
        }
        
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
    // Background maintenance tasks
    if (g_web_status == WEB_STATUS_RUNNING) {
        // Periodic cleanup of old client data
        static uint64_t last_cleanup = 0;
        uint64_t current_time = esp_timer_get_time();
        
        if (current_time - last_cleanup > 300000000ULL) { // 5 minutes
            for (int i = 0; i < g_client_count; i++) {
                if (current_time - g_client_info[i].last_request_time > 600000000ULL) { // 10 minutes
                    // Remove inactive client
                    memmove(&g_client_info[i], &g_client_info[i + 1], 
                           (g_client_count - i - 1) * sizeof(web_client_info_t));
                    g_client_count--;
                    i--; // Adjust index
                }
            }
            last_cleanup = current_time;
        }
    }
    
    return ESP_OK;
}

// ═══════════════════════════════════════════════════════════════════════════════
// UTILITY FUNCTIONS IMPLEMENTATION
// ═══════════════════════════════════════════════════════════════════════════════

esp_err_t web_get_default_config(web_interface_config_t* config) {
    if (config == NULL) return ESP_ERR_INVALID_ARG;
    
    config->server_port = WEB_SERVER_PORT;
    config->max_open_sockets = WEB_MAX_OPEN_SOCKETS;
    config->enable_cors = true;
    config->enable_rate_limiting = true;
    config->enable_command_logging = true;
    config->enable_real_time_updates = false;
    strcpy(config->server_name, "ESP32S3_TROLLEY_3MODE");
    
    return ESP_OK;
}

const char* web_interface_status_to_string(web_interface_status_t status) {
    switch (status) {
        case WEB_STATUS_STOPPED: return "Stopped";
        case WEB_STATUS_STARTING: return "Starting";
        case WEB_STATUS_RUNNING: return "Running";
        case WEB_STATUS_ERROR: return "Error";
        case WEB_STATUS_STOPPING: return "Stopping";
        default: return "Unknown";
    }
}

uint64_t web_get_uptime(void) {
    return (esp_timer_get_time() - g_server_stats.server_start_time) / 1000;
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

esp_err_t web_reset_statistics(void) {
    uint64_t server_start_time = g_server_stats.server_start_time;
    memset(&g_server_stats, 0, sizeof(g_server_stats));
    g_server_stats.server_start_time = server_start_time;
    ESP_LOGI(TAG, "Web server statistics reset");
    return ESP_OK;
}

esp_err_t web_restart_server(void) {
    ESP_LOGI(TAG, "Restarting web server...");
    
    esp_err_t result = web_interface_stop();
    if (result != ESP_OK) {
        return result;
    }
    
    vTaskDelay(pdMS_TO_TICKS(1000)); // Wait 1 second
    
    return web_interface_start();
}// components/web_interface/src/web_interface.cpp
// ═══════════════════════════════════════════════════════════════════════════════
// WEB_INTERFACE.CPP - WEB UI AND HTTP HANDLING IMPLEMENTATION
// ═══════════════════════════════════════════════════════════════════════════════
// 
// SINGLE RESPONSIBILITY: Web UI and HTTP handling
// - Complete 3-mode web interface with sensor validation
// - Real-time status updates and command processing
// - WiFi AP management and security
// - Rate limiting and error handling
// ═══════════════════════════════════════════════════════════════════════════════

#include "web_interface.h"
#include "mode_coordinator.h"
#include "hardware_control.h"
#include "sensor_health.h"
#include "wire_learning_mode.h"
#include "automatic_mode.h"
#include "manual_mode.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "esp_http_server.h"
#include <cstring>
#include <cmath>

static const char* TAG = "WEB_INTERFACE";

// ═══════════════════════════════════════════════════════════════════════════════
// GLOBAL WEB INTERFACE STATE
// ═══════════════════════════════════════════════════════════════════════════════

static web_interface_config_t g_web_config = {0};
static web_server_stats_t g_server_stats = {0};
static web_interface_status_t g_web_status = WEB_STATUS_STOPPED;
static httpd_handle_t g_server_handle = NULL;
static bool g_web_initialized = false;

// Rate limiting and security
static web_client_info_t g_client_info[10] = {0}; // Track up to 10 clients
static int g_client_count = 0;

// ═══════════════════════════════════════════════════════════════════════════════
// WIFI ACCESS POINT MANAGEMENT
// ═══════════════════════════════════════════════════════════════════════════════

static void wifi_event_handler(void* arg, esp_event_base_t event_base,
                              int32_t event_id, void* event_data) {
    if (event_base == WIFI_EVENT) {
        switch (event_id) {
            case WIFI_EVENT_AP_START:
                ESP_LOGI(TAG, "WiFi AP started successfully");
                break;
            case WIFI_EVENT_AP_STOP:
                ESP_LOGW(TAG, "WiFi AP stopped");
                break;
            case WIFI_EVENT_AP_STACONNECTED:
                ESP_LOGI(TAG, "Station connected to AP");
                g_server_stats.active_connections++;
                if (g_server_stats.active_connections > g_server_stats.max_concurrent_connections) {
                    g_server_stats.max_concurrent_connections = g_server_stats.active_connections;
                }
                break;
            case WIFI_EVENT_AP_STADISCONNECTED:
                ESP_LOGI(TAG, "Station disconnected from AP");
                if (g_server_stats.active_connections > 0) {
                    g_server_stats.active_connections--;
                }
                break;
            default:
                ESP_LOGD(TAG, "WiFi Event: %ld", event_id);
                break;
        }
    }
}

esp_err_t web_wifi_init_ap(const char* ssid, const char* password) {
    ESP_LOGI(TAG, "Initializing WiFi Access Point...");
    
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    
    // Register event handler
    ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_event_handler, NULL));
    
    esp_netif_create_default_wifi_ap();
    
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    
    // Configure WiFi AP
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
    
    ESP_LOGI(TAG, "WiFi AP initialized: SSID='%s', Channel=%d", ssid, wifi_config.ap.channel);
    ESP_LOGI(TAG, "Connect to: http://192.168.4.1");
    
    return ESP_OK;
}

esp_err_t web_wifi_get_info(char* info_buffer, size_t buffer_size) {
    if (info_buffer == NULL) return ESP_ERR_INVALID_ARG;
    
    wifi_ap_record_t ap_info;
    esp_err_t result = esp_wifi_sta_get_ap_info(&ap_info);
    
    if (result == ESP_OK) {
        snprintf(info_buffer, buffer_size,
                "WiFi AP: %s\nChannel: %d\nConnected Clients: %d\nIP: 192.168.4.1",
                (char*)ap_info.ssid, ap_info.primary, g_server_stats.active_connections);
    } else {
        snprintf(info_buffer, buffer_size,
                "WiFi AP: ESP32S3_TROLLEY_3MODE\nIP: 192.168.4.1\nConnected Clients: %d",
                g_server_stats.active_connections);
    }
    
    return ESP_OK;
}

bool web_wifi_is_ap_running(void) {
    wifi_mode_t mode;
    esp_err_t result = esp_wifi_get_mode(&mode);
    return (result == ESP_OK && mode == WIFI_MODE_AP);
}

uint8_t web_wifi_get_client_count(void) {
    return g_server_stats.active_connections;
}

// ═══════════════════════════════════════════════════════════════════════════════
// RATE LIMITING AND SECURITY
// ═══════════════════════════════════════════════════════════════════════════════

static int find_client_index(const char* client_ip) {
    for (int i = 0; i < g_client_count; i++) {
        if (strcmp(g_client_info[i].ip_address, client_ip) == 0) {
            return i;
        }
    }
    return -1;
}

bool web_is_client_rate_limited(const char* client_ip) {
    if (!g_web_config.enable_rate_limiting) {
        return false;
    }
    
    int index = find_client_index(client_ip);
    if (index < 0) {
        return false; // New client, not rate limited
    }
    
    web_client_info_t* client = &g_client_info[index];
    uint64_t current_time = esp_timer_get_time();
    
    // Reset counter every minute
    if (current_time - client->last_request_time > 60000000ULL) {
        client->requests_sent = 0;
        client->rate_limited = false;
    }
    
    return client->rate_limited;
}

esp_err_t web_update_rate_limiting(const char* client_ip) {
    if (!g_web_config.enable_rate_limiting) {
        return ESP_OK;
    }
    
    int index = find_client_index(client_ip);
    if (index < 0) {
        // Add new client
        if (g_client_count < 10) {
            index = g_client_count++;
            g_client_info[index].client_id = g_client_count;
            strncpy(g_client_info[index].ip_address, client_ip, sizeof(g_client_info[index].ip_address) - 1);
            g_client_info[index].connect_time = esp_timer_get_time();
            g_client_info[index].requests_sent = 0;
            g_client_info[
			
// ═══════════════════════════════════════════════════════════════════════════════
// MISSING PORTION - ADD TO END OF web_interface.cpp
// ═══════════════════════════════════════════════════════════════════════════════

esp_err_t web_set_config(const web_interface_config_t* config) {
    if (config == NULL) return ESP_ERR_INVALID_ARG;
    
    if (web_interface_is_running()) {
        ESP_LOGW(TAG, "Cannot change config while server is running");
        return ESP_ERR_INVALID_STATE;
    }
    
    memcpy(&g_web_config, config, sizeof(web_interface_config_t));
    ESP_LOGI(TAG, "Web interface configuration updated");
    
    return ESP_OK;
}

esp_err_t web_get_config(web_interface_config_t* config) {
    if (config == NULL) return ESP_ERR_INVALID_ARG;
    
    memcpy(config, &g_web_config, sizeof(web_interface_config_t));
    return ESP_OK;
}

esp_err_t web_block_client(const char* client_ip, uint32_t duration_ms) {
    if (client_ip == NULL) return ESP_ERR_INVALID_ARG;
    
    int index = find_client_index(client_ip);
    if (index >= 0) {
        g_client_info[index].rate_limited = true;
        ESP_LOGW(TAG, "Client %s blocked for %lu ms", client_ip, duration_ms);
    }
    
    return ESP_OK;
}

esp_err_t web_get_command_help(char* help_buffer, size_t buffer_size) {
    if (help_buffer == NULL) return ESP_ERR_INVALID_ARG;
    
    snprintf(help_buffer, buffer_size,
        "3-MODE TROLLEY WEB COMMANDS:\n"
        "Mode Control:\n"
        "  W = Activate Wire Learning Mode\n"
        "  U = Activate Automatic Mode (5 m/s cycling)\n"
        "  M = Activate Manual Mode\n"
        "  Q = Stop Current Mode\n"
        "  I = Interrupt Current Mode\n"
        "\n"
        "Sensor Validation:\n"
        "  V = Start Sensor Validation\n"
        "  H = Confirm Hall Sensor (after wheel rotation)\n"
        "  C = Confirm Accelerometer (after trolley shake)\n"
        "\n"
        "Manual Mode (when active):\n"
        "  A = Arm ESC\n"
        "  D = Disarm ESC\n"
        "  F = Forward Movement\n"
        "  B = Backward Movement\n"
        "  S = Stop Movement\n"
        "  + = Increase Speed\n"
        "  - = Decrease Speed\n"
        "\n"
        "System:\n"
        "  T = Status\n"
        "  E = Emergency Stop\n"
        "  R = Reset System");
    
    return ESP_OK;
}

esp_err_t web_generate_stats_json(char* json_buffer, size_t buffer_size) {
    if (json_buffer == NULL) return ESP_ERR_INVALID_ARG;
    
    web_server_stats_t stats = web_interface_get_stats();
    
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
        "\"free_heap\": %u,"
        "\"min_free_heap\": %u,"
        "\"wifi_clients\": %d"
        "},"
        "\"rate_limiting\": {"
        "\"tracked_clients\": %d,"
        "\"rate_limiting_enabled\": %s"
        "}"
        "}",
        
        stats.total_requests,
        stats.successful_requests,
        stats.failed_requests,
        stats.commands_executed,
        stats.status_requests,
        stats.active_connections,
        stats.max_concurrent_connections,
        web_get_uptime(),
        esp_get_free_heap_size(),
        esp_get_minimum_free_heap_size(),
        web_wifi_get_client_count(),
        g_client_count,
        g_web_config.enable_rate_limiting ? "true" : "false");
    
    return ESP_OK;
}

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
        "\"/status\","
        "\"/command\","
        "\"/api/info\","
        "\"/api/stats\""
        "]"
        "}",
        __DATE__, __TIME__);
    
    return ESP_OK;
}

esp_err_t web_log_error(const char* error_message, const char* client_ip) {
    ESP_LOGE(TAG, "Web Error from %s: %s", client_ip ? client_ip : "unknown", error_message);
    return ESP_OK;
}

esp_err_t web_get_error_log(char* log_buffer, size_t buffer_size) {
    if (log_buffer == NULL) return ESP_ERR_INVALID_ARG;
    
    snprintf(log_buffer, buffer_size, "Error logging not implemented in this version");
    return ESP_OK;
}

esp_err_t web_clear_error_log(void) {
    ESP_LOGI(TAG, "Error log cleared");
    return ESP_OK;
}

esp_err_t web_get_debug_info(char* debug_buffer, size_t buffer_size) {
    if (debug_buffer == NULL) return ESP_ERR_INVALID_ARG;
    
    snprintf(debug_buffer, buffer_size,
        "Web Interface Debug Info:\n"
        "Status: %s\n"
        "Server Handle: %p\n"
        "Config Port: %d\n"
        "Max Sockets: %d\n"
        "Rate Limiting: %s\n"
        "CORS: %s\n"
        "Command Logging: %s\n"
        "Tracked Clients: %d\n"
        "Free Heap: %u bytes\n"
        "Uptime: %llu ms",
        web_interface_status_to_string(g_web_status),
        g_server_handle,
        g_web_config.server_port,
        g_web_config.max_open_sockets,
        g_web_config.enable_rate_limiting ? "enabled" : "disabled",
        g_web_config.enable_cors ? "enabled" : "disabled",
        g_web_config.enable_command_logging ? "enabled" : "disabled",
        g_client_count,
        esp_get_free_heap_size(),
        web_get_uptime());
    
    return ESP_OK;
}

esp_err_t web_generate_error_page(int error_code, const char* error_message,
                                 char* html_buffer, size_t buffer_size) {
    if (html_buffer == NULL) return ESP_ERR_INVALID_ARG;
    
    const char* error_template = 
        "<!DOCTYPE html>"
        "<html><head>"
        "<title>Error %d - ESP32-S3 Trolley</title>"
        "<style>"
        "body { font-family: Arial; margin: 50px; text-align: center; }"
        "h1 { color: #dc3545; }"
        ".error-box { background: #f8d7da; padding: 20px; border-radius: 8px; margin: 20px 0; }"
        "a { color: #007bff; }"
        "</style></head>"
        "<body>"
        "<h1>Error %d</h1>"
        "<div class='error-box'>"
        "<p>%s</p>"
        "</div>"
        "<p><a href='/'>Return to Main Page</a></p>"
        "</body></html>";
    
    snprintf(html_buffer, buffer_size, error_template, 
            error_code, error_code, error_message ? error_message : "Unknown error");
    
    return ESP_OK;
}

// Real-time updates (placeholder for future WebSocket implementation)
esp_err_t web_enable_real_time_updates(bool enable) {
    g_web_config.enable_real_time_updates = enable;
    ESP_LOGI(TAG, "Real-time updates %s", enable ? "enabled" : "disabled");
    return ESP_OK;
}

esp_err_t web_send_real_time_update(const char* update_type, const char* data) {
    // Placeholder for WebSocket implementation
    ESP_LOGD(TAG, "Real-time update: %s - %s", update_type, data);
    return ESP_OK;
}

esp_err_t web_register_for_updates(const char* client_ip) {
    ESP_LOGD(TAG, "Client %s registered for updates", client_ip);
    return ESP_OK;
}

esp_err_t web_unregister_from_updates(const char* client_ip) {
    ESP_LOGD(TAG, "Client %s unregistered from updates", client_ip);
    return ESP_OK;
}

// Theme and customization (placeholder)
esp_err_t web_set_theme(const char* theme_name) {
    ESP_LOGI(TAG, "Theme set to: %s", theme_name);
    return ESP_OK;
}

esp_err_t web_get_available_themes(char* themes_buffer, size_t buffer_size) {
    if (themes_buffer == NULL) return ESP_ERR_INVALID_ARG;
    
    snprintf(themes_buffer, buffer_size, "default,dark,minimal");
    return ESP_OK;
}

esp_err_t web_set_page_branding(const char* title, const char* subtitle) {
    ESP_LOGI(TAG, "Page branding: %s - %s", title, subtitle);
    return ESP_OK;
}