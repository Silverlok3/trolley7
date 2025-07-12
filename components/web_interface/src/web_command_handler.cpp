// components/web_interface/src/web_command_handler.cpp - FIXED VERSION
// ═══════════════════════════════════════════════════════════════════════════════
// WEB_COMMAND_HANDLER.CPP - COMMAND PROCESSING AND ROUTING
// ═══════════════════════════════════════════════════════════════════════════════

#include "web_interface.h"
#include "mode_coordinator.h"
#include "wire_learning_mode.h"
#include "automatic_mode.h"
#include "manual_mode.h"
#include "esp_log.h"
#include <cstring>
#include <cctype>

static const char* TAG = "WEB_COMMAND";

// FIXED: Add missing constant definitions
#define MANUAL_MODE_MAX_SPEED_MS        2.0f      // Maximum allowed manual speed
#define MANUAL_MODE_DEFAULT_SPEED_MS    0.5f      // Default speed for forward/backward

// ═══════════════════════════════════════════════════════════════════════════════
// COMMAND VALIDATION
// ═══════════════════════════════════════════════════════════════════════════════

bool web_validate_command(char command_char, const char* client_ip) {
    // Convert to uppercase for consistency
    command_char = toupper(command_char);
    
    // Check if command is in valid set
    const char* valid_commands = "WUMHCVADFSB+-QIETRLK";
    if (strchr(valid_commands, command_char) == NULL) {
        ESP_LOGW(TAG, "Invalid command character: '%c' from %s", command_char, client_ip);
        return false;
    }
    
    // Additional validation could be added here:
    // - Rate limiting per client
    // - Command history analysis
    // - Security checks
    
    return true;
}

// ═══════════════════════════════════════════════════════════════════════════════
// COMMAND PROCESSING
// ═══════════════════════════════════════════════════════════════════════════════

esp_err_t web_process_command(char command_char, const char* client_ip,
                             char* response_buffer, size_t buffer_size) {
    if (response_buffer == NULL) return ESP_ERR_INVALID_ARG;
    
    // Validate command
    if (!web_validate_command(command_char, client_ip)) {
        snprintf(response_buffer, buffer_size, "Invalid command '%c'", command_char);
        return ESP_ERR_INVALID_ARG;
    }
    
    esp_err_t result = ESP_OK;
    command_char = toupper(command_char);
    
    ESP_LOGI(TAG, "Processing command '%c' from %s", command_char, client_ip ? client_ip : "unknown");
    
    switch (command_char) {
        // ═══════════════════════════════════════════════════════════════════════
        // MODE ACTIVATION COMMANDS
        // ═══════════════════════════════════════════════════════════════════════
        
        case 'W': // Wire Learning Mode
            result = mode_coordinator_activate_wire_learning();
            if (result == ESP_OK) {
                snprintf(response_buffer, buffer_size, 
                        "🔍 Wire learning mode activated - Finding wire length with gradual speed progression (0.1→1.0 m/s)");
            } else {
                snprintf(response_buffer, buffer_size, 
                        "❌ Failed to activate wire learning: %s", 
                        mode_coordinator_get_error_message());
            }
            break;
            
        case 'U': // Automatic Mode
            result = mode_coordinator_activate_automatic();
            if (result == ESP_OK) {
                snprintf(response_buffer, buffer_size, 
                        "🚀 Automatic mode activated - 5 m/s cycling with coasting calibration started");
            } else {
                snprintf(response_buffer, buffer_size, 
                        "❌ Failed to activate automatic mode: %s", 
                        mode_coordinator_get_error_message());
            }
            break;
            
        case 'M': // Manual Mode
            result = mode_coordinator_activate_manual();
            if (result == ESP_OK) {
                snprintf(response_buffer, buffer_size, 
                        "🎮 Manual mode activated - Use ARM ESC button to enable motor control");
            } else {
                snprintf(response_buffer, buffer_size, 
                        "❌ Failed to activate manual mode: %s", 
                        mode_coordinator_get_error_message());
            }
            break;
            
        // ═══════════════════════════════════════════════════════════════════════
        // SENSOR VALIDATION COMMANDS
        // ═══════════════════════════════════════════════════════════════════════
        
        case 'V': // Start sensor validation
            result = mode_coordinator_start_sensor_validation();
            if (result == ESP_OK) {
                snprintf(response_buffer, buffer_size, 
                        "📋 Sensor validation started - Step 1: ROTATE THE WHEEL manually to test Hall sensor");
            } else {
                snprintf(response_buffer, buffer_size, 
                        "❌ Failed to start sensor validation");
            }
            break;
            
        case 'H': // Confirm Hall sensor
            result = mode_coordinator_confirm_hall_validation();
            if (result == ESP_OK) {
                snprintf(response_buffer, buffer_size, 
                        "✅ Hall sensor validation confirmed - Step 2: SHAKE THE TROLLEY to test accelerometer");
            } else {
                snprintf(response_buffer, buffer_size, 
                        "⚠️ Hall sensor validation not ready - ensure wheel rotation is detected first");
            }
            break;
            
        case 'C': // Confirm Accelerometer
            result = mode_coordinator_confirm_accel_validation();
            if (result == ESP_OK) {
                snprintf(response_buffer, buffer_size, 
                        "✅ Accelerometer validation confirmed - All sensors validated! Modes now available");
            } else {
                snprintf(response_buffer, buffer_size, 
                        "⚠️ Accelerometer validation not ready - ensure trolley shake is detected first");
            }
            break;
            
        // ═══════════════════════════════════════════════════════════════════════
        // MANUAL MODE COMMANDS (only when manual mode is active)
        // ═══════════════════════════════════════════════════════════════════════
        
        case 'A': // Arm ESC
            if (manual_mode_is_active()) {
                result = manual_mode_arm_esc();
                if (result == ESP_OK) {
                    snprintf(response_buffer, buffer_size, 
                            "⚡ ESC armed successfully - Motor control enabled, ready for movement commands");
                } else {
                    snprintf(response_buffer, buffer_size, 
                            "❌ Failed to arm ESC - Check hardware connections");
                }
            } else {
                snprintf(response_buffer, buffer_size, 
                        "⚠️ Manual mode not active - Activate manual mode first");
                result = ESP_ERR_INVALID_STATE;
            }
            break;
            
        case 'D': // Disarm ESC
            if (manual_mode_is_active()) {
                result = manual_mode_disarm_esc();
                snprintf(response_buffer, buffer_size, 
                        "🛑 ESC disarmed - Motor control disabled, system safe");
            } else {
                snprintf(response_buffer, buffer_size, 
                        "⚠️ Manual mode not active");
                result = ESP_ERR_INVALID_STATE;
            }
            break;
            
        case 'F': // Forward
            if (manual_mode_is_active()) {
                result = manual_mode_move_forward();
                if (result == ESP_OK) {
                    snprintf(response_buffer, buffer_size, 
                            "➡️ Moving forward at %.1f m/s - Use +/- to adjust speed", 
                            manual_mode_get_current_speed());
                } else {
                    snprintf(response_buffer, buffer_size, 
                            "❌ Failed to move forward - Ensure ESC is armed first");
                }
            } else {
                snprintf(response_buffer, buffer_size, 
                        "⚠️ Manual mode not active");
                result = ESP_ERR_INVALID_STATE;
            }
            break;
            
        case 'B': // Backward
            if (manual_mode_is_active()) {
                result = manual_mode_move_backward();
                if (result == ESP_OK) {
                    snprintf(response_buffer, buffer_size, 
                            "⬅️ Moving backward at %.1f m/s - Use +/- to adjust speed", 
                            manual_mode_get_current_speed());
                } else {
                    snprintf(response_buffer, buffer_size, 
                            "❌ Failed to move backward - Ensure ESC is armed first");
                }
            } else {
                snprintf(response_buffer, buffer_size, 
                        "⚠️ Manual mode not active");
                result = ESP_ERR_INVALID_STATE;
            }
            break;
            
        case 'S': // Stop movement
            if (manual_mode_is_active()) {
                result = manual_mode_stop_movement();
                snprintf(response_buffer, buffer_size, 
                        "⏹️ Motor stopped - ESC remains armed for further commands");
            } else {
                snprintf(response_buffer, buffer_size, 
                        "⚠️ Manual mode not active");
                result = ESP_ERR_INVALID_STATE;
            }
            break;
            
        case '+': // Increase speed
            if (manual_mode_is_active()) {
                result = manual_mode_increase_speed();
                if (result == ESP_OK) {
                    snprintf(response_buffer, buffer_size, 
                            "⬆️ Speed increased to %.1f m/s", 
                            manual_mode_get_current_speed());
                } else {
                    snprintf(response_buffer, buffer_size, 
                            "⚠️ Cannot increase speed - At maximum (%.1f m/s) or ESC not armed", 
                            MANUAL_MODE_MAX_SPEED_MS);
                }
            } else {
                snprintf(response_buffer, buffer_size, 
                        "⚠️ Manual mode not active");
                result = ESP_ERR_INVALID_STATE;
            }
            break;
            
        case '-': // Decrease speed
            if (manual_mode_is_active()) {
                result = manual_mode_decrease_speed();
                if (result == ESP_OK) {
                    snprintf(response_buffer, buffer_size, 
                            "⬇️ Speed decreased to %.1f m/s", 
                            manual_mode_get_current_speed());
                } else {
                    snprintf(response_buffer, buffer_size, 
                            "⚠️ Cannot decrease speed further");
                }
            } else {
                snprintf(response_buffer, buffer_size, 
                        "⚠️ Manual mode not active");
                result = ESP_ERR_INVALID_STATE;
            }
            break;
            
        // ═══════════════════════════════════════════════════════════════════════
        // MODE CONTROL COMMANDS
        // ═══════════════════════════════════════════════════════════════════════
        
        case 'Q': // Stop current mode gracefully
            result = mode_coordinator_stop_current_mode(false);
            snprintf(response_buffer, buffer_size, 
                    "⏹️ Current mode stopping gracefully - Will complete current operation safely");
            break;
            
        case 'I': // Interrupt current mode
            if (automatic_mode_is_active()) {
                result = automatic_mode_interrupt();
                snprintf(response_buffer, buffer_size, 
                        "⚠️ Automatic mode interrupted - Will stop at next wire end");
            } else {
                result = mode_coordinator_stop_current_mode(true);
                snprintf(response_buffer, buffer_size, 
                        "⏹️ Current mode stopped immediately");
            }
            break;
            
        // ═══════════════════════════════════════════════════════════════════════
        // EMERGENCY AND SYSTEM COMMANDS
        // ═══════════════════════════════════════════════════════════════════════
        
        case 'E': // Emergency stop
            result = mode_coordinator_emergency_stop();
            snprintf(response_buffer, buffer_size, 
                    "🚨 EMERGENCY STOP ACTIVATED - All modes stopped, motor halted immediately");
            break;
            
        case 'R': // Reset system
            result = mode_coordinator_reset_system();
            snprintf(response_buffer, buffer_size, 
                    "🔄 System reset complete - Sensor validation required before operation");
            break;
            
        case 'T': // Status
            {
                system_mode_status_t status = mode_coordinator_get_status();
                hardware_status_t hw_status = hardware_get_status();
                
                snprintf(response_buffer, buffer_size, 
                        "📊 Status: Mode=%s, Sensors=%s, ESC=%s, Speed=%.1f m/s, Position=%.1f m", 
                        mode_coordinator_mode_to_string(status.current_mode),
                        status.sensors_validated ? "✅ Validated" : "❌ Not Validated",
                        hw_status.esc_armed ? "✅ Armed" : "❌ Disarmed",
                        hw_status.current_speed_ms,
                        hw_status.current_position_m);
            }
            break;
            
        // ═══════════════════════════════════════════════════════════════════════
        // HELP AND UNKNOWN COMMANDS
        // ═══════════════════════════════════════════════════════════════════════
        
        default:
            snprintf(response_buffer, buffer_size, 
                    "❓ Unknown command '%c' - Valid commands: V(validation), W(wire learning), U(automatic), M(manual), A(arm), F(forward), B(backward), S(stop), E(emergency)", 
                    command_char);
            result = ESP_ERR_INVALID_ARG;
            break;
    }
    
    // Log command execution
    ESP_LOGI(TAG, "Command '%c' from %s: %s (result: %s)", 
            command_char, 
            client_ip ? client_ip : "unknown",
            response_buffer,
            result == ESP_OK ? "SUCCESS" : "FAILED");
    
    return result;
}

// ═══════════════════════════════════════════════════════════════════════════════
// COMMAND HELP AND DOCUMENTATION
// ═══════════════════════════════════════════════════════════════════════════════

esp_err_t web_get_command_help(char* help_buffer, size_t buffer_size) {
    if (help_buffer == NULL) return ESP_ERR_INVALID_ARG;
    
    snprintf(help_buffer, buffer_size,
        "ESP32-S3 TROLLEY - 3-MODE SYSTEM COMMANDS\n"
        "\n"
        "🛡️ SENSOR VALIDATION (Required First):\n"
        "  V = Start Sensor Validation Process\n"
        "  H = Confirm Hall Sensor (after wheel rotation detected)\n"
        "  C = Confirm Accelerometer (after trolley shake detected)\n"
        "\n"
        "🔄 MODE CONTROL:\n"
        "  W = Activate Wire Learning Mode (0.1→1.0 m/s gradual)\n"
        "  U = Activate Automatic Mode (5 m/s cycling with coasting)\n"
        "  M = Activate Manual Mode (full user control)\n"
        "  Q = Stop Current Mode (graceful)\n"
        "  I = Interrupt Current Mode (immediate)\n"
        "\n"
        "🎮 MANUAL MODE COMMANDS (when manual mode active):\n"
        "  A = ARM ESC (required before movement)\n"
        "  D = DISARM ESC (safe state)\n"
        "  F = Move Forward (default speed)\n"
        "  B = Move Backward (default speed)\n"
        "  S = Stop Movement\n"
        "  + = Increase Speed (+0.1 m/s)\n"
        "  - = Decrease Speed (-0.1 m/s)\n"
        "\n"
        "🚨 EMERGENCY & SYSTEM:\n"
        "  E = Emergency Stop (immediate halt)\n"
        "  R = Reset System (clear all data)\n"
        "  T = System Status (current state)\n"
        "\n"
        "📝 USAGE NOTES:\n"
        "• Sensor validation must be completed before any mode operation\n"
        "• Wire learning must be completed before automatic mode\n"
        "• Manual mode requires ESC arming before movement\n"
        "• Emergency stop is available in all modes\n"
        "• Maximum speeds: 1.0 m/s (learning), 5.0 m/s (automatic), 2.0 m/s (manual)\n"
        "\n"
        "🌐 Full interface available at: http://192.168.4.1"
    );
    
    return ESP_OK;
}

esp_err_t web_get_available_commands(char* commands_buffer, size_t buffer_size) {
    if (commands_buffer == NULL) return ESP_ERR_INVALID_ARG;
    
    system_mode_status_t status = mode_coordinator_get_status();
    
    if (!status.sensors_validated) {
        snprintf(commands_buffer, buffer_size, "V,H,C,T,E,R");
    } else if (status.current_mode == TROLLEY_MODE_NONE) {
        snprintf(commands_buffer, buffer_size, "W,U,M,T,E,R");
    } else if (status.current_mode == TROLLEY_MODE_MANUAL) {
        snprintf(commands_buffer, buffer_size, "A,D,F,B,S,+,-,Q,I,T,E,R");
    } else {
        snprintf(commands_buffer, buffer_size, "Q,I,T,E,R");
    }
    
    return ESP_OK;
}