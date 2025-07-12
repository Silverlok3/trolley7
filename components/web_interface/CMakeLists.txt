# components/web_interface/CMakeLists.txt
# ═══════════════════════════════════════════════════════════════════════════════
# WEB INTERFACE COMPONENT BUILD CONFIGURATION
# ═══════════════════════════════════════════════════════════════════════════════

idf_component_register(
    SRCS 
        "src/web_interface_main.cpp"
        "src/web_status_handler.cpp"
        "src/web_command_handler.cpp"
        "src/web_utils.cpp"
    INCLUDE_DIRS 
        "include"
    REQUIRES 
        mode_coordinator
        hardware_control
        sensor_health
        wire_learning_mode
        automatic_mode
        manual_mode
        esp_http_server
        esp_wifi
        esp_event
        esp_netif
        esp_timer
        freertos
        nvs_flash
        esp_hw_support
    PRIV_REQUIRES 
        log
)