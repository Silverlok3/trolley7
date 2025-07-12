# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
# FILE: components/manual_mode/CMakeLists.txt  
# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

idf_component_register(
    SRCS "src/manual_mode.cpp"
    INCLUDE_DIRS "include"
    REQUIRES 
        hardware_control
        sensor_health
        freertos 
        esp_timer 
        nvs_flash
        esp_hw_support
		mode_coordinator
    PRIV_REQUIRES 
        log
)