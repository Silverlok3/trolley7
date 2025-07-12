# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
# FILE: components/automatic_mode/CMakeLists.txt - FIXED
# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

idf_component_register(
    SRCS "src/automatic_mode.cpp"
    INCLUDE_DIRS "include"
    REQUIRES 
        hardware_control
        sensor_health
        mode_coordinator        # FIXED: Add this dependency
        freertos 
        esp_timer 
        nvs_flash
        esp_hw_support
    PRIV_REQUIRES 
        log
)