# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
# FILE: components/mode_coordinator/CMakeLists.txt
# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

idf_component_register(
    SRCS "src/mode_coordinator.cpp"
    INCLUDE_DIRS "include"
    REQUIRES 
        hardware_control
        sensor_health
        wire_learning_mode
        automatic_mode
        manual_mode
        freertos 
        esp_timer 
        nvs_flash
        esp_hw_support
    PRIV_REQUIRES 
        log
)