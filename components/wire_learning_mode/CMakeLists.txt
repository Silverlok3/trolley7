# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
# FILE: components/wire_learning_mode/CMakeLists.txt  
# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

idf_component_register(
    SRCS "src/wire_learning_mode.cpp"
    INCLUDE_DIRS "include"
    REQUIRES 
        hardware_control
        sensor_health
        freertos 
        esp_timer 
        nvs_flash
        esp_hw_support
    PRIV_REQUIRES 
        log
)