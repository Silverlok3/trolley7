# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
# FILE: components/hardware_control/CMakeLists.txt
# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

idf_component_register(
    SRCS "src/hardware_control.cpp"
    INCLUDE_DIRS "include"
    REQUIRES 
        driver 
        freertos 
        esp_timer 
        esp_hw_support
    PRIV_REQUIRES 
        log
)