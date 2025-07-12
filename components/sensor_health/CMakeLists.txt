# components/sensor_health/CMakeLists.txt

idf_component_register(
    SRCS "src/sensor_health.cpp"
    INCLUDE_DIRS "include"
    REQUIRES 
        MPUdriver 
        I2Cbus 
        freertos 
        esp_timer 
        nvs_flash
        esp_hw_support
    PRIV_REQUIRES 
        log
)