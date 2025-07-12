# main/CMakeLists.txt - ESP32-S3 Trolley 3-Mode System
# ═══════════════════════════════════════════════════════════════════════════════
# MAIN APPLICATION BUILD CONFIGURATION
# ═══════════════════════════════════════════════════════════════════════════════

idf_component_register(
    SRCS "main.cpp"
    INCLUDE_DIRS "."
    REQUIRES 
        # ═══════════════════════════════════════════════════════════════════════
        # CUSTOM PROJECT COMPONENTS (New Modular Architecture)
        # ═══════════════════════════════════════════════════════════════════════
        hardware_control        # Low-level hardware abstraction (ESC, Hall, GPIO)
        mode_coordinator        # 3-mode system management and coordination
        wire_learning_mode      # Mode 1: Wire learning implementation
        automatic_mode          # Mode 2: Autonomous cycling implementation  
        manual_mode             # Mode 3: Manual control implementation
        web_interface           # Web UI and HTTP server (NEW: split into 4 files)
        sensor_health           # Sensor validation and health monitoring
        

        # ═══════════════════════════════════════════════════════════════════════
        # EXTERNAL HARDWARE DRIVERS
        # ═══════════════════════════════════════════════════════════════════════
        MPUdriver               # MPU6050 accelerometer/gyroscope driver
        I2Cbus                  # I2C bus driver (required by MPUdriver)
        
        # ═══════════════════════════════════════════════════════════════════════
        # ESP-IDF CORE COMPONENTS
        # ═══════════════════════════════════════════════════════════════════════
        esp_timer               # High-resolution timer
        driver                  # GPIO, I2C, LEDC drivers
        nvs_flash               # Non-volatile storage (WiFi, settings)
        freertos                # Real-time operating system
        esp_hw_support          # Hardware support functions
        
        # ═══════════════════════════════════════════════════════════════════════
        # WIFI AND NETWORKING COMPONENTS
        # ═══════════════════════════════════════════════════════════════════════
        esp_wifi                # WiFi driver and stack
        esp_event               # Event handling system
        esp_netif               # Network interface abstraction
        esp_http_server         # HTTP server for web interface
        
        # ═══════════════════════════════════════════════════════════════════════
        # USB AND COMMUNICATION (ESP32-S3 Specific)
        # ═══════════════════════════════════════════════════════════════════════
        usb                     # USB support (ESP32-S3 has native USB)
        
        # ═══════════════════════════════════════════════════════════════════════
        # OPTIONAL ADDITIONAL COMPONENTS (uncomment if needed)
        # ═══════════════════════════════════════════════════════════════════════
        # console               # Console command support
        # app_update            # OTA update support
        # spiffs                # SPI Flash File System
        # fatfs                 # FAT file system
        # json                  # JSON parsing (if complex JSON needed)
        # esp_https_server      # HTTPS server (if security needed)
        # mdns                  # mDNS service discovery
        # esp_websocket_client  # WebSocket client
        # protocomm             # Protocol communication
)