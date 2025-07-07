# CMakeLists.txt of ESP32ENCODER component

# Src files
file(GLOB_RECURSE MOTOR_SRCS
    src/*.c
    src/*.cpp
)
# Include
set(MOTOR_INCS
    "include/")

# Public component requirement
set(MOTOR_REQUIRES
    driver
    esp_timer
    ESP32Encoder
    esp32_BNO08x
)

# Private component requirement
set(MOTOR_PRIV_REQUIRES
)

# Register component
idf_component_register(
    SRCS ${MOTOR_SRCS}
    INCLUDE_DIRS ${MOTOR_INCS}
    REQUIRES ${MOTOR_REQUIRES}
    PRIV_REQUIRES ${MOTOR_PRIV_REQUIRES}
)
