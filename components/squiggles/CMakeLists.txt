# CMakeLists.txt of ESP32ENCODER component

# Src files
file(GLOB_RECURSE SRCS
    src/*.c
    src/*.cpp
)
# Include
set(INCS
    "include/"
)

# Public component requirement
set(REQUIRES
    driver
    esp_timer
)

# Private component requirement
set(PRIV_REQUIRES
)


# Register component
idf_component_register(
    SRCS ${SRCS}
    INCLUDE_DIRS ${INCS}
    REQUIRES ${REQUIRES}
    PRIV_REQUIRES ${PRIV_REQUIRES}
)
target_compile_options(${COMPONENT_LIB} PRIVATE -fexceptions)