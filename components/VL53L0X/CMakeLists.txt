idf_component_register(
			SRCS "src/vl53l0x.c" "src/VL53L0X.cpp"
			INCLUDE_DIRS "include"
			REQUIRES "driver" "esp_timer"
)