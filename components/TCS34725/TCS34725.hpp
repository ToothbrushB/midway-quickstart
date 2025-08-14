/**
 *  @file TCS34725.hpp
 *
 *  @mainpage Driver for the TCS34725 digital color sensors for ESP32
 *
 *  @author Tollsimy, KTOWN (Adafruit Industries)
 * 
 *  @section license License
 * 
 *  BSD (see LICENSE)
 *
 *  @version v0.2 (C++ version)
 * 
 */

#pragma once

#include <stdint.h>
#include <stdbool.h>
#include <esp_err.h>
#include <driver/i2c.h>
#include "LedStripHelper.hpp"
#include <esp_timer.h>

// TCS34725 I2C Address
#define TCS34725_ADDRESS          (0x29)

// TCS34725 Command Bit
#define TCS34725_COMMAND_BIT      (0x80)

// TCS34725 Register Map
#define TCS34725_ENABLE           (0x00)
#define TCS34725_ATIME            (0x01)
#define TCS34725_WTIME            (0x03)
#define TCS34725_AILTL            (0x04)
#define TCS34725_AILTH            (0x05)
#define TCS34725_AIHTL            (0x06)
#define TCS34725_AIHTH            (0x07)
#define TCS34725_PERS             (0x0C)
#define TCS34725_CONFIG           (0x0D)
#define TCS34725_CONTROL          (0x0F)
#define TCS34725_ID               (0x12)
#define TCS34725_STATUS           (0x13)
#define TCS34725_CDATAL           (0x14)
#define TCS34725_CDATAH           (0x15)
#define TCS34725_RDATAL           (0x16)
#define TCS34725_RDATAH           (0x17)
#define TCS34725_GDATAL           (0x18)
#define TCS34725_GDATAH           (0x19)
#define TCS34725_BDATAL           (0x1A)
#define TCS34725_BDATAH           (0x1B)

// TCS34725 Enable Register
#define TCS34725_ENABLE_AIEN      (0x10)    // RGBC Interrupt Enable
#define TCS34725_ENABLE_WEN       (0x08)    // Wait enable - Writing 1 activates the wait timer
#define TCS34725_ENABLE_AEN       (0x02)    // RGBC Enable - Writing 1 actives the ADC, 0 disables it
#define TCS34725_ENABLE_PON       (0x01)    // Power on - Writing 1 activates the internal oscillator, 0 disables it

// TCS34725 Time Register
#define TCS34725_INTEGRATIONTIME_2_4MS  (0xFF)   // 2.4ms - 1 cycle    - Max Count: 1024
#define TCS34725_INTEGRATIONTIME_24MS   (0xF6)   // 24ms  - 10 cycles  - Max Count: 10240
#define TCS34725_INTEGRATIONTIME_50MS   (0xEB)   // 50ms  - 20 cycles  - Max Count: 20480
#define TCS34725_INTEGRATIONTIME_101MS  (0xD5)   // 101ms - 42 cycles  - Max Count: 43008
#define TCS34725_INTEGRATIONTIME_154MS  (0xC0)   // 154ms - 64 cycles  - Max Count: 65535
#define TCS34725_INTEGRATIONTIME_700MS  (0x00)   // 700ms - 256 cycles - Max Count: 65535

// TCS34725 Gain
typedef enum {
    TCS34725_GAIN_1X  = 0x00,   //  1x gain
    TCS34725_GAIN_4X  = 0x01,   //  4x gain
    TCS34725_GAIN_16X = 0x02,   // 16x gain
    TCS34725_GAIN_60X = 0x03    // 60x gain
} tcs34725Gain_t;

class TCS34725 {
public:
    // Constructor
    TCS34725();
    
    // Destructor
    ~TCS34725();
    
    // Initialize the sensor
    esp_err_t init(i2c_port_t i2c_port);
    
    // Enable/disable the device
    void enable();
    void disable();
    
    // Interrupt functions
    void setInterrupt(bool enable);
    void setIntLimits(uint16_t low, uint16_t high);
    void clearInterrupt();
    
    // Configuration functions
    void setIntegrationTime(uint8_t it);
    void setGain(tcs34725Gain_t gain);
    
    // Data reading functions
    void getRawData(uint16_t *r, uint16_t *g, uint16_t *b, uint16_t *c);
    void getRawDataOneShot(uint16_t *r, uint16_t *g, uint16_t *b, uint16_t *c);
    Color getColor() {return current_color;}
    
    // Color temperature calculation
    uint16_t calculateColorTemperature_dn40(uint16_t r, uint16_t g, uint16_t b, uint16_t c);
    
    uint16_t getColorTemperature() {return color_temperature;}
    uint16_t getLux() {return lux;}
    uint16_t getClear() {return _sum;}

    
    // Check if initialized
    bool isInitialized() const { return _initialized; }
    bool hasError() const { return _err; }

private:
    // Private member variables
    i2c_port_t _i2c_port;
    bool _initialized;
    tcs34725Gain_t _gain;
    uint8_t _integrationTime;
    
    // Private I2C functions
    void write8(uint8_t reg, uint32_t value);
    uint8_t read8(uint8_t reg);
    uint16_t read16(uint8_t reg);
    
    esp_timer_handle_t timer;
    Color current_color;
    uint16_t color_temperature;
    uint16_t lux;

    // Static tag for logging
    static const char* TAG;

    void getRGB(float *r, float *g, float *b);

    // Static utility functions
    static uint16_t calculateColorTemperature(uint16_t r, uint16_t g, uint16_t b);
    static uint16_t calculateLux(uint16_t r, uint16_t g, uint16_t b);

    void configure_tcs();
    void read_data();
    bool _err;
    uint16_t _sum; 
};
