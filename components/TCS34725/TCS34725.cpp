/**
 *  @file TCS34725.cpp
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

#include "TCS34725.hpp"
#include <cmath>
#include <cstdlib>
#include <esp_log.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "LED.hpp"
#include "SettingsHelper.hpp"
#include "Telemetry.hpp"
const char* TCS34725::TAG = "TCS34725";

// Constructor
TCS34725::TCS34725() 
    : _i2c_port(I2C_NUM_0) // Default to I2C_NUM_0
    , _initialized(false)
    , _gain(TCS34725_GAIN_1X)
    , _integrationTime(TCS34725_INTEGRATIONTIME_2_4MS)
{
}

// Destructor
TCS34725::~TCS34725() {
    if (_initialized) {
        disable();
    }
}

// Private I2C functions
void TCS34725::write8(uint8_t reg, uint32_t value) {
    uint8_t buffer[2] = {static_cast<uint8_t>(TCS34725_COMMAND_BIT | reg), static_cast<uint8_t>(value & 0xFF)};
    _err = i2c_master_write_to_device(_i2c_port, TCS34725_ADDRESS, buffer, 2, 1000 / portTICK_PERIOD_MS);
}

uint8_t TCS34725::read8(uint8_t reg) {
    uint8_t buffer[1] = {static_cast<uint8_t>(TCS34725_COMMAND_BIT | reg)};
    _err = i2c_master_write_to_device(_i2c_port, TCS34725_ADDRESS, buffer, 1, 1000 / portTICK_PERIOD_MS) & 
                  i2c_master_read_from_device(_i2c_port, TCS34725_ADDRESS, buffer, 1, 1000 / portTICK_PERIOD_MS);
    return buffer[0];
}

uint16_t TCS34725::read16(uint8_t reg) {
    uint8_t buffer[2] = {static_cast<uint8_t>(TCS34725_COMMAND_BIT | reg), 0};
    _err = i2c_master_write_to_device(_i2c_port, TCS34725_ADDRESS, buffer, 1, 1000 / portTICK_PERIOD_MS) & i2c_master_read_from_device(_i2c_port, TCS34725_ADDRESS, buffer, 2, 1000 / portTICK_PERIOD_MS);
    return (((uint16_t)buffer[1]) << 8) | (((uint16_t)buffer[0]) & 0xFF);
}

// Public functions

esp_err_t TCS34725::init(i2c_port_t i2c_port) {
    if (_initialized) {
        ESP_LOGE(TAG, "TCS34725 already initialized");
        return ESP_FAIL;
    }

    _i2c_port = i2c_port;
    _gain = TCS34725_GAIN_1X;
    _integrationTime = TCS34725_INTEGRATIONTIME_2_4MS;

    // Make sure we're actually connected
    uint8_t x = read8(TCS34725_ID);
    if ((x != 0x4d) && (x != 0x44) && (x != 0x10)) {
        ESP_LOGE(TAG, "TCS34725 not found, ID = 0x%02x", x);
        return ESP_FAIL;
    }
    
    _initialized = true;

    // Set default integration time and gain
    setIntegrationTime(_integrationTime);
    setGain(_gain);

    // Note: by default, the device is in power down mode on bootup
    enable();

    // Set up the settings
    SettingsHelper::addIntSetting("tcs_gain", TCS34725_GAIN_1X);
    SettingsHelper::addIntSetting("tcs_int_time", TCS34725_INTEGRATIONTIME_50MS);
    SettingsHelper::registerIntCallback("tcs_gain", [this](const std::pair<const char*, int>& setting) {
        this->configure_tcs();
        ESP_LOGI(TAG, "TCS34725 Gain changed: %d", setting.second);
    });
    SettingsHelper::registerIntCallback("tcs_int_time", [this](const std::pair<const char*, int>& setting) {
        this->configure_tcs();
        ESP_LOGI(TAG, "TCS34725 Integration Time changed: %d", setting.second);
    });

    // configure the settings
    configure_tcs();


    // Set up a periodic timer to read data
    const esp_timer_create_args_t timer_args = {
        .callback = [](void* arg) {
            static_cast<TCS34725*>(arg)->read_data();
        },
        .arg = this,
        .dispatch_method = ESP_TIMER_TASK,
        .name = "TCS34725 Timer",
        .skip_unhandled_events = false
    };
    esp_timer_create(&timer_args, &timer);
    esp_timer_start_periodic(timer, 100000); // 100 ms interval

    // publish to telemetry
    Telemetry::registerPeriodicCallback([this]() {
        // Periodic callback code here
        // publish current color in rgb, lux, and color temperature to telemetry using snprintf in json
        if (hasError()) {
            Telemetry::publishData("tcs", "{\"status\": \"error\"}", 0);
            return;
        } else if (!isInitialized()) {
            Telemetry::publishData("tcs", "{\"status\": \"not_initialized\"}", 0);
            return;
        } else {
            char json[128];
            snprintf(json, sizeof(json), "{\"r\":%d, \"g\":%d, \"b\":%d, \"c\":%d, \"lux\":%d, \"color_temperature\":%d, \"status\":\"ok\"}",
                    this->current_color.r, this->current_color.g, this->current_color.b,
                    this->getClear(), this->lux, this->color_temperature);
            Telemetry::publishData("tcs", json, 0);
        }


    }, PublishFrequency::HZ_1); // Adjust frequency as needed    


    return ESP_OK;
}

void TCS34725::read_data(void) {
    float red, green, blue;
    getRGB(&red, &green, &blue);
    current_color = {static_cast<uint8_t>(red), static_cast<uint8_t>(green), static_cast<uint8_t>(blue)};
    color_temperature = calculateColorTemperature(red, green, blue);
    lux = calculateLux(red, green, blue);
}

void TCS34725::configure_tcs(void) {
    // Configure the TCS34725 sensor
    setGain(static_cast<tcs34725Gain_t>(SettingsHelper::getIntSetting("tcs_gain")));
    setIntegrationTime(SettingsHelper::getIntSetting("tcs_int_time"));
}

void TCS34725::enable() {
    if (!_initialized) {
        ESP_LOGE(TAG, "TCS34725 not initialized");
        return;
    }
    
    write8(TCS34725_ENABLE, TCS34725_ENABLE_PON);
    vTaskDelay(3 / portTICK_PERIOD_MS);
    write8(TCS34725_ENABLE, TCS34725_ENABLE_PON | TCS34725_ENABLE_AEN);
    
    // Set a delay for the integration time.
    // This is only necessary in the case where enabling and then
    // immediately trying to read values back. This is because setting
    // AEN triggers an automatic integration, so if a read RGBC is
    // performed too quickly, the data is not yet valid and all 0's are
    // returned
    // 12/5 = 2.4, add 1 to account for integer truncation
    vTaskDelay(((256 - _integrationTime) * 12 / 5 + 1) / portTICK_PERIOD_MS);
}

void TCS34725::disable() {
    if (!_initialized) {
        ESP_LOGE(TAG, "TCS34725 not initialized");
        return;
    }
    
    // Turn the device off to save power
    uint8_t reg = read8(TCS34725_ENABLE);
    write8(TCS34725_ENABLE, reg & ~(TCS34725_ENABLE_PON | TCS34725_ENABLE_AEN));
}

void TCS34725::setInterrupt(bool enable) {
    if (!_initialized) {
        ESP_LOGE(TAG, "TCS34725 not initialized");
        return;
    }
    
    uint8_t r = read8(TCS34725_ENABLE);
    if (enable) {
        r |= TCS34725_ENABLE_AIEN;
    } else {
        r &= ~TCS34725_ENABLE_AIEN;
    }
    write8(TCS34725_ENABLE, r);
}

void TCS34725::setIntLimits(uint16_t low, uint16_t high) {
    if (!_initialized) {
        ESP_LOGE(TAG, "TCS34725 not initialized");
        return;
    }
    
    write8(0x04, low & 0xFF);
    write8(0x05, low >> 8);
    write8(0x06, high & 0xFF);
    write8(0x07, high >> 8);
}

void TCS34725::clearInterrupt() {
    if (!_initialized) {
        ESP_LOGE(TAG, "TCS34725 not initialized");
        return;
    }
    
    uint8_t buffer[1] = {TCS34725_COMMAND_BIT | 0x66};
    ESP_ERROR_CHECK(i2c_master_write_to_device(_i2c_port, TCS34725_ADDRESS, buffer, 1, 1000 / portTICK_PERIOD_MS));
}

void TCS34725::setIntegrationTime(uint8_t it) {
    if (!_initialized) {
        ESP_LOGE(TAG, "TCS34725 not initialized");
        return;
    }
    
    // Update the timing register
    write8(TCS34725_ATIME, it);
    
    // Update value placeholders
    _integrationTime = it;
}

void TCS34725::setGain(tcs34725Gain_t gain) {
    if (!_initialized) {
        ESP_LOGE(TAG, "TCS34725 not initialized");
        return;
    }
    
    // Update the timing register
    write8(TCS34725_CONTROL, gain);
    
    // Update value placeholders
    _gain = gain;
}

void TCS34725::getRawData(uint16_t *r, uint16_t *g, uint16_t *b, uint16_t *c) {
    if (!_initialized) {
        ESP_LOGE(TAG, "TCS34725 not initialized");
        return;
    }

    *c = read16(TCS34725_CDATAL);
    *r = read16(TCS34725_RDATAL);
    *g = read16(TCS34725_GDATAL);
    *b = read16(TCS34725_BDATAL);

    // Set a delay for the integration time
    // 12/5 = 2.4, add 1 to account for integer truncation
    vTaskDelay(((256 - _integrationTime) * 12 / 5 + 1) / portTICK_PERIOD_MS);
}

void TCS34725::getRawDataOneShot(uint16_t *r, uint16_t *g, uint16_t *b, uint16_t *c) {
    if (!_initialized) {
        ESP_LOGE(TAG, "TCS34725 not initialized");
        return;
    }
    
    enable();
    getRawData(r, g, b, c);
    disable();
}

void TCS34725::getRGB(float *r, float *g, float *b) {
    if (!_initialized) {
        ESP_LOGE(TAG, "TCS34725 not initialized");
        return;
    }
    
    uint16_t red, green, blue, clear;
    getRawData(&red, &green, &blue, &clear);
    _sum = clear;

    // Avoid divide by zero errors ... if clear = 0 return black
    if (clear == 0) {
        *r = *g = *b = 0;
        return;
    }

    *r = (float)red / _sum * 255.0f;
    *g = (float)green / _sum * 255.0f;
    *b = (float)blue / _sum * 255.0f;
}

uint16_t TCS34725::calculateColorTemperature_dn40(uint16_t r, uint16_t g, uint16_t b, uint16_t c) {
    if (!_initialized) {
        ESP_LOGE(TAG, "TCS34725 not initialized");
        return 0;
    }
    
    uint16_t r2, b2; // RGB values minus IR component
    uint16_t sat;    // Digital saturation level
    uint16_t ir;     // Inferred IR content

    if (c == 0) {
        return 0;
    }

    // Analog/Digital saturation:
    //
    // (a) As light becomes brighter, the clear channel will tend to
    //     saturate first since R+G+B is approximately equal to C.
    // (b) The TCS34725 accumulates 1024 counts per 2.4ms of integration
    //     time, up to a maximum values of 65535. This means analog
    //     saturation can occur up to an integration time of 153.6ms
    //     (64*2.4ms=153.6ms).
    // (c) If the integration time is > 153.6ms, digital saturation will
    //     occur before analog saturation. Digital saturation occurs when
    //     the count reaches 65535.
    if ((256 - _integrationTime) > 63) {
        // Track digital saturation
        sat = 65535;
    } else {
        // Track analog saturation
        sat = 1024 * (256 - _integrationTime);
    }

    // Ripple rejection:
    //
    // (a) An integration time of 50ms or multiples of 50ms are required to
    //     reject both 50Hz and 60Hz ripple.
    // (b) If an integration time faster than 50ms is required, you may need
    //     to average a number of samples over a 50ms period to reject ripple
    //     from fluorescent and incandescent light sources.
    //
    // Ripple saturation notes:
    //
    // (a) If there is ripple in the received signal, the value read from C
    //     will be less than the max, but still have some effects of being
    //     saturated. This means that you can be below the 'sat' value, but
    //     still be saturating. At integration times >150ms this can be
    //     ignored, but <= 150ms you should calculate the 75% saturation
    //     level to avoid this problem.
    if ((256 - _integrationTime) <= 63) {
        // Adjust sat to 75% to avoid analog saturation if atime < 153.6ms
        sat -= sat / 4;
    }

    // Check for saturation and mark the sample as invalid if true
    if (c >= sat) {
        return 0;
    }

    // AMS RGB sensors have no IR channel, so the IR content must be
    // calculated indirectly.
    ir = (r + g + b > c) ? (r + g + b - c) / 2 : 0;

    // Remove the IR component from the raw RGB values
    r2 = r - ir;
    b2 = b - ir;

    if (r2 == 0) {
        return 0;
    }

    // A simple method of measuring color temp is to use the ratio of blue
    // to red light, taking IR cancellation into account.
    uint16_t cct = (3810 * (uint32_t)b2) / // Color temp coefficient.
                       (uint32_t)r2 +
                   1391; // Color temp offset.

    return cct;
}

// Static utility functions
uint16_t TCS34725::calculateColorTemperature(uint16_t r, uint16_t g, uint16_t b) {
    float X, Y, Z; // RGB to XYZ correlation
    float xc, yc;  // Chromaticity co-ordinates
    float n;       // McCamy's formula
    float cct;

    if (r == 0 && g == 0 && b == 0) {
        return 0;
    }

    // 1. Map RGB values to their XYZ counterparts.
    // Based on 6500K fluorescent, 3000K fluorescent
    // and 60W incandescent values for a wide range.
    // Note: Y = Illuminance or lux
    X = (-0.14282f * r) + (1.54924f * g) + (-0.95641f * b);
    Y = (-0.32466f * r) + (1.57837f * g) + (-0.73191f * b);
    Z = (-0.68202f * r) + (0.77073f * g) + (0.56332f * b);

    // 2. Calculate the chromaticity co-ordinates
    xc = (X) / (X + Y + Z);
    yc = (Y) / (X + Y + Z);

    // 3. Use McCamy's formula to determine the CCT
    n = (xc - 0.3320f) / (0.1858f - yc);

    // Calculate the final CCT
    cct = (449.0f * std::pow(n, 3)) + (3525.0f * std::pow(n, 2)) + (6823.3f * n) + 5520.33f;

    // Return the results in degrees Kelvin
    return (uint16_t)cct;
}

uint16_t TCS34725::calculateLux(uint16_t r, uint16_t g, uint16_t b) {
    float illuminance;

    // This only uses RGB ... how can we integrate clear or calculate lux
    // based exclusively on clear since this might be more reliable?
    illuminance = (-0.32466f * r) + (1.57837f * g) + (-0.73191f * b);

    return (uint16_t)illuminance;
}
