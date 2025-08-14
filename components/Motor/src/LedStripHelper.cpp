#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "led_strip.h"
#include "esp_log.h"
#include "esp_err.h"
#include "LedStripHelper.hpp"
#include "Telemetry.hpp"
#include "SettingsHelper.hpp"

// 10MHz resolution, 1 tick = 0.1us (led strip needs a high resolution)
#define LED_STRIP_RMT_RES_HZ  (10 * 1000 * 1000)

static const char *TAG = "LedStripHelper";
// Static members initialization
int LedStripHelper::LED_STRIP_LED_COUNT = 300; // Default count of LEDs
int LedStripHelper::LED_STRIP_GPIO_PIN = 2; // Default GPIO pin for LED strip
led_strip_handle_t LedStripHelper::led_strip;
Color LedStripHelper::current_color; // Default color red
void LedStripHelper::init(void)
{
    
    SettingsHelper::addIntSetting("led_count", LED_STRIP_LED_COUNT);
    SettingsHelper::setIntSetting("led_count", LED_STRIP_LED_COUNT);
    SettingsHelper::addIntSetting("led_pin", LED_STRIP_GPIO_PIN);
    SettingsHelper::addStringSetting("led_color", "255,0,0"); // Default color red

    configure_led();

    SettingsHelper::registerIntCallback("led_count", [](const std::pair<const char*, int>& setting) {
        ESP_LOGI(TAG, "LED Count changed: %d", setting.second);
        configure_led();
    });
    SettingsHelper::registerIntCallback("led_pin", [](const std::pair<const char*, int>& setting) {
        ESP_LOGI(TAG, "LED Pin changed: %d", setting.second);
        configure_led();
    });


    Telemetry::registerPeriodicCallback([]() {
        // Periodic callback code here
        // publish current color to telemetry in rgb using snprintf
        char color_str[32];
        snprintf(color_str, sizeof(color_str), "%d,%d,%d", current_color.r, current_color.g, current_color.b);
        Telemetry::publishData("leds/color", color_str, 0);
    }, PublishFrequency::HZ_1); // Adjust frequency as needed

}

led_strip_handle_t LedStripHelper::configure_led(void) {
    // LED strip general initialization, according to your led board design
    LED_STRIP_GPIO_PIN = SettingsHelper::getIntSetting("led_pin");
    LED_STRIP_LED_COUNT = SettingsHelper::getIntSetting("led_count");
    ESP_LOGI(TAG, "Configuring LED strip with GPIO pin %d and LED count %d", LED_STRIP_GPIO_PIN, LED_STRIP_LED_COUNT);
    led_strip_config_t strip_config = {
        .strip_gpio_num = LED_STRIP_GPIO_PIN, // The GPIO that connected to the LED strip's data line
        .max_leds = static_cast<uint32_t>(LED_STRIP_LED_COUNT),      // The number of LEDs in the strip,
        .led_model = LED_MODEL_WS2812,        // LED strip model
        .color_component_format = LED_STRIP_COLOR_COMPONENT_FMT_GRB, // The color order of the strip: GRB
        .flags = {
            .invert_out = false, // don't invert the output signal
        }
    };

    // LED strip backend configuration: RMT
    // led_strip_rmt_config_t rmt_config = {
    //     .clk_src = RMT_CLK_SRC_DEFAULT,        // different clock source can lead to different power consumption
    //     .resolution_hz = LED_STRIP_RMT_RES_HZ, // RMT counter clock frequency
    //     .mem_block_symbols = 0, // the memory block size used by the RMT channel
    //     .flags = {
    //         .with_dma = 0,     // Using DMA can improve performance when driving more LEDs
    //     }
    // };

    // LED Strip object handle
    // ESP_ERROR_CHECK(led_strip_new_rmt_device(&strip_config, &rmt_config, &led_strip));
    // ESP_LOGI(TAG, "Created LED strip object with RMT backend");

    // LED strip backend configuration: SPI
    led_strip_spi_config_t spi_config = {
        .clk_src = SPI_CLK_SRC_DEFAULT, // different clock source can lead to different power consumption
        .spi_bus = SPI3_HOST,           // SPI bus ID; IMU uses 2
        .flags = {
            .with_dma = 1, // Using DMA can improve performance and help drive more LEDs
        }
    };

    // LED Strip object handle
    ESP_ERROR_CHECK(led_strip_new_spi_device(&strip_config, &spi_config, &led_strip));
    ESP_LOGI(TAG, "Created LED strip object with SPI backend");

    // set the color to color from settings
    std::string color_str = SettingsHelper::getStringSetting("led_color");
    if (!color_str.empty()) {
        int r, g, b;
        if (sscanf(color_str.c_str(), "%d,%d,%d", &r, &g, &b) == 3) {
            set_color_rgb(Color{static_cast<uint8_t>(r), static_cast<uint8_t>(g), static_cast<uint8_t>(b)});
        }
    }
    return led_strip;
}

void LedStripHelper::set_color_rgb(Color c) {
    current_color = c;
    for (int i = 0; i < LED_STRIP_LED_COUNT; i++) {
        ESP_ERROR_CHECK(led_strip_set_pixel(led_strip, i, c.r, c.g, c.b));
    }
    ESP_ERROR_CHECK(led_strip_refresh(led_strip));
}

void LedStripHelper::set_color_hsv(int hue, int saturation, int value) {
    set_color_rgb(hsv2rgb(hue, saturation, value));
}

void LedStripHelper::set_color_rgb(int r, int g, int b) {
    set_color_rgb(Color{static_cast<uint8_t>(r), static_cast<uint8_t>(g), static_cast<uint8_t>(b)});
}

void LedStripHelper::off() {
    for (int i = 0; i < LED_STRIP_LED_COUNT; i++) {
        ESP_ERROR_CHECK(led_strip_set_pixel(led_strip, i, 0, 0, 0));
    }
    ESP_ERROR_CHECK(led_strip_refresh(led_strip));
}

Color LedStripHelper::hsv2rgb(uint16_t hue, uint8_t saturation, uint8_t value) {

    uint32_t red = 0;
    uint32_t green = 0;
    uint32_t blue = 0;

    uint32_t rgb_max = value;
    uint32_t rgb_min = rgb_max * (255 - saturation) / 255.0f;

    uint32_t i = hue / 60;
    uint32_t diff = hue % 60;

    // RGB adjustment amount by hue
    uint32_t rgb_adj = (rgb_max - rgb_min) * diff / 60;

    switch (i) {
    case 0:
        red = rgb_max;
        green = rgb_min + rgb_adj;
        blue = rgb_min;
        break;
    case 1:
        red = rgb_max - rgb_adj;
        green = rgb_max;
        blue = rgb_min;
        break;
    case 2:
        red = rgb_min;
        green = rgb_max;
        blue = rgb_min + rgb_adj;
        break;
    case 3:
        red = rgb_min;
        green = rgb_max - rgb_adj;
        blue = rgb_max;
        break;
    case 4:
        red = rgb_min + rgb_adj;
        green = rgb_min;
        blue = rgb_max;
        break;
    default:
        red = rgb_max;
        green = rgb_min;
        blue = rgb_max - rgb_adj;
        break;
    }
    return {static_cast<uint8_t>(red), static_cast<uint8_t>(green), static_cast<uint8_t>(blue)};
}

HSVColor LedStripHelper::rgb2hsv(Color c) {
    uint8_t max = std::max(std::max(c.r, c.g), c.b);
    uint8_t min = std::min(std::min(c.r, c.g), c.b);
    uint8_t delta = max - min;

    HSVColor hsv;
    if (delta == 0) {
        hsv.hue = 0;
    } else if (max == c.r) {
        hsv.hue = (60 * ((c.g - c.b) / delta) + 360) % 360;
    } else if (max == c.g) {
        hsv.hue = (60 * ((c.b - c.r) / delta) + 120) % 360;
    } else {
        hsv.hue = (60 * ((c.r - c.g) / delta) + 240) % 360;
    }

    hsv.saturation = (max == 0) ? 0 : (delta * 255 / max);
    hsv.value = max;

    return hsv;
}

