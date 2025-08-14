#include "SettingsHelper.hpp"
#include "Telemetry.hpp"
#include "esp_log.h"
#include <map>
#include <string>
#include <functional>
#include <Preferences.hpp>

static const char* TAG = "SettingsHelper";
static const char* BASE_TOPIC = "settings/";
std::map<const char*, std::string> SettingsHelper::defaultStringSettings;
std::map<const char*, int> SettingsHelper::defaultIntSettings;
std::map<const char*, double> SettingsHelper::defaultDoubleSettings;
std::map<const char*, bool> SettingsHelper::defaultBoolSettings;
std::multimap<const char*, std::pair<bool, std::function<void(std::pair<const char*, std::string>)>>> SettingsHelper::stringCallbacks;
std::multimap<const char*, std::pair<bool, std::function<void(std::pair<const char*, int>)>>> SettingsHelper::intCallbacks;
std::multimap<const char*, std::pair<bool, std::function<void(std::pair<const char*, double>)>>> SettingsHelper::doubleCallbacks;
std::multimap<const char*, std::pair<bool, std::function<void(std::pair<const char*, bool>)>>> SettingsHelper::boolCallbacks;
Preferences SettingsHelper::preferences;

void SettingsHelper::addStringSetting(const char* key, std::string defaultValue) {
    defaultStringSettings[key] = defaultValue;
    if (!preferences.isKey(key)) {
        ESP_LOGI(TAG, "Adding string setting: %s = %s", key, defaultValue.c_str());
        preferences.putString(key, defaultValue);
    }
}
void SettingsHelper::addIntSetting(const char* key, int defaultValue) {
    defaultIntSettings[key] = defaultValue;
    if (!preferences.isKey(key)) {
        ESP_LOGI(TAG, "Adding int setting: %s = %d", key, defaultValue);
        preferences.putInt(key, defaultValue);
    }
}
void SettingsHelper::addDoubleSetting(const char* key, double defaultValue) {
    defaultDoubleSettings[key] = defaultValue;
    if (!preferences.isKey(key)) {
        ESP_LOGI(TAG, "Adding double setting: %s = %f", key, defaultValue);
        preferences.putDouble(key, defaultValue);
    }
}
void SettingsHelper::addBoolSetting(const char* key, bool defaultValue) {
    defaultBoolSettings[key] = defaultValue;
    if (!preferences.isKey(key)) {
        ESP_LOGI(TAG, "Adding bool setting: %s = %s", key, defaultValue ? "true" : "false");
        preferences.putBool(key, defaultValue);
    }
}
std::string SettingsHelper::getStringSetting(const char* key) {
    return preferences.getString(key, defaultStringSettings[key]);
}
int SettingsHelper::getIntSetting(const char* key) {
    return preferences.getInt(key, defaultIntSettings[key]);
}
double SettingsHelper::getDoubleSetting(const char* key) {
    return preferences.getDouble(key, defaultDoubleSettings[key]);
}
bool SettingsHelper::getBoolSetting(const char* key) {
    return preferences.getBool(key, defaultBoolSettings[key]);
}
void SettingsHelper::setStringSetting(const char* key, std::string value) {
    preferences.putString(key, value);
    auto range = stringCallbacks.equal_range(key);
    for (auto it = range.first; it != range.second; ++it) {
        it->second.first = true; // Mark as updated
    }
    std::string topic = std::string(BASE_TOPIC) + key;
    Telemetry::publishData(topic.c_str(), value.c_str());
}
void SettingsHelper::setIntSetting(const char* key, int value) {
    preferences.putInt(key, value);
    auto range = intCallbacks.equal_range(key);
    for (auto it = range.first; it != range.second; ++it) {
        it->second.first = true; // Mark as updated
    }
    std::string topic = std::string(BASE_TOPIC) + key;
    Telemetry::publishData(topic.c_str(), value);
}
void SettingsHelper::setDoubleSetting(const char* key, double value) {
    preferences.putDouble(key, value);
    auto range = doubleCallbacks.equal_range(key);
    for (auto it = range.first; it != range.second; ++it) {
        it->second.first = true; // Mark as updated
    }
    std::string topic = std::string(BASE_TOPIC) + key;
    Telemetry::publishData(topic.c_str(), value);
}
void SettingsHelper::setBoolSetting(const char* key, bool value) {
    preferences.putBool(key, value);
    auto range = boolCallbacks.equal_range(key);
    for (auto it = range.first; it != range.second; ++it) {
        it->second.first = true; // Mark as updated
    }
    std::string topic = std::string(BASE_TOPIC) + key;
    Telemetry::publishData(topic.c_str(), value);
}


void SettingsHelper::init() {
    if(preferences.begin("settings", false, "nvs_pref")) {
        ESP_LOGI(TAG, "Preferences initialized");
        printRemainingStorage();
    }
    else {
        ESP_LOGE(TAG, "Failed to initialize Preferences");
    }
}

void SettingsHelper::printRemainingStorage() {
    size_t freeEntries = preferences.freeEntries();
    ESP_LOGI(TAG, "Remaining storage entries: %zu", freeEntries);
}

void SettingsHelper::applyAllSettings() {
    for (const auto& pair : stringCallbacks) {
            pair.second.second({pair.first, preferences.getString(pair.first, defaultStringSettings[pair.first])});
    }
    for (const auto& pair : intCallbacks) {
            pair.second.second({pair.first, preferences.getInt(pair.first, defaultIntSettings[pair.first])});
    }
    for (const auto& pair : doubleCallbacks) {
            pair.second.second({pair.first, preferences.getDouble(pair.first, defaultDoubleSettings[pair.first])});
    }
    for (const auto& pair : boolCallbacks) {
            pair.second.second({pair.first, preferences.getBool(pair.first, defaultBoolSettings[pair.first])});
    }
}

void SettingsHelper::applySettings() {
    for (const auto& pair : stringCallbacks) {
        if (pair.second.first) {
            // If the value is updated, call it with the current value
            pair.second.second({pair.first, preferences.getString(pair.first, defaultStringSettings[pair.first])});
        }
    }
    for (const auto& pair : intCallbacks) {
        if (pair.second.first) {
            // If the value is updated, call it with the current value
            pair.second.second({pair.first, preferences.getInt(pair.first, defaultIntSettings[pair.first])});
        }
    }
    for (const auto& pair : doubleCallbacks) {
        if (pair.second.first) {
            // If the value is updated, call it with the current value
            pair.second.second({pair.first, preferences.getDouble(pair.first, defaultDoubleSettings[pair.first])});
        }
    }
    for (const auto& pair : boolCallbacks) {
        if (pair.second.first) {
            // If the value is updated, call it with the current value
            pair.second.second({pair.first, preferences.getBool(pair.first, defaultBoolSettings[pair.first])});
        }
    }
    ESP_LOGI(TAG, "Settings applied");
}

void SettingsHelper::resetAllSettings() {
    preferences.clear();
    for (const auto& pair : defaultStringSettings) {
        preferences.putString(pair.first, pair.second);
    }
    for (const auto& pair : defaultIntSettings) {
        preferences.putInt(pair.first, pair.second);
    }
    for (const auto& pair : defaultDoubleSettings) {
        preferences.putDouble(pair.first, pair.second);
    }
    for (const auto& pair : defaultBoolSettings) {
        preferences.putBool(pair.first, pair.second);
    }
    for (auto& pair : stringCallbacks) {
        pair.second.first = true; // Mark as updated
    }
    for (auto& pair : intCallbacks) {
        pair.second.first = true; // Mark as updated
    }
    for (auto& pair : doubleCallbacks) {
        pair.second.first = true; // Mark as updated
    }
    for (auto& pair : boolCallbacks) {
        pair.second.first = true; // Mark as updated
    }
    publishAllSettings();
    ESP_LOGI(TAG, "All settings reset to defaults");
}

void SettingsHelper::publishAllSettings() {
    for (const auto& pair : defaultStringSettings) {
        std::string topic = std::string(BASE_TOPIC) + pair.first;
        Telemetry::publishData(topic.c_str(), getStringSetting(pair.first).c_str());
    }
    for (const auto& pair : defaultIntSettings) {
        std::string topic = std::string(BASE_TOPIC) + pair.first;
        Telemetry::publishData(topic.c_str(), getIntSetting(pair.first));
    }
    for (const auto& pair : defaultDoubleSettings) {
        std::string topic = std::string(BASE_TOPIC) + pair.first;
        Telemetry::publishData(topic.c_str(), getDoubleSetting(pair.first));
    }
    for (const auto& pair : defaultBoolSettings) {
        std::string topic = std::string(BASE_TOPIC) + pair.first;
        Telemetry::publishData(topic.c_str(), getBoolSetting(pair.first));
    }
}

void SettingsHelper::resetBoolSetting(const char* key) {
    if (defaultBoolSettings.find(key) != defaultBoolSettings.end()) {
        preferences.putBool(key, defaultBoolSettings[key]);
        auto range = boolCallbacks.equal_range(key);
        for (auto it = range.first; it != range.second; ++it) {
            it->second.first = true; // Mark as updated
        }
    } else {
        ESP_LOGW(TAG, "No default value for bool setting: %s", key);
    }
}
void SettingsHelper::resetIntSetting(const char* key) {
    if (defaultIntSettings.find(key) != defaultIntSettings.end()) {
        preferences.putInt(key, defaultIntSettings[key]);
        auto range = intCallbacks.equal_range(key);
        for (auto it = range.first; it != range.second; ++it) {
            it->second.first = true; // Mark as updated
        }
    } else {
        ESP_LOGW(TAG, "No default value for int setting: %s", key);
    }
}

void SettingsHelper::resetDoubleSetting(const char* key) {
    if (defaultDoubleSettings.find(key) != defaultDoubleSettings.end()) {
        preferences.putDouble(key, defaultDoubleSettings[key]);
        auto range = doubleCallbacks.equal_range(key);
        for (auto it = range.first; it != range.second; ++it) {
            it->second.first = true; // Mark as updated
        }
    } else {
        ESP_LOGW(TAG, "No default value for double setting: %s", key);
    }
}
void SettingsHelper::resetStringSetting(const char* key) {
    if (defaultStringSettings.find(key) != defaultStringSettings.end()) {
        preferences.putString(key, defaultStringSettings[key]);
        auto range = stringCallbacks.equal_range(key);
        for (auto it = range.first; it != range.second; ++it) {
            it->second.first = true; // Mark as updated
        }
    } else {
        ESP_LOGW(TAG, "No default value for string setting: %s", key);
    }
}

void SettingsHelper::registerStringCallback(const char* key, std::function<void(const std::pair<const char*, std::string>&)> callback) {
    if (preferences.isKey(key)) {
        stringCallbacks.insert({key, {false, callback}});
    } else {
        ESP_LOGW(TAG, "Key %s does not exist in preferences", key);
    }
}
void SettingsHelper::registerIntCallback(const char* key, std::function<void(const std::pair<const char*, int>&)> callback) {
    if (preferences.isKey(key)) {
        intCallbacks.insert({key, {false, callback}});
    } else {
        ESP_LOGW(TAG, "Key %s does not exist in preferences", key);
    }
}
void SettingsHelper::registerDoubleCallback(const char* key, std::function<void(const std::pair<const char*, double>&)> callback) {
    if (preferences.isKey(key)) {
        doubleCallbacks.insert({key, {false, callback}});
    } else {
        ESP_LOGW(TAG, "Key %s does not exist in preferences", key);
    }
}
void SettingsHelper::registerBoolCallback(const char* key, std::function<void(const std::pair<const char*, bool>&)> callback) {
    if (preferences.isKey(key)) {
        boolCallbacks.insert({key, {false, callback}});
    } else {
        ESP_LOGW(TAG, "Key %s does not exist in preferences", key);
    }
}

void SettingsHelper::printAllSettings() {
    ESP_LOGI(TAG, "Current settings:");
    for (const auto& pair : defaultStringSettings) {
        ESP_LOGI(TAG, "  %s: %s", pair.first, getStringSetting(pair.first).c_str());
    }
    for (const auto& pair : defaultIntSettings) {
        ESP_LOGI(TAG, "  %s: %d", pair.first, getIntSetting(pair.first));
    }
    for (const auto& pair : defaultDoubleSettings) {
        ESP_LOGI(TAG, "  %s: %f", pair.first, getDoubleSetting(pair.first));
    }
    for (const auto& pair : defaultBoolSettings) {
        ESP_LOGI(TAG, "  %s: %s", pair.first, getBoolSetting(pair.first) ? "true" : "false");
    }
}

void SettingsHelper::registerTelemetryListener(std::string topic) {
    // Register a telemetry listener for the given topic
    Telemetry::subscribe(topic.c_str(), [topic](const char* t, const char* data, int data_len) {
        // If SET command, parse and set the setting
        // If GET command, publish the current setting value using response method
        // If RESET command, reset to default value
        // If APPLY command, apply the setting
    });
}
