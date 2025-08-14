#include <map>
#include <string>
#include <functional>
#include <Preferences.hpp>
#pragma once
class SettingsHelper {
public:
    static void addStringSetting(const char* key, std::string defaultValue);
    static std::string getStringSetting(const char* key);
    static void setStringSetting(const char* key, std::string value);
    static void resetStringSetting(const char* key);

    static void addIntSetting(const char* key, int defaultValue);
    static int getIntSetting(const char* key);
    static void setIntSetting(const char* key, int value);
    static void resetIntSetting(const char* key);

    static void addDoubleSetting(const char* key, double defaultValue);
    static double getDoubleSetting(const char* key);
    static void setDoubleSetting(const char* key, double value);
    static void resetDoubleSetting(const char* key);

    static void addBoolSetting(const char* key, bool defaultValue);
    static bool getBoolSetting(const char* key);
    static void setBoolSetting(const char* key, bool value);
    static void resetBoolSetting(const char* key);

    static void applySettings();
    static void applyAllSettings();
    static void resetAllSettings();
    static void printAllSettings();

    static void init();

    static void printRemainingStorage();

    static void registerStringCallback(const char* key, std::function<void(const std::pair<const char*, std::string>&)> callback);
    static void registerIntCallback(const char* key, std::function<void(const std::pair<const char*, int>&)> callback);
    static void registerDoubleCallback(const char* key, std::function<void(const std::pair<const char*, double>&)> callback);
    static void registerBoolCallback(const char* key, std::function<void(const std::pair<const char*, bool>&)> callback);

    static void registerTelemetryListener(std::string topic);
private:
    static std::map<const char*, std::string> defaultStringSettings;
    static std::map<const char*, int> defaultIntSettings;
    static std::map<const char*, double> defaultDoubleSettings;
    static std::map<const char*, bool> defaultBoolSettings;

    // Map settings name to pair(needs to be updated, callback function(name, value))
    static std::multimap<const char*, std::pair<bool, std::function<void(std::pair<const char*, std::string>)>>> stringCallbacks;
    static std::multimap<const char*, std::pair<bool, std::function<void(std::pair<const char*, int>)>>> intCallbacks;
    static std::multimap<const char*, std::pair<bool, std::function<void(std::pair<const char*, double>)>>> doubleCallbacks;
    static std::multimap<const char*, std::pair<bool, std::function<void(std::pair<const char*, bool>)>>> boolCallbacks;
    static Preferences preferences;
    static void publishAllSettings();
};