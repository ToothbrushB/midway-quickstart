#pragma once

class WifiHelper {
public:
    static void init();
    static void deinit();
    static char* get_mac_string(void);

private:
    static uint8_t macAddress[6];
};