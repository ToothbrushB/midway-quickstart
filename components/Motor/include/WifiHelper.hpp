#pragma once

class WifiHelper {
public:
    static void init();
    static int getRssi();
    static void deinit();

private:
    static uint8_t macAddress[6];
};