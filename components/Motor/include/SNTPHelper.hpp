#pragma once
#include <ctime>

class SNTPHelper {
public:
    static void init();
    static void deinit();
    static void setTimeZone(const char* timezone);
    static void waitForSync();
    static void start();
    static void print_servers();
    static void print_current_time();

private:
    static void time_sync_notification_cb(struct timeval *tv);

};