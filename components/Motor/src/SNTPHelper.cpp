#include "SNTPHelper.hpp"
#include <time.h>
#include <sys/time.h>
#include "esp_netif_sntp.h"
#include "esp_sntp.h"
#include "esp_log.h"
#include "freertos/portmacro.h"
#include "esp_event.h"
#include "nvs_flash.h"
#include "esp_netif.h"


static const char* TAG = "SNTPHelper";
#ifndef INET6_ADDRSTRLEN
#define INET6_ADDRSTRLEN 48
#endif


void SNTPHelper::print_servers(void)
{
    ESP_LOGI(TAG, "List of configured NTP servers:");

    for (uint8_t i = 0; i < SNTP_MAX_SERVERS; ++i){
        if (esp_sntp_getservername(i)){
            ESP_LOGI(TAG, "server %d: %s", i, esp_sntp_getservername(i));
        } else {
            // we have either IPv4 or IPv6 address, let's print it
            char buff[INET6_ADDRSTRLEN];
            ip_addr_t const *ip = esp_sntp_getserver(i);
            if (ipaddr_ntoa_r(ip, buff, INET6_ADDRSTRLEN) != NULL)
                ESP_LOGI(TAG, "server %d: %s", i, buff);
        }
    }
}

void SNTPHelper::time_sync_notification_cb(struct timeval *tv) // static method
{
    ESP_LOGI(TAG, "Notification of a time synchronization event");
    print_current_time();
}

void SNTPHelper::print_current_time() {
    // Print current time code
    time_t now;
    struct tm timeinfo;
    char strftime_buf[64];

    time(&now);
    localtime_r(&now, &timeinfo);
    strftime(strftime_buf, sizeof(strftime_buf), "%c", &timeinfo);
    ESP_LOGI(TAG, "Current time: %s", strftime_buf);
}

// must be called before getting IP
void SNTPHelper::init() {
    // Initialization code
    /**
     * NTP server address could be acquired via DHCP,
     * see following menuconfig options:
     * 'LWIP_DHCP_GET_NTP_SRV' - enable STNP over DHCP
     * 'LWIP_SNTP_DEBUG' - enable debugging messages
     *
     * NOTE: This call should be made BEFORE esp acquires IP address from DHCP,
     * otherwise NTP option would be rejected by default.
     */
    ESP_LOGI(TAG, "Initializing SNTP");
    ESP_ERROR_CHECK( nvs_flash_init() );
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK( esp_event_loop_create_default() );
    esp_sntp_config_t config = {
        .smooth_sync = false, // smooth sync is not needed for this example
        .server_from_dhcp = true, // accept NTP offers from DHCP server
        .wait_for_sync = true, // create a semaphore to signal time sync event
        .start = false, // do not start SNTP service automatically
        .sync_cb = time_sync_notification_cb, // callback function on time sync event
        .renew_servers_after_new_IP = true, // refresh server list if NTP provided by DHCP
        .ip_event_to_renew = IP_EVENT_STA_GOT_IP, // set the IP event id on which we refresh server list
        .index_of_first_server = 0, // updates from server num 1, leaving server 0 (preconfigured) intact
        .num_of_servers = 0, // number of preconfigured NTP servers
        .servers = {} // list of servers
    };
    esp_netif_sntp_init(&config);
}

// must be called after getting IP
void SNTPHelper::start() {
    // Start SNTP service
    ESP_ERROR_CHECK(esp_netif_sntp_start());
}

void SNTPHelper::deinit() {
    // Deinitialization code
    esp_netif_sntp_deinit();
}


void SNTPHelper::setTimeZone(const char* timezone) {
    // Set time zone code
    setenv("TZ", timezone, 1);
    tzset();
}

void SNTPHelper::waitForSync() {
    // Wait for synchronization code
    while (esp_netif_sntp_sync_wait(1000 / portTICK_PERIOD_MS) == ESP_ERR_TIMEOUT) {
        ESP_LOGI(TAG, "Waiting for system time to be set...");
    }
}
