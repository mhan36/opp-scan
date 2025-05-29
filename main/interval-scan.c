#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/timers.h"
#include "esp_system.h"
#include "esp_log.h"
#include "esp_mac.h"
#include "esp_wifi.h"
#include "esp_netif.h"
#include "nvs_flash.h"
#include "esp_timer.h"
#include "uthash.h"

#define CHANNEL 1
#define SECONDS_TO_USEC(s) ((s) * 1000000ULL)

static const char *TAG = "DEBUG ";
static esp_timer_handle_t periodic_timer;

// Timing units are in milliseconds
static wifi_scan_config_t scan_config = {
    .ssid = NULL,
    .bssid = NULL,
    .channel = 1,
    .show_hidden = true,
    .scan_type = WIFI_SCAN_TYPE_ACTIVE,
    .scan_time = {
        .active = {
            .min = 0,
            .max = 120 // ms per channel
        }
    },
    .home_chan_dwell_time = 250 // ms
};

// Initialize Wi-Fi stack to inject packets.
void wifi_init()
{
    // Set default config.
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    // Set country for US
    wifi_country_t wifi_country = {
        .cc = "US",
        .schan = 1,
        .nchan = 11,
        .policy = WIFI_COUNTRY_POLICY_AUTO};
    ESP_ERROR_CHECK(esp_wifi_set_country(&wifi_country));

    // Set storage to RAM
    ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_RAM));

    // Disable power saving.
    ESP_ERROR_CHECK(esp_wifi_set_ps(WIFI_PS_NONE));

    // Set Wi-Fi to station mode
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));

    // Set transmit PHY rate to lowest in 802.11n
    // (see https://github.com/espressif/esp-idf/blob/master/components/esp_wifi/include/esp_wifi_types_generic.h#L874)
    ESP_ERROR_CHECK(esp_wifi_config_80211_tx_rate(WIFI_IF_STA, WIFI_PHY_RATE_1M_L));

    // Set protocol.
    ESP_ERROR_CHECK(esp_wifi_set_protocol(WIFI_IF_STA, WIFI_PROTOCOL_11B));

    // Start Wi-Fi stack.
    ESP_ERROR_CHECK(esp_wifi_start());

    // Set max TX power.
    ESP_ERROR_CHECK(esp_wifi_set_max_tx_power(84));

    // Set channel.
    ESP_ERROR_CHECK(esp_wifi_set_channel(CHANNEL, WIFI_SECOND_CHAN_NONE));
    // ESP_ERROR_CHECK(esp_wifi_set_channel(chan_arr[chan_idx], WIFI_SECOND_CHAN_NONE));

    // Set bandwidth.
    ESP_ERROR_CHECK(esp_wifi_set_bandwidth(WIFI_IF_STA, WIFI_BW_HT20));
}

void periodic_scan(void *arg)
{
    esp_err_t ret = esp_wifi_scan_start(&scan_config, true);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Scan failed: %s", esp_err_to_name(ret));
        return;
    }
    
    // Process scan results
    uint16_t ap_count = 20;
    wifi_ap_record_t ap_records[ap_count];
    esp_wifi_scan_get_ap_records(&ap_count, ap_records);
    ESP_LOGI(TAG, "Scan completed. Found %d APs", ap_count);

    ESP_ERROR_CHECK(esp_wifi_scan_get_ap_records(&ap_count, ap_records));
    ESP_LOGI(TAG, "Found %d access points:", ap_count);

    for (int i = 0; i < ap_count; ++i)
    {
        ESP_LOGI(TAG, "  SSID: %s, RSSI: %d, Channel: %d",
                 ap_records[i].ssid,
                 ap_records[i].rssi,
                 ap_records[i].primary);
    }

    esp_wifi_clear_ap_list();
}

void app_main(void)
{
    ESP_ERROR_CHECK(nvs_flash_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    //ESP_ERROR_CHECK(esp_netif_init()); // may be unnecessary

    // wifi_init();
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_start());

    // Create periodic timer (30 seconds)
    const esp_timer_create_args_t timer_args = {
        .callback = &periodic_scan,
        .name = "periodic_scan"
    };
    ESP_ERROR_CHECK(esp_timer_create(&timer_args, &periodic_timer));
    ESP_ERROR_CHECK(esp_timer_start_periodic(periodic_timer, SECONDS_TO_USEC(30)));
    
    uint8_t mac[6];
    esp_read_mac(mac, ESP_MAC_WIFI_STA);
    printf("ESP MAC Address: %02X:%02X:%02X:%02X:%02X:%02X\n",
           mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);

    // Keep task alive
    while(1) vTaskDelay(pdMS_TO_TICKS(1000));
}