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
// #include "esp_timer.h"

static const char *TAG = "DEBUG ";
int counter = 0;

static TimerHandle_t listen_timer;

// Construct the probe request frame
static uint8_t probe_request[64] = {
    0x40, 0x00,                         // Frame Control (0x40 = probe request)
    0x00, 0x00,                         // Duration
    0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, // Destination MAC (broadcast)
    0x84, 0xF7, 0x03, 0x07, 0xC3, 0x10, // Source MAC (ESP32's MAC address)
    0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, // BSSID (broadcast)
    0x00, 0x00,                         // Sequence Control

    // SSID Information Element
    0x00, 0x00, // SSID Element ID (0x00), Length (0 for wildcard SSID)

    // Supported Rates Information Element
    0x01, 0x08, 0x82, 0x84, 0x8B, 0x96, 0x12, 0x24, 0x48, 0x6C, // Supported Rates (1, 2, 5.5, 11, 18, 36, 72, 96 Mbps)

    // Extended Supported Rates Information Element
    0x32, 0x04, 0x0C, 0x18, 0x30, 0x60 // Extended Supported Rates (6, 12, 24, 54 Mbps)
};

static void send_probe_request()
{
    esp_wifi_80211_tx(WIFI_IF_STA, probe_request, sizeof(probe_request), false);
    ESP_LOGI(TAG, "Wildcard probe request sent.");
}

static void listen_cb(TimerHandle_t xTimer)
{
    send_probe_request();
}

/**
 * Callback when packets are received in monitor mode
 */
static void listen_handler(void *buff, wifi_promiscuous_pkt_type_t type)
{
    if (type != WIFI_PKT_MGMT) // filter for management frames only
    {
        return;
    }

    wifi_promiscuous_pkt_t *ppkt = (wifi_promiscuous_pkt_t *)buff;
    uint8_t *payload = ppkt->payload;
    int packet_len = ppkt->rx_ctrl.sig_len;

    // Check if it's a Probe Request
    if ((payload[0] & 0xFC) == 0x40)
    {
        if (counter == 10)
        {
            send_probe_request();
            xTimerReset(listen_timer, 0);
            counter = 0;
            return;
        }

        // 24 bytes of MAC header, start parsing ie
        uint8_t *ies = payload + 24;
        int ies_len = packet_len - 24;

        int pos = 0;
        char ssid[33] = {0}; // null terminate after 32 bytes
        while (pos < ies_len)
        {
            uint8_t id = ies[pos];
            uint8_t length = ies[pos + 1];

            if (id == 0x00) // SSID Element ID
            {
                if (length == 0)
                {
                    strcpy(ssid, "WILDCARD");
                }
                else
                {
                    int copy_len = (length > 32) ? 32 : length;
                    memcpy(ssid, ies + pos + 2, copy_len);
                    ssid[copy_len] = '\0'; // null termination
                    break;
                }
            }
            pos += 2 + length;
        }

        uint8_t *bssid = ppkt->payload + 10; // BSSID is located at offset 10

        int8_t rssi = ppkt->rx_ctrl.rssi;
        uint8_t channel = ppkt->rx_ctrl.channel;

        printf("BSSID: %02x:%02x:%02x:%02x:%02x:%02x, Channel: %d, RSSI: %d, SSID: %s\n",
               bssid[0], bssid[1], bssid[2], bssid[3], bssid[4], bssid[5],
               channel,
               rssi,
               ssid);
    }
    counter++;
}

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

    // Set promiscuous filter
    wifi_promiscuous_filter_t filter = {
        .filter_mask = WIFI_PROMIS_FILTER_MASK_MGMT};
    ESP_ERROR_CHECK(esp_wifi_set_promiscuous_filter(&filter));

    // Setup promiscuous mode
    ESP_ERROR_CHECK(esp_wifi_set_promiscuous(true));

    // Set the promiscuous mode receive callback.
    // ESP_ERROR_CHECK(esp_wifi_set_promiscuous_rx_cb(wifi_sniffer_packet_handler));
    ESP_ERROR_CHECK(esp_wifi_set_promiscuous_rx_cb(listen_handler));

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
    ESP_ERROR_CHECK(esp_wifi_set_channel(10, WIFI_SECOND_CHAN_NONE));

    // Set bandwidth.
    ESP_ERROR_CHECK(esp_wifi_set_bandwidth(WIFI_IF_STA, WIFI_BW_HT20));
}

void app_main(void)
{
    ESP_ERROR_CHECK(nvs_flash_init());
    ESP_ERROR_CHECK(esp_netif_init()); // may be unnecessary
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    // initialize wifi
    wifi_init();

    // creates a timer with a callback "listen_cb"
    listen_timer = xTimerCreate("listen_timer", pdMS_TO_TICKS(1000), pdFALSE, NULL, listen_cb);
    xTimerStart(listen_timer, 0);

    ESP_LOGI(TAG, "Scanning started.");

    // uint8_t mac[6];
    // esp_read_mac(mac, ESP_MAC_WIFI_STA); // Read the MAC address for Wi-Fi station

    // printf("MAC Address: %02X:%02X:%02X:%02X:%02X:%02X\n",
    //        mac[0], mac[1], mac[2],
    //        mac[3], mac[4], mac[5]);
}
