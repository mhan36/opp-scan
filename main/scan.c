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

#define PROBE_PERIOD_MS 120
#define MAX_SCAN_RESULTS 30
#define CHANNELS_SCANNED 3

// Struct to hold scan results, from inject.c
typedef struct scan_result_t
{
    uint8_t bssid[6];  // BSSID (MAC address)
    uint8_t ssid[33];  // SSID
    uint8_t channel;   // Wi-Fi channel
    int8_t rssi;       // Signal strength (RSSI)
    bool updated;      // Flag to indicate if this entry was updated in the current iteration
    UT_hash_handle hh; // Hash table handle
    int recvResponse;  // Flag to indicate that a probe response was heard for this particular ssid
} scan_result_t;

// Manually construct the probe request frame, from inject.c
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

static const char *TAG = "DEBUG ";

static DRAM_ATTR scan_result_t *scan_results = NULL; // Hash table for storing unique scan results, from inject.c

static int64_t last_sniff_time_us = 0; // timestamp that the last packet was sniffed

static int num_scan_results = 0;

static const uint8_t chan_arr[] = {1, 6, 11};
static int chan_idx = 2;

/************************************************************
 *                SCAN RESULTS AND HASH                     *
 *                -Primarily sourced from inject.c          *
 ************************************************************/

// Add a scan result to the hash set
static inline void IRAM_ATTR add_scan_result(
    uint8_t *bssid,
    uint8_t *ssid,
    uint8_t ssid_len,
    uint8_t channel,
    int8_t rssi,
    bool is_probe_resp)
{
    // Check if we exceed maximum scan count.
    if (HASH_COUNT(scan_results) >= MAX_SCAN_RESULTS)
        return;

    // Check if the BSSID is already in the hash set
    scan_result_t *result;
    HASH_FIND(hh, scan_results, bssid, 6, result);

    if (result)
    {
        // Update the existing entry
        result->channel = channel;
        result->rssi = rssi;
        result->updated = true; // Mark as updated

        if (is_probe_resp)
        {
            result->recvResponse = 1;
        }
    }
    else
    {
        if (is_probe_resp) // only add new entry if it is a probe req, change this later to support probe resp
        {
            return;
        }
        // Create a new entry
        result = (scan_result_t *)malloc(sizeof(scan_result_t));
        memcpy(result->bssid, bssid, 6);
        memcpy(result->ssid, ssid, ssid_len);
        result->ssid[ssid_len] = '\0'; // Ensure SSID is null-terminated
        result->channel = channel;
        result->rssi = rssi;
        result->updated = true; // Mark as updated

        // Add to the hash set
        HASH_ADD(hh, scan_results, bssid, 6, result);
    }
}

// Function to clear the entire hash set
static inline void IRAM_ATTR clear_scan_results()
{
    scan_result_t *current_entry, *tmp;

    // Iterate over the hash set and reset each entry
    HASH_ITER(hh, scan_results, current_entry, tmp)
    {
        current_entry->updated = false; // Reset the updated flag
    }
}

// Debug Function to print the number of items in the hash set
static inline void IRAM_ATTR print_num_scan_results()
{
    unsigned int num_items = HASH_COUNT(scan_results); // Use HASH_COUNT to get the number of items
    printf("Scan results: %u\n", num_items);
}

// Debug Function to print all entries in the hash set
static inline void IRAM_ATTR print_scan_results()
{
    scan_result_t *current_entry, *tmp;

    // Iterate over the hash set and print each entry
    HASH_ITER(hh, scan_results, current_entry, tmp)
    {
        printf("BSSID: %02x:%02x:%02x:%02x:%02x:%02x, SSID: %s, Channel: %d, RSSI: %d dBm\n",
               current_entry->bssid[0], current_entry->bssid[1], current_entry->bssid[2],
               current_entry->bssid[3], current_entry->bssid[4], current_entry->bssid[5],
               current_entry->ssid, current_entry->channel, current_entry->rssi);
    }
}

/************************************************************
 *                      PROBING BEHAVIOR                    *
 ************************************************************/
static void switch_channels()
{
    if (chan_idx + 1 == CHANNELS_SCANNED)
    {
        chan_idx = 0;
    }
    else
    {
        chan_idx += 1;
    }

    ESP_ERROR_CHECK(esp_wifi_set_channel(chan_arr[chan_idx], WIFI_SECOND_CHAN_NONE)); // switch channels
}

bool is_probe_request(void *buff)
{
    wifi_promiscuous_pkt_t *ppkt = (wifi_promiscuous_pkt_t *)buff;
    uint8_t *payload = ppkt->payload;
    return (payload[0] & 0xFC) == 0x40;
}

bool is_probe_response(void *buff)
{
    wifi_promiscuous_pkt_t *ppkt = (wifi_promiscuous_pkt_t *)buff;
    uint8_t *payload = ppkt->payload;
    return (payload[0] & 0xFC) == 0x50;
}

// timer_cb and send_probe_request() can probably be combined for simplicity if not needed
static void send_probe_request()
{
    esp_wifi_80211_tx(WIFI_IF_STA, probe_request, sizeof(probe_request), false);
    ESP_LOGI(TAG, "Wildcard probe request sent. Channel : %d ", chan_arr[chan_idx]);
}

static void timer_cb()
{
    int64_t now = esp_timer_get_time();
    int64_t time_since_sniff = now - last_sniff_time_us;

    if (time_since_sniff >= 3000000 || last_sniff_time_us == 0) // this is "time delta" between probe bursts when in a silent env
    {
        switch_channels();
        send_probe_request(); // if no probe detected for 3 seconds (magic number), we probe ourselves (wildcard for now)
    }
    else
    {
        // ESP_LOGI(TAG, "SKIPPED PROBE");
    }
}

// Callback when packets are received in monitor mode
void IRAM_ATTR listen_handler(void *buff, wifi_promiscuous_pkt_type_t type)
{
    if (type != WIFI_PKT_MGMT) // filter for management frames only
    {
        return;
    }

    bool is_probe_req = is_probe_request(buff);
    bool is_probe_resp = is_probe_response(buff);

    if (!is_probe_req && !is_probe_resp)
    { // drop packet if its not a probe resp or req
        return;
    }

    switch_channels();
    last_sniff_time_us = esp_timer_get_time(); // update last time we sniffed

    wifi_promiscuous_pkt_t *ppkt = (wifi_promiscuous_pkt_t *)buff;
    uint8_t *payload = ppkt->payload;
    int packet_len = ppkt->rx_ctrl.sig_len;
    if (packet_len < 38)
    {
        return; // bad packet, causes memory to crash
    }

    // 24 bytes of MAC header, start parsing ie
    uint8_t *ies = payload + 24;
    int ies_len = packet_len - 24;

    int pos = 0;
    char ssid_str[33] = {0}; // null terminate after 32 bytes
    while (pos < ies_len)
    {
        uint8_t id = ies[pos];
        uint8_t length = ies[pos + 1];

        if (id == 0x00) // SSID Element ID
        {
            if (length == 0)
            {
                strcpy(ssid_str, "WILDCARD");
            }
            else
            {
                int copy_len = (length > 32) ? 32 : length;
                memcpy(ssid_str, ies + pos + 2, copy_len);
                ssid_str[copy_len] = '\0'; // null termination
                break;
            }
        }
        pos += 2 + length;
    }

    uint8_t *bssid = ppkt->payload + 10; // BSSID is located at offset 10

    int8_t ssid_len = ppkt->payload[37]; // SSID length is 1 byte after the Element ID
    uint8_t *ssid = ppkt->payload + 38;  // SSID starts after length byte

    int8_t rssi = ppkt->rx_ctrl.rssi;
    uint8_t channel = ppkt->rx_ctrl.channel;

    if (num_scan_results < MAX_SCAN_RESULTS)
    {
        add_scan_result(bssid, ssid, ssid_len, channel, rssi, is_probe_resp);
        num_scan_results += 1;
    }
    // else
    // {
    //     // print_scan_results();
    //     // clear_scan_results();
    //     // num_scan_results = 0;
    //     return;
    // }

    scan_result_t *result;
    HASH_FIND(hh, scan_results, bssid, 6, result);
    int found = 0;
    if (result)
    {
        found = result->recvResponse;
    }

    printf("BSSID: %02x:%02x:%02x:%02x:%02x:%02x, Channel: %d, RSSI: %d, SSID: %s, RESP RECV : %d \n",
           bssid[0], bssid[1], bssid[2], bssid[3], bssid[4], bssid[5],
           channel,
           rssi,
           ssid_str,
           found);
}

/************************************************************
 *                  WIFI INIT AND MAIN                      *
 ************************************************************/

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
    // ESP_ERROR_CHECK(esp_wifi_set_channel(11, WIFI_SECOND_CHAN_NONE));
    ESP_ERROR_CHECK(esp_wifi_set_channel(chan_arr[chan_idx], WIFI_SECOND_CHAN_NONE));

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

    // Create a timer to periodically send probe requests.
    esp_timer_create_args_t timer_args = {
        .callback = &timer_cb,             // Callback function
        .arg = NULL,                       // Argument passed to the callback
        .dispatch_method = ESP_TIMER_TASK, // ESP_TIEMR_ISR does not seem to be supported for version 5.x
        // .dispatch_method = ESP_TIMER_ISR, // Call callback in a task context
        .name = "probe_cb" // Name of the timer (for debugging)
    };

    esp_timer_handle_t timer_handle;
    ESP_ERROR_CHECK(esp_timer_create(&timer_args, &timer_handle));
    ESP_ERROR_CHECK(esp_timer_start_periodic(timer_handle, PROBE_PERIOD_MS * 1000)); // 1,000,000 microseconds = 1 second

    ESP_LOGI(TAG, "opp-scan start");

    // uint8_t mac[6];
    // esp_read_mac(mac, ESP_MAC_WIFI_STA); // Read the MAC address for Wi-Fi station

    // printf("MAC Address: %02X:%02X:%02X:%02X:%02X:%02X\n",
    //        mac[0], mac[1], mac[2],
    //        mac[3], mac[4], mac[5]);
}
