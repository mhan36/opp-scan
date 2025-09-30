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

#define WIFI_SSID "ssid"
#define WIFI_PASS "pass"

// times are in ms
#define PROBE_DELAY 20      // how much delay before each burst of probes
#define PROBE_INTERVAL 30   // time between each 802.11 probe request frame within a burst
#define NUM_PROBES 3        // number of probe requests in each burst
#define CHAN_DWELL_TIME 100 // how long we stay on channel for each "probe event"
// #define LISTEN_TIME 10 //how long we listen on the channel for responses
#define SCAN_INTERVAL 60000 // how long each we wait between scan events
#define MAX_SCAN_RESULTS 30 // how many results we store
#define NUM_CHANNELS 14     // 14 chan on 2.4 ghz

static void probe_timer_cb();
static void chanDwell_timer_cb();

static void switch_to_next_channel();
static void init_timers();
static void send_probe_request();
static void finished_dynamo_probe();
static void IRAM_ATTR listen_handler(void *buff, wifi_promiscuous_pkt_type_t type);
static inline void IRAM_ATTR print_scan_results();

// Timer handlers
static esp_timer_handle_t probe_timer_handler;
static esp_timer_handle_t chanDwell_timer_handler;

//***********************************************************************
//*                                                                     *
//************************************************************************

// Struct to hold scan results, from inject.c
typedef struct scan_result_t
{
    uint8_t bssid[6];  // BSSID (MAC address)
    uint8_t ssid[33];  // SSID
    uint8_t channel;   // Wi-Fi channel
    int8_t rssi;       // Signal strength (RSSI)
    UT_hash_handle hh; // Hash table handle
    bool recvResponse; // Flag to indicate that a probe response was heard for this particular ssid
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

static const char *PRINT = "[ PRINT ]";
// static const char *DEBUG = "[ DEBUG ]";

static DRAM_ATTR scan_result_t *scan_results = NULL; // Hash table for storing unique scan results, from inject.c

static int num_scan_results = 0;

static const uint8_t wifi_channels[NUM_CHANNELS] = {1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14};
static int curr_chan_idx = 0;

static bool scan_finish = false;

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
 *                 TIMERS AND CALLBACKS                     *
 *                                                          *
 ************************************************************/

// Handler for Wi-Fi and IP events
// static void event_handler(
//     void *arg, esp_event_base_t event_base, int32_t event_id, void *event_data)
// {
//     if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START)
//     {
//         esp_wifi_connect();
//         ESP_LOGI(TAG, "Connecting to AP...");
//     }
//     else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED)
//     {
//         ESP_LOGW(TAG, "Disconnected. Reconnecting...");
//         esp_wifi_connect();
//     }
//     else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP)
//     {
//         ip_event_got_ip_t *event = (ip_event_got_ip_t *)event_data;
//         ESP_LOGI(TAG, "Got IP: " IPSTR, IP2STR(&event->ip_info.ip));
//     }
// }

static void init_timers()
{
    // Probe delay timer
    esp_timer_create_args_t probe_timer_args = {
        .callback = &probe_timer_cb,       // Callback function
        .arg = NULL,                       // Argument passed to the callback
        .dispatch_method = ESP_TIMER_TASK, // ESP_TIMER_ISR does not seem to be supported for version 5.x
        // .dispatch_method = ESP_TIMER_ISR, // Call callback in a task context
        .name = "probe_delay_timer" // Name of the timer (for debugging)
    };
    ESP_ERROR_CHECK(esp_timer_create(&probe_timer_args, &probe_timer_handler));

    // Chan Dwell Timer, opportunisticly triggered
    esp_timer_create_args_t chanDwell_timer_args = {
        .callback = &chanDwell_timer_cb,   // Callback function
        .arg = NULL,                       // Argument passed to the callback
        .dispatch_method = ESP_TIMER_TASK, // ESP_TIMER_ISR does not seem to be supported for version 5.x
        // .dispatch_method = ESP_TIMER_ISR, // Call callback in a task context
        .name = "chan_dwell_timer" // Name of the timer (for debugging)
    };
    ESP_ERROR_CHECK(esp_timer_create(&chanDwell_timer_args, &chanDwell_timer_handler));
}

static void switch_to_next_channel()
{
    if (scan_finish)
    {
        return;
    }

    if (curr_chan_idx >= NUM_CHANNELS - 1)
    {
        finished_dynamo_probe();
        return;
    }
    curr_chan_idx += 1;

    uint8_t next_chan = wifi_channels[curr_chan_idx];

    ESP_ERROR_CHECK(esp_wifi_set_channel(next_chan, WIFI_SECOND_CHAN_NONE)); // switch channels
    // ESP_LOGI(PRINT, "Channel switched to %d : ", next_chan);

    // restart the probe delay timer on duration PROBE_DELAY because we finished scanning this channel
    if (esp_timer_is_active(probe_timer_handler))
    {
        ESP_ERROR_CHECK(esp_timer_stop(probe_timer_handler));
    }
    ESP_LOGI(PRINT, "RESTART PROBE TIMER ON CHAN %d", next_chan);
    ESP_ERROR_CHECK(esp_timer_start_once(probe_timer_handler, PROBE_DELAY * 1000)); // 1,000,000 microseconds = 1 second, this value is PROBE_DELAY ms
}

static void send_probe_request()
{
    ESP_ERROR_CHECK(esp_wifi_80211_tx(WIFI_IF_STA, probe_request, sizeof(probe_request), false));
    ESP_LOGI(PRINT, "Wildcard probe request sent. Channel : %d ", wifi_channels[curr_chan_idx]);
}

// triggered when probe_delay expires without being interrupted by listen event
static void probe_timer_cb()
{
    ESP_LOGI(PRINT, "PROBE DELAY EXPIRED");
    if (scan_finish)
    {
        return;
    }
    // probeDelay expires, trigger an active probe on curr channel
    send_probe_request();

    // // Start chan dwell timer to dwell on this channel, if we have not already
    // if (!esp_timer_is_active(chanDwell_timer_handler))
    // {
    //     ESP_ERROR_CHECK(esp_timer_start_once(chanDwell_timer_handler, CHAN_DWELL_TIME * 1000)); // 1,000,000 microseconds = 1 second, this value is CHAN_DWELL_TIME ms
    // }

    // refresh chanDwell timer, in case we started it before probe_delay expired
    if (esp_timer_is_active(chanDwell_timer_handler))
    {
        ESP_ERROR_CHECK(esp_timer_stop(chanDwell_timer_handler));
    }

    if (!esp_timer_is_active(chanDwell_timer_handler))
    {
        ESP_LOGI(PRINT, "START CHAN DWELL LISTENING AFTER SENDING PROBE");
        ESP_ERROR_CHECK(esp_timer_start_once(chanDwell_timer_handler, PROBE_DELAY * 1000)); // 1,000,000 microseconds = 1 second, this value is PROBE_DELAY ms
    }

    // // start probeDelay timer again for next channel
    // if (esp_timer_is_active(probe_timer_handler))
    // {
    //     ESP_ERROR_CHECK(esp_timer_stop(probe_timer_handler));
    // }
    // ESP_ERROR_CHECK(esp_timer_start_once(probe_timer_handler, PROBE_DELAY * 1000)); // 1,000,000 microseconds = 1 second, this value is PROBE_DELAY
}

// triggered when chanDwell expires
static void chanDwell_timer_cb()
{
    ESP_LOGI(PRINT, "CHAN DWELL EXPIRED");
    if (scan_finish)
    {
        return;
    }

    // switch channel, only in chanDwell because we want to dwell on the channel after sending a probe, or hear a probe being transmitted
    switch_to_next_channel();
}

static void finished_dynamo_probe()
{

    // bool flag control for a single scan event
    if (scan_finish)
    {
        return;
    }
    scan_finish = true;
    ESP_LOGI(PRINT, "FINISHED SCANNING");

    if (esp_timer_is_active(probe_timer_handler))
    {
        ESP_ERROR_CHECK(esp_timer_stop(probe_timer_handler));
    }

    if (esp_timer_is_active(chanDwell_timer_handler))
    {
        ESP_ERROR_CHECK(esp_timer_stop(chanDwell_timer_handler));
    }
    ESP_LOGI(PRINT, "STOP ALL TIMERS");
    ESP_ERROR_CHECK(esp_timer_delete(probe_timer_handler));
    ESP_ERROR_CHECK(esp_timer_delete(chanDwell_timer_handler));

    esp_wifi_set_promiscuous(false);
    ESP_LOGI(PRINT, "Disabled promiscuous mode");
    esp_wifi_set_promiscuous_rx_cb(NULL);

    print_scan_results();

    // ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &event_handler, NULL));
    // ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &event_handler, NULL));

    // restart wifi in STA mode
    ESP_ERROR_CHECK(esp_wifi_stop());
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_start());

    wifi_config_t wifi_config = {
        .sta = {
            .ssid = WIFI_SSID,
            .password = WIFI_PASS,
            .threshold.authmode = WIFI_AUTH_WPA2_PSK,
        },
    };
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_connect());
    ESP_LOGI(PRINT, "Connecting to AP...");

    return;
}

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
        if (is_probe_resp)
        {
            result->recvResponse = true;
        }
    }
    else
    {
        // Create a new entry, we have not seen this BSSID before
        result = (scan_result_t *)malloc(sizeof(scan_result_t));
        memcpy(result->bssid, bssid, 6);
        memcpy(result->ssid, ssid, ssid_len);
        result->ssid[ssid_len] = '\0'; // Ensure SSID is null-terminated
        result->channel = channel;
        result->rssi = rssi;
        // Add to the hash set
        HASH_ADD(hh, scan_results, bssid, 6, result);
    }
}

// Function to clear the entire hash set
static inline void IRAM_ATTR clear_scan_results()
{
    scan_result_t *current_entry, *tmp;

    // // Iterate over the hash set and reset each entry
    // HASH_ITER(hh, scan_results, current_entry, tmp)
    // {
    //     current_entry->updated = false; // Reset the updated flag
    // }
}

// Debug Function to print the number of items in the hash set
static inline void IRAM_ATTR print_num_scan_results()
{
    unsigned int num_items = HASH_COUNT(scan_results); // Use HASH_COUNT to get the number of items
    printf("Scan results: %u\n", num_items);
}

/************************************************************
 *                      PROBING BEHAVIOR                    *
 ************************************************************/

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

// Callback when packets are received in monitor mode
void IRAM_ATTR listen_handler(void *buff, wifi_promiscuous_pkt_type_t type)
{
    if (scan_finish)
    {
        return;
    }

    if (type != WIFI_PKT_MGMT) // filter for management frames only
    {
        return;
    }

    bool is_probe_req = is_probe_request(buff);
    bool is_probe_resp = is_probe_response(buff);

    if (!is_probe_req && !is_probe_resp)
    { // drop packet if its not a probe resp or req
        // keep probe delay timer running
        return;
    }

    if (is_probe_req)
    {
        ESP_LOGI(PRINT, "####### PROBE REQUEST SNIIFED #######");
    }
    else if (is_probe_resp)
    {
        ESP_LOGI(PRINT, "####### PROBE RESPONSE SNIFFED #######");
    }

    // If probe delay active, Stop probe_delay timer upon sniffing a relevant packet, continue to sniff on this chan for chanDwell

    if (esp_timer_is_active(probe_timer_handler))
    {
        ESP_LOGI(PRINT, "****** STOP PROBE TIMER IN LISTEN HANDLER ****");
        ESP_ERROR_CHECK(esp_timer_stop(probe_timer_handler));

        // Start chanDwell timer on this channel, if it has not already been started by a previous listen event
        // if it has been started by probe_delay expiring, we do not start
        if (!esp_timer_is_active(chanDwell_timer_handler))
        {
            ESP_LOGI(PRINT, "****** STARTED CHAN DWELL TIMER IN LISTEN HANDLER ****");
            ESP_ERROR_CHECK(esp_timer_start_once(chanDwell_timer_handler, CHAN_DWELL_TIME * 1000)); // 1,000,000 microseconds = 1 second, this value is CHAN_DWELL_TIME ms
        }
    }
    else
    {
        ESP_LOGI(PRINT, "****** PROBE DELAY IS NOT ACTIVE ****");
    }

    // // Start chanDwell timer on this channel, if it has not already been started by a previous listen event
    // // if it has been started by probe_delay expiring, we do not start
    // if (!esp_timer_is_active(chanDwell_timer_handler))
    // {
    //     ESP_LOGI(PRINT, "****** STARTED CHAN DWELL TIMER IN LISTEN HANDLER ****");
    //     ESP_ERROR_CHECK(esp_timer_start_once(chanDwell_timer_handler, CHAN_DWELL_TIME * 1000)); // 1,000,000 microseconds = 1 second, this value is CHAN_DWELL_TIME ms
    // }

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
        ESP_LOGI(PRINT, "########### ADDED A SCAN RESULT ################");
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

    // scan_result_t *result;
    // HASH_FIND(hh, scan_results, bssid, 6, result);
    // bool found = false;
    // if (result)
    // {
    //     found = result->recvResponse;
    // }

    // printf("BSSID: %02x:%02x:%02x:%02x:%02x:%02x, Channel: %d, RSSI: %d, SSID: %s, RESP RECV : %d \n",
    //        bssid[0], bssid[1], bssid[2], bssid[3], bssid[4], bssid[5],
    //        channel,
    //        rssi,
    //        ssid_str,
    //        found);
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

    // wifi_config_t wifi_config = {
    //     .sta = {
    //         .ssid = WIFI_SSID,
    //         .password = WIFI_PASS,
    //         .threshold.authmode = WIFI_AUTH_WPA2_PSK,
    //     },
    // };

    // Start Wi-Fi stack.
    ESP_ERROR_CHECK(esp_wifi_start());

    // Set max TX power.
    ESP_ERROR_CHECK(esp_wifi_set_max_tx_power(84));

    // Set channel.
    // ESP_ERROR_CHECK(esp_wifi_set_channel(11, WIFI_SECOND_CHAN_NONE));
    ESP_ERROR_CHECK(esp_wifi_set_channel(wifi_channels[curr_chan_idx], WIFI_SECOND_CHAN_NONE));

    // Set bandwidth, 2.4 ghz
    ESP_ERROR_CHECK(esp_wifi_set_bandwidth(WIFI_IF_STA, WIFI_BW_HT20));

    // ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
}

void app_main(void)
{
    ESP_ERROR_CHECK(nvs_flash_init());
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    wifi_init();
    init_timers();

    // start probe delay timer, triggers active scan upon expiration if not interrupted by listen
    ESP_ERROR_CHECK(esp_timer_start_once(probe_timer_handler, PROBE_DELAY * 1000)); // 1,000,000 microseconds = 1 second, this value is PROBE_DELAY ms

    ESP_LOGI(PRINT, "~~~~~~~~~~~~~~~~~~~~~~ START  ~~~~~~~~~~~~~~~~~~~~~~");
    ESP_LOGI(PRINT, "FIRST PROBE DELAY STARTS HERE");

    // uint8_t mac[6];
    // esp_read_mac(mac, ESP_MAC_WIFI_STA); // Read the MAC address for Wi-Fi station

    // printf("MAC Address: %02X:%02X:%02X:%02X:%02X:%02X\n",
    //        mac[0], mac[1], mac[2],
    //        mac[3], mac[4], mac[5]);
}