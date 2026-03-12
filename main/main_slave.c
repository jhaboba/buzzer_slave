/* ESPNOW Example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/

/*
   This example shows how to use ESPNOW.
   Prepare two device, one for sending ESPNOW data and another for receiving
   ESPNOW data.
*/
#include <stdlib.h>
#include <inttypes.h>
#include <string.h>
#include <assert.h>
#include "freertos/FreeRTOS.h"
#include "freertos/timers.h"
#include "nvs_flash.h"
#include "esp_random.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "esp_wifi.h"
#include "esp_log.h"
#include "esp_mac.h"
#include "esp_now.h"
#include "esp_crc.h"
#include "driver/gpio.h"
#include "espnow_example.h"

#define ESPNOW_MAXDELAY 512
#define SLAVE_TRIGGER_GPIO GPIO_NUM_0
#define SLAVE_TRIGGER_DEBOUNCE_MS 120
#define SLAVE_LED_GPIO GPIO_NUM_8
#define SLAVE_LED_ON_LEVEL 0
#define SLAVE_LED_OFF_LEVEL 1
#define SLAVE_LED_ON_MS 1000

static const char *TAG = "espnow_example";

static QueueHandle_t s_example_espnow_queue = NULL;
static TimerHandle_t s_slave_led_off_timer = NULL;
static const uint8_t s_example_fixed_peer_mac[ESP_NOW_ETH_ALEN] = { 0x10, 0x00, 0x3b, 0xcd, 0xd9, 0xbd };
static uint16_t s_example_espnow_seq = 0;
static uint8_t s_example_sta_mac[ESP_NOW_ETH_ALEN] = { 0 };
static volatile TickType_t s_last_gpio_trigger_tick = 0;

static void example_espnow_deinit(example_espnow_send_param_t *send_param);
static void example_gpio_init(void);
static void example_slave_led_init(void);
static void example_slave_led_blink(void);

static void example_slave_led_off_timer_cb(TimerHandle_t xTimer)
{
    (void)xTimer;
    gpio_set_level(SLAVE_LED_GPIO, SLAVE_LED_OFF_LEVEL);
}

static void example_slave_led_init(void)
{
    gpio_config_t led_conf = {
        .pin_bit_mask = 1ULL << SLAVE_LED_GPIO,
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };

    ESP_ERROR_CHECK(gpio_config(&led_conf));
    ESP_ERROR_CHECK(gpio_set_level(SLAVE_LED_GPIO, SLAVE_LED_OFF_LEVEL));

    s_slave_led_off_timer = xTimerCreate("slave_led_off",
                                         pdMS_TO_TICKS(SLAVE_LED_ON_MS),
                                         pdFALSE,
                                         NULL,
                                         example_slave_led_off_timer_cb);
    if (s_slave_led_off_timer == NULL) {
        ESP_LOGE(TAG, "Create slave LED timer fail");
    }
}

static void example_slave_led_blink(void)
{
    if (s_slave_led_off_timer == NULL) {
        return;
    }

    ESP_ERROR_CHECK(gpio_set_level(SLAVE_LED_GPIO, SLAVE_LED_ON_LEVEL));
    xTimerStop(s_slave_led_off_timer, 0);
    xTimerStart(s_slave_led_off_timer, 0);
}

static void IRAM_ATTR example_gpio_isr_handler(void *arg)
{
    (void)arg;
    BaseType_t high_task_wakeup = pdFALSE;
    TickType_t now = xTaskGetTickCountFromISR();
    example_espnow_event_t evt = { .id = EXAMPLE_ESPNOW_GPIO_CB };

    if ((now - s_last_gpio_trigger_tick) < pdMS_TO_TICKS(SLAVE_TRIGGER_DEBOUNCE_MS)) {
        return;
    }
    s_last_gpio_trigger_tick = now;

    xQueueSendFromISR(s_example_espnow_queue, &evt, &high_task_wakeup);
    if (high_task_wakeup == pdTRUE) {
        portYIELD_FROM_ISR();
    }
}

static void example_gpio_init(void)
{
    gpio_config_t io_conf = {
        .pin_bit_mask = 1ULL << SLAVE_TRIGGER_GPIO,
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_NEGEDGE,
    };

    ESP_ERROR_CHECK(gpio_config(&io_conf));
    ESP_ERROR_CHECK(gpio_install_isr_service(0));
    ESP_ERROR_CHECK(gpio_isr_handler_add(SLAVE_TRIGGER_GPIO, example_gpio_isr_handler, NULL));
    ESP_LOGI(TAG, "GPIO0 trigger enabled (pull-up, active low)");
}

/* WiFi should start before using ESPNOW */
static void example_wifi_init(void)
{
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK( esp_wifi_init(&cfg) );
    ESP_ERROR_CHECK( esp_wifi_set_storage(WIFI_STORAGE_RAM) );
    ESP_ERROR_CHECK( esp_wifi_set_mode(WIFI_MODE_STA) );
    ESP_ERROR_CHECK( esp_wifi_start());
    ESP_ERROR_CHECK( esp_wifi_set_channel(CONFIG_ESPNOW_CHANNEL, WIFI_SECOND_CHAN_NONE));
    ESP_ERROR_CHECK( esp_wifi_get_mac(ESP_IF_WIFI_STA, s_example_sta_mac) );
    ESP_LOGI(TAG, "--- SLAVE APP Local AP MAC: "MACSTR, MAC2STR(s_example_sta_mac));

#if CONFIG_ESPNOW_ENABLE_LONG_RANGE
    ESP_ERROR_CHECK( esp_wifi_set_protocol(ESP_IF_WIFI_STA, WIFI_PROTOCOL_11B|WIFI_PROTOCOL_11G|WIFI_PROTOCOL_11N|WIFI_PROTOCOL_LR) );
#endif
}

/* ESPNOW sending or receiving callback function is called in WiFi task.
 * Users should not do lengthy operations from this task. Instead, post
 * necessary data to a queue and handle it from a lower priority task. */
static void example_espnow_send_cb(const esp_now_send_info_t *tx_info, esp_now_send_status_t status)
{
    example_espnow_event_t evt;
    example_espnow_event_send_cb_t *send_cb = &evt.info.send_cb;

    if (tx_info == NULL) {
        ESP_LOGE(TAG, "Send cb arg error");
        return;
    }

    evt.id = EXAMPLE_ESPNOW_SEND_CB;
    memcpy(send_cb->mac_addr, tx_info->des_addr, ESP_NOW_ETH_ALEN);
    send_cb->status = status;
    if (xQueueSend(s_example_espnow_queue, &evt, ESPNOW_MAXDELAY) != pdTRUE) {
        ESP_LOGW(TAG, "Send send queue fail");
    }
}

static void example_espnow_recv_cb(const esp_now_recv_info_t *recv_info, const uint8_t *data, int len)
{
    example_espnow_event_t evt;
    example_espnow_event_recv_cb_t *recv_cb = &evt.info.recv_cb;
    const uint8_t *mac_addr = NULL;

    if (recv_info == NULL || data == NULL || len <= 0) {
        ESP_LOGE(TAG, "Receive cb arg error");
        return;
    }

    mac_addr = recv_info->src_addr;
    if (mac_addr == NULL) {
        ESP_LOGE(TAG, "Receive cb src mac error");
        return;
    }

    evt.id = EXAMPLE_ESPNOW_RECV_CB;
    memcpy(recv_cb->mac_addr, mac_addr, ESP_NOW_ETH_ALEN);
    recv_cb->data = malloc(len);
    if (recv_cb->data == NULL) {
        ESP_LOGE(TAG, "Malloc receive data fail");
        return;
    }
    memcpy(recv_cb->data, data, len);
    recv_cb->data_len = len;
    if (xQueueSend(s_example_espnow_queue, &evt, ESPNOW_MAXDELAY) != pdTRUE) {
        ESP_LOGW(TAG, "Send receive queue fail");
        free(recv_cb->data);
    }
}

/* Parse received ESPNOW data. */
int example_espnow_data_parse(uint8_t *data, uint16_t data_len, uint8_t *state, uint16_t *seq, uint32_t *magic)
{
    example_espnow_data_t *buf = (example_espnow_data_t *)data;
    uint16_t crc, crc_cal = 0;

    if (data_len < sizeof(example_espnow_data_t)) {
        ESP_LOGE(TAG, "Receive ESPNOW data too short, len:%d", data_len);
        return -1;
    }

    *state = buf->state;
    *seq = buf->seq_num;
    *magic = buf->magic;
    crc = buf->crc;
    buf->crc = 0;
    crc_cal = esp_crc16_le(UINT16_MAX, (uint8_t const *)buf, data_len);

    if (crc_cal == crc) {
        return buf->type;
    }

    return -1;
}

/* Prepare ESPNOW data to be sent. */
void example_espnow_data_prepare(example_espnow_send_param_t *send_param)
{
    example_espnow_data_t *buf = (example_espnow_data_t *)send_param->buffer;

    assert(send_param->len >= sizeof(example_espnow_data_t));

    buf->type = EXAMPLE_ESPNOW_DATA_UNICAST;
    buf->state = send_param->state;
    buf->seq_num = s_example_espnow_seq++;
    buf->crc = 0;
    buf->magic = send_param->magic;
    /* Fill all remaining bytes after the data with random values */
    esp_fill_random(buf->payload, send_param->len - sizeof(example_espnow_data_t));
    buf->crc = esp_crc16_le(UINT16_MAX, (uint8_t const *)buf, send_param->len);
}

static void example_espnow_task(void *pvParameter)
{
    example_espnow_event_t evt;
    uint8_t recv_state = 0;
    uint16_t recv_seq = 0;
    uint32_t recv_magic = 0;
    int ret;
    example_espnow_send_param_t *send_param = (example_espnow_send_param_t *)pvParameter;

    ESP_LOGI(TAG, "Waiting for GPIO0 low to send packet to: "MACSTR, MAC2STR(send_param->dest_mac));

    while (xQueueReceive(s_example_espnow_queue, &evt, portMAX_DELAY) == pdTRUE) {
        switch (evt.id) {
            case EXAMPLE_ESPNOW_SEND_CB:
            {
                example_espnow_event_send_cb_t *send_cb = &evt.info.send_cb;
                ESP_LOGD(TAG, "Send data to "MACSTR", status: %d", MAC2STR(send_cb->mac_addr), send_cb->status);
                break;
            }
            case EXAMPLE_ESPNOW_RECV_CB:
            {
                example_espnow_event_recv_cb_t *recv_cb = &evt.info.recv_cb;
                example_slave_led_blink();

                ret = example_espnow_data_parse(recv_cb->data, recv_cb->data_len, &recv_state, &recv_seq, &recv_magic);
                free(recv_cb->data);
                if (ret < 0) {
                    ESP_LOGI(TAG, "Receive error data from: "MACSTR"", MAC2STR(recv_cb->mac_addr));
                } else {
                    ESP_LOGI(TAG, "Receive %dth data(type=%d, state=%u, magic=%"PRIu32") from: "MACSTR", len: %d",
                             recv_seq, ret, recv_state, recv_magic, MAC2STR(recv_cb->mac_addr), recv_cb->data_len);
                }
                break;
            }
            case EXAMPLE_ESPNOW_GPIO_CB:
            {
                if (gpio_get_level(SLAVE_TRIGGER_GPIO) != 0) {
                    break;
                }

                example_espnow_data_prepare(send_param);
                if (esp_now_send(send_param->dest_mac, send_param->buffer, send_param->len) != ESP_OK) {
                    ESP_LOGE(TAG, "Send error");
                    example_espnow_deinit(send_param);
                    vTaskDelete(NULL);
                } else {
                    ESP_LOGI(TAG, "GPIO0 low -> sent packet to "MACSTR, MAC2STR(send_param->dest_mac));
                }
                break;
            }
            default:
                ESP_LOGE(TAG, "Callback type error: %d", evt.id);
                break;
        }
    }
}

static esp_err_t example_espnow_init(void)
{
    example_espnow_send_param_t *send_param;

    s_example_espnow_queue = xQueueCreate(ESPNOW_QUEUE_SIZE, sizeof(example_espnow_event_t));
    if (s_example_espnow_queue == NULL) {
        ESP_LOGE(TAG, "Create queue fail");
        return ESP_FAIL;
    }
    example_gpio_init();

    /* Initialize ESPNOW and register sending and receiving callback function. */
    ESP_ERROR_CHECK( esp_now_init() );
    ESP_ERROR_CHECK( esp_now_register_send_cb(example_espnow_send_cb) );
    ESP_ERROR_CHECK( esp_now_register_recv_cb(example_espnow_recv_cb) );
#if CONFIG_ESPNOW_ENABLE_POWER_SAVE
    ESP_ERROR_CHECK( esp_now_set_wake_window(CONFIG_ESPNOW_WAKE_WINDOW) );
    ESP_ERROR_CHECK( esp_wifi_connectionless_module_set_wake_interval(CONFIG_ESPNOW_WAKE_INTERVAL) );
#endif
    /* Add fixed unicast peer information to peer list. */
    esp_now_peer_info_t *peer = malloc(sizeof(esp_now_peer_info_t));
    if (peer == NULL) {
        ESP_LOGE(TAG, "Malloc peer information fail");
        vQueueDelete(s_example_espnow_queue);
        s_example_espnow_queue = NULL;
        esp_now_deinit();
        return ESP_FAIL;
    }
    memset(peer, 0, sizeof(esp_now_peer_info_t));
    peer->channel = CONFIG_ESPNOW_CHANNEL;
    peer->ifidx = ESP_IF_WIFI_STA;
    peer->encrypt = false;
    memcpy(peer->peer_addr, s_example_fixed_peer_mac, ESP_NOW_ETH_ALEN);
    ESP_ERROR_CHECK( esp_now_add_peer(peer) );
    free(peer);

    /* Initialize sending parameters. */
    send_param = malloc(sizeof(example_espnow_send_param_t));
    if (send_param == NULL) {
        ESP_LOGE(TAG, "Malloc send parameter fail");
        vQueueDelete(s_example_espnow_queue);
        s_example_espnow_queue = NULL;
        esp_now_deinit();
        return ESP_FAIL;
    }
    memset(send_param, 0, sizeof(example_espnow_send_param_t));
    send_param->unicast = true;
    send_param->broadcast = false;
    send_param->state = 1;
    send_param->magic = esp_random();
    // send_param->count = CONFIG_ESPNOW_SEND_COUNT;
    // send_param->delay = CONFIG_ESPNOW_SEND_DELAY;
    send_param->len = CONFIG_ESPNOW_SEND_LEN;
    send_param->buffer = malloc(CONFIG_ESPNOW_SEND_LEN);
    if (send_param->buffer == NULL) {
        ESP_LOGE(TAG, "Malloc send buffer fail");
        free(send_param);
        vQueueDelete(s_example_espnow_queue);
        s_example_espnow_queue = NULL;
        esp_now_deinit();
        return ESP_FAIL;
    }
    memcpy(send_param->dest_mac, s_example_fixed_peer_mac, ESP_NOW_ETH_ALEN);
    example_espnow_data_prepare(send_param);

    xTaskCreate(example_espnow_task, "example_espnow_task", 2560, send_param, 4, NULL);

    return ESP_OK;
}

static void example_espnow_deinit(example_espnow_send_param_t *send_param)
{
    if (s_slave_led_off_timer != NULL) {
        xTimerStop(s_slave_led_off_timer, portMAX_DELAY);
        xTimerDelete(s_slave_led_off_timer, portMAX_DELAY);
        s_slave_led_off_timer = NULL;
    }
    gpio_isr_handler_remove(SLAVE_TRIGGER_GPIO);
    gpio_uninstall_isr_service();
    free(send_param->buffer);
    free(send_param);
    vQueueDelete(s_example_espnow_queue);
    s_example_espnow_queue = NULL;
    esp_now_deinit();
}

void app_main(void)
{
    // Initialize NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK( nvs_flash_erase() );
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK( ret );

    example_slave_led_init();
    ESP_LOGD(TAG, "INIT OF SLAVE APPLICATION");

    example_wifi_init();
    example_espnow_init();
}
