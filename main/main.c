#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_timer.h"

#include "easy_gpio.h"
#include "lora.h"
#include "oled.h"

#if CONFIG_FREERTOS_HZ < 1000
#error "Set CONFIG_FREERTOS_HZ=1000 (menuconfig > Component config > FreeRTOS > Kernel > configTICK_RATE_HZ)"
#endif

static const char *TAG = "MAINCTRL";

#define PEDAL_LEFT_PIN     21
#define PEDAL_RIGHT_PIN    20
#define PEDAL_ACTIVE_LEVEL LOW

#define PIN_LORA_MOSI      6
#define PIN_LORA_MISO      5
#define PIN_LORA_SCK       4
#define PIN_LORA_NSS       7
#define PIN_LORA_RST       10
#define PIN_LORA_DIO0      3

#define OLED_SDA_PIN       0
#define OLED_SCL_PIN       1

#define LORA_FREQ_HZ       433000000

#define PEDAL_POLL_MS      5
#define PEDAL_DEBOUNCE_N   3
#define PEDAL_CHORD_MS     60
#define PEDAL_TAP_GAP_MS   150

#define CMD_BURST_CYCLES   3
#define CMD_ZERO_CYCLES    1

#define RING_WAIT_HOPS     4
#define RING_MARGIN_HOPS   2
#define LINK_LOST_STREAK   10

#define PKT_LEN 3
_Static_assert(PKT_LEN == 3, "paket harus 3 byte");

enum { NODE_MAIN = 1, NODE_PANTRY, NODE_ICE, NODE_OVEN, NODE_DRONE };

static const uint8_t s_node_id = NODE_MAIN;
static uint32_t      s_hop_ms  = 5;

static uint32_t ring_timeout_ms(void)
{
    return (RING_WAIT_HOPS + RING_MARGIN_HOPS) * s_hop_ms;
}

enum {
    CMD_NONE     = 0x0,
    CMD_MAIN1    = 0x1,
    CMD_MAIN2    = 0x2,
    CMD_OPT1     = 0x3,
    CMD_OPT2     = 0x4,
    CMD_SW_DRONE = 0x5,
    CMD_SW_CAM   = 0x6,
};

static const char *cmd_name(uint8_t c)
{
    switch (c) {
    case CMD_MAIN1:    return "MAIN1";
    case CMD_MAIN2:    return "MAIN2";
    case CMD_OPT1:     return "OPT1";
    case CMD_OPT2:     return "OPT2";
    case CMD_SW_DRONE: return "SW-DRONE";
    case CMD_SW_CAM:   return "SW-CAM";
    default:           return "-";
    }
}

typedef struct {
    uint8_t  red, green, pantry_land;
    uint8_t  ball, ice_land;
    uint8_t  cube, oven_land;
    uint8_t  drone;
    uint16_t payload;
    uint8_t  last_cmd;
    uint32_t cycle_us;
    uint32_t timeouts;
    uint32_t streak;
    bool     link_ok;
} ring_state_t;

static ring_state_t s_st = { .link_ok = false };
static portMUX_TYPE s_mux = portMUX_INITIALIZER_UNLOCKED;

static QueueHandle_t s_cmd_q;

static void gesture_emit(uint8_t mask, uint8_t n);

typedef struct {
    bool     down[2];
    uint8_t  cnt[2];
    int8_t   pend;
    uint32_t pend_t;
    uint8_t  seq_mask;
    uint8_t  seq_n;
    uint32_t last_edge;
} gesture_t;

static void gesture_tap(gesture_t *g, uint8_t mask)
{
    if (g->seq_n && g->seq_mask != mask) {
        gesture_emit(g->seq_mask, g->seq_n);
        g->seq_n = 0;
    }
    g->seq_mask = mask;
    if (++g->seq_n >= 2) {
        gesture_emit(mask, 2);
        g->seq_n = 0;
    }
}

static void gesture_step(gesture_t *g, uint32_t now, const bool raw[2])
{

    if (g->pend >= 0 && (now - g->pend_t) > PEDAL_CHORD_MS) {
        gesture_tap(g, (uint8_t)(1u << g->pend));
        g->pend = -1;
    }

    for (int i = 0; i < 2; i++) {
        if (raw[i] == g->down[i]) {
            g->cnt[i] = 0;
            continue;
        }
        if (++g->cnt[i] < PEDAL_DEBOUNCE_N) {
            continue;
        }
        g->down[i] = raw[i];
        g->cnt[i] = 0;
        g->last_edge = now;
        if (!raw[i]) {
            continue;
        }
        if (g->pend < 0) {
            g->pend = (int8_t)i;
            g->pend_t = now;
        } else if (g->pend != i) {
            gesture_tap(g, 0x03);
            g->pend = -1;
        }
    }

    if (g->seq_n && g->pend < 0 && (now - g->last_edge) >= PEDAL_TAP_GAP_MS) {
        gesture_emit(g->seq_mask, g->seq_n);
        g->seq_n = 0;
    }
}

static void gesture_emit(uint8_t mask, uint8_t n)
{
    uint8_t cmd = CMD_NONE;
    if      (mask == 0x02 && n == 1) cmd = CMD_MAIN1;
    else if (mask == 0x01 && n == 1) cmd = CMD_MAIN2;
    else if (mask == 0x02 && n == 2) cmd = CMD_OPT1;
    else if (mask == 0x01 && n == 2) cmd = CMD_OPT2;
    else if (mask == 0x03 && n == 2) cmd = CMD_SW_DRONE;
    else if (mask == 0x03 && n == 1) cmd = CMD_SW_CAM;

    if (cmd != CMD_NONE) {
        xQueueSend(s_cmd_q, &cmd, 0);
    }
}

static void pedal_task(void *arg)
{
    (void)arg;
    const uint8_t pins[2] = { PEDAL_LEFT_PIN, PEDAL_RIGHT_PIN };
    for (int i = 0; i < 2; i++) {
        ESP_ERROR_CHECK(pinMode(pins[i], INPUT_PULLUP));
    }

    gesture_t g = { .pend = -1 };
    TickType_t wake = xTaskGetTickCount();
    for (;;) {
        vTaskDelayUntil(&wake, pdMS_TO_TICKS(PEDAL_POLL_MS));
        bool raw[2];
        for (int i = 0; i < 2; i++) {
            raw[i] = (digitalRead(pins[i]) == PEDAL_ACTIVE_LEVEL);
        }
        gesture_step(&g, millis(), raw);
    }
}

static lora_t           *s_radio;
static SemaphoreHandle_t s_tx_mtx;
static uint32_t          s_cycle;
static int64_t           s_last_tx_us;
static uint8_t           s_burst_cmd, s_burst_left, s_zero_left;
static TaskHandle_t      s_ring_task;

static uint8_t cmd_next(void)
{
    if (s_burst_left > 0) {
        if (--s_burst_left == 0) {
            s_zero_left = CMD_ZERO_CYCLES;
        }
        return s_burst_cmd;
    }
    if (s_zero_left > 0) {
        s_zero_left--;
        return CMD_NONE;
    }
    uint8_t ev;
    if (xQueueReceive(s_cmd_q, &ev, 0) == pdTRUE) {
        s_burst_cmd = ev;
        s_burst_left = CMD_BURST_CYCLES - 1;
        if (s_burst_left == 0) {
            s_zero_left = CMD_ZERO_CYCLES;
        }
        portENTER_CRITICAL(&s_mux);
        s_st.last_cmd = ev;
        portEXIT_CRITICAL(&s_mux);
        return ev;
    }
    return CMD_NONE;
}

static void ring_broadcast_locked(void)
{
    uint8_t cmd = cmd_next();
    const uint8_t pkt[PKT_LEN] = { s_node_id, (uint8_t)(cmd << 4), 0x00 };

    int64_t t = esp_timer_get_time();
    (void)lora_send(s_radio, pkt, PKT_LEN, 0);
    s_cycle++;

    portENTER_CRITICAL(&s_mux);
    if (s_last_tx_us != 0) {
        s_st.cycle_us = (uint32_t)(t - s_last_tx_us);
    }
    portEXIT_CRITICAL(&s_mux);
    s_last_tx_us = t;
}

static void on_rx(lora_t *radio, const lora_packet_t *p, void *ctx)
{
    (void)radio;
    (void)ctx;
    if (!p->crc_ok || p->len != PKT_LEN) {
        return;
    }
    const uint8_t id = p->data[0], b1 = p->data[1], b2 = p->data[2];
    if (id == s_node_id) {
        return;
    }

    portENTER_CRITICAL(&s_mux);
    switch (id) {
    case NODE_PANTRY:
        s_st.red = b1 >> 4;
        s_st.green = b1 & 0x0F;
        s_st.pantry_land = b2 >> 4;
        break;
    case NODE_ICE:
        s_st.ball = b1 >> 4;
        s_st.ice_land = b1 & 0x0F;
        break;
    case NODE_OVEN:
        s_st.cube = b1 >> 4;
        s_st.oven_land = b1 & 0x0F;
        break;
    case NODE_DRONE:
        s_st.drone = b1 >> 4;
        s_st.payload = (uint16_t)(((b1 & 0x0F) << 8) | b2);
        s_st.streak = 0;
        s_st.link_ok = true;
        break;
    default:
        break;
    }
    portEXIT_CRITICAL(&s_mux);

    if (id != NODE_DRONE) {
        return;
    }

    xSemaphoreTake(s_tx_mtx, portMAX_DELAY);
    ring_broadcast_locked();
    xSemaphoreGive(s_tx_mtx);
    if (s_ring_task) {
        xTaskNotifyGive(s_ring_task);
    }
}

static void ring_task(void *arg)
{
    (void)arg;

    xSemaphoreTake(s_tx_mtx, portMAX_DELAY);
    ring_broadcast_locked();
    uint32_t seen = s_cycle;
    xSemaphoreGive(s_tx_mtx);

    for (;;) {
        if (ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(ring_timeout_ms())) > 0) {
            xSemaphoreTake(s_tx_mtx, portMAX_DELAY);
            seen = s_cycle;
            xSemaphoreGive(s_tx_mtx);
            continue;
        }
        xSemaphoreTake(s_tx_mtx, portMAX_DELAY);
        if (s_cycle == seen) {
            portENTER_CRITICAL(&s_mux);
            s_st.timeouts++;
            if (++s_st.streak >= LINK_LOST_STREAK) {
                s_st.link_ok = false;
            }
            portEXIT_CRITICAL(&s_mux);
            ring_broadcast_locked();
        }
        seen = s_cycle;
        xSemaphoreGive(s_tx_mtx);
    }
}

static esp_err_t radio_init(void)
{
    lora_config_t cfg = LORA_CONFIG_DEFAULT(PIN_LORA_MOSI, PIN_LORA_MISO, PIN_LORA_SCK,
                                            PIN_LORA_NSS, PIN_LORA_RST, PIN_LORA_DIO0);
    cfg.frequency_hz = LORA_FREQ_HZ;
    cfg.sf = LORA_SF6;
    cfg.bw = LORA_BW_500K;
    cfg.cr = LORA_CR_4_5;
    cfg.preamble_len = 6;
    cfg.crc = true;
    cfg.header_mode = LORA_HEADER_IMPLICIT;
    cfg.implicit_payload_len = PKT_LEN;
    cfg.ldro = LORA_LDRO_OFF;

    esp_err_t err = lora_begin(&cfg, &s_radio);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "lora_begin gagal: %s", esp_err_to_name(err));
        return err;
    }

    ESP_LOGI(TAG, "simbol %.1f us (PDF 128.0), airtime %lu ms (PDF 3)",
             lora_get_symbol_time_us(s_radio),
             (unsigned long)lora_get_airtime_ms(s_radio, PKT_LEN));

    const uint8_t r31 = lora_read_reg(s_radio, 0x31), r37 = lora_read_reg(s_radio, 0x37);
    const uint8_t r36 = lora_read_reg(s_radio, 0x36), r3a = lora_read_reg(s_radio, 0x3A);
#if LORA_FREQ_HZ <= 525000000UL
    const bool regs_ok = ((r31 & 0x07) == 0x05) && r37 == 0x0C && r36 == 0x02 && r3a == 0x7F;
#else
    const bool regs_ok = ((r31 & 0x07) == 0x05) && r37 == 0x0C && r36 == 0x02;
#endif
    if (regs_ok) {
        ESP_LOGI(TAG, "register SF6/BW500 OK (0x31=%02X 0x37=%02X 0x36=%02X 0x3A=%02X)", r31, r37, r36, r3a);
    } else {
        ESP_LOGW(TAG, "register SF6/BW500 TIDAK sesuai: 0x31=%02X 0x37=%02X 0x36=%02X 0x3A=%02X", r31, r37, r36, r3a);
    }
    return ESP_OK;
}

static void oled_bus_check(void)
{
    pinMode(OLED_SDA_PIN, INPUT_PULLUP);
    pinMode(OLED_SCL_PIN, INPUT_PULLUP);
    delay(5);

    if (digitalRead(OLED_SDA_PIN) == LOW && digitalRead(OLED_SCL_PIN) == HIGH) {
        ESP_LOGW(TAG, "I2C: SDA tertahan LOW (OLED macet di tengah transfer), memulihkan bus");
        pinMode(OLED_SCL_PIN, OUTPUT_OPEN_DRAIN);
        gpio_set_pull_mode((gpio_num_t)OLED_SCL_PIN, GPIO_PULLUP_ONLY);
        for (int i = 0; i < 9 && digitalRead(OLED_SDA_PIN) == LOW; i++) {
            digitalWrite(OLED_SCL_PIN, LOW);
            delayMicroseconds(10);
            digitalWrite(OLED_SCL_PIN, HIGH);
            delayMicroseconds(10);
        }
        pinMode(OLED_SDA_PIN, INPUT_PULLUP);
        pinMode(OLED_SCL_PIN, INPUT_PULLUP);
        delay(5);
        ESP_LOGI(TAG, "I2C pemulihan bus: SDA kini %d", digitalRead(OLED_SDA_PIN));
    }

    const int sda_pu = digitalRead(OLED_SDA_PIN);
    const int scl_pu = digitalRead(OLED_SCL_PIN);

    pinMode(OLED_SDA_PIN, INPUT_PULLDOWN);
    pinMode(OLED_SCL_PIN, INPUT_PULLDOWN);
    delay(5);
    const int sda_pd = digitalRead(OLED_SDA_PIN);
    const int scl_pd = digitalRead(OLED_SCL_PIN);

    pinMode(OLED_SDA_PIN, INPUT_PULLUP);
    pinMode(OLED_SCL_PIN, INPUT_PULLUP);

    ESP_LOGI(TAG, "I2C diam: pull-up SDA=%d SCL=%d | pull-down SDA=%d SCL=%d (normal: 1 1 | 1 1)",
             sda_pu, scl_pu, sda_pd, scl_pd);

    if (!sda_pu || !scl_pu) {
        ESP_LOGW(TAG, "I2C: jalur %s%s masih tertahan LOW (korsleting, OLED tidak berdaya, atau urutan pin salah)",
                 sda_pu ? "" : "SDA ", scl_pu ? "" : "SCL");
    } else if (!sda_pd || !scl_pd) {
        ESP_LOGW(TAG, "I2C: tidak ada pull-up eksternal di jalur %s%s, tambahkan 4.7k ke 3V3",
                 sda_pd ? "" : "SDA ", scl_pd ? "" : "SCL");
    } else {
        ESP_LOGI(TAG, "I2C: pull-up eksternal terdeteksi di SDA dan SCL");
    }
}

static void display_task(void *arg)
{
    (void)arg;
    for (;;) {
        ring_state_t s;
        portENTER_CRITICAL(&s_mux);
        s = s_st;
        portEXIT_CRITICAL(&s_mux);

        oled_printf(0, 0, "DRONE:%X LINK:%s", s.drone, s.link_ok ? "OK" : "LOST");
        oled_printf(0, 1, "CMD:%s", cmd_name(s.last_cmd));
        oled_printf(0, 2, "PANTRY R:%X G:%X L:%X", s.red, s.green, s.pantry_land);
        oled_printf(0, 3, "ICE    BALL:%X L:%X", s.ball, s.ice_land);
        oled_printf(0, 4, "OVEN   CUBE:%X L:%X", s.cube, s.oven_land);
        oled_printf(0, 5, "PAYLOAD:%03X", s.payload);
        oled_printf(0, 6, "CYCLE:%lu US", (unsigned long)s.cycle_us);
        oled_printf(0, 7, "TIMEOUT:%lu", (unsigned long)s.timeouts);
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

void app_main(void)
{
    s_cmd_q = xQueueCreate(4, sizeof(uint8_t));
    s_tx_mtx = xSemaphoreCreateMutex();
    configASSERT(s_cmd_q && s_tx_mtx);

    oled_bus_check();
    oled_init();
    oled_clear();
    oled_printf(0, 0, "MAIN CONTROL");
    oled_printf(0, 1, "LORA INIT");

    ESP_ERROR_CHECK(radio_init());
    ESP_ERROR_CHECK(lora_start_receive(s_radio, on_rx, NULL));
    ESP_LOGI(TAG, "node %u, hop %lu ms, timeout %lu ms", s_node_id,
             (unsigned long)s_hop_ms, (unsigned long)ring_timeout_ms());

    xTaskCreate(pedal_task,   "pedal",   3072, NULL, 4,  NULL);
    xTaskCreate(display_task, "display", 3072, NULL, 2,  NULL);
    xTaskCreate(ring_task,    "ring",    3072, NULL, 10, &s_ring_task);
}