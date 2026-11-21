#include <stdbool.h>
#include <stdint.h>
#include <stddef.h>

#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "driver/rmt_tx.h"
#include "driver/rmt_encoder.h"
#include "hal/rmt_types.h"

#include "easy_gpio.h"
#include "lora.h"
#include "oled.h"

#if CONFIG_FREERTOS_HZ < 1000
#error "Set CONFIG_FREERTOS_HZ=1000 (menuconfig > Component config > FreeRTOS > Kernel > configTICK_RATE_HZ)"
#endif

static const char *TAG = "MAINCTRL";

#define PEDAL_LEFT_PIN     20
#define PEDAL_RIGHT_PIN    21
#define PEDAL_ACTIVE_LEVEL LOW

#define PIN_LORA_MOSI      6
#define PIN_LORA_MISO      5
#define PIN_LORA_SCK       4
#define PIN_LORA_NSS       7
#define PIN_LORA_RST       10
#define PIN_LORA_DIO0      3

#define OLED_SDA_PIN       0
#define OLED_SCL_PIN       1

#define PIN_PPM_OUT        GPIO_NUM_2
#define PPM_RESOLUTION_HZ  1000000         
#define PPM_FRAME_US       22500
#define PPM_PULSE_US       300
#define PPM_PULSE_LEVEL    1             
#define PPM_IDLE_LEVEL     0
#define PPM_CH_NUM         8

uint16_t PPM_S_Drone     = 1000;
uint16_t PPM_S_Cam       = 1000;

#define BUZZER_PIN         8
#define BUZZER_ACTIVE_LEVEL HIGH
#define BUZZER_BEEP_MS     55
#define BUZZER_GAP_MS      90

#define LORA_FREQ_HZ       433000000

#define PEDAL_POLL_MS      5
#define PEDAL_DEBOUNCE_N   3
#define PEDAL_CHORD_MS     300
#define PEDAL_TAP_GAP_MS   400

#define CMD_BURST_CYCLES   3
#define CMD_ZERO_CYCLES    1

#define RING_WAIT_HOPS     4
#define RING_MARGIN_HOPS   2
#define LINK_LOST_STREAK   10

#define PKT_LEN 3
_Static_assert(PKT_LEN == 3, "paket harus 3 byte");

enum {
    NODE_MAIN   = 1,
    NODE_PANTRY = 2,
    NODE_ICE    = 3,
    NODE_OVEN   = 4,
    NODE_DRONE  = 5
};

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
    case CMD_MAIN1:    return "PICK/DROP G/BALL";
    case CMD_MAIN2:    return "PICK/DROP R/CUBE";
    case CMD_OPT1:     return "PICK ONE BALL";
    case CMD_OPT2:     return "PICK ONE CUBE";
    case CMD_SW_DRONE: return "SWITCH DRONE";
    case CMD_SW_CAM:   return "SWITCH CAM";
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
static QueueHandle_t s_buzzer_q;

typedef enum {
    BUZZER_EVENT_CMD = 1,
    BUZZER_EVENT_SWITCH_DRONE,
    BUZZER_EVENT_SWITCH_CAM
} buzzer_event_t;

typedef struct {
    uint16_t hidup_ms;
    uint16_t jeda_ms;
} buzzer_step_t;

static const buzzer_step_t s_startup_pattern[] = {
    {110, 70}, {110, 90}, {170, 70}, {80, 55},
    {80, 80}, {150, 65}, {220, 0}
};

static void buzzer_play_step(uint32_t hidup_ms, uint32_t jeda_ms)
{
    digitalWrite(BUZZER_PIN, BUZZER_ACTIVE_LEVEL);
    vTaskDelay(pdMS_TO_TICKS(hidup_ms));
    digitalWrite(BUZZER_PIN, !BUZZER_ACTIVE_LEVEL);
    if (jeda_ms > 0) {
        vTaskDelay(pdMS_TO_TICKS(jeda_ms));
    }
}

static void buzzer_task(void *arg)
{
    (void)arg;

    for (size_t i = 0; i < sizeof(s_startup_pattern) / sizeof(s_startup_pattern[0]); i++) {
        buzzer_play_step(s_startup_pattern[i].hidup_ms, s_startup_pattern[i].jeda_ms);
    }

    for (;;) {
        buzzer_event_t event;
        if (xQueueReceive(s_buzzer_q, &event, portMAX_DELAY) != pdTRUE) {
            continue;
        }

        int jumlah_beep = 1;
        if (event == BUZZER_EVENT_SWITCH_DRONE) jumlah_beep = 2;
        else if (event == BUZZER_EVENT_SWITCH_CAM) jumlah_beep = 3;

        for (int i = 0; i < jumlah_beep; i++) {
            buzzer_play_step(BUZZER_BEEP_MS, i + 1 < jumlah_beep ? BUZZER_GAP_MS : 0);
        }
    }
}

static void gesture_emit(uint8_t mask, uint8_t n);

typedef struct {
    bool     down[2];
    uint8_t  cnt[2];
    bool     press_active;
    bool     press_invalid;
    uint8_t  press_mask;
    uint32_t press_started_ms;
    uint8_t  seq_mask;
    uint8_t  seq_n;
    uint32_t seq_released_ms;
} gesture_t;

static void gesture_emit(uint8_t mask, uint8_t n)
{
    uint8_t cmd = CMD_NONE;
    if      (mask == 0x02 && n == 1) cmd = CMD_MAIN1;
    else if (mask == 0x01 && n == 1) cmd = CMD_MAIN2;
    else if (mask == 0x02 && n == 2) cmd = CMD_OPT1;
    else if (mask == 0x01 && n == 2) cmd = CMD_OPT2;
    else if (mask == 0x03 && n == 2) cmd = CMD_SW_DRONE;
    else if (mask == 0x03 && n == 1) cmd = CMD_SW_CAM;

    if (cmd == CMD_NONE) return;

    if (xQueueSend(s_cmd_q, &cmd, 0) != pdTRUE) {
        ESP_LOGW(TAG, "Antrean perintah penuh, CMD 0x%02X tidak tersimpan", (unsigned)cmd);
        return;
    }

    if (cmd == CMD_SW_CAM) PPM_S_Cam = (PPM_S_Cam == 2000) ? 1000 : 2000;
    if (cmd == CMD_SW_DRONE)   PPM_S_Drone   = (PPM_S_Drone   == 1000) ? 1500 : (PPM_S_Drone == 1500) ? 2000 : 1000;

    buzzer_event_t bunyi = BUZZER_EVENT_CMD;
    if (cmd == CMD_SW_DRONE) bunyi = BUZZER_EVENT_SWITCH_DRONE;
    else if (cmd == CMD_SW_CAM) bunyi = BUZZER_EVENT_SWITCH_CAM;
    (void)xQueueSend(s_buzzer_q, &bunyi, 0);
}

static void gesture_finish_tap(gesture_t *g, uint32_t now)
{
    const uint8_t mask = g->press_mask;
    const uint32_t started_ms = g->press_started_ms;
    const bool valid = !g->press_invalid;

    g->press_active = false;
    g->press_invalid = false;
    g->press_mask = 0;

    if (!valid) {
        return;
    }

    if (g->seq_n) {
        const bool in_gap = (uint32_t)(started_ms - g->seq_released_ms) <= PEDAL_TAP_GAP_MS;
        if (in_gap && g->seq_mask == mask) {
            gesture_emit(mask, 2);
            g->seq_n = 0;
            return;
        }
        
        gesture_emit(g->seq_mask, 1);
        g->seq_n = 0;
    }

    g->seq_mask = mask;
    g->seq_n = 1;
    g->seq_released_ms = now;
}

static void gesture_step(gesture_t *g, uint32_t now, const bool raw[2])
{
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

        if (raw[i]) {
            if (!g->press_active) {
                if (g->seq_n &&
                    (uint32_t)(now - g->seq_released_ms) > PEDAL_TAP_GAP_MS) {
                    gesture_emit(g->seq_mask, 1);
                    g->seq_n = 0;
                }
                g->press_active = true;
                g->press_invalid = false;
                g->press_mask = (uint8_t)(1u << i);
                g->press_started_ms = now;
            } else if (g->press_mask != 0x03) {
                const uint8_t other = (uint8_t)(1u << (1 - i));
                if ((g->press_mask & other) && g->down[1 - i] &&
                    (uint32_t)(now - g->press_started_ms) <= PEDAL_CHORD_MS) {
                    g->press_mask = 0x03;
                } else {
                    g->press_invalid = true;
                }
            }
        }
    }

    if (g->press_active && !g->down[0] && !g->down[1]) {
        gesture_finish_tap(g, now);
    }

    if (!g->press_active && g->seq_n &&
        (uint32_t)(now - g->seq_released_ms) >= PEDAL_TAP_GAP_MS) {
        gesture_emit(g->seq_mask, 1);
        g->seq_n = 0;
    }
}

static void pedal_task(void *arg)
{
    (void)arg;
    const uint8_t pins[2] = { PEDAL_LEFT_PIN, PEDAL_RIGHT_PIN };
    for (int i = 0; i < 2; i++) {
        ESP_ERROR_CHECK(pinMode(pins[i], INPUT_PULLUP));
    }

    gesture_t g = {0};
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
static bool              s_cmd_logged;
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
        s_cmd_logged = false;
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
    esp_err_t err = lora_send(s_radio, pkt, PKT_LEN, 0);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Gagal kirim LoRa: %s", esp_err_to_name(err));
    } else if (cmd != CMD_NONE && !s_cmd_logged) {
        ESP_LOGI(TAG, "KIRIM CMD 0x%02X: %s", (unsigned)cmd, cmd_name(cmd));
        ESP_LOGI(TAG, "PPM VALUE: %d, %d", PPM_S_Drone, PPM_S_Cam);
        s_cmd_logged = true;
    }
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
    cfg.tx_power_mw = 9;

    esp_err_t err = lora_begin(&cfg, &s_radio);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "lora_begin gagal: %s", esp_err_to_name(err));
        return err;
    }

    return ESP_OK;
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
        oled_printf(0, 2, "----- DATA PRS -----");
        oled_printf(0, 3, "PANTRY R:%X G:%X L:%X", s.red, s.green, s.pantry_land);
        oled_printf(0, 4, "ICE B:%X/%X OVEN C:%X/%X",
                    s.ball, s.ice_land, s.cube, s.oven_land);
        oled_printf(0, 5, "----- DATA RING -----");
        oled_printf(0, 6, "PAY:%03X CYC:%luUS", s.payload, (unsigned long)s.cycle_us);
        oled_printf(0, 7, "TIMEOUT:%lu", (unsigned long)s.timeouts);
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

static inline int ppmClamp(int v) {
    if (v < 1000) return 1000;
    if (v > 2000) return 2000;
    return v;
}

static void ppmOut(void *arg) {
    rmt_channel_handle_t tx_chan = NULL;
    rmt_encoder_handle_t enc = NULL;

    rmt_tx_channel_config_t tx_cfg = {
        .gpio_num = PIN_PPM_OUT,
        .clk_src = RMT_CLK_SRC_DEFAULT,
        .resolution_hz = PPM_RESOLUTION_HZ,
        .mem_block_symbols = SOC_RMT_MEM_WORDS_PER_CHANNEL,
        .trans_queue_depth = 4,
        .flags.init_level = PPM_IDLE_LEVEL,
    };
    ESP_ERROR_CHECK(rmt_new_tx_channel(&tx_cfg, &tx_chan));

    rmt_copy_encoder_config_t enc_cfg = {};
    ESP_ERROR_CHECK(rmt_new_copy_encoder(&enc_cfg, &enc));
    ESP_ERROR_CHECK(rmt_enable(tx_chan));

    rmt_transmit_config_t trans_cfg = {
        .loop_count = 0,
        .flags.eot_level = PPM_IDLE_LEVEL,
    };

    rmt_symbol_word_t sym[PPM_CH_NUM + 1];   

    while (1) {
        int ch[PPM_CH_NUM] = { ppmClamp(PPM_S_Drone), ppmClamp(PPM_S_Cam),
                               1500, 1500, 1500, 1500, 1500, 1500 };
        int total = 0;

        for (int i = 0; i < PPM_CH_NUM; i++) {
            sym[i].level0    = PPM_PULSE_LEVEL;
            sym[i].duration0 = PPM_PULSE_US;
            sym[i].level1    = PPM_IDLE_LEVEL;
            sym[i].duration1 = ch[i] - PPM_PULSE_US;
            total += ch[i];
        }

        sym[PPM_CH_NUM].level0    = PPM_PULSE_LEVEL;
        sym[PPM_CH_NUM].duration0 = PPM_PULSE_US;
        sym[PPM_CH_NUM].level1    = PPM_IDLE_LEVEL;
        sym[PPM_CH_NUM].duration1 = PPM_FRAME_US - total - PPM_PULSE_US;

        ESP_ERROR_CHECK(rmt_transmit(tx_chan, enc, sym, sizeof(sym), &trans_cfg));
        ESP_ERROR_CHECK(rmt_tx_wait_all_done(tx_chan, portMAX_DELAY));
    }
}

void app_main(void)
{
    esp_log_level_set("lora", ESP_LOG_WARN);
    esp_log_level_set("oled", ESP_LOG_WARN);

    s_cmd_q = xQueueCreate(4, sizeof(uint8_t));
    s_buzzer_q = xQueueCreate(8, sizeof(buzzer_event_t));
    s_tx_mtx = xSemaphoreCreateMutex();
    configASSERT(s_cmd_q && s_buzzer_q && s_tx_mtx);

    ESP_ERROR_CHECK(pinMode(BUZZER_PIN, OUTPUT));
    digitalWrite(BUZZER_PIN, !BUZZER_ACTIVE_LEVEL);
    BaseType_t buzzer_ok = xTaskCreate(buzzer_task, "buzzer", 2048, NULL, 2, NULL);
    configASSERT(buzzer_ok == pdPASS);

    oled_init();
    oled_clear();
    oled_printf(0, 0, "MAIN CONTROL");
    oled_printf(0, 1, "LORA INIT");

    ESP_ERROR_CHECK(radio_init());
    ESP_ERROR_CHECK(lora_start_receive(s_radio, on_rx, NULL));
    xTaskCreate(pedal_task,   "pedal",   3072, NULL, 4,  NULL);
    xTaskCreate(display_task, "display", 3072, NULL, 2,  NULL);
    xTaskCreate(ring_task,    "ring",    3072, NULL, 10, &s_ring_task);
    xTaskCreate(ppmOut, "ppmOut", 3072, NULL, 5, NULL);
}