#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/uart.h"
#include "esp_log.h"
#include <string.h>
#include <stdint.h>

static const char *TAG = "mr24hpc";

// UART configuration
#define UART_NUM            UART_NUM_2   // koristim UART2
#define UART_RX_PIN         16           // ESP32 RX <-- MR24 TX
#define UART_TX_PIN         17           // ESP32 TX --> MR24 RX
#define UART_BUF_SIZE       1024

// Frame markers
#define FRAME_HDR0 0x53
#define FRAME_HDR1 0x59
#define FRAME_TAIL0 0x54
#define FRAME_TAIL1 0x43

// ================================ Helper functions =======================================

static uint32_t last_hb = 0;

void watchdog_task(void *p) {
    while (1) {
        if ((xTaskGetTickCount() - last_hb) > pdMS_TO_TICKS(3000)) {
            ESP_LOGE(TAG, "MR24 heartbeat LOST!\n");
        }
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

void dump_hex(const uint8_t *data, int len) {
    printf("DATA: ");
    for (int i = 0; i < len; i++) printf("%02X ", data[i]);
    printf("\n");
}

// ================================== MR24 handling =========================================

static uint8_t calc_checksum(const uint8_t *buf, int len) {
    uint32_t sum = 0;
    for (int i = 0; i < len; ++i) sum += buf[i];
    return (uint8_t)(sum & 0xFF);
}

static void uart_init_mr24(void) {
    const uart_config_t uart_config = {
        .baud_rate = 9600,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE
    };
    uart_param_config(UART_NUM, &uart_config);
    uart_set_pin(UART_NUM, UART_TX_PIN, UART_RX_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE); 
    // RX buffer (interrupt driven)
    uart_driver_install(UART_NUM, UART_BUF_SIZE * 2, 0, 0, NULL, 0);
}

static void mr24_task(void *arg) {
    uint8_t buf[UART_BUF_SIZE];
    uint8_t frame[512];
    int frame_pos = 0;
    bool in_frame = false;
    while (1) {
        int len = uart_read_bytes(UART_NUM, buf, sizeof(buf), pdMS_TO_TICKS(200)); //200ms čeka (timeout), nakon 200ms vraća 0
        if (len <= 0) {
            vTaskDelay(pdMS_TO_TICKS(50));
            continue;
        }
        for (int i = 0; i < len; ++i) {
            uint8_t b = buf[i];
            if (!in_frame) {
                // tražim header 0x53 0x59
                if (frame_pos == 0 && b == FRAME_HDR0) {
                    frame[frame_pos++] = b;
                } else if (frame_pos == 1) {
                    if (b == FRAME_HDR1) {
                        frame[frame_pos++] = b;
                        in_frame = true;
                    } else {
                        frame_pos = 0;
                        if (b == FRAME_HDR0) frame_pos = 1;
                    }
                } else {
                    frame_pos = 0;
                    if (b == FRAME_HDR0) frame_pos = 1;
                }
            } else {
                // akumiliramo dok ne vidimo tail 0x54 0x43;
                frame[frame_pos++] = b;
                // minimal frame length check (hdr(2)+ctrl(1)+cmd(1)+len(2)+checksum(1)+tail(2) => 9)
                if (frame_pos >= 9) {
                    // duljina podataka (u bajtovima) je na poziciji 4(length_H) i 5(length_L)
                    uint16_t data_len = ((uint16_t)frame[4] << 8) | frame[5]; //operacije nad bitovima, zato sto frame sprema 8-bitne blokove
                    int expected_len = 2 /*hdr*/ + 1 /*ctrl*/ + 1 /*cmd*/ + 2 /*len*/ + data_len + 1 /*chksum*/ + 2 /*tail*/;
                    if (frame_pos == expected_len) {
                        // verificiraj tail
                        if (frame[frame_pos-2] == FRAME_TAIL0 && frame[frame_pos-1] == FRAME_TAIL1) {
                            // verificiraj checksum:
                            // prema dokumentaciji ==> checksum = sum(frame header + control + command + length + data) lower 8 bits
                            uint8_t expected_chksum = frame[frame_pos - 3];
                            uint8_t calc = calc_checksum(frame, 2 + 1 + 1 + 2 + data_len);
                            if (calc == expected_chksum) {
                                // dobar frame — procesiraj
                                uint8_t control = frame[2];
                                uint8_t command = frame[3];
                                uint8_t *data = &frame[6]; // data starts at index 6
                                ESP_LOGI(TAG, "Got frame: ctrl=0x%02X cmd=0x%02X data_len=%u\n", control, command, data_len);
                                
                                // poziv za debugiranje - ispiši podatke
                                dump_hex(data, data_len);
                                
                                if (control == 0x01 && command == 0x01) {
                                    ESP_LOGI(TAG, "Heartbeat OK\n");
                                    last_hb = xTaskGetTickCount();
                                }

                            } else {
                                ESP_LOGW(TAG, "Checksum mismatch (calc=0x%02X expected=0x%02X)", calc, expected_chksum);
                            }
                        } else {
                            ESP_LOGW(TAG, "Invalid tail bytes.");
                        }
                        // reset za iduci frame
                        frame_pos = 0;
                        in_frame = false;
                    } else if (frame_pos > expected_len) {
                        // reset
                        frame_pos = 0;
                        in_frame = false;
                    }
                }
            }
        }
    }
}

// ==================================== Main app ============================================

void app_main(void) {
    ESP_LOGI(TAG, "Starting MR24 example");
    uart_init_mr24();
    xTaskCreate(mr24_task, "mr24_task", 4096, NULL, 10, NULL);
    xTaskCreate(watchdog_task, "wd_task", 2048, NULL, 5, NULL);
}
