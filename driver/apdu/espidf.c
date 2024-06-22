#include "espidf.h"

#include <arpa/inet.h>
#include <driver/gpio.h>
#include <driver/uart.h>
#include <esp_log.h>
#include <euicc/interface.h>
#include <freertos/FreeRTOS.h>
#include <stdio.h>
#include <string.h>

#define SIM_ON_PIN GPIO_NUM_13
#define RESET_PIN GPIO_NUM_10
#define RX_PIN GPIO_NUM_12
#define TX_PIN GPIO_NUM_11
#define TAG "apdu_driver_espidf"

#define EUICC_INTERFACE_BUFSZ 264

#define APDU_TERMINAL_CAPABILITIES \
    "\x80\xAA\x00\x00\x0A\xA9\x08\x81\x00\x82\x01\x01\x83\x01\x07"
#define APDU_OPENLOGICCHANNEL "\x00\x70\x00\x00\x01"
#define APDU_CLOSELOGICCHANNEL "\x00\x70\x80\xFF\x00"
#define APDU_SELECT_HEADER "\x00\xA4\x04\x00\xFF"

static void _reset_card() {
    // First deactivation
    gpio_set_level(RESET_PIN, 0);
    gpio_set_level(SIM_ON_PIN, 0);
    vTaskDelay(100 / portTICK_PERIOD_MS);

    // Cold reset
    gpio_set_level(SIM_ON_PIN, 1);
    vTaskDelay(10 / portTICK_PERIOD_MS);
    uart_flush(UART_NUM_1);
    gpio_set_level(RESET_PIN, 1);
}

static int _send_and_clear_echo(const uint8_t *tx, uint32_t tx_len) {
    if (uart_write_bytes(UART_NUM_1, tx, tx_len) != tx_len) {
        ESP_LOGE(TAG, "Fail to transmit data");
        return -1;
    }
    uint8_t rx_buf[EUICC_INTERFACE_BUFSZ] = {0};
    if (uart_read_bytes(UART_NUM_1, rx_buf, tx_len, 500 / portTICK_PERIOD_MS) !=
        tx_len) {
        size_t s;
        uart_get_buffered_data_len(UART_NUM_1, &s);
        ESP_LOGE(TAG, "Fail to clear echo, TX len: %ld, RX len: %d", tx_len, s);
        return -1;
    }
    if (strncmp((char *)rx_buf, (char *)tx, tx_len) != 0) {
        ESP_LOGE(TAG, "Echo mismatch");
        for (int i = 0; i < tx_len; ++i) {
            ESP_LOGE(TAG, "TX%d: 0x%02X RX: 0x%02X", i, tx[i], rx_buf[i]);
        }
        return -1;
    }
    return 0;
}

static int _transmit_raw(uint8_t *rx, uint32_t *rx_len, const uint8_t *tx,
                         const uint8_t tx_len) {
    if (tx_len < 5) {
        ESP_LOGE(TAG, "Invalid APDU command");
        return -1;
    }
    ESP_LOGD(
        TAG,
        "Sending APDU command header: %02X %02X %02X %02X %02X, Tx len: %d",
        tx[0], tx[1], tx[2], tx[3], tx[4], tx_len);
    if (_send_and_clear_echo(tx, 5) != 0) {
        ESP_LOGE(TAG, "Fail to transmit APDU command header");
        return -1;
    }
    uint8_t ack;
    while (1) {
        if (uart_read_bytes(UART_NUM_1, &ack, 1, 3000 / portTICK_PERIOD_MS) !=
            1) {
            ESP_LOGE(TAG, "No ack");
            return -1;
        }
        if (ack == tx[1]) {
            ESP_LOGD(TAG, "Received ack");
            break;
        }
        if (ack != 0x60) {
            uint8_t sw1 = ack;
            uint8_t sw2, nil;
            if (uart_read_bytes(UART_NUM_1, &sw2, 1,
                                500 / portTICK_PERIOD_MS) != 1) {
                ESP_LOGE(TAG, "No sw2");
                return -1;
            }
            if (uart_read_bytes(UART_NUM_1, &nil, 1,
                                500 / portTICK_PERIOD_MS) == 1) {
                ESP_LOGE(TAG, "Expected nothing received");
                return -1;
            }
            *rx_len = 2;
            rx[0] = sw1;
            rx[1] = sw2;
            ESP_LOGD(TAG, "Received SW1 SW2: 0x%02X 0x%02X", sw1, sw2);
            return 0;
        }
    }
    if (tx_len > 5) {
        if (_send_and_clear_echo(tx + 5, tx_len - 5) != 0) {
            ESP_LOGE(TAG, "Fail to transmit APDU command data");
            return -1;
        }
    }
    int to_recv = tx[4] + 7 - tx_len;

    // Extend
    if (tx[4] == 0 && tx_len > 5) {
        // HEADER | LE
        if (tx_len == 7) {
            uint16_t le = ntohs(*(uint16_t *)(tx + 5));
            if (le == 0) {
                to_recv = 65535;
            } else {
                to_recv = le + 2;
            }
        } else {
            uint16_t body_len = tx_len - 4;
            uint16_t lc = ntohs(*(uint16_t *)(tx + 5));
            if (lc != body_len - 3 && lc != body_len - 5) {
                to_recv = 65535;
            } else if (tx_len == 7 + lc) {
                // HEADER | LC | DATA
                to_recv = 2;
            } else {
                to_recv = ntohs(*(uint16_t *)(tx + tx_len - 2));
                if (to_recv == 0) {
                    to_recv = 65535;
                } else {
                    to_recv += 2;
                }
            }
        }
    } else if (tx_len == 5) {
        // HEADER | LE
        to_recv = tx[4];
        if (to_recv == 0) {
            to_recv = 65535;
        } else {
            to_recv += 2;
        }
    } else {
        int body_len = tx_len - 4;
        int lc = tx[4];
        if (lc != body_len - 1 && lc != body_len - 2) {
            to_recv = 65535;
        } else if (tx_len == 5 + lc) {
            // HEADER | LC | DATA
            to_recv = 2;
        } else {
            to_recv = tx[tx_len - 1];
            if (to_recv == 0) {
                to_recv = 65535;
            } else {
                if (tx_len == 5 + lc) {
                    // HEADER | LC | DATA
                    to_recv = 2;
                } else {
                    to_recv = tx[tx_len - 1];
                    if (to_recv == 0) {
                        to_recv = 65535;
                    } else {
                        to_recv += 2;
                    }
                }
            }
        }
    }

    ESP_LOGD(TAG, "To receive: %d", to_recv);
    *rx_len = 0;
    int has_recv = 0;
    int wait_time = 0;
    while (*rx_len < to_recv) {
        if (!has_recv) {
            wait_time = 5000;
        } else {
            wait_time = 300;
        }
        if (uart_read_bytes(UART_NUM_1, rx + (*rx_len), 1,
                            wait_time / portTICK_PERIOD_MS) != 1) {
            break;
        }
        if (to_recv == 2 && rx[*rx_len] == 0x60) {
            continue;
        }
        has_recv = 1;
        ++(*rx_len);
    }
    if (*rx_len != to_recv && to_recv != 65535) {
        ESP_LOGE(TAG, "Response invalid, received %ld bytes", *rx_len);
        return -1;
    }
    size_t s;
    uart_get_buffered_data_len(UART_NUM_1, &s);
    if (s > 0) {
        ESP_LOGE(TAG, "Extra data to read: %d", s);
    }
    return 0;
}

static int _wait_for_card() {
    int retry = 0;
    while (retry++ < 1) {
        uint8_t data;
        if (uart_read_bytes(UART_NUM_1, &data, 1, 1000 / portTICK_PERIOD_MS) ==
            1) {
            ESP_LOGD(TAG, "Response: 0x%02X", data);
            if (data != 0x3B) {
                size_t s;
                uart_get_buffered_data_len(UART_NUM_1, &s);
                ESP_LOGE(TAG, "Invalid response: 0x%02X, rest length: %d", data,
                         s);
                return -1;
            }
            while (data == 0x3B) {
                if (uart_read_bytes(UART_NUM_1, &data, 1,
                                    500 / portTICK_PERIOD_MS) != 1) {
                    ESP_LOGE(TAG, "No response T0");
                    return -1;
                }
                ESP_LOGD(TAG, "Response: 0x%02X", data);
                if (data != 0x3B) break;
            }
            uint8_t t0 = data;
            ESP_LOGD(TAG, "T0: 0x%02X", t0);
            for (uint8_t i = 0; i < 4; ++i) {
                if (t0 & (0x10 << i)) {
                    if (uart_read_bytes(UART_NUM_1, &data, 1,
                                        500 / portTICK_PERIOD_MS) != 1) {
                        ESP_LOGE(TAG, "No response 1");
                        return -1;
                    }
                    ESP_LOGD(TAG, "Ti%d: 0x%02X", i, data);
                }
            }
            for (uint8_t i = 0; i < (t0 & 0xF); ++i) {
                if (uart_read_bytes(UART_NUM_1, &data, 1,
                                    500 / portTICK_PERIOD_MS) != 1) {
                    ESP_LOGE(TAG, "No response 2");
                    return -1;
                }
                ESP_LOGD(TAG, "Historical%d: 0x%02X", i, data);
            }
            vTaskDelay(500 / portTICK_PERIOD_MS);
            size_t s;
            uart_get_buffered_data_len(UART_NUM_1, &s);
            while (s) {
                uart_read_bytes(UART_NUM_1, &data, 1, 500 / portTICK_PERIOD_MS);
                ESP_LOGD(TAG, "Extra: 0x%02X", data);
                --s;
            }
            return 0;
        }
    }
    return -1;
}

static int apdu_interface_connect(struct euicc_ctx *ctx) {
    int retry = 0;
    while (retry++ < 1) {
        _reset_card();
        if (_wait_for_card() == 0) {
            uint8_t rx_buf[EUICC_INTERFACE_BUFSZ];
            uint32_t rx_len;
            ESP_LOGD(TAG, "Card online, sending terminal capabilities");
            return _transmit_raw(rx_buf, &rx_len,
                                 (const uint8_t *)APDU_TERMINAL_CAPABILITIES,
                                 sizeof(APDU_TERMINAL_CAPABILITIES) - 1);
        }
    }
    ESP_LOGE(TAG, "Timeout wait for card ATR");
    return -1;
}

static void apdu_interface_disconnect(struct euicc_ctx *ctx) {
    gpio_set_level(RESET_PIN, 0);
    gpio_set_level(SIM_ON_PIN, 0);
}

static int apdu_interface_logic_channel_open(struct euicc_ctx *ctx,
                                             const uint8_t *aid,
                                             uint8_t aid_len) {
    uint8_t rx_buf[EUICC_INTERFACE_BUFSZ];
    uint32_t rx_len;
    uint8_t channel = 0;
    ESP_LOGD(TAG, "Opening logic channel");
    if (_transmit_raw(rx_buf, &rx_len, (const uint8_t *)APDU_OPENLOGICCHANNEL,
                      sizeof(APDU_OPENLOGICCHANNEL) - 1) != 0) {
        ESP_LOGE(TAG, "Fail to open logic channel");
        return -1;
    }
    if (rx_len != 3) {
        ESP_LOGE(TAG, "Fail to open logic channel, rx_len != 3");
        return -1;
    }
    if ((rx_buf[1] & 0xF0) != 0x90) {
        ESP_LOGE(TAG, "Fail to open logic channel, checksum != 0x90");
        return -1;
    }
    channel = rx_buf[0];

    uint32_t tx_len = sizeof(APDU_SELECT_HEADER) + aid_len - 1;
    uint8_t tx_buf[tx_len];
    memcpy(tx_buf, APDU_SELECT_HEADER, sizeof(APDU_SELECT_HEADER) - 1);
    memcpy(tx_buf + sizeof(APDU_SELECT_HEADER) - 1, aid, aid_len);
    tx_buf[0] = (tx_buf[0] & 0xF0) | channel;
    tx_buf[4] = aid_len;
    if (_transmit_raw(rx_buf, &rx_len, tx_buf, tx_len) != 0) {
        ESP_LOGE(TAG, "Fail to select ISD-R");
        return -1;
    }
    if (rx_len < 2) {
        ESP_LOGE(TAG, "Select ISD-R response invalid");
        return -1;
    }
    switch (rx_buf[rx_len - 2]) {
        case 0x90:
        case 0x61:
            ESP_LOGD(TAG, "Get channel: %d", channel);
            return channel;
        default:
            ESP_LOGD(TAG, "Channel get failed, %d", rx_buf[rx_len - 2]);
            return -1;
    }
    return -1;
}

static void apdu_interface_logic_channel_close(struct euicc_ctx *ctx,
                                               uint8_t channel) {
    if (channel) {
        ESP_LOGD(TAG, "Closing channel %d", channel);
        uint8_t rx_buf[EUICC_INTERFACE_BUFSZ];
        uint32_t rx_len;
        _transmit_raw(rx_buf, &rx_len, (const uint8_t *)APDU_CLOSELOGICCHANNEL,
                      sizeof(APDU_CLOSELOGICCHANNEL) - 1);
    }
}

static int apdu_interface_transmit(struct euicc_ctx *ctx, uint8_t **rx,
                                   uint32_t *rx_len, const uint8_t *tx,
                                   uint32_t tx_len) {
    *rx = malloc(EUICC_INTERFACE_BUFSZ);
    if (!*rx) {
        ESP_LOGE(TAG, "Failed to malloc transmit memory");
        return -1;
    }
    *rx_len = EUICC_INTERFACE_BUFSZ;
    if (_transmit_raw(*rx, rx_len, tx, tx_len) != 0) {
        free(*rx);
        *rx_len = 0;
        return -1;
    }
    return 0;
}

static int libapduinterface_init(struct euicc_apdu_interface *ifstruct) {
    // GPIO configuration for reset card and sim on
    gpio_set_direction(SIM_ON_PIN, GPIO_MODE_OUTPUT);
    gpio_set_pull_mode(SIM_ON_PIN, GPIO_FLOATING);
    gpio_set_level(SIM_ON_PIN, 0);
    gpio_set_direction(RESET_PIN, GPIO_MODE_OUTPUT);
    gpio_set_pull_mode(RESET_PIN, GPIO_FLOATING);
    gpio_set_level(RESET_PIN, 0);
    gpio_set_direction(TX_PIN, GPIO_MODE_INPUT_OUTPUT_OD);

    // UART configuration
    ESP_LOGD(TAG, "Initializing UART");
    const uart_config_t uart_config = {.baud_rate = 9600,
                                       .data_bits = UART_DATA_8_BITS,
                                       .parity = UART_PARITY_EVEN,
                                       .stop_bits = UART_STOP_BITS_2,
                                       .flow_ctrl = UART_HW_FLOWCTRL_DISABLE};
    uart_param_config(UART_NUM_1, &uart_config);
    uart_set_pin(UART_NUM_1, TX_PIN, RX_PIN, UART_PIN_NO_CHANGE,
                 UART_PIN_NO_CHANGE);
    uart_driver_install(UART_NUM_1, EUICC_INTERFACE_BUFSZ * 2, 0, 0, NULL, 0);
    ESP_LOGD(TAG, "UART initialized");

    memset(ifstruct, 0, sizeof(struct euicc_apdu_interface));

    ifstruct->connect = apdu_interface_connect;
    ifstruct->disconnect = apdu_interface_disconnect;
    ifstruct->logic_channel_open = apdu_interface_logic_channel_open;
    ifstruct->logic_channel_close = apdu_interface_logic_channel_close;
    ifstruct->transmit = apdu_interface_transmit;
    return 0;
}

static int libapduinterface_main(int argc, char **argv) { return 0; }

static void libapduinterface_fini(void) {
    uart_driver_delete(UART_NUM_1);
    gpio_set_level(RESET_PIN, 0);
    gpio_set_level(SIM_ON_PIN, 0);
}

const struct euicc_driver driver_apdu_espidf = {
    .type = DRIVER_APDU,
    .name = "espidf",
    .init = (int (*)(void *))libapduinterface_init,
    .main = libapduinterface_main,
    .fini = libapduinterface_fini,
};