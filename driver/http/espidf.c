#include "espidf.h"

#include <esp_http_client.h>
#include <esp_log.h>
#include <esp_spiffs.h>
#include <esp_tls.h>
#include <euicc/interface.h>
#include <inttypes.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/param.h>

#define TAG "http_driver_espidf"

static char *output_buffer = NULL;
static uint32_t output_len = 0;
static uint32_t receive_len = 0;
static unsigned char *certs_buf = NULL;
static size_t certs_buf_len = 0;

esp_err_t _http_event_handler(esp_http_client_event_t *evt) {
    switch (evt->event_id) {
        case HTTP_EVENT_ERROR:
            ESP_LOGD(TAG, "HTTP_EVENT_ERROR");
            break;
        case HTTP_EVENT_ON_CONNECTED:
            ESP_LOGD(TAG, "HTTP_EVENT_ON_CONNECTED");
            break;
        case HTTP_EVENT_HEADER_SENT:
            ESP_LOGD(TAG, "HTTP_EVENT_HEADER_SENT");
            break;
        case HTTP_EVENT_ON_HEADER:
            ESP_LOGD(TAG, "HTTP_EVENT_ON_HEADER, key=%s, value=%s",
                     evt->header_key, evt->header_value);
            break;
        case HTTP_EVENT_ON_DATA:
            ESP_LOGD(TAG, "HTTP_EVENT_ON_DATA, len=%d", evt->data_len);
            receive_len += evt->data_len;
            if (output_buffer == NULL) {
                output_buffer = (char *)malloc(evt->data_len + 1);
                output_len = 0;
                if (output_buffer == NULL) {
                    ESP_LOGE(TAG,
                             "Failed to allocate memory for output "
                             "buffer, length=%d",
                             evt->data_len);
                    return ESP_FAIL;
                }
            } else {
                output_buffer =
                    realloc(output_buffer, output_len + evt->data_len + 1);
                if (output_buffer == NULL) {
                    ESP_LOGE(TAG,
                             "Failed to reallocate memory for output "
                             "buffer, length=%ld",
                             output_len + evt->data_len + 1);
                    return ESP_FAIL;
                }
            }
            memcpy(output_buffer + output_len, evt->data, evt->data_len);
            output_len += evt->data_len;

            break;
        case HTTP_EVENT_ON_FINISH:
            ESP_LOGD(TAG, "HTTP_EVENT_ON_FINISH");
            break;
        case HTTP_EVENT_DISCONNECTED:
            ESP_LOGD(TAG, "HTTP_EVENT_DISCONNECTED");
            int mbedtls_err = 0;
            esp_err_t err = esp_tls_get_and_clear_last_error(
                (esp_tls_error_handle_t)evt->data, &mbedtls_err, NULL);
            if (err != 0 && err != ESP_ERR_ESP_TLS_TCP_CLOSED_FIN) {
                ESP_LOGE(TAG, "Last esp error code: 0x%x", err);
                ESP_LOGE(TAG, "Last mbedtls failure: 0x%x", mbedtls_err);
            }
            break;
        case HTTP_EVENT_REDIRECT:
            ESP_LOGD(TAG, "HTTP_EVENT_REDIRECT");
            esp_http_client_set_redirection(evt->client);
            break;
    }
    return ESP_OK;
}

static int transmit(struct euicc_ctx *ctx, const char *url, uint32_t *rcode,
                    uint8_t **rx, uint32_t *rx_len, const uint8_t *tx,
                    uint32_t tx_len, const char **h) {
    esp_http_client_config_t config = {0};
    config.url = url;
    config.event_handler = _http_event_handler;
    config.timeout_ms = 60000;
    if (certs_buf != NULL) {
        config.cert_pem = (char *)certs_buf;
        config.cert_len = certs_buf_len;
    }

    esp_http_client_handle_t client = esp_http_client_init(&config);

    if (tx != NULL) {
        esp_http_client_set_method(client, HTTP_METHOD_POST);
        esp_http_client_set_post_field(client, (const char *)tx, tx_len);
    }

    for (int i = 0; h[i] != NULL; i++) {
        const char *delimiter = strchr(h[i], ':');
        if (delimiter != NULL) {
            int key_length = delimiter - h[i];
            char key[key_length + 1];
            strncpy(key, h[i], key_length);
            key[key_length] = '\0';

            const char *value =
                delimiter + 2;  // 假设值与键之间有一个冒号和一个空格

            esp_http_client_set_header(client, key, value);
        }
    }

    output_buffer = NULL;
    output_len = 0;
    receive_len = 0;

    esp_err_t err = esp_http_client_perform(client);

    if (output_len != receive_len) {
        ESP_LOGE(TAG, "Data may be lost");
        err = ESP_FAIL;
        if (output_buffer != NULL) {
            free(output_buffer);
            output_buffer = NULL;
        }
        *rx = NULL;
        *rx_len = 0;
        *rcode = 0;
    } else if (err == ESP_OK) {
        *rcode = esp_http_client_get_status_code(client);
        ESP_LOGD(TAG, "HTTPS Status = %ld, content_length = %ld", *rcode,
                 output_len);
        if (output_len > 0 && output_buffer != NULL) {
            *rx_len = output_len + 1;
            *rx = (uint8_t *)output_buffer;
            (*rx)[output_len] = 0;
            ESP_LOGD(TAG, "Response: %s", *rx);
        } else {
            *rx_len = 0;
            *rx = NULL;
        }
    } else {
        ESP_LOGE(TAG, "Error perform http request %s", esp_err_to_name(err));
        *rcode = esp_http_client_get_status_code(client);
        *rx_len = 0;
        *rx = NULL;
        if (output_buffer != NULL) {
            free(output_buffer);
            output_buffer = NULL;
        }
    }
    output_len = 0;
    receive_len = 0;
    esp_http_client_cleanup(client);
    return (err == ESP_OK) ? 0 : -1;
}

static int espidf_httpinterface_init(struct euicc_http_interface *ifstruct) {
    memset(ifstruct, 0, sizeof(struct euicc_http_interface));
    ifstruct->transmit = transmit;

    esp_vfs_spiffs_conf_t conf = {.base_path = "/storage",
                                  .partition_label = NULL,
                                  .max_files = 2,
                                  .format_if_mount_failed = false};
    esp_err_t ret = esp_vfs_spiffs_register(&conf);

    if (ret == ESP_OK) {
        FILE *f = fopen("/storage/gsma_certs.pem", "r");
        if (f != NULL) {
            fseek(f, 0, SEEK_END);
            certs_buf_len = ftell(f);
            certs_buf = malloc(certs_buf_len + 1);
            fseek(f, 0, SEEK_SET);
            fread(certs_buf, 1, certs_buf_len, f);
            fclose(f);
            certs_buf[certs_buf_len] = 0;
            certs_buf_len += 1;
            ESP_LOGI(TAG, "GSMA CA certificates loaded, size=%d", certs_buf_len);
        }
    }
    return 0;
}

static int espidf_httpinterface_main(int argc, char **argv) { return 0; }

static void espidf_httpinterface_fini(void) {
    if (output_buffer != NULL) {
        free(output_buffer);
        output_buffer = NULL;
    }
    if (certs_buf != NULL) {
        free(certs_buf);
        certs_buf = NULL;
        esp_vfs_spiffs_unregister(NULL);
    }
}

const struct euicc_driver driver_http_espidf = {
    .type = DRIVER_HTTP,
    .name = "espidf",
    .init = (int (*)(void *))espidf_httpinterface_init,
    .main = espidf_httpinterface_main,
    .fini = espidf_httpinterface_fini,
};
