/* ESP HTTP Client Example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/

#include <string.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_system.h"
#include "nvs_flash.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "protocol_examples_common.h"
#include "esp_tls.h"
#if CONFIG_MBEDTLS_CERTIFICATE_BUNDLE
#include "esp_crt_bundle.h"
#endif

#include "esp_http_client.h"
#include "driver/gpio.h"
#include "driver/uart.h"
#include "dfu_file.h"
#include "slip.h"
#include "mem_buf.h"

#define MAX_HTTP_RECV_BUFFER 1024
#define MAX_HTTP_DAT_FILE_BUFFER 139
#define MAX_HTTP_BIN_FILE_BUFFER 2212

#define MTU_SIZE 129
#define MAX_ACTUAL_PAYLOAD (MTU_SIZE / 2 - 2) // 62

static const char *TAG = "main";

esp_err_t _http_event_handler(esp_http_client_event_t *evt)
{
    static char *output_buffer; // Buffer to store response of http request from event handler
    static int output_len;      // Stores number of bytes read
    switch (evt->event_id)
    {
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
        ESP_LOGD(TAG, "HTTP_EVENT_ON_HEADER, key=%s, value=%s", evt->header_key, evt->header_value);
        break;
    case HTTP_EVENT_ON_DATA:
        ESP_LOGD(TAG, "HTTP_EVENT_ON_DATA, len=%d", evt->data_len);
        /*
         *  Check for chunked encoding is added as the URL for chunked encoding used in this example returns binary data.
         *  However, event handler can also be used in case chunked encoding is used.
         */
        if (!esp_http_client_is_chunked_response(evt->client))
        {
            // If user_data buffer is configured, copy the response into the buffer
            if (evt->user_data)
            {
                memcpy(evt->user_data + output_len, evt->data, evt->data_len);
            }
            else
            {
                if (output_buffer == NULL)
                {
                    output_buffer = (char *)malloc(esp_http_client_get_content_length(evt->client));
                    output_len = 0;
                    if (output_buffer == NULL)
                    {
                        ESP_LOGE(TAG, "Failed to allocate memory for output buffer");
                        return ESP_FAIL;
                    }
                }
                memcpy(output_buffer + output_len, evt->data, evt->data_len);
            }
            output_len += evt->data_len;
        }

        break;
    case HTTP_EVENT_ON_FINISH:
        ESP_LOGD(TAG, "HTTP_EVENT_ON_FINISH");
        if (output_buffer != NULL)
        {
            // Response is accumulated in output_buffer. Uncomment the below line to print the accumulated response
            // ESP_LOG_BUFFER_HEX(TAG, output_buffer, output_len);
            free(output_buffer);
            output_buffer = NULL;
        }
        output_len = 0;
        break;
    case HTTP_EVENT_DISCONNECTED:
        ESP_LOGI(TAG, "HTTP_EVENT_DISCONNECTED");
        int mbedtls_err = 0;
        esp_err_t err = esp_tls_get_and_clear_last_error(evt->data, &mbedtls_err, NULL);
        if (err != 0)
        {
            ESP_LOGI(TAG, "Last esp error code: 0x%x", err);
            ESP_LOGI(TAG, "Last mbedtls failure: 0x%x", mbedtls_err);
        }
        if (output_buffer != NULL)
        {
            free(output_buffer);
            output_buffer = NULL;
        }
        output_len = 0;
        break;
    }
    return ESP_OK;
}

static void http_get_dat_file(void)
{
    char local_response_buffer[MAX_HTTP_DAT_FILE_BUFFER] = {0};
    /**
     * NOTE: All the configuration parameters for http_client must be spefied either in URL or as host and path parameters.
     * If host and path parameters are not set, query parameter will be ignored. In such cases,
     * query parameter should be specified in URL.
     *
     * If URL as well as host and path parameters are specified, values of host and path will be considered.
     */
    esp_http_client_config_t config = {
        .url = "http://192.168.1.12:8000/blinky.dat",
        .event_handler = _http_event_handler,
        .user_data = local_response_buffer, // Pass address of local buffer to get response
        .disable_auto_redirect = true,
    };

    esp_http_client_handle_t client = esp_http_client_init(&config);

    // GET
    esp_err_t err = esp_http_client_perform(client);
    if (err == ESP_OK)
    {
        ESP_LOGI(TAG, "HTTP GET Status = %d, content_length = %d",
                 esp_http_client_get_status_code(client),
                 esp_http_client_get_content_length(client));
    }
    else
    {
        ESP_LOGE(TAG, "HTTP GET request failed: %s", esp_err_to_name(err));
    }

    // ESP_LOG_BUFFER_HEX(TAG, local_response_buffer, strlen(local_response_buffer));
    ESP_LOGI(TAG, "================== .dat file ==================");
    ESP_LOG_BUFFER_HEX(TAG, local_response_buffer, MAX_HTTP_DAT_FILE_BUFFER);
    ESP_LOGI(TAG, "===============================================");
    ESP_LOGI(TAG, "Received .dat file");

    esp_http_client_cleanup(client);

    // uart start
    // int txBytes = uart_write_bytes(UART_NUM_1, local_response_buffer, MAX_HTTP_DAT_FILE_BUFFER);
    // ESP_LOGI(TAG, "UART wrote %d bytes", txBytes);
}
static void http_get_bin_file(void)
{
    char local_response_buffer[MAX_HTTP_BIN_FILE_BUFFER] = {0};
    /**
     * NOTE: All the configuration parameters for http_client must be spefied either in URL or as host and path parameters.
     * If host and path parameters are not set, query parameter will be ignored. In such cases,
     * query parameter should be specified in URL.
     *
     * If URL as well as host and path parameters are specified, values of host and path will be considered.
     */
    esp_http_client_config_t config = {
        .url = "http://192.168.1.12:8000/blinky.bin",
        .event_handler = _http_event_handler,
        .user_data = local_response_buffer, // Pass address of local buffer to get response
        .disable_auto_redirect = true,
    };

    esp_http_client_handle_t client = esp_http_client_init(&config);

    // GET
    esp_err_t err = esp_http_client_perform(client);
    if (err == ESP_OK)
    {
        ESP_LOGI(TAG, "HTTP GET Status = %d, content_length = %d",
                 esp_http_client_get_status_code(client),
                 esp_http_client_get_content_length(client));
    }
    else
    {
        ESP_LOGE(TAG, "HTTP GET request failed: %s", esp_err_to_name(err));
    }

    // ESP_LOG_BUFFER_HEX(TAG, local_response_buffer, strlen(local_response_buffer));
    ESP_LOGI(TAG, "================== .bin file ==================");
    ESP_LOG_BUFFER_HEX(TAG, local_response_buffer, MAX_HTTP_BIN_FILE_BUFFER);
    ESP_LOGI(TAG, "===============================================");
    // ESP_LOGI(TAG, "Received .dat file");

    esp_http_client_cleanup(client);
    // uart start
    // int txBytes = uart_write_bytes(UART_NUM_1, local_response_buffer, MAX_HTTP_BIN_FILE_BUFFER);
    // ESP_LOGI(TAG, "UART wrote %d bytes", txBytes);
}

#define TXD_PIN (GPIO_NUM_22)
#define RXD_PIN (GPIO_NUM_21)

static const int RX_BUF_SIZE = 1024;
static const uint16_t PREAMBLE_MTU_SIZE = 200;
static MemBuf *txBuf;

static void uart_init(void)
{
    const uart_config_t uart_config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_APB,
    };
    // We won't use a buffer for sending data.
    uart_driver_install(UART_NUM_1, RX_BUF_SIZE * 2, 0, 0, NULL, 0);
    uart_param_config(UART_NUM_1, &uart_config);
    uart_set_pin(UART_NUM_1, TXD_PIN, RXD_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    ESP_LOGI(TAG, "Initialized UART");
}

/// @brief send_image_packet()
/// @param 
static void send_image_packet(void)
{
    // ESP_LOGI(TAG, "Size of binFile: %d", sizeof(bin_file));

    static uint8_t payload[6] = {0};
    static const uint8_t opcode = 0x08;
    uint32_t binFileLength = sizeof(bin_file);
    uint32_t index = 0;

    // Select command
    // ESP_LOGI(TAG, "Send select command");
    // payload[0] = 0x06;
    // payload[1] = 0x02;
    // // encode payload
    // MemBufReset(txBuf);
    // SlipEncodeAppend(txBuf, payload, 2);
    // SlipEncodeAddEndMarker(txBuf);
    // // TODO: uart send
    // int txBytes = uart_write_bytes(UART_NUM_1, txBuf->data, txBuf->curSize);

    // Ask for serial MTU
    // ESP_LOGI(TAG, "Ask for serial MTU");
    // payload[0] = 0x07;
    // encode payload
    // MemBufReset(txBuf);
    // SlipEncodeAppend(txBuf, payload, 1);
    // SlipEncodeAddEndMarker(txBuf);
    // TODO: uart send
    // txBytes = uart_write_bytes(UART_NUM_1, txBuf->data, txBuf->curSize);
    // return;

    // ESP_LOGI(TAG, "binFileLength: %d", binFileLength);
    // Create Object
    ESP_LOGI(TAG, "Send create object");
    payload[0] = 0x01;
    // payload[1] = 0x01; // Command Object
    payload[1] = 0x02; // Command Object
    payload[2] = binFileLength;
    payload[3] = binFileLength >> 8;
    payload[4] = binFileLength >> 16;
    payload[5] = binFileLength >> 24;

    // ESP_LOGI(TAG, "Raw init payload: ");
    // ESP_LOG_BUFFER_HEX(TAG, payload, sizeof(payload));


    ////////////////////// Init ////////////////////////
    // encode payload
    MemBufReset(txBuf);
    SlipEncodeAppend(txBuf, payload, sizeof(payload));
    SlipEncodeAddEndMarker(txBuf);

    // ESP_LOGI(TAG, "Encoded init payload: ");
    // ESP_LOG_BUFFER_HEX(TAG, txBuf->data, txBuf->curSize);

    // uart send
    int txBytes = uart_write_bytes(UART_NUM_1, txBuf->data, txBuf->curSize);
    // ESP_LOGI(TAG, "UART wrote %d bytes", txBytes);

    vTaskDelay(100 / portTICK_PERIOD_MS);

    while (binFileLength > 0)
    {
        if (binFileLength >= MAX_ACTUAL_PAYLOAD)
        {           
            // ESP_LOGI(TAG, "binFileLength: %d", binFileLength);

            MemBufReset(txBuf);
            SlipEncodeAppend(txBuf, &opcode, sizeof(opcode));
            SlipEncodeAppend(txBuf, &bin_file[index], MAX_ACTUAL_PAYLOAD);
            SlipEncodeAddEndMarker(txBuf);
            
            binFileLength -= MAX_ACTUAL_PAYLOAD;
            index += MAX_ACTUAL_PAYLOAD;
        }
        else
        {
            MemBufReset(txBuf);
            SlipEncodeAppend(txBuf, &opcode, sizeof(opcode));
            SlipEncodeAppend(txBuf, &bin_file[index], binFileLength);
            SlipEncodeAddEndMarker(txBuf);
            binFileLength = 0;
        }
        // ESP_LOGI(TAG, "Encoded datFile payload: ");
        // ESP_LOG_BUFFER_HEX(TAG, txBuf->data, txBuf->curSize);
        // TODO: uart send
        txBytes = uart_write_bytes(UART_NUM_1, txBuf->data, txBuf->curSize);
        // ESP_LOGI(TAG, "UART wrote %d bytes", txBytes);
        vTaskDelay(50 / portTICK_PERIOD_MS);
    }

    // Ask for CRC
    payload[0] = 0x03;
    // encode payload
    MemBufReset(txBuf);
    SlipEncodeAppend(txBuf, payload, 1);
    SlipEncodeAddEndMarker(txBuf);
    // TODO: uart send
    txBytes = uart_write_bytes(UART_NUM_1, txBuf->data, txBuf->curSize);
    // ESP_LOGI(TAG, "UART wrote %d bytes", txBytes);
    // To-DO: check CRC
    
    vTaskDelay(5 / portTICK_PERIOD_MS);

    // Execute init packet
    payload[0] = 0x04;
    // encode payload
    MemBufReset(txBuf);
    SlipEncodeAppend(txBuf, payload, 1);
    SlipEncodeAddEndMarker(txBuf);
    // TODO: uart send
    txBytes = uart_write_bytes(UART_NUM_1, txBuf->data, txBuf->curSize);
    // ESP_LOGI(TAG, "UART wrote %d bytes", txBytes);
    ESP_LOGI(TAG, "Wrote image");
}

/// @brief send_init_packet()
/// @param 
static void send_init_packet(void)
{
    // ESP_LOGI(TAG, "dat file");
    // ESP_LOG_BUFFER_HEX(TAG, dat_file, sizeof(dat_file));
    static uint8_t payload[6] = {0};
    static const uint8_t opcode = 0x08;
    uint32_t datFileLength = sizeof(dat_file);
    uint32_t index = 0;

    // Create Object
    payload[0] = 0x01;
    payload[1] = 0x01; // Command Object
    payload[2] = datFileLength;
    payload[3] = datFileLength >> 8;
    payload[4] = datFileLength >> 16;
    payload[5] = datFileLength >> 24;

    // ESP_LOGI(TAG, "Raw init payload: ");
    // ESP_LOG_BUFFER_HEX(TAG, payload, sizeof(payload));

    ////////////////////// Init ////////////////////////
    // encode payload
    MemBufReset(txBuf);
    SlipEncodeAppend(txBuf, payload, sizeof(payload));
    SlipEncodeAddEndMarker(txBuf);

    // ESP_LOGI(TAG, "Encoded init payload: ");
    // ESP_LOG_BUFFER_HEX(TAG, txBuf->data, txBuf->curSize);

    // uart send
    int txBytes = uart_write_bytes(UART_NUM_1, txBuf->data, txBuf->curSize);
    // ESP_LOGI(TAG, "UART wrote %d bytes", txBytes);
    vTaskDelay(5 / portTICK_PERIOD_MS);
    while (datFileLength > 0)
    {
        if (datFileLength >= MAX_ACTUAL_PAYLOAD)
        {           
            // ESP_LOGI(TAG, "datFileLength: %d", datFileLength);

            MemBufReset(txBuf);
            SlipEncodeAppend(txBuf, &opcode, sizeof(opcode));
            SlipEncodeAppend(txBuf, &dat_file[index], MAX_ACTUAL_PAYLOAD);
            SlipEncodeAddEndMarker(txBuf);
            
            datFileLength -= MAX_ACTUAL_PAYLOAD;
            index += MAX_ACTUAL_PAYLOAD;
        }
        else
        {
            MemBufReset(txBuf);
            SlipEncodeAppend(txBuf, &opcode, sizeof(opcode));
            SlipEncodeAppend(txBuf, &dat_file[index], datFileLength);
            SlipEncodeAddEndMarker(txBuf);
            datFileLength = 0;
        }
        // ESP_LOGI(TAG, "Encoded datFile payload: ");
        // ESP_LOG_BUFFER_HEX(TAG, txBuf->data, txBuf->curSize);
        // TODO: uart send
        txBytes = uart_write_bytes(UART_NUM_1, txBuf->data, txBuf->curSize);
        // ESP_LOGI(TAG, "UART wrote %d bytes", txBytes);
        vTaskDelay(5 / portTICK_PERIOD_MS);
    }
    // Ask for CRC
    payload[0] = 0x03;
    // encode payload
    MemBufReset(txBuf);
    SlipEncodeAppend(txBuf, payload, 1);
    SlipEncodeAddEndMarker(txBuf);
    // TODO: uart send
    txBytes = uart_write_bytes(UART_NUM_1, txBuf->data, txBuf->curSize);
    // ESP_LOGI(TAG, "UART wrote %d bytes", txBytes);
    // To-DO: check CRC
    
    vTaskDelay(5 / portTICK_PERIOD_MS);

    // Execute init packet
    payload[0] = 0x04;
    // encode payload
    MemBufReset(txBuf);
    SlipEncodeAppend(txBuf, payload, 1);
    SlipEncodeAddEndMarker(txBuf);
    // TODO: uart send
    txBytes = uart_write_bytes(UART_NUM_1, txBuf->data, txBuf->curSize);
    // ESP_LOGI(TAG, "UART wrote %d bytes", txBytes);
    ESP_LOGI(TAG, "Wrote init packet");
}

/// @brief send_ping()
/// @param
static void send_ping(void)
{
    const uint8_t payload[2] = {0x09, 0x01};

    MemBufReset(txBuf);
    SlipEncodeAppend(txBuf, payload, sizeof(payload));
    SlipEncodeAddEndMarker(txBuf);

    ESP_LOGI(TAG, "Ping: ");
    ESP_LOG_BUFFER_HEX(TAG, txBuf->data, txBuf->curSize);

    int txBytes = uart_write_bytes(UART_NUM_1, txBuf->data, txBuf->curSize);
    ESP_LOGI(TAG, "UART wrote %d bytes", txBytes);
}



static void main_task(void *pvParameters)
{
    // send ping command
    send_ping();
    vTaskDelay(500 / portTICK_PERIOD_MS);
    // send init (.dat)
    send_init_packet();
    vTaskDelay(500 / portTICK_PERIOD_MS);
    // send image (.bin)
    send_image_packet();
    vTaskDelay(500 / portTICK_PERIOD_MS);
    // delete task
    vTaskDelete(NULL);
}

void app_main(void)
{
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND)
    {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    // ESP_ERROR_CHECK(ret);
    // ESP_ERROR_CHECK(esp_netif_init());
    // ESP_ERROR_CHECK(esp_event_loop_create_default());

    // Init uart
    uart_init();

    // ESP_ERROR_CHECK(example_connect());
    // ESP_LOGI(TAG, "Connected to AP, begin http example");

    txBuf = AllocMemBuf(PREAMBLE_MTU_SIZE);

    xTaskCreate(&main_task, "main_task", 8192, NULL, 5, NULL);
}
