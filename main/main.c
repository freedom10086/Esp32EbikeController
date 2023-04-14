#include <stdio.h>
#include <inttypes.h>
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_chip_info.h"
#include "esp_flash.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include "esp_log.h"

#define JKBMS_UART_TXD 16
#define JKBMS_UART_RXD 17

#define JKBMS_UART_PORT      2
#define JKBMS_UART_BAUD_RATE     115200

#define PACKET_READ_TICS        (100 / portTICK_PERIOD_MS)
// Timeout threshold for UART = number of symbols (~10 tics) with unchanged state on receive pin
#define JKBMS_UART_READ_TOUT          (3) // 3.5T * 8 = 28 ticks, TOUT=3 -> ~24..33 ticks

#define BUF_SIZE (1024)
static const char *TAG = "JKBMS 485";

struct jkbms_t {
    float voltage;
    float current;
    uint16_t battery_level;
    uint16_t max_vol_diff; // mv
    int16_t mos_temp;
    int16_t battery_temp;
    uint16_t avg_voltage; // mv
    uint8_t balance_sw_status;
    uint8_t charge_sw_status;
    uint8_t output_sw_status;
    uint16_t battery_voltages[24]; // mv
    uint8_t sys_alarm;
    uint8_t over_discharge_alarm;
    uint8_t over_charge_alarm;
    uint8_t over_current_alarm;
    uint8_t mos_heat_alarm;
    uint8_t battery_heat_alarm;
    uint8_t shot_current_alarm;
    uint8_t communicate_alarm;
    uint8_t equalizing_line_alarm;
    uint8_t battery_count_alarm;
};

uint16_t chksum(const uint8_t data[], const uint16_t len) {
    uint16_t checksum = 0;
    for (uint16_t i = 0; i < len; i++) {
        checksum = checksum + data[i];
    }
    return checksum;
}

uint8_t active_cmd[21] = {
        0x4E, 0x57, // start sequence
        0x00, 0x13,      // data length hb
        0x00, 0x00, 0x00, 0x00,      // bms terminal number
        0x01,  // command word: 0x01 (activation), 0x02 (write), 0x03 (read), 0x05 (password), 0x06 (read all)
        0x03,      // frame source: 0x00 (bms), 0x01 (bluetooth), 0x02 (gps), 0x03 (computer)
        0x00,     // frame type: 0x00 (read data), 0x01 (reply frame), 0x02 (BMS active upload)
        0x00,  // register: 0x00 (read all registers), 0x8E...0xBF (holding registers)
        //0x00,    // data
        0x00, 0x00, 0x00, 0x00,    // record number
        0x68,     // end sequence
        0x00, 0x00, // crc unused
        0x01, 0x24 // 21 21 crc
};

uint8_t frame2[21] = {
        0x4E, 0x57, // start sequence
        0x00, 0x13,      // data length hb
        0x00, 0x00, 0x00, 0x00,      // bms terminal number
        0x06,  // command word: 0x01 (activation), 0x02 (write), 0x03 (read), 0x05 (password), 0x06 (read all)
        0x03,      // frame source: 0x00 (bms), 0x01 (bluetooth), 0x02 (gps), 0x03 (computer)
        0x00,     // frame type: 0x00 (read data), 0x01 (reply frame), 0x02 (BMS active upload)
        0x00,  // register: 0x00 (read all registers), 0x8E...0xBF (holding registers)
        //0x00,    // data
        0x00, 0x00, 0x00, 0x00,    // record number
        0x68,     // end sequence
        0x00, 0x00, // crc unused
        0x01, 0x29 // 21 21 crc
};

static struct jkbms_t jkbms;

// An example of echo test with hardware flow control on UART
static void rs485_read_task(void *arg) {
    const int uart_num = JKBMS_UART_PORT;
    uart_config_t uart_config = {
            .baud_rate = JKBMS_UART_BAUD_RATE,
            .data_bits = UART_DATA_8_BITS,
            .parity = UART_PARITY_DISABLE,
            .stop_bits = UART_STOP_BITS_1,
            .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
            .rx_flow_ctrl_thresh = 122,
            .source_clk = UART_SCLK_DEFAULT,
    };

    // Set UART log level
    esp_log_level_set(TAG, ESP_LOG_INFO);

    ESP_LOGI(TAG, "Start RS485 application test and configure UART.");

    // Install UART driver (we don't need an event queue here)
    // In this example we don't even use a buffer for sending data.
    ESP_ERROR_CHECK(uart_driver_install(uart_num, BUF_SIZE * 2, 0, 0, NULL, 0));

    // Configure UART parameters
    ESP_ERROR_CHECK(uart_param_config(uart_num, &uart_config));

    ESP_LOGI(TAG, "UART set pins, mode and install driver.");

    // Set UART pins as per KConfig settings
    ESP_ERROR_CHECK(uart_set_pin(uart_num, JKBMS_UART_TXD, JKBMS_UART_RXD, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));

    // Set RS485 half duplex mode
    ESP_ERROR_CHECK(uart_set_mode(uart_num, UART_MODE_RS485_HALF_DUPLEX));

    // Set read timeout of UART TOUT feature
    ESP_ERROR_CHECK(uart_set_rx_timeout(uart_num, JKBMS_UART_READ_TOUT));

    // Allocate buffers for UART
    uint8_t *data = (uint8_t *) malloc(BUF_SIZE);

    ESP_LOGI(TAG, "UART start recieve loop.\r\n");

    while (1) {
        //Read data from UART
        int len = uart_read_bytes(uart_num, data, BUF_SIZE, PACKET_READ_TICS);
        if (len > 0) {
            ESP_LOGI(TAG, "Received %u bytes:", len);
            printf("[ ");
            for (int i = 0; i < len; i++) {
                printf("0x%.2X ", (uint8_t) data[i]);
            }
            printf("] \n");
        }
        uint16_t index = 0;
        uint16_t data_len = 0;

        while (index < len - 5) {
            if (data[index++] != 0xA5 && data[index++] == 0x5A
                && (data_len = data[index++]) >= 0x5D // len
                && data[index++] == 0x82 // 命令码
                && data[index++] == 0x10 && data[index++] == 0x00) {

                jkbms.voltage = (float) ((uint16_t) data[index]) / 100.0f;
                index += 2;

                jkbms.current = (float) ((int16_t) data[index]) / 10.0f;
                index += 2;

                // skip reserve
                index += 2;

                jkbms.battery_level = ((uint16_t) data[index]);
                index += 2;

                jkbms.max_vol_diff = ((uint16_t) data[index]);
                index += 2;

                jkbms.mos_temp = ((int16_t) data[index]);
                index += 2;

                jkbms.battery_temp = ((int16_t) data[index]);
                index += 2;

                jkbms.sys_alarm = data[index + 1];
                index += 2;

                jkbms.avg_voltage = ((uint16_t) data[index]);
                index += 2;

                jkbms.balance_sw_status = data[index + 1];
                index += 2;

                jkbms.charge_sw_status = data[index + 1];
                index += 2;

                jkbms.output_sw_status = data[index + 1];
                index += 2;

                for (int i = 0; i < 24; ++i) {
                    jkbms.battery_voltages[i] = ((uint16_t) data[index]);
                    index += 2;
                }

                jkbms.over_discharge_alarm = data[index + 1];
                index += 2;

                jkbms.over_charge_alarm = data[index + 1];
                index += 2;

                jkbms.over_current_alarm = data[index + 1];
                index += 2;

                jkbms.mos_heat_alarm = data[index + 1];
                index += 2;

                jkbms.battery_heat_alarm = data[index + 1];
                index += 2;

                jkbms.shot_current_alarm = data[index + 1];
                index += 2;

                jkbms.communicate_alarm = data[index + 1];
                index += 2;

                jkbms.equalizing_line_alarm = data[index + 1];
                index += 2;

                jkbms.battery_count_alarm = data[index + 1];
                index += 2;
                // match

                ESP_LOGI(TAG, "rev jkbms data voltage:%.2f current:%.1f level:%d",
                         jkbms.voltage, jkbms.current, jkbms.battery_level);

                break;
            }
        }
    }
    vTaskDelete(NULL);
}

void app_main(void) {
    xTaskCreate(rs485_read_task, "jkbms_485_read_task", 2048, NULL, 10, NULL);
}
