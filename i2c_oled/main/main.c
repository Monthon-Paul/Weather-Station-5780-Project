/*
 * Mini Weather station, implement with reading sensors with I2C,
 * and displaying on OLED screen with I2C
 */
#include <stdio.h>
#include <string.h>

#include "bmx280.h"
#include "dht11.h"
#include "driver/gpio.h"
#include "driver/i2c.h"
#include "driver/uart.h"
#include "esp_err.h"
#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_ops.h"
#include "esp_log.h"
#include "esp_lvgl_port.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "lvgl.h"
#include "sdkconfig.h"
#include "soc/uart_periph.h"

#if CONFIG_EXAMPLE_LCD_CONTROLLER_SH1107
#include "esp_lcd_sh1107.h"
#else
#include "esp_lcd_panel_vendor.h"
#endif

static const char *TAG = "TPH";

#define I2C_BUS_PORT 0
#define I2C_BUS_PORT2 1

// Defining UART Pins
#define UART_NUM UART_NUM_0

#define UARTS_BAUD_RATE 115200
#define BUF_SIZE 1024
#define RD_BUF_SIZE BUF_SIZE
#define PATTERN_CHR_NUM 3
#define TXD_PIN 43
#define RXD_PIN 44

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////// Please update the following configuration according to your LCD spec //////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
#define EXAMPLE_LCD_PIXEL_CLOCK_HZ (400 * 1000)
#define EXAMPLE_PIN_NUM_SDA 3
#define EXAMPLE_PIN_NUM_SCL 4
#define EXAMPLE_PIN_NUM_RST -1
#define EXAMPLE_I2C_HW_ADDR 0x3C

// The pixel number in horizontal and vertical
#if CONFIG_EXAMPLE_LCD_CONTROLLER_SSD1306
#define EXAMPLE_LCD_H_RES 128
#define EXAMPLE_LCD_V_RES 64
#elif CONFIG_EXAMPLE_LCD_CONTROLLER_SH1107
#define EXAMPLE_LCD_H_RES 64
#define EXAMPLE_LCD_V_RES 128
#endif
// Bit number used to represent command and parameter
#define EXAMPLE_LCD_CMD_BITS 8
#define EXAMPLE_LCD_PARAM_BITS 8

void uart_event_task(void *);
extern void example_lvgl_demo_ui(lv_disp_t *disp, char *message, int i);

char *dtmp;

void temp_sens(lv_disp_t *disp) {
    // Entry Point
    ESP_LOGI(TAG, "Initialize I2C bus for sensors");
    // Create I2C bus handle for sensor readings
    i2c_config_t i2c_cfg = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = GPIO_NUM_10,
        .scl_io_num = GPIO_NUM_11,
        .sda_pullup_en = false,
        .scl_pullup_en = false,

        .master = {
            .clk_speed = 100000}};

    ESP_ERROR_CHECK(i2c_param_config(I2C_NUM_1, &i2c_cfg));
    ESP_ERROR_CHECK(i2c_driver_install(I2C_NUM_1, I2C_MODE_MASTER, 0, 0, 0));

    bmx280_t *bmx280 = bmx280_create(I2C_NUM_1);

    if (!bmx280) {
        ESP_LOGE("test", "Could not create bmx280 driver.");
        return;
    }

    ESP_ERROR_CHECK(bmx280_init(bmx280));

    bmx280_config_t bmx_cfg = BMX280_DEFAULT_CONFIG;
    ESP_ERROR_CHECK(bmx280_configure(bmx280, &bmx_cfg));

    DHT11_init(GPIO_NUM_5);

    size_t i = 0;
    char dis[10];

    ESP_LOGI(TAG, "Display LVGL Scroll Text");
    // Lock the mutex due to the LVGL APIs are not thread-safe
    while (1) {
        if (lvgl_port_lock(0)) {
            ESP_ERROR_CHECK(bmx280_setMode(bmx280, BMX280_MODE_FORCE));
            do {
                vTaskDelay(pdMS_TO_TICKS(1));
            } while (bmx280_isSampling(bmx280));

            float temp = 0, pres = 0, hum = 0;
            ESP_ERROR_CHECK(bmx280_readoutFloat(bmx280, &temp, &pres, &hum));
            hum = DHT11_read().humidity;
            ESP_LOGI(TAG, "Read Values: temp = %.2f, pres = %.2f, hum = %.2f", temp, pres, hum);
            ESP_LOGI(TAG, "Status code is %d", DHT11_read().status);
            example_lvgl_demo_ui(disp, dis, i);
            if (*dtmp == 'b') {
                i++;
                if (i % 3 == 1)
                    sprintf(dis, "%.2f", pres);
                else if (i % 3 == 2)
                    sprintf(dis, "%.0f", hum);
                else
                    sprintf(dis, "%.0f", temp);
                example_lvgl_demo_ui(disp, dis, i % 3);
            } else {
                if (i % 3 == 1)
                    sprintf(dis, "%.2f", pres);
                else if (i % 3 == 2)
                    sprintf(dis, "%.0f", hum);
                else
                    sprintf(dis, "%.0f", temp);
                example_lvgl_demo_ui(disp, dis, i++ % 3);
            }
            if (*dtmp == 'p') {
                sprintf(dis, "%.2f", pres);
                example_lvgl_demo_ui(disp, dis, 1);
                i = 4;
            } else if (*dtmp == 'h') {
                sprintf(dis, "%.0f", hum);
                example_lvgl_demo_ui(disp, dis, 2);
                i = 5;
            } else if (*dtmp == 't') {
                sprintf(dis, "%.0f", temp);
                example_lvgl_demo_ui(disp, dis, 0);
                i = 3;
            }
            vTaskDelay(pdMS_TO_TICKS(3000));
            *dtmp = 'z';
            // Release the mutex
            lvgl_port_unlock();
        }
    }
}

void uart_init() {
    esp_log_level_set(TAG, ESP_LOG_INFO);
    uart_config_t uart_config = {
        .baud_rate = UARTS_BAUD_RATE,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };

    uart_driver_install(UART_NUM, BUF_SIZE * 2, BUF_SIZE * 2, 20, NULL, 0);
    uart_param_config(UART_NUM, &uart_config);

    //Set UART log level
    esp_log_level_set(TAG, ESP_LOG_INFO);
    //Set UART pins (using UART0 default pins ie no changes.)
    uart_set_pin(UART_NUM, TXD_PIN, RXD_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);

    //Set uart pattern detect function.
    uart_enable_pattern_det_baud_intr(UART_NUM, '+', 3, 9, 0, 0);
    //Reset the pattern queue length to record at most 20 pattern positions.
    uart_pattern_queue_reset(UART_NUM, 20);

    //Create a task to handler UART event from ISR
    xTaskCreate(uart_event_task, "uart_event_task", 2048, NULL, 12, NULL);
}

void uart_event_task(void *pvParameters) {
    dtmp = (char *)calloc(BUF_SIZE, sizeof(char *));
    while (1) {
        // Read data from the UART
        int len = uart_read_bytes(UART_NUM, dtmp, BUF_SIZE, 20 / portTICK_PERIOD_MS);
        if (len > 0)
            ESP_LOGI(TAG, "Received %d bytes: '%s'", len, dtmp); // Print the received data
    }
    free(dtmp);
    dtmp = NULL;
    vTaskDelete(NULL);
}

void app_main(void) {
    ESP_LOGI(TAG, "Initialize I2C");
    i2c_config_t i2c_cfg = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = EXAMPLE_PIN_NUM_SDA,
        .scl_io_num = EXAMPLE_PIN_NUM_SCL,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,

        .master = {
            .clk_speed = EXAMPLE_LCD_PIXEL_CLOCK_HZ}};

    ESP_ERROR_CHECK(i2c_param_config(I2C_NUM_0, &i2c_cfg));
    ESP_ERROR_CHECK(i2c_driver_install(I2C_NUM_0, I2C_MODE_MASTER, 0, 0, 0));

    ESP_LOGI(TAG, "Install panel IO");
    esp_lcd_panel_io_handle_t io_handle = NULL;
    esp_lcd_panel_io_i2c_config_t io_config = {
        .dev_addr = EXAMPLE_I2C_HW_ADDR,
        //.scl_speed_hz = EXAMPLE_LCD_PIXEL_CLOCK_HZ,
        .control_phase_bytes = 1,                // According to SSD1306 datasheet
        .lcd_cmd_bits = EXAMPLE_LCD_CMD_BITS,    // According to SSD1306 datasheet
        .lcd_param_bits = EXAMPLE_LCD_CMD_BITS,  // According to SSD1306 datasheet
#if CONFIG_EXAMPLE_LCD_CONTROLLER_SSD1306
        .dc_bit_offset = 6,  // According to SSD1306 datasheet
#elif CONFIG_EXAMPLE_LCD_CONTROLLER_SH1107
        .dc_bit_offset = 0,  // According to SH1107 datasheet
        .flags =
            {
                .disable_control_phase = 1,
            }
#endif
    };
    ESP_ERROR_CHECK(esp_lcd_new_panel_io_i2c(I2C_NUM_0, &io_config, &io_handle));

    ESP_LOGI(TAG, "Install SSD1306 panel driver");
    esp_lcd_panel_handle_t panel_handle = NULL;
    esp_lcd_panel_dev_config_t panel_config = {
        .bits_per_pixel = 1,
        .reset_gpio_num = EXAMPLE_PIN_NUM_RST,
    };
#if CONFIG_EXAMPLE_LCD_CONTROLLER_SSD1306
    ESP_ERROR_CHECK(esp_lcd_new_panel_ssd1306(io_handle, &panel_config, &panel_handle));
#elif CONFIG_EXAMPLE_LCD_CONTROLLER_SH1107
    ESP_ERROR_CHECK(esp_lcd_new_panel_sh1107(io_handle, &panel_config, &panel_handle));
#endif

    ESP_ERROR_CHECK(esp_lcd_panel_reset(panel_handle));
    ESP_ERROR_CHECK(esp_lcd_panel_init(panel_handle));
    ESP_ERROR_CHECK(esp_lcd_panel_disp_on_off(panel_handle, true));

#if CONFIG_EXAMPLE_LCD_CONTROLLER_SH1107
    ESP_ERROR_CHECK(esp_lcd_panel_invert_color(panel_handle, true));
#endif

    ESP_LOGI(TAG, "Initialize LVGL");
    const lvgl_port_cfg_t lvgl_cfg = ESP_LVGL_PORT_INIT_CONFIG();
    lvgl_port_init(&lvgl_cfg);

    const lvgl_port_display_cfg_t disp_cfg = {
        .io_handle = io_handle,
        .panel_handle = panel_handle,
        .buffer_size = EXAMPLE_LCD_H_RES * EXAMPLE_LCD_V_RES,
        .double_buffer = true,
        .hres = EXAMPLE_LCD_H_RES,
        .vres = EXAMPLE_LCD_V_RES,
        .monochrome = true,
        .rotation = {
            .swap_xy = false,
            .mirror_x = false,
            .mirror_y = false,
        }};
    lv_disp_t *disp = lvgl_port_add_disp(&disp_cfg);

    /* Rotation of the screen */
    lv_disp_set_rotation(disp, LV_DISP_ROT_NONE);

    uart_init();
    temp_sens(disp);
}