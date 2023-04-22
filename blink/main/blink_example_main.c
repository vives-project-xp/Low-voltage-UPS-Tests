/* Blink Example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "led_strip.h"
#include "sdkconfig.h"

static const char *TAG = "example";

/* Use project configuration menu (idf.py menuconfig) to choose the GPIO to blink,
   or you can edit the following line and set a number here.
*/
#define BLINK_GPIO GPIO_NUM_13
#define BLINK_GPIO2 GPIO_NUM_14


static uint8_t led_state = 0;

void blink_led(void)
{
    gpio_set_level(BLINK_GPIO, led_state);
    gpio_set_level(BLINK_GPIO2, led_state);
}

void app_main(void)
{
    while (1) {
        ESP_LOGI(TAG, "Turning the LED %s!", led_state == true ? "ON" : "OFF");
        blink_led();
        /* Toggle the LED state */
        led_state = !led_state;
        vTaskDelay(100);
    }
}
