#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_log.h"

#define LED_PIN GPIO_NUM_2

static uint8_t led_state = 0;

void blink_led(void)
{
    gpio_set_level(LED_PIN, led_state);
}

void app_main(void)
{
    gpio_set_direction(LED_PIN, GPIO_MODE_OUTPUT);

    while (1)
    {
        ESP_LOGI("ESP32", "Hello World!");
        ESP_LOGI("LED CONTROL", "Turning the LED %s!", led_state == true ? "ON" : "OFF");
        blink_led();
        led_state = !led_state;

        vTaskDelay(100); // Add 1 tick delay (10 ms) so that current task does not starve idle task and trigger watchdog timer
    }
}