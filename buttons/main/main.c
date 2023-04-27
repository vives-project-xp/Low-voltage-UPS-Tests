#include "driver/gpio.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "freertos/queue.h"
#include "freertos/event_groups.h"

#define BUTTON_1 GPIO_NUM_12

void app_main(void){
    gpio_set_direction(GPIO_NUM_12, GPIO_MODE_INPUT);
    gpio_set_pull_mode(BUTTON_1, GPIO_PULLUP_ONLY);
    
    while (1){
        int button_state = gpio_get_level(BUTTON_1);
        ESP_LOGI("button state", "%d", button_state);
        vTaskDelay(10);
    }
}