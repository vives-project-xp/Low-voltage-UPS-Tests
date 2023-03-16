#include "driver/gpio.h" 
#include "mqtt_client.h"
#include "esp_log.h"    
#include "esp_wifi.h"   
#include "esp_system.h"
#include "esp_event.h"
#include "nvs_flash.h"
#include "esp_netif.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "freertos/queue.h"
#include "freertos/event_groups.h"

//Wifi dependencies working
#define WIFI_CONNECTED_BIT BIT0
#define WIFI_FAIL_BIT      BIT1
static int s_retry_num = 0;
static EventGroupHandle_t s_wifi_event_group;
#define EXAMPLE_ESP_MAXIMUM_RETRY  5


#define LED_PIN GPIO_NUM_2

#define WIFI_SSID "devbit"
#define WIFI_PASS "Dr@@dloos!"

static esp_mqtt_client_handle_t client;


static uint8_t led_state = 0;
//

//led blink test working
void blink_led(void)
{
    gpio_set_level(LED_PIN, led_state);
}
//

//wifi event handler
static void event_handler(void* arg, esp_event_base_t event_base,
                                int32_t event_id, void* event_data)
{
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        if (s_retry_num < EXAMPLE_ESP_MAXIMUM_RETRY) {
            esp_wifi_connect();
            s_retry_num++;
            ESP_LOGI("WIFI", "retry to connect to the AP");
        } else {
            xEventGroupSetBits(s_wifi_event_group, WIFI_FAIL_BIT);
        }
        ESP_LOGI("WIFI","connect to the AP fail");
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t* event = (ip_event_got_ip_t*) event_data;
        ESP_LOGI("WIFI", "got ip:" IPSTR, IP2STR(&event->ip_info.ip));
        s_retry_num = 0;
        xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
    }
}
//

//Wifi setup
void wifi_init_sta(void)
{
    s_wifi_event_group = xEventGroupCreate();

    ESP_ERROR_CHECK(esp_netif_init());

    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();

    esp_event_handler_instance_t instance_any_id;
    esp_event_handler_instance_t instance_got_ip;
    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT,
                                                        ESP_EVENT_ANY_ID,
                                                        &event_handler,
                                                        NULL,
                                                        &instance_any_id));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT,
                                                        IP_EVENT_STA_GOT_IP,
                                                       &event_handler,
                                                        NULL,
                                                        &instance_got_ip));

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    wifi_config_t wifi_config = {
        .sta = {
            .ssid = WIFI_SSID,
            .password = WIFI_PASS,
        },
    };
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());
}
//

//mqtt event handler
static void mqtt_event_handler(void *handler_args, esp_event_base_t base, int32_t event_id, void *event_data)
{
    esp_mqtt_event_handle_t event = event_data;
    esp_mqtt_client_handle_t intrnclient = event->client;
    int msg_id;
    switch ((esp_mqtt_event_id_t)event_id) {
    case MQTT_EVENT_CONNECTED:
        ESP_LOGI("MQTT", "MQTT_EVENT_CONNECTED");
        msg_id = esp_mqtt_client_publish(intrnclient, "homeassistant/battery/state", handler_args, 0, 1, 0);
        ESP_LOGI("MQTT", "sent publish successful, msg_id=%d", msg_id);

        msg_id = esp_mqtt_client_subscribe(intrnclient, "homeassistant/battery/reset", 0);
        ESP_LOGI("MQTT", "sent subscribe successful, msg_id=%d", msg_id);

        /*msg_id = esp_mqtt_client_subscribe(intrnclient, "/topic/qos1", 1);
        ESP_LOGI("MQTT", "sent subscribe successful, msg_id=%d", msg_id);*/

        /*msg_id = esp_mqtt_client_unsubscribe(intrnclient, "/topic/qos1");
        ESP_LOGI("MQTT", "sent unsubscribe successful, msg_id=%d", msg_id);*/
        break;
    case MQTT_EVENT_DISCONNECTED:
        ESP_LOGI("MQTT", "MQTT_EVENT_DISCONNECTED");
        break;

    case MQTT_EVENT_SUBSCRIBED:
        /*ESP_LOGI("MQTT", "MQTT_EVENT_SUBSCRIBED, msg_id=%d", event->msg_id);
        msg_id = esp_mqtt_client_publish(intrnclient, "/topic/qos0", "data", 0, 0, 0);
        ESP_LOGI("MQTT", "sent publish successful, msg_id=%d", msg_id);*/
        break;
    case MQTT_EVENT_UNSUBSCRIBED:
        ESP_LOGI("MQTT", "MQTT_EVENT_UNSUBSCRIBED, msg_id=%d", event->msg_id);
        break;
    case MQTT_EVENT_PUBLISHED:
        ESP_LOGI("MQTT", "MQTT_EVENT_PUBLISHED, msg_id=%d", event->msg_id);
        break;
    case MQTT_EVENT_DATA:
        ESP_LOGI("MQTT", "MQTT_EVENT_DATA");
        printf("TOPIC=%.*s\r\n", event->topic_len, event->topic);
        printf("DATA=%.*s\r\n", event->data_len, event->data);
        break;
    case MQTT_EVENT_ERROR:
        ESP_LOGI("MQTT", "MQTT_EVENT_ERROR");
        break;
    default:
        ESP_LOGI("MQTT", "Other event id:%d", event->event_id);
        break;
    }
}
//

//mqtt setup
static void mqtt_app_start(void)
{
    esp_mqtt_client_config_t mqtt_cfg = {
        .broker.address.uri = "mqtt://10.10.78.77:1883",
    };

    client = esp_mqtt_client_init(&mqtt_cfg);
    /* The last argument may be used to pass data to the event handler, in this example mqtt_event_handler */
    esp_mqtt_client_register_event(client, ESP_EVENT_ANY_ID, mqtt_event_handler, NULL);
    esp_mqtt_client_start(client);
}
//

//mqtt publisher
static void mqtt_publish_task(char *message)
{
    int message_id;
    message_id = esp_mqtt_client_publish(client, "homeassistant/battery/state", message, 0, 0, 0);
    ESP_LOGI("MQTT", "Published message with ID %d: %s", message_id, message);
}


void app_main(void)
{
    //Wifi code working
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
      ESP_ERROR_CHECK(nvs_flash_erase());
      ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    wifi_init_sta();
    vTaskDelay(200);
    gpio_set_direction(LED_PIN, GPIO_MODE_OUTPUT);
    //

    //mqtt code
    /*ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());*/

    mqtt_app_start();


    while (1)
    {
        //blinky led
        ESP_LOGI("ESP32", "Hello World!");
        ESP_LOGI("LED CONTROL", "Turning the LED %s!", led_state == true ? "ON" : "OFF");
        blink_led();
        led_state = !led_state;
        //
        //mqtt message publish
        char message[30];
        sprintf(message, "LED is %s!", led_state == true ? "ON" : "OFF");
        mqtt_publish_task(message);
        //
        vTaskDelay(100); // Add 1 tick delay (10 ms) so that current task does not starve idle task and trigger watchdog timer
    }
}