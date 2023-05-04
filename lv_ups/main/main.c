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

#include "esp_bt.h"
#include "esp_gap_ble_api.h"
#include "esp_gatts_api.h"
#include "esp_bt_main.h"
#include "gatts_table.h"
#include "esp_gatt_common_api.h"

#include "cJSON.h"
#include "esp_timer.h"

// wifi, mqtt and led toggle
#define WIFI_CONNECTED_BIT BIT0
#define WIFI_FAIL_BIT BIT1
static int s_retry_num = 0;
static EventGroupHandle_t s_wifi_event_group;
#define EXAMPLE_ESP_MAXIMUM_RETRY 5
int leds[] = {GPIO_NUM_13, GPIO_NUM_14};
#define BLE_LED 0
#define MQTT_LED 1
#define BUTTON_1 GPIO_NUM_12
/*#define BLE_LED_PIN GPIO_NUM_13
#define MQTT_LED_PIN GPIO_NUM_14*/
static esp_mqtt_client_handle_t client;
// static uint8_t led_state = 0;
bool wifi_online = false;
//

// ble and wifi variables
char ssid[32] = "";
size_t ssid_size = sizeof(ssid);
char pass[64] = "";
size_t pass_size = sizeof(pass);
char mqtt_ip[21] = "";
size_t mqtt_ip_size = sizeof(mqtt_ip);
char mqtt_url[28] = "";
//

// mqtt variables
char name_device[32] = "";
size_t name_size = sizeof(name_device);
//char commands[] = "{\"Battery_charge\":false,\"Battery_size\":32000,\"Min_battery_charge\":10,\"Max_battery_charge\":90,\"Restart\":false,\"Use_battery\":false}";
char commands[] = "";
size_t command_len;
char *lvups_status = "";
char topic_prefix[] = "lvups/";
char info_topic[64] = "";
char state_topic[64] = "";
char command_topic[64] = "";
char info_name[] = "";
char state_name[] = "";
char command_name[] = "";


//

// sensor variables
bool charging = false;
//

//battery values
int battery_size = 32000;
int battery_charge_min = 10;
int battery_charge_max = 90;
//

// ble dependencies, defines and variables
bool bluetooth_online = false;
bool device_connected = false;

int ssid_handle = 30;
int pass_handle = 30;
int mqtt_handle = 30;
int name_handle = 30;
int status_handle = 30;
int cnt = 0;
//

// flash memory defines
#define FLASH_NAMESPACE "wifi_creds"
nvs_handle_t wifi_handle;

//

#define GATTS_TABLE_TAG "GATTS_TABLE"

#define PROFILE_NUM 1
#define PROFILE_APP_IDX 0
#define ESP_APP_ID 0x55
#define BLE_DEVICE_NAME "UPS-1"
#define SVC_INST_ID 0

/* The max length of characteristic value. When the GATT client performs a write or prepare write operation,
 *  the data length must be less than GATTS_DEMO_CHAR_VAL_LEN_MAX.
 */
#define GATTS_DEMO_CHAR_VAL_LEN_MAX 500
#define PREPARE_BUF_MAX_SIZE 1024
#define CHAR_DECLARATION_SIZE (sizeof(uint8_t))

#define ADV_CONFIG_FLAG (1 << 0)
#define SCAN_RSP_CONFIG_FLAG (1 << 1)

static uint8_t adv_config_done = 0;

uint16_t heart_rate_handle_table[HRS_IDX_NB];

typedef struct
{
    uint8_t *prepare_buf;
    int prepare_len;
} prepare_type_env_t;

static prepare_type_env_t prepare_write_env;

#define CONFIG_SET_RAW_ADV_DATA
#ifdef CONFIG_SET_RAW_ADV_DATA
static uint8_t raw_adv_data[] = {
    /* flags */
    0x02, 0x01, 0x06,
    /* tx power*/
    0x02, 0x0a, 0xeb,
    /* service uuid */
    0x03, 0x03, 0xFF, 0x00,
    /* device name */
    0x06, 0x09, 'U', 'P', 'S', '-', '1'};
static uint8_t raw_scan_rsp_data[] = {
    /* flags */
    0x02, 0x01, 0x06,
    /* tx power */
    0x02, 0x0a, 0xeb,
    /* service uuid */
    0x03, 0x03, 0xFF, 0x00};

#else
static uint8_t service_uuid[16] = {
    /* LSB <--------------------------------------------------------------------------------> MSB */
    // first uuid, 16bit, [12],[13] is the value
    0xfb,
    0x34,
    0x9b,
    0x5f,
    0x80,
    0x00,
    0x00,
    0x80,
    0x00,
    0x10,
    0x00,
    0x00,
    0xFF,
    0x00,
    0x00,
    0x00,
};

/* The length of adv data must be less than 31 bytes */
static esp_ble_adv_data_t adv_data = {
    .set_scan_rsp = false,
    .include_name = true,
    .include_txpower = true,
    .min_interval = 0x0006, // slave connection min interval, Time = min_interval * 1.25 msec
    .max_interval = 0x0010, // slave connection max interval, Time = max_interval * 1.25 msec
    .appearance = 0x00,
    .manufacturer_len = 0,       // TEST_MANUFACTURER_DATA_LEN,
    .p_manufacturer_data = NULL, // test_manufacturer,
    .service_data_len = 0,
    .p_service_data = NULL,
    .service_uuid_len = sizeof(service_uuid),
    .p_service_uuid = service_uuid,
    .flag = (ESP_BLE_ADV_FLAG_GEN_DISC | ESP_BLE_ADV_FLAG_BREDR_NOT_SPT),
};

// scan response data
static esp_ble_adv_data_t scan_rsp_data = {
    .set_scan_rsp = true,
    .include_name = true,
    .include_txpower = true,
    .min_interval = 0x0006,
    .max_interval = 0x0010,
    .appearance = 0x00,
    .manufacturer_len = 0,       // TEST_MANUFACTURER_DATA_LEN,
    .p_manufacturer_data = NULL, //&test_manufacturer[0],
    .service_data_len = 0,
    .p_service_data = NULL,
    .service_uuid_len = sizeof(service_uuid),
    .p_service_uuid = service_uuid,
    .flag = (ESP_BLE_ADV_FLAG_GEN_DISC | ESP_BLE_ADV_FLAG_BREDR_NOT_SPT),
};
#endif /* CONFIG_SET_RAW_ADV_DATA */

static esp_ble_adv_params_t adv_params = {
    .adv_int_min = 0x20,
    .adv_int_max = 0x40,
    .adv_type = ADV_TYPE_IND,
    .own_addr_type = BLE_ADDR_TYPE_PUBLIC,
    .channel_map = ADV_CHNL_ALL,
    .adv_filter_policy = ADV_FILTER_ALLOW_SCAN_ANY_CON_ANY,
};

struct gatts_profile_inst
{
    esp_gatts_cb_t gatts_cb;
    uint16_t gatts_if;
    uint16_t app_id;
    uint16_t conn_id;
    uint16_t service_handle;
    esp_gatt_srvc_id_t service_id;
    uint16_t char_handle;
    esp_bt_uuid_t char_uuid;
    esp_gatt_perm_t perm;
    esp_gatt_char_prop_t property;
    uint16_t descr_handle;
    esp_bt_uuid_t descr_uuid;
};

static void gatts_profile_event_handler(esp_gatts_cb_event_t event,
                                        esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param);

/* One gatt-based profile one app_id and one gatts_if, this array will store the gatts_if returned by ESP_GATTS_REG_EVT */
static struct gatts_profile_inst heart_rate_profile_tab[PROFILE_NUM] = {
    [PROFILE_APP_IDX] = {
        .gatts_cb = gatts_profile_event_handler,
        .gatts_if = ESP_GATT_IF_NONE, /* Not get the gatt_if, so initial is ESP_GATT_IF_NONE */
    },
};

/* Service */
static const uint16_t GATTS_SERVICE_UUID_TEST = 0x00FF;
static const uint16_t GATTS_CHAR_UUID_A = 0xFF01;
static const uint16_t GATTS_CHAR_UUID_B = 0xFF02;
static const uint16_t GATTS_CHAR_UUID_C = 0xFF03;
static const uint16_t GATTS_CHAR_UUID_D = 0xFF04;
static const uint16_t GATTS_CHAR_UUID_E = 0xFF05;

static const uint16_t primary_service_uuid = ESP_GATT_UUID_PRI_SERVICE;
static const uint16_t character_declaration_uuid = ESP_GATT_UUID_CHAR_DECLARE;
// static const uint16_t character_client_config_uuid = ESP_GATT_UUID_CHAR_CLIENT_CONFIG;
// static const uint8_t char_prop_read = ESP_GATT_CHAR_PROP_BIT_READ;
// static const uint8_t char_prop_write = ESP_GATT_CHAR_PROP_BIT_WRITE;
static const uint8_t char_prop_read_write = ESP_GATT_CHAR_PROP_BIT_READ | ESP_GATT_CHAR_PROP_BIT_WRITE;
static const uint8_t char_prop_read_write_notify = ESP_GATT_CHAR_PROP_BIT_WRITE | ESP_GATT_CHAR_PROP_BIT_READ | ESP_GATT_CHAR_PROP_BIT_NOTIFY;

static const uint8_t char_value[4] = {0x11, 0x22, 0x33, 0x44};
// end of ble variables

// ble table creation and event handlers
/* Full Database Description - Used to add attributes into the database */
static const esp_gatts_attr_db_t gatt_db[HRS_IDX_NB] =
    {
        // Service Declaration
        [IDX_SVC] =
            {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&primary_service_uuid, ESP_GATT_PERM_READ, sizeof(uint16_t), sizeof(GATTS_SERVICE_UUID_TEST), (uint8_t *)&GATTS_SERVICE_UUID_TEST}},

        /* Characteristic Declaration */
        [IDX_CHAR_A] =
            {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_declaration_uuid, ESP_GATT_PERM_READ, CHAR_DECLARATION_SIZE, CHAR_DECLARATION_SIZE, (uint8_t *)&char_prop_read_write}},

        /* Characteristic Value */
        [IDX_CHAR_VAL_A] =
            {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&GATTS_CHAR_UUID_A, ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE, GATTS_DEMO_CHAR_VAL_LEN_MAX, sizeof(char_value), (uint8_t *)char_value}},

        /* Characteristic Declaration */
        [IDX_CHAR_B] =
            {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_declaration_uuid, ESP_GATT_PERM_READ, CHAR_DECLARATION_SIZE, CHAR_DECLARATION_SIZE, (uint8_t *)&char_prop_read_write}},

        /* Characteristic Value */
        [IDX_CHAR_VAL_B] =
            {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&GATTS_CHAR_UUID_B, ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE, GATTS_DEMO_CHAR_VAL_LEN_MAX, sizeof(char_value), (uint8_t *)char_value}},

        /* Characteristic Declaration */
        [IDX_CHAR_C] =
            {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_declaration_uuid, ESP_GATT_PERM_READ, CHAR_DECLARATION_SIZE, CHAR_DECLARATION_SIZE, (uint8_t *)&char_prop_read_write}},

        /* Characteristic Value */
        [IDX_CHAR_VAL_C] =
            {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&GATTS_CHAR_UUID_C, ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE, GATTS_DEMO_CHAR_VAL_LEN_MAX, sizeof(char_value), (uint8_t *)char_value}},

        /* Characteristic Declaration */
        [IDX_CHAR_D] =
            {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_declaration_uuid, ESP_GATT_PERM_READ, CHAR_DECLARATION_SIZE, CHAR_DECLARATION_SIZE, (uint8_t *)&char_prop_read_write}},

        /* Characteristic Value */
        [IDX_CHAR_VAL_D] =
            {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&GATTS_CHAR_UUID_D, ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE, GATTS_DEMO_CHAR_VAL_LEN_MAX, sizeof(char_value), (uint8_t *)char_value}},

        /* Characteristic Declaration */
        [IDX_CHAR_E] =
            {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_declaration_uuid, ESP_GATT_PERM_READ, CHAR_DECLARATION_SIZE, CHAR_DECLARATION_SIZE, (uint8_t *)&char_prop_read_write_notify}},

        /* Characteristic Value */
        [IDX_CHAR_VAL_E] =
            {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&GATTS_CHAR_UUID_E, ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE, GATTS_DEMO_CHAR_VAL_LEN_MAX, sizeof(char_value), (uint8_t *)char_value}},

};

static void gap_event_handler(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param)
{
    switch (event)
    {
#ifdef CONFIG_SET_RAW_ADV_DATA
    case ESP_GAP_BLE_ADV_DATA_RAW_SET_COMPLETE_EVT:
        adv_config_done &= (~ADV_CONFIG_FLAG);
        if (adv_config_done == 0)
        {
            esp_ble_gap_start_advertising(&adv_params);
        }
        break;
    case ESP_GAP_BLE_SCAN_RSP_DATA_RAW_SET_COMPLETE_EVT:
        adv_config_done &= (~SCAN_RSP_CONFIG_FLAG);
        if (adv_config_done == 0)
        {
            esp_ble_gap_start_advertising(&adv_params);
        }
        break;
#else
    case ESP_GAP_BLE_ADV_DATA_SET_COMPLETE_EVT:
        adv_config_done &= (~ADV_CONFIG_FLAG);
        if (adv_config_done == 0)
        {
            esp_ble_gap_start_advertising(&adv_params);
        }
        break;
    case ESP_GAP_BLE_SCAN_RSP_DATA_SET_COMPLETE_EVT:
        adv_config_done &= (~SCAN_RSP_CONFIG_FLAG);
        if (adv_config_done == 0)
        {
            esp_ble_gap_start_advertising(&adv_params);
        }
        break;
#endif
    case ESP_GAP_BLE_ADV_START_COMPLETE_EVT:
        /* advertising start complete event to indicate advertising start successfully or failed */
        if (param->adv_start_cmpl.status != ESP_BT_STATUS_SUCCESS)
        {
            ESP_LOGE(GATTS_TABLE_TAG, "advertising start failed");
        }
        else
        {
            ESP_LOGI(GATTS_TABLE_TAG, "advertising start successfully");
        }
        break;
    case ESP_GAP_BLE_ADV_STOP_COMPLETE_EVT:
        if (param->adv_stop_cmpl.status != ESP_BT_STATUS_SUCCESS)
        {
            ESP_LOGE(GATTS_TABLE_TAG, "Advertising stop failed");
        }
        else
        {
            ESP_LOGI(GATTS_TABLE_TAG, "Stop adv successfully\n");
        }
        break;
    case ESP_GAP_BLE_UPDATE_CONN_PARAMS_EVT:
        ESP_LOGI(GATTS_TABLE_TAG, "update connection params status = %d, min_int = %d, max_int = %d,conn_int = %d,latency = %d, timeout = %d",
                 param->update_conn_params.status,
                 param->update_conn_params.min_int,
                 param->update_conn_params.max_int,
                 param->update_conn_params.conn_int,
                 param->update_conn_params.latency,
                 param->update_conn_params.timeout);
        break;
    default:
        break;
    }
}

void example_prepare_write_event_env(esp_gatt_if_t gatts_if, prepare_type_env_t *prepare_write_env, esp_ble_gatts_cb_param_t *param)
{
    ESP_LOGI(GATTS_TABLE_TAG, "prepare write, handle = %d, value len = %d", param->write.handle, param->write.len);
    esp_gatt_status_t status = ESP_GATT_OK;
    if (prepare_write_env->prepare_buf == NULL)
    {
        prepare_write_env->prepare_buf = (uint8_t *)malloc(PREPARE_BUF_MAX_SIZE * sizeof(uint8_t));
        prepare_write_env->prepare_len = 0;
        if (prepare_write_env->prepare_buf == NULL)
        {
            ESP_LOGE(GATTS_TABLE_TAG, "%s, Gatt_server prep no mem", __func__);
            status = ESP_GATT_NO_RESOURCES;
        }
    }
    else
    {
        if (param->write.offset > PREPARE_BUF_MAX_SIZE)
        {
            status = ESP_GATT_INVALID_OFFSET;
        }
        else if ((param->write.offset + param->write.len) > PREPARE_BUF_MAX_SIZE)
        {
            status = ESP_GATT_INVALID_ATTR_LEN;
        }
    }
    /*send response when param->write.need_rsp is true */
    if (param->write.need_rsp)
    {
        esp_gatt_rsp_t *gatt_rsp = (esp_gatt_rsp_t *)malloc(sizeof(esp_gatt_rsp_t));
        if (gatt_rsp != NULL)
        {
            gatt_rsp->attr_value.len = param->write.len;
            gatt_rsp->attr_value.handle = param->write.handle;
            gatt_rsp->attr_value.offset = param->write.offset;
            gatt_rsp->attr_value.auth_req = ESP_GATT_AUTH_REQ_NONE;
            memcpy(gatt_rsp->attr_value.value, param->write.value, param->write.len);
            esp_err_t response_err = esp_ble_gatts_send_response(gatts_if, param->write.conn_id, param->write.trans_id, status, gatt_rsp);
            if (response_err != ESP_OK)
            {
                ESP_LOGE(GATTS_TABLE_TAG, "Send response error");
            }
            free(gatt_rsp);
        }
        else
        {
            ESP_LOGE(GATTS_TABLE_TAG, "%s, malloc failed", __func__);
        }
    }
    if (status != ESP_GATT_OK)
    {
        return;
    }
    memcpy(prepare_write_env->prepare_buf + param->write.offset,
           param->write.value,
           param->write.len);
    prepare_write_env->prepare_len += param->write.len;
}

void example_exec_write_event_env(prepare_type_env_t *prepare_write_env, esp_ble_gatts_cb_param_t *param)
{
    if (param->exec_write.exec_write_flag == ESP_GATT_PREP_WRITE_EXEC && prepare_write_env->prepare_buf)
    {
        esp_log_buffer_hex(GATTS_TABLE_TAG, prepare_write_env->prepare_buf, prepare_write_env->prepare_len);
    }
    else
    {
        ESP_LOGI(GATTS_TABLE_TAG, "ESP_GATT_PREP_WRITE_CANCEL");
    }
    if (prepare_write_env->prepare_buf)
    {
        free(prepare_write_env->prepare_buf);
        prepare_write_env->prepare_buf = NULL;
    }
    prepare_write_env->prepare_len = 0;
}

static void gatts_profile_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param)
{
    switch (event)
    {
    case ESP_GATTS_REG_EVT:
    {
        esp_err_t set_dev_name_ret = esp_ble_gap_set_device_name(BLE_DEVICE_NAME);
        if (set_dev_name_ret)
        {
            ESP_LOGE(GATTS_TABLE_TAG, "set device name failed, error code = %x", set_dev_name_ret);
        }
#ifdef CONFIG_SET_RAW_ADV_DATA
        esp_err_t raw_adv_ret = esp_ble_gap_config_adv_data_raw(raw_adv_data, sizeof(raw_adv_data));
        if (raw_adv_ret)
        {
            ESP_LOGE(GATTS_TABLE_TAG, "config raw adv data failed, error code = %x ", raw_adv_ret);
        }
        adv_config_done |= ADV_CONFIG_FLAG;
        esp_err_t raw_scan_ret = esp_ble_gap_config_scan_rsp_data_raw(raw_scan_rsp_data, sizeof(raw_scan_rsp_data));
        if (raw_scan_ret)
        {
            ESP_LOGE(GATTS_TABLE_TAG, "config raw scan rsp data failed, error code = %x", raw_scan_ret);
        }
        adv_config_done |= SCAN_RSP_CONFIG_FLAG;
#else
        // config adv data
        esp_err_t ret = esp_ble_gap_config_adv_data(&adv_data);
        if (ret)
        {
            ESP_LOGE(GATTS_TABLE_TAG, "config adv data failed, error code = %x", ret);
        }
        adv_config_done |= ADV_CONFIG_FLAG;
        // config scan response data
        ret = esp_ble_gap_config_adv_data(&scan_rsp_data);
        if (ret)
        {
            ESP_LOGE(GATTS_TABLE_TAG, "config scan response data failed, error code = %x", ret);
        }
        adv_config_done |= SCAN_RSP_CONFIG_FLAG;
#endif
        esp_err_t create_attr_ret = esp_ble_gatts_create_attr_tab(gatt_db, gatts_if, HRS_IDX_NB, SVC_INST_ID);
        if (create_attr_ret)
        {
            ESP_LOGE(GATTS_TABLE_TAG, "create attr table failed, error code = %x", create_attr_ret);
        }
    }
    break;
    case ESP_GATTS_READ_EVT:
        ESP_LOGI(GATTS_TABLE_TAG, "ESP_GATTS_READ_EVT");
        switch (cnt)
        {
        case 0:
            ssid_handle = param->write.handle;
            cnt = 1;
            break;
        case 1:
            pass_handle = param->write.handle;
            cnt = 2;
            break;
        case 2:
            mqtt_handle = param->write.handle;
            cnt = 3;
            break;
        case 3:
            name_handle = param->write.handle;
            cnt = 4;
            break;
        case 4:
            status_handle = param->write.handle;
            cnt = 0;
            break;
        default:
            break;
        }
        ESP_LOGI(GATTS_TABLE_TAG, "ssid_handle: %d, pass_handle: %d, mqtt_handle: %d, name_handle: %d", ssid_handle, pass_handle, mqtt_handle, name_handle);
        break;
    case ESP_GATTS_WRITE_EVT:
        if (!param->write.is_prep)
        {
            // the data length of gattc write  must be less than GATTS_DEMO_CHAR_VAL_LEN_MAX.
            ESP_LOGI(GATTS_TABLE_TAG, "GATT_WRITE_EVT, handle = %d, value len = %d, value :", param->write.handle, param->write.len);
            ESP_LOGI(GATTS_TABLE_TAG, "%s", param->write.value);
            int length = param->write.len;
            if (param->write.handle == ssid_handle)
            {
                memcpy(ssid, param->write.value, length);
            }
            else if (param->write.handle == pass_handle)
            {
                memcpy(pass, param->write.value, length);
            }
            else if (param->write.handle == mqtt_handle)
            {
                memcpy(mqtt_ip, param->write.value, length);
            }
            else if (param->write.handle == name_handle)
            {
                memcpy(name_device, param->write.value, length);
            }
            /* send response when param->write.need_rsp is true*/
            if (param->write.need_rsp)
            {
                esp_ble_gatts_send_response(gatts_if, param->write.conn_id, param->write.trans_id, ESP_GATT_OK, NULL);
            }
        }
        else
        {
            /* handle prepare write */
            example_prepare_write_event_env(gatts_if, &prepare_write_env, param);
        }
        break;
    case ESP_GATTS_EXEC_WRITE_EVT:
        // the length of gattc prepare write data must be less than GATTS_DEMO_CHAR_VAL_LEN_MAX.
        ESP_LOGI(GATTS_TABLE_TAG, "ESP_GATTS_EXEC_WRITE_EVT");
        example_exec_write_event_env(&prepare_write_env, param);
        break;
    case ESP_GATTS_MTU_EVT:
        ESP_LOGI(GATTS_TABLE_TAG, "ESP_GATTS_MTU_EVT, MTU %d", param->mtu.mtu);
        break;
    case ESP_GATTS_CONF_EVT:
        ESP_LOGI(GATTS_TABLE_TAG, "ESP_GATTS_CONF_EVT, status = %d, attr_handle %d", param->conf.status, param->conf.handle);
        break;
    case ESP_GATTS_START_EVT:
        ESP_LOGI(GATTS_TABLE_TAG, "SERVICE_START_EVT, status %d, service_handle %d", param->start.status, param->start.service_handle);
        break;
    case ESP_GATTS_CONNECT_EVT:
        ESP_LOGI(GATTS_TABLE_TAG, "ESP_GATTS_CONNECT_EVT, conn_id = %d", param->connect.conn_id);
        esp_log_buffer_hex(GATTS_TABLE_TAG, param->connect.remote_bda, 6);
        esp_ble_conn_update_params_t conn_params = {0};
        memcpy(conn_params.bda, param->connect.remote_bda, sizeof(esp_bd_addr_t));
        /* For the iOS system, please refer to Apple official documents about the BLE connection parameters restrictions. */
        conn_params.latency = 0;
        conn_params.max_int = 0x20; // max_int = 0x20*1.25ms = 40ms
        conn_params.min_int = 0x10; // min_int = 0x10*1.25ms = 20ms
        conn_params.timeout = 400;  // timeout = 400*10ms = 4000ms
        // start sent the update connection parameters to the peer device.
        esp_ble_gap_update_conn_params(&conn_params);
        device_connected = true;
        break;
    case ESP_GATTS_DISCONNECT_EVT:
        ESP_LOGI(GATTS_TABLE_TAG, "ESP_GATTS_DISCONNECT_EVT, reason = 0x%x", param->disconnect.reason);
        device_connected = false;
        esp_ble_gap_start_advertising(&adv_params);
        break;
    case ESP_GATTS_CREAT_ATTR_TAB_EVT:
    {
        if (param->add_attr_tab.status != ESP_GATT_OK)
        {
            ESP_LOGE(GATTS_TABLE_TAG, "create attribute table failed, error code=0x%x", param->add_attr_tab.status);
        }
        else if (param->add_attr_tab.num_handle != HRS_IDX_NB)
        {
            ESP_LOGE(GATTS_TABLE_TAG, "create attribute table abnormally, num_handle (%d) \
                        doesn't equal to HRS_IDX_NB(%d)",
                     param->add_attr_tab.num_handle, HRS_IDX_NB);
        }
        else
        {
            ESP_LOGI(GATTS_TABLE_TAG, "create attribute table successfully, the number handle = %d\n", param->add_attr_tab.num_handle);
            memcpy(heart_rate_handle_table, param->add_attr_tab.handles, sizeof(heart_rate_handle_table));
            esp_ble_gatts_start_service(heart_rate_handle_table[IDX_SVC]);
        }
        break;
    }
    case ESP_GATTS_STOP_EVT:
    case ESP_GATTS_OPEN_EVT:
    case ESP_GATTS_CANCEL_OPEN_EVT:
    case ESP_GATTS_CLOSE_EVT:
    case ESP_GATTS_LISTEN_EVT:
    case ESP_GATTS_CONGEST_EVT:
    case ESP_GATTS_UNREG_EVT:
    case ESP_GATTS_DELETE_EVT:
    default:
        break;
    }
}

static void gatts_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param)
{

    /* If event is register event, store the gatts_if for each profile */
    if (event == ESP_GATTS_REG_EVT)
    {
        if (param->reg.status == ESP_GATT_OK)
        {
            heart_rate_profile_tab[PROFILE_APP_IDX].gatts_if = gatts_if;
        }
        else
        {
            ESP_LOGE(GATTS_TABLE_TAG, "reg app failed, app_id %04x, status %d",
                     param->reg.app_id,
                     param->reg.status);
            return;
        }
    }
    do
    {
        int idx;
        for (idx = 0; idx < PROFILE_NUM; idx++)
        {
            /* ESP_GATT_IF_NONE, not specify a certain gatt_if, need to call every profile cb function */
            if (gatts_if == ESP_GATT_IF_NONE || gatts_if == heart_rate_profile_tab[idx].gatts_if)
            {
                if (heart_rate_profile_tab[idx].gatts_cb)
                {
                    heart_rate_profile_tab[idx].gatts_cb(event, gatts_if, param);
                }
            }
        }
    } while (0);
}

void ble_setup(void)
{
    esp_err_t ret;

    ESP_ERROR_CHECK(esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT));

    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    ret = esp_bt_controller_init(&bt_cfg);
    if (ret)
    {
        ESP_LOGE(GATTS_TABLE_TAG, "%s enable controller failed: %s", __func__, esp_err_to_name(ret));
        return;
    }

    ret = esp_bt_controller_enable(ESP_BT_MODE_BLE);
    if (ret)
    {
        ESP_LOGE(GATTS_TABLE_TAG, "%s enable controller failed: %s", __func__, esp_err_to_name(ret));
        return;
    }

    ret = esp_bluedroid_init();
    if (ret)
    {
        ESP_LOGE(GATTS_TABLE_TAG, "%s init bluetooth failed: %s", __func__, esp_err_to_name(ret));
        return;
    }

    ret = esp_bluedroid_enable();
    if (ret)
    {
        ESP_LOGE(GATTS_TABLE_TAG, "%s enable bluetooth failed: %s", __func__, esp_err_to_name(ret));
        return;
    }

    ret = esp_ble_gatts_register_callback(gatts_event_handler);
    if (ret)
    {
        ESP_LOGE(GATTS_TABLE_TAG, "gatts register error, error code = %x", ret);
        return;
    }

    ret = esp_ble_gap_register_callback(gap_event_handler);
    if (ret)
    {
        ESP_LOGE(GATTS_TABLE_TAG, "gap register error, error code = %x", ret);
        return;
    }

    ret = esp_ble_gatts_app_register(ESP_APP_ID);
    if (ret)
    {
        ESP_LOGE(GATTS_TABLE_TAG, "gatts app register error, error code = %x", ret);
        return;
    }

    esp_err_t local_mtu_ret = esp_ble_gatt_set_local_mtu(500);
    if (local_mtu_ret)
    {
        ESP_LOGE(GATTS_TABLE_TAG, "set local  MTU failed, error code = %x", local_mtu_ret);
    }

    bluetooth_online = true;
}
//

// led blink test working
void blink_led(int LED, int freq)
{
    gpio_set_level(leds[LED], 0);
    vTaskDelay(freq / 2);
    gpio_set_level(leds[LED], 1);
    vTaskDelay(freq / 2);
}
//
// wifi event handler
static void event_handler(void *arg, esp_event_base_t event_base,
                          int32_t event_id, void *event_data)
{
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START)
    {
        esp_wifi_connect();
    }
    else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED)
    {
        if (s_retry_num < EXAMPLE_ESP_MAXIMUM_RETRY)
        {
            esp_wifi_connect();
            s_retry_num++;
            ESP_LOGI("WIFI", "retry to connect to the AP");
        }
        else
        {
            xEventGroupSetBits(s_wifi_event_group, WIFI_FAIL_BIT);
        }
        ESP_LOGI("WIFI", "connect to the AP fail");
    }
    else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP)
    {
        ip_event_got_ip_t *event = (ip_event_got_ip_t *)event_data;
        ESP_LOGI("WIFI", "got ip:" IPSTR, IP2STR(&event->ip_info.ip));
        s_retry_num = 0;
        xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
    }
}
//

// mqtt event handler
static void mqtt_event_handler(void *handler_args, esp_event_base_t base, int32_t event_id, void *event_data)
{
    esp_mqtt_event_handle_t event = event_data;
    esp_mqtt_client_handle_t intrnclient = event->client;
    int msg_id;
    switch ((esp_mqtt_event_id_t)event_id)
    {
    case MQTT_EVENT_CONNECTED:
        ESP_LOGI("MQTT", "MQTT_EVENT_CONNECTED");
        //msg_id = esp_mqtt_client_publish(intrnclient, state_topic, handler_args, 0, 1, 0);
        //ESP_LOGI("MQTT", "sent publish successful, msg_id=%d", msg_id);

        //msg_id = esp_mqtt_client_publish(intrnclient, info_topic, handler_args, 0, 1, 1);
        //ESP_LOGI("MQTT", "sent publish successful, msg_id=%d", msg_id);

        msg_id = esp_mqtt_client_subscribe(intrnclient, command_topic, 0);
        ESP_LOGI("MQTT", "sent subscribe successful, msg_id=%d", msg_id);

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
        command_len = event->data_len;
        memcpy(commands, event->data, command_len);
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

// mqtt setup
void mqtt_app_start(void)
{
    sprintf(info_topic, "lvups/%s/INFO", name_device);
    sprintf(state_topic, "lvups/%s/STATE", name_device);
    sprintf(command_topic, "lvups/%s/COMMAND",name_device);
    sprintf(mqtt_url, "mqtt://%s", mqtt_ip);
    esp_mqtt_client_config_t mqtt_cfg = {
        .broker.address.uri = mqtt_url,
    };

    client = esp_mqtt_client_init(&mqtt_cfg);
    /* The last argument may be used to pass data to the event handler, in this example mqtt_event_handler */
    esp_mqtt_client_register_event(client, ESP_EVENT_ANY_ID, mqtt_event_handler, NULL);
    esp_mqtt_client_start(client);
    vTaskDelay(200);
    esp_mqtt_client_publish(client, info_topic , "{\"Firmware_version\":\"v0.0.1\",\"Hardware_version\":\"sucks\"}", 0, 1, 1);
}
//

// Wifi setup
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
            .ssid = "",
            .password = "",
        },
    };

    strncpy((char *)wifi_config.sta.ssid, ssid, sizeof(wifi_config.sta.ssid) - 1);
    strncpy((char *)wifi_config.sta.password, pass, sizeof(wifi_config.sta.password) - 1);

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());
    vTaskDelay(200);
    mqtt_app_start();
}
//

void wifi_flash(void)
{
    esp_err_t ret = nvs_open(FLASH_NAMESPACE, NVS_READWRITE, &wifi_handle);
    ESP_LOGI("flash mem", "NVS open return: %s", esp_err_to_name(ret));

    ret = nvs_get_str(wifi_handle, "ssid_key", &ssid, &ssid_size);
    ret = nvs_get_str(wifi_handle, "pass_key", &pass, &pass_size);
    ret = nvs_get_str(wifi_handle, "mqtt_key", &mqtt_ip, &mqtt_ip_size);
    ret = nvs_get_str(wifi_handle, "name_key", &name_device, &name_size);
    
    if (ret != ESP_OK)
    {
        ESP_LOGI("flash mem", "Failed to read NVS flash");
        ESP_LOGI("flash mem", "ERROR: %s", esp_err_to_name(ret));
    }
    else
    {
        ESP_LOGI("flash mem", "Variables read");
    }
    nvs_close(wifi_handle);

    if (strlen(ssid) != 0 && strlen(pass) != 0 && strlen(mqtt_ip) != 0)
    {
        if (wifi_online == false)
        {
            wifi_init_sta();
        }
    }
    else
    {
        if (bluetooth_online == false)
        {
            ble_setup();
        }
    }
}

void gpio_setup(){
    gpio_set_direction(leds[BLE_LED], GPIO_MODE_OUTPUT);
    gpio_set_direction(leds[MQTT_LED], GPIO_MODE_OUTPUT);
    gpio_set_direction(BUTTON_1, GPIO_MODE_INPUT);
    gpio_set_pull_mode(BUTTON_1, GPIO_PULLUP_ONLY);
    gpio_set_level(leds[BLE_LED], 1);
    gpio_set_level(leds[MQTT_LED], 1);
}

cJSON* json_setup(void){
    cJSON *lvups_json = cJSON_CreateObject();
    cJSON_AddNumberToObject(lvups_json, "Uptime", 0);
    cJSON_AddBoolToObject(lvups_json, "Charging_battery", false);
    cJSON_AddNumberToObject(lvups_json, "Battery_percentage", 0);
    cJSON_AddBoolToObject(lvups_json, "Using_battery", false);
    cJSON_AddBoolToObject(lvups_json, "Receiving_power", false);
    cJSON_AddStringToObject(lvups_json, "Charge_time", "0");
    cJSON_AddStringToObject(lvups_json, "Discharge_time", "0");
    cJSON_AddStringToObject(lvups_json, "Discharge_time_ml", "0");
    return lvups_json;
}

void app_main(void)
{
    esp_timer_early_init();
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND)
    {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    gpio_setup();
    wifi_flash();
    cJSON *lvups_json = json_setup();

    while (bluetooth_online == true)
    {
        if (strlen(ssid) != 0 && strlen(pass) != 0 && strlen(mqtt_ip) != 0)
        {
            ESP_LOGI("SETTINGS", "ACCEPTED");

            ret = nvs_open(FLASH_NAMESPACE, NVS_READWRITE, &wifi_handle);
            if (ret != ESP_OK)
            {
                ESP_LOGI("flash mem", "Error (%s) opening NVS handle", esp_err_to_name(ret));
            }
            else
            {
                ret = nvs_set_str(wifi_handle, "ssid_key", ssid);
                ret = nvs_set_str(wifi_handle, "pass_key", pass);
                ret = nvs_set_str(wifi_handle, "mqtt_key", mqtt_ip);
                ret = nvs_set_str(wifi_handle, "name_key", name_device);

                if (ret != ESP_OK)
                {
                    ESP_LOGI("flash mem", "Error (%s) writing NVS variables", esp_err_to_name(ret));
                }
                else
                {
                    ESP_LOGI("flash mem", "Variables written to NVS successfully");
                }
                ret = nvs_commit(wifi_handle);
                if (ret != ESP_OK)
                {
                    ESP_LOGI("nvs commit", "Error (%s)", esp_err_to_name(ret));
                }
            }

            nvs_close(wifi_handle);
            esp_restart();
        }

        ESP_LOGI("device name", "%s", name_device);

        if (device_connected == false)
        {
            blink_led(BLE_LED, 50);
        }
        else
        {
            gpio_set_level(leds[BLE_LED], 0);
            vTaskDelay(10);
        }
    }

    while (1)
    {
        cJSON *commands_json = cJSON_Parse(commands);

        cJSON *charging_command = cJSON_GetObjectItem(commands_json, "Charge_battery");
        cJSON *restart_command = cJSON_GetObjectItem(commands_json, "Restart");
        //cJSON *use_battery_command = cJSON_GetObjectItem(commands_json, "Use_battery");
        cJSON *size_command = cJSON_GetObjectItem(commands_json, "Battery_size");
        cJSON *min_charge_command = cJSON_GetObjectItem(commands_json, "Min_battery_charge");
        cJSON *max_charge_command = cJSON_GetObjectItem(commands_json, "Max_battery_charge");

        if (size_command != NULL){
            battery_size = cJSON_GetNumberValue(size_command);
        }

        if (min_charge_command != NULL){
            battery_charge_min = cJSON_GetNumberValue(min_charge_command);
        }

        if (max_charge_command != NULL){
            battery_charge_max = cJSON_GetNumberValue(max_charge_command);
        }
        
        if (cJSON_IsTrue(charging_command))
        {
            charging = true;
        }
        else
        {
            charging = false;
        }

        if (cJSON_IsTrue(restart_command)){
            esp_restart();
        }

        ESP_LOGI("charging", "%d", charging);
        ESP_LOGI("size", "%d", battery_size);
        ESP_LOGI("min charge", "%d", battery_charge_min);
        ESP_LOGI("max charge", "%d", battery_charge_max);
        ESP_LOGI("device name", "%s", name_device);
        ESP_LOGI("topics", "%s, %s, %s", info_topic, state_topic, command_topic);
        
        cJSON_Delete(commands_json);

        int64_t uptime = esp_timer_get_time();
        uptime = uptime / 1000000;
        printf("uptime %lld \n", uptime);

        cJSON_ReplaceItemInObject(lvups_json, "Uptime", cJSON_CreateNumber(uptime));

        lvups_status = cJSON_Print(lvups_json);
        esp_mqtt_client_publish(client, state_topic, lvups_status, 0, 0, 0);

        int nvs_button = gpio_get_level(BUTTON_1);
        if (nvs_button == 0)
        {
            nvs_flash_erase();
            ESP_LOGI("button", "nvs erased");
        }

        blink_led(MQTT_LED, 10);

        vTaskDelay(100);
    }
}