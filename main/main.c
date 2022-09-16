#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "nvs_flash.h"
#include "esp_log.h"
#include "esp_system.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"

#include "esp_bt.h"
#include "esp_gap_ble_api.h"
#include "esp_gatts_api.h"
#include "esp_bt_defs.h"
#include "esp_bt_main.h"
#include "esp_gatt_common_api.h"


#define PROFILE_TRACTIAN_APP_ID     0
#define GATTS_SERVICE_UUID_FILE     0x00FF
#define GATTS_CHAR_UUID_FILE        0xFF01
#define GATTS_DESCR_UUID_FILE       0x3333
#define GATTS_NUM_HANDLE_FILE       4

#define TEST_DEVICE_NAME "TRACTIAN_CHLGR"   
#define PREPARE_BUF_MAX_SIZE 1024
#define ADV_CONFIG_FLAG      (1 << 0)


typedef struct 
{
    uint8_t                 *prepare_buf;
    int                     prepare_len;
} prepare_type_env_t;


// The length of adv data must be less than 31 bytes
//adv data
static esp_ble_adv_data_t adv_data = {
    .set_scan_rsp = false,
    .include_name = true,
    .include_txpower = false,
    .min_interval = 0x0006, //slave connection min interval, Time = min_interval * 1.25 msec
    .max_interval = 0x0010, //slave connection max interval, Time = max_interval * 1.25 msec
    .appearance = 0x00,
    .manufacturer_len = sizeof(manufacturer_name), //TEST_MANUFACTURER_DATA_LEN,
    .p_manufacturer_data =  (uint8_t*)manufacturer_name, //&test_manufacturer[0],
    .service_data_len = 0,
    .p_service_data = NULL,
    .service_uuid_len = sizeof(service_uuid128),
    .p_service_uuid = service_uuid128,
    .flag = (ESP_BLE_ADV_FLAG_GEN_DISC | ESP_BLE_ADV_FLAG_BREDR_NOT_SPT),
};

static esp_ble_adv_params_t adv_params = {
    .adv_int_min        = 0x20,
    .adv_int_max        = 0x40,
    .adv_type           = ADV_TYPE_IND,
    .own_addr_type      = BLE_ADDR_TYPE_PUBLIC,
    .channel_map        = ADV_CHNL_ALL,
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

static struct gatts_profile_inst gl_profile_tab = {
        .gatts_cb = gatts_profile_event_handler,
        .gatts_if = ESP_GATT_IF_NONE,       /* Not get the gatt_if, so initial is ESP_GATT_IF_NONE */
};

static prepare_type_env_t a_prepare_write_env;
static esp_gatt_char_prop_t a_property = 0;
static uint8_t adv_config_done = 0;
static char manufacturer_name[] = "MarconeTenorio";
const char TAG[] = "TRACTIAN_CHALLENGE";
static uint8_t service_uuid128[32] = {
    /* LSB <--------------------------------------------------------------------------------> MSB */
    0xfb, 0x34, 0x9b, 0x5f, 0x80, 0x00, 0x00, 0x80, 0x00, 0x10, 0x00, 0x00, 0xEE, 0x00, 0x00, 0x00,
    0xfb, 0x34, 0x9b, 0x5f, 0x80, 0x00, 0x00, 0x80, 0x00, 0x10, 0x00, 0x00, 0xFF, 0x00, 0x00, 0x00,
};

void example_write_event_env(esp_gatt_if_t gatts_if, prepare_type_env_t *prepare_write_env, esp_ble_gatts_cb_param_t *param);
static void gatts_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param);
static void gatts_profile_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param);
static void gap_event_handler(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param);

void setup(void)
{
    esp_err_t ret;

    /*Será necessário iniciar o acesso a flash para acessar
    o arquivo de 500kb pois a memória ram das esp32 possui apenas 512kb */

    ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) 
    {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK( ret );

    ESP_LOGI(TAG, "NVS inicializado com sucesso");

    // evita consumo de memória desnecessário uma vez que apenas irei utilizar o bluetooth low energy e não o bluetooth classic
    ESP_ERROR_CHECK(esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT));
    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    esp_bt_controller_init(&bt_cfg);
    esp_bt_controller_enable(ESP_BT_MODE_BLE);
    esp_bluedroid_init();
    esp_bluedroid_enable();
    esp_ble_gatts_register_callback(gatts_event_handler);
    esp_ble_gap_register_callback(gap_event_handler);
    esp_ble_gatts_app_register(PROFILE_TRACTIAN_APP_ID);
    esp_ble_gatt_set_local_mtu(500);
}

void loop_routine(void* pvParameter)
{
    while(1)
    {
        vTaskDelay(100/portTICK_PERIOD_MS); //delay para evitar watchdog
    }
}

void app_main(void)
{
    setup();
    xTaskCreate(loop_routine, "BLUETOOTH_ROUTINE", 60*1024, NULL, 3, NULL);
}

static void gatts_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param)
{
    /* If event is register event, store the gatts_if for each profile */
    if (event == ESP_GATTS_REG_EVT) 
    {
        if (param->reg.status == ESP_GATT_OK) 
        {
            gl_profile_tab.gatts_if = gatts_if;
        } else {
            ESP_LOGI(TAG, "Reg app failed, app_id %04x, status %d\n",
                    param->reg.app_id,
                    param->reg.status);
            return;
        }
    }

    /* If the gatts_if equal to profile A, call profile A cb handler,
     * so here call each profile's callback */
    do {
        if (gatts_if == ESP_GATT_IF_NONE || /* ESP_GATT_IF_NONE, not specify a certain gatt_if, need to call every profile cb function */
                gatts_if == gl_profile_tab.gatts_if) 
        {
            if (gl_profile_tab.gatts_cb) 
            {
                gl_profile_tab.gatts_cb(event, gatts_if, param);
            }
        }
    } while (0);
}

static void gatts_profile_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param) 
{
    ESP_LOGE(TAG, "EVENT GATT: %d", (int)event);
    switch (event) 
    {
        case ESP_GATTS_REG_EVT:
            ESP_LOGI(TAG, "REGISTER_APP_EVT, status %d, app_id %d\n", param->reg.status, param->reg.app_id);
            gl_profile_tab.service_id.is_primary = true;
            gl_profile_tab.service_id.id.inst_id = 0x00;
            gl_profile_tab.service_id.id.uuid.len = ESP_UUID_LEN_16;
            gl_profile_tab.service_id.id.uuid.uuid.uuid16 = GATTS_SERVICE_UUID_FILE;

            esp_err_t set_dev_name_ret = esp_ble_gap_set_device_name(TEST_DEVICE_NAME);
            if (set_dev_name_ret) ESP_LOGE(TAG, "set device name failed, error code = %x", set_dev_name_ret);

            //config adv data
            esp_err_t ret = esp_ble_gap_config_adv_data(&adv_data);
            if (ret) ESP_LOGE(TAG, "config adv data failed, error code = %x", ret);
            adv_config_done |= ADV_CONFIG_FLAG;

            esp_ble_gatts_create_service(gatts_if, &gl_profile_tab.service_id, GATTS_NUM_HANDLE_FILE);
            break;

        case ESP_GATTS_READ_EVT: 
        {
            ESP_LOGI(TAG, "GATT_READ_EVT, conn_id %d, trans_id %d, handle %d\n", param->read.conn_id, param->read.trans_id, param->read.handle);
            
            esp_gatt_rsp_t rsp;
            memset(&rsp, 0, sizeof(esp_gatt_rsp_t));
            
            rsp.attr_value.handle = param->read.handle;
            rsp.attr_value.len = 4;
            rsp.attr_value.value[0] = 0xde;
            rsp.attr_value.value[1] = 0xed;
            rsp.attr_value.value[2] = 0xbe;
            rsp.attr_value.value[3] = 0xef;

            esp_ble_gatts_send_response(gatts_if, param->read.conn_id, param->read.trans_id,
                                        ESP_GATT_OK, &rsp);
            break;
        }

        case ESP_GATTS_WRITE_EVT: 
        {
            ESP_LOGI(TAG, "GATT_WRITE_EVT, conn_id %d, trans_id %d, handle %d", param->write.conn_id, param->write.trans_id, param->write.handle);
            if (!param->write.is_prep)
            {
                ESP_LOGI(TAG, "GATT_WRITE_EVT, value len %d, value :", param->write.len);
                esp_log_buffer_hex(TAG, param->write.value, param->write.len);
            }
            example_write_event_env(gatts_if, &a_prepare_write_env, param);
            break;
        }        

        case ESP_GATTS_CREATE_EVT:
            ESP_LOGI(TAG, "CREATE_SERVICE_EVT, status %d,  service_handle %d\n", param->create.status, param->create.service_handle);
            gl_profile_tab.service_handle = param->create.service_handle;
            gl_profile_tab.char_uuid.len = ESP_UUID_LEN_16;
            gl_profile_tab.char_uuid.uuid.uuid16 = GATTS_CHAR_UUID_FILE;

            esp_ble_gatts_start_service(gl_profile_tab.service_handle);
            a_property = ESP_GATT_CHAR_PROP_BIT_READ | ESP_GATT_CHAR_PROP_BIT_WRITE;
            esp_err_t add_char_ret = esp_ble_gatts_add_char(gl_profile_tab.service_handle, &gl_profile_tab.char_uuid,
                                                            ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
                                                            a_property,
                                                            NULL, NULL);

            if (add_char_ret) ESP_LOGE(TAG, "add char failed, error code =%x",add_char_ret);
            break;

        case ESP_GATTS_ADD_CHAR_EVT: 
        {
            uint16_t length = 0;
            const uint8_t *prf_char;

            ESP_LOGI(TAG, "ADD_CHAR_EVT, status %d,  attr_handle %d, service_handle %d\n",
                    param->add_char.status, param->add_char.attr_handle, param->add_char.service_handle);

            gl_profile_tab.char_handle = param->add_char.attr_handle;
            gl_profile_tab.descr_uuid.len = ESP_UUID_LEN_16;
            gl_profile_tab.descr_uuid.uuid.uuid16 = ESP_GATT_UUID_CHAR_CLIENT_CONFIG;

            esp_err_t get_attr_ret = esp_ble_gatts_get_attr_value(param->add_char.attr_handle, &length, &prf_char);

            if (get_attr_ret == ESP_FAIL) ESP_LOGE(TAG, "ILLEGAL HANDLE"); 

            esp_err_t add_descr_ret = esp_ble_gatts_add_char_descr(gl_profile_tab.service_handle, &gl_profile_tab.descr_uuid,
                                                                    ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE, NULL, NULL);
            if (add_descr_ret) ESP_LOGE(TAG, "add char descr failed, error code =%x", add_descr_ret);
            break;
        }

        case ESP_GATTS_START_EVT:
            ESP_LOGI(TAG, "SERVICE_START_EVT, status %d, service_handle %d\n",
                    param->start.status, param->start.service_handle);
            break;

        case ESP_GATTS_CONNECT_EVT: 
        {
            esp_ble_conn_update_params_t conn_params = {0};
            memcpy(conn_params.bda, param->connect.remote_bda, sizeof(esp_bd_addr_t));
            /* For the IOS system, please reference the apple official documents about the ble connection parameters restrictions. */
            conn_params.latency = 0;
            conn_params.max_int = 0x20;    // max_int = 0x20*1.25ms = 40ms
            conn_params.min_int = 0x10;    // min_int = 0x10*1.25ms = 20ms
            conn_params.timeout = 400;    // timeout = 400*10ms = 4000ms
            ESP_LOGI(TAG, "ESP_GATTS_CONNECT_EVT, conn_id %d, remote %02x:%02x:%02x:%02x:%02x:%02x:",
                    param->connect.conn_id,
                    param->connect.remote_bda[0], param->connect.remote_bda[1], param->connect.remote_bda[2],
                    param->connect.remote_bda[3], param->connect.remote_bda[4], param->connect.remote_bda[5]);
            gl_profile_tab.conn_id = param->connect.conn_id;
            //start sent the update connection parameters to the peer device.
            esp_ble_gap_update_conn_params(&conn_params);
            break;
        }

        case ESP_GATTS_DISCONNECT_EVT:
            ESP_LOGI(TAG, "ESP_GATTS_DISCONNECT_EVT, disconnect reason 0x%x", param->disconnect.reason);
            esp_ble_gap_start_advertising(&adv_params);
            break;

        case ESP_GATTS_CONF_EVT:
            ESP_LOGI(TAG, "ESP_GATTS_CONF_EVT, status %d attr_handle %d", param->conf.status, param->conf.handle);
            if (param->conf.status != ESP_GATT_OK) esp_log_buffer_hex(TAG, param->conf.value, param->conf.len);
            break;

        case ESP_GATTS_OPEN_EVT:
        case ESP_GATTS_CANCEL_OPEN_EVT:
        case ESP_GATTS_CLOSE_EVT:
        case ESP_GATTS_LISTEN_EVT:
        case ESP_GATTS_CONGEST_EVT:

        default:
            break;
        }
}

static void gap_event_handler(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param)
{
    ESP_LOGE(TAG, "EVENT GAP: %d", (int)event);
    switch (event) 
    {
        case ESP_GAP_BLE_ADV_DATA_SET_COMPLETE_EVT:
            adv_config_done &= (~ADV_CONFIG_FLAG);
            if (adv_config_done == 0)
            {
                esp_ble_gap_start_advertising(&adv_params);
            }
            break;

        case ESP_GAP_BLE_ADV_START_COMPLETE_EVT:
            //advertising start complete event to indicate advertising start successfully or failed
            if (param->adv_start_cmpl.status != ESP_BT_STATUS_SUCCESS)
            {
                ESP_LOGE(TAG, "Advertising start failed\n");
            }
            break;

        case ESP_GAP_BLE_ADV_STOP_COMPLETE_EVT:
            if (param->adv_stop_cmpl.status != ESP_BT_STATUS_SUCCESS)
            {
                ESP_LOGE(TAG, "Advertising stop failed\n");
            } else {
                ESP_LOGI(TAG, "Stop adv successfully\n");
            }
            break;

        case ESP_GAP_BLE_UPDATE_CONN_PARAMS_EVT:
            ESP_LOGI(TAG, "update connection params status = %d, min_int = %d, max_int = %d,conn_int = %d,latency = %d, timeout = %d",
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

void example_write_event_env(esp_gatt_if_t gatts_if, prepare_type_env_t *prepare_write_env, esp_ble_gatts_cb_param_t *param)
{
    esp_gatt_status_t status = ESP_GATT_OK;
    if (param->write.need_rsp)
    {
        if (param->write.is_prep)
        {
            if (prepare_write_env->prepare_buf == NULL) 
            {
                prepare_write_env->prepare_buf = (uint8_t *)malloc(PREPARE_BUF_MAX_SIZE*sizeof(uint8_t));
                prepare_write_env->prepare_len = 0;

                if (prepare_write_env->prepare_buf == NULL) 
                {
                    ESP_LOGE(TAG, "Gatt_server prep no mem\n");
                    status = ESP_GATT_NO_RESOURCES;
                }
            } else {
                if(param->write.offset > PREPARE_BUF_MAX_SIZE) 
                {
                    status = ESP_GATT_INVALID_OFFSET;
                } else if ((param->write.offset + param->write.len) > PREPARE_BUF_MAX_SIZE) {
                    status = ESP_GATT_INVALID_ATTR_LEN;
                }
            }

            esp_gatt_rsp_t *gatt_rsp = (esp_gatt_rsp_t *)malloc(sizeof(esp_gatt_rsp_t));
            gatt_rsp->attr_value.len = param->write.len;
            gatt_rsp->attr_value.handle = param->write.handle;
            gatt_rsp->attr_value.offset = param->write.offset;
            gatt_rsp->attr_value.auth_req = ESP_GATT_AUTH_REQ_NONE;
            memcpy(gatt_rsp->attr_value.value, param->write.value, param->write.len);
            
            esp_err_t response_err = esp_ble_gatts_send_response(gatts_if, param->write.conn_id, param->write.trans_id, status, gatt_rsp);
            if (response_err != ESP_OK) ESP_LOGE(TAG, "Send response error\n"); 

            free(gatt_rsp);
            if (status != ESP_GATT_OK) return; 

            memcpy(prepare_write_env->prepare_buf + param->write.offset,
                   param->write.value,
                   param->write.len);
            prepare_write_env->prepare_len += param->write.len;

        } else {
            esp_ble_gatts_send_response(gatts_if, param->write.conn_id, param->write.trans_id, status, NULL);
        }
    }
}