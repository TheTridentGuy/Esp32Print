#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_bt.h"
#include "esp_bt_main.h"
#include "esp_gap_ble_api.h"
#include "esp_gattc_api.h"
#include "nvs_flash.h"


#define INIT_TAG "INIT"
#define GATTC_TAG "GATT_CLIENT"
#define GAP_TAG "GAP"

static esp_gatt_if_t client_if;
static esp_bd_addr_t printer_addr;
static bool found_printer = false;


void gattc_cb(esp_gattc_cb_event_t event, esp_gatt_if_t gattc_if, esp_ble_gattc_cb_param_t *param){
    if(event == ESP_GATTC_REG_EVT){
        ESP_LOGI(GATTC_TAG, "GATT client registered...");
        client_if = gattc_if;
        esp_ble_scan_params_t scan_params = {
            .scan_type = BLE_SCAN_TYPE_ACTIVE,
            .own_addr_type = BLE_ADDR_TYPE_PUBLIC,
            .scan_filter_policy = BLE_SCAN_FILTER_ALLOW_ALL,
            .scan_interval = 0x50,
            .scan_window = 0x30,
        };
        ESP_ERROR_CHECK(esp_ble_gap_set_scan_params(&scan_params));
    }else if(event == ESP_GATTC_OPEN_EVT){
        ESP_LOGI(GATTC_TAG, "Connected to printer...");
    }else if(event == ESP_GATTC_CLOSE_EVT){
        ESP_LOGI(GATTC_TAG, "Disconnected from printer...");
    }else if(event == ESP_GATTC_READ_CHAR_EVT){
        ESP_LOGI(GATTC_TAG, "Read characteristic...");
    }else if(event == ESP_GATTC_WRITE_CHAR_EVT){
        ESP_LOGI(GATTC_TAG, "Wrote to characteristic...");
    }else{
        ESP_LOGI(GATTC_TAG, "GATT event: %d", event);
    }
}

void gap_ble_cb(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param){
    if(event == ESP_GAP_BLE_SCAN_PARAM_SET_COMPLETE_EVT){
        ESP_LOGI(GAP_TAG, "Scan params set...");
        ESP_ERROR_CHECK(esp_ble_gap_start_scanning(10));
    }else if(event == ESP_GAP_BLE_SCAN_RESULT_EVT){
        ESP_LOGI(GAP_TAG, "Device found: %02X:%02X:%02X:%02X:%02X:%02X",
            param->scan_rst.bda[0], param->scan_rst.bda[1], param->scan_rst.bda[2],
            param->scan_rst.bda[3], param->scan_rst.bda[4], param->scan_rst.bda[5]);
        if(!found_printer && param->scan_rst.search_evt == ESP_GAP_SEARCH_INQ_RES_EVT){
            char *device_name = (char *)param->scan_rst.ble_adv;
            ESP_LOGI(GAP_TAG, "Device name: %s", device_name);
            if(strstr(device_name, "MXW01") != NULL){
                found_printer = true;
                memcpy(printer_addr, param->scan_rst.bda, sizeof(esp_bd_addr_t));
                ESP_ERROR_CHECK(esp_ble_gap_stop_scanning());
                ESP_ERROR_CHECK(esp_ble_gattc_open(client_if, printer_addr, BLE_ADDR_TYPE_PUBLIC, true));
            }
        }
    }else if(event == ESP_GAP_BLE_SCAN_STOP_COMPLETE_EVT){
        ESP_LOGI(GAP_TAG, "Scan stopped...");
    }else{
        ESP_LOGI(GAP_TAG, "GAP event: %d", event);
    }
}


void app_main(){
    ESP_LOGI(INIT_TAG, "Initializing NVS...");
    ESP_ERROR_CHECK(nvs_flash_init());
    ESP_ERROR_CHECK(esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT));
    ESP_LOGI(INIT_TAG, "Initializing bt controller...");
    esp_bt_controller_config_t cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_bt_controller_init(&cfg));
    ESP_LOGI(INIT_TAG, "Enabling bt controller...");
    ESP_ERROR_CHECK(esp_bt_controller_enable(ESP_BT_MODE_BLE));
    ESP_LOGI(INIT_TAG, "Initializing bluedroid...");
    ESP_ERROR_CHECK(esp_bluedroid_init());
    ESP_LOGI(INIT_TAG, "Enabling bluedroid...");
    ESP_ERROR_CHECK(esp_bluedroid_enable());
    ESP_LOGI(INIT_TAG, "Registering GATT client callback...");
    ESP_ERROR_CHECK(esp_ble_gattc_register_callback(gattc_cb));
    ESP_LOGI(INIT_TAG, "Registering GAP BLE callback...");
    ESP_ERROR_CHECK(esp_ble_gap_register_callback(gap_ble_cb));
    ESP_LOGI(INIT_TAG, "Registering GATT client...");
    ESP_ERROR_CHECK(esp_ble_gattc_app_register(0));
}