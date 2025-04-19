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
        if(param->open.status == ESP_GATT_OK){
            ESP_LOGI(GATTC_TAG, "Connected to printer...");
            uint8_t set_intensity_data[] = {0x22, 0x21, 0xA2, 0x00, 0b1000, 0b0000, 0x5d, 0x94, 0xFF};
            uint8_t* set_intensity = set_intensity_data;
            ESP_ERROR_CHECK(esp_ble_gattc_write_char(client_if, param->open.conn_id, 0xAE01, sizeof(set_intensity), set_intensity, ESP_GATT_WRITE_TYPE_NO_RSP, ESP_GATT_AUTH_REQ_NONE));
            ESP_LOGI(GATTC_TAG, "Intensity set...");
            vTaskDelay(pdMS_TO_TICKS(100));
            uint8_t print_request_data[] = {0x22, 0x21, 0xA9, 0x00, 0b1100, 0b0000, 0x01, 0x30, 0x00, 0x92, 0xFF};
            uint8_t* print_request = print_request_data;
            ESP_ERROR_CHECK(esp_ble_gattc_write_char(client_if, param->open.conn_id, 0xAE01, sizeof(print_request), print_request, ESP_GATT_WRITE_TYPE_NO_RSP, ESP_GATT_AUTH_REQ_NONE));
            ESP_LOGI(GATTC_TAG, "Print request sent...");
            vTaskDelay(pdMS_TO_TICKS(10000));
            for(int i=0; i<48; i++){
                uint8_t print_data_data[] = {0xFF};
                uint8_t* print_data = print_data_data;
                ESP_ERROR_CHECK(esp_ble_gattc_write_char(client_if, param->open.conn_id, 0xAE03, sizeof(print_data), print_data, ESP_GATT_WRITE_TYPE_NO_RSP, ESP_GATT_AUTH_REQ_NONE));
                vTaskDelay(pdMS_TO_TICKS(50));
            }
            ESP_LOGI(GATTC_TAG, "Print data sent...");
            vTaskDelay(pdMS_TO_TICKS(100));
            uint8_t print_flush_data[] = {0x22, 0x21, 0xAD, 0x00, 0b1000, 0b0000, 0x00, 0x00, 0xFF};
            uint8_t* print_flush = print_flush_data;
            ESP_ERROR_CHECK(esp_ble_gattc_write_char(client_if, param->open.conn_id, 0xAE01, sizeof(print_flush), print_flush, ESP_GATT_WRITE_TYPE_NO_RSP, ESP_GATT_AUTH_REQ_NONE));
            ESP_LOGI(GATTC_TAG, "Print data flushed...");
        }else{
            ESP_LOGE(GATTC_TAG, "Failed to connect to printer, status: %d", param->open.status);
        }
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
            uint8_t length = 0;
            uint8_t *adv_data = esp_ble_resolve_adv_data(param->scan_rst.ble_adv, ESP_BLE_AD_TYPE_NAME_CMPL, &length);
            if(length > 0 && adv_data != NULL){
                char device_name[32] = {0};
                memcpy(device_name, adv_data, length);
                device_name[length] = '\0';
                ESP_LOGI(GAP_TAG, "Device name: %.*s", length, adv_data);
                if(strstr(device_name, "MXW01") != NULL){
                    found_printer = true;
                    memcpy(printer_addr, param->scan_rst.bda, sizeof(esp_bd_addr_t));
                    ESP_ERROR_CHECK(esp_ble_gap_stop_scanning());
                    ESP_ERROR_CHECK(esp_ble_gattc_open(client_if, printer_addr, BLE_ADDR_TYPE_PUBLIC, true));
                }
            }
        }
    }else if(event == ESP_GAP_BLE_SCAN_STOP_COMPLETE_EVT){
        ESP_LOGI(GAP_TAG, "Scan stopped...");
        if(!found_printer){
            ESP_LOGW(GAP_TAG, "Unable to find a printer.");
        }
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