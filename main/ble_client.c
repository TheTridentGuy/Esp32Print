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


void gattc_cb(esp_gattc_cb_event_t event, esp_gatt_if_t gattc_if, esp_ble_gattc_cb_param_t *param){
    if(event == ESP_GATTC_REG_EVT){
        ESP_LOGI(GATTC_TAG, "GATT client registered...");
    }else if(event == ESP_GATTC_CONNECT_EVT){
        ESP_LOGI(GATTC_TAG, "Connected to GATT server...");
    }else if(event == ESP_GATTC_DISCONNECT_EVT){
        ESP_LOGI(GATTC_TAG, "Disconnected from GATT server...");
    }else if(event == ESP_GATTC_READ_CHAR_EVT){
        ESP_LOGI(GATTC_TAG, "Read characteristic...");
    }else if(event == ESP_GATTC_WRITE_CHAR_EVT){
        ESP_LOGI(GATTC_TAG, "Wrote to characteristic...");
    }else{
        ESP_LOGI(GATTC_TAG, "GATT event: %d", event);
    }
}


void app_main(){
    esp_err_t ret;
    ESP_LOGI(INIT_TAG, "Initializing NVS...");
    ret = nvs_flash_init();
    ESP_LOGI(INIT_TAG, "Initializing bt controller...");
    if(ret != ESP_OK){
        ESP_LOGE(INIT_TAG, "NVS init failed: %s", esp_err_to_name(ret));
        return;
    }
    esp_bt_controller_config_t cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    ret = esp_bt_controller_init(&cfg);
    if(ret != ESP_OK){
        ESP_LOGE(INIT_TAG, "Controller init failed: %s", esp_err_to_name(ret));
        return;
    }
    ESP_LOGI(INIT_TAG, "Enabling bt controller...");
    ret = esp_bt_controller_enable(ESP_BT_MODE_BLE);
    if(ret != ESP_OK){
        ESP_LOGE(INIT_TAG, "Controller enable failed: %s", esp_err_to_name(ret));
        return;
    }
    ESP_LOGI(INIT_TAG, "NVS and BT controller initialized and enabled successfully.");
}