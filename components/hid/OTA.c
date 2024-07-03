#include <stdio.h>
#include "OTA.h"

#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include <freertos/task.h>
#include <freertos/event_groups.h>
#include <freertos/timers.h>
#include <freertos/semphr.h>
#include "esp_system.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_ota_ops.h"
#include "esp_http_client.h"
#include "esp_https_ota.h"
#include "string.h"
#ifdef CONFIG_EXAMPLE_USE_CERT_BUNDLE
#include "esp_crt_bundle.h"
#endif
#include "esp_netif.h"
#include "nvs.h"
#include "nvs_flash.h"
#include <sys/socket.h>
#include "esp_wifi.h"
#include "esp_netif_defaults.h"

static const char *TAG = "OTA";



extern const uint8_t server_cert_pem_start[] asm("_binary_ca_cert_pem_start");
extern const uint8_t server_cert_pem_end[] asm("_binary_ca_cert_pem_end");
extern EventGroupHandle_t s_wifi_event_group;
extern const int WIFI_CONNECTED_BIT;

//char running_fw_version[32] = "invalid";

esp_http_client_config_t config = {
    .skip_cert_common_name_check = true,
    // .use_global_ca_store = true,
    .cert_pem = (char *)server_cert_pem_start,
    .url = OTA_URL,
    .timeout_ms = CONFIG_EXAMPLE_OTA_RECV_TIMEOUT,
    .keep_alive_enable = true,
};


static esp_err_t _http_client_init_cb(esp_http_client_handle_t http_client) {
    esp_err_t err = ESP_OK;
    /* Uncomment to add custom headers to HTTP request */
    // err = esp_http_client_set_header(http_client, "Custom-Header", "Value");
    return err;
}


esp_https_ota_config_t ota_config = {
    .http_config = &config,
    .http_client_init_cb = _http_client_init_cb, // Register a callback to be invoked after esp_http_client is initialized
};




esp_err_t validate_image_header(esp_app_desc_t *new_app_info) {
    if (new_app_info == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    const esp_partition_t *running = esp_ota_get_running_partition();
    esp_app_desc_t running_app_info;
    if (esp_ota_get_partition_description(running, &running_app_info) == ESP_OK) {
        ESP_LOGI(TAG, "Running firmware version: %s", running_app_info.version);
        //running_fw_version[32] = running_app_info.version;
    }

#if (CONFIG_EXAMPLE_SKIP_VERSION_CHECK == true)
    if (memcmp(new_app_info->version, running_app_info.version, sizeof(new_app_info->version)) == 0) {
        ESP_LOGW(TAG, "Current running version is the same as a new. We will not continue the update.");
        return ESP_FAIL;
    }
#endif

    return ESP_OK;
}




void https_ota() {
        esp_err_t ota_finish_err = ESP_OK;
        esp_https_ota_handle_t https_ota_handle = NULL;
        esp_err_t err = esp_https_ota_begin(&ota_config, &https_ota_handle);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "ESP HTTPS OTA Begin failed");
        }

        esp_app_desc_t app_desc;
        err = esp_https_ota_get_img_desc(https_ota_handle, &app_desc);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "esp_https_ota_read_img_desc failed");
            goto ota_end;
        }
        err = validate_image_header(&app_desc);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "image header verification failed");
            goto ota_end;
        }

        while (1) {
            err = esp_https_ota_perform(https_ota_handle);
            if (err != ESP_ERR_HTTPS_OTA_IN_PROGRESS) {
                break;
            }
            // esp_https_ota_perform returns after every read operation which gives user the ability to
            // monitor the status of OTA upgrade by calling esp_https_ota_get_image_len_read, which gives length of image
            // data read so far.
            ESP_LOGD(TAG, "Image bytes read: %d", esp_https_ota_get_image_len_read(https_ota_handle));
        }

        if (esp_https_ota_is_complete_data_received(https_ota_handle) != true) {
            // the OTA image was not completely received and user can customize the response to this situation.
            ESP_LOGE(TAG, "Complete data was not received.");
        } else {
            ota_finish_err = esp_https_ota_finish(https_ota_handle);
            if ((err == ESP_OK) && (ota_finish_err == ESP_OK)) {
                ESP_LOGI(TAG, "ESP_HTTPS_OTA upgrade successful. Rebooting ...");
                vTaskDelay(pdMS_TO_TICKS(2000));
                esp_restart();
            } else {
                if (ota_finish_err == ESP_ERR_OTA_VALIDATE_FAILED) {
                    ESP_LOGE(TAG, "Image validation failed, image is corrupted");
                }
                ESP_LOGE(TAG, "ESP_HTTPS_OTA upgrade failed 0x%x", ota_finish_err);
            }
        }

ota_end:
    esp_https_ota_abort(https_ota_handle);
    ESP_LOGE(TAG, "ESP_HTTPS_OTA upgrade failed");
}



void ota_task(void *pvParameter) {

    while(1) {
        xEventGroupWaitBits(s_wifi_event_group, WIFI_CONNECTED_BIT, false, false, portMAX_DELAY);
        https_ota();
        vTaskDelay(pdMS_TO_TICKS(WAIT_TIME_FOR_UPDATE));
    }
}


void ota_start() {
    xTaskCreate(&ota_task, "ota_example_task", 1024*8, NULL, 0, NULL);
}
