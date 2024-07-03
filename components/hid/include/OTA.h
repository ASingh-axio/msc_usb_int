#ifndef OTA_H_
#define OTA_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>

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


#define WAIT_TIME_FOR_UPDATE                60000UL
#define CONFIG_EXAMPLE_OTA_RECV_TIMEOUT     60000UL
#define CONFIG_EXAMPLE_SKIP_VERSION_CHECK   true
#define OTA_URL                             "https://axioelectronics.com/white_firmware.bin"

typedef void *esp_ws_ota_handle_t;
//extern char running_fw_version[32];

/**
 * @brief Validate firmware version
 * 
 * @param new_app_info esp_app_desc_t for new image
 * @return esp_err_t  ESP_OK: success
 *                    ESP_FAIL: failure
 */
esp_err_t validate_image_header(esp_app_desc_t *new_app_info);

/**
 * @brief Main task for the OTA
 * 
 * @param pvParameter 
 */
void ota_task(void *pvParameter);

/**
 * @brief Allocate heap memory for the OTA Task and start the OTA RTOS task
 * 
 */
void ota_start();

/**
 * @brief Do ota using https request
 * 
 * @param url https url for OTA
 */
void https_ota();


#ifdef __cplusplus
}
#endif

#endif /* OTA_H_ */