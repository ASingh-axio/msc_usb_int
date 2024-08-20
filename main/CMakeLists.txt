idf_component_register(SRCS "esp_hid_gap.c" "hid_host_example.c"
                       INCLUDE_DIRS "." "../components/hid/include"
                       REQUIRES esp_hid driver usb bt fatfs vfs log nvs_flash lwip esp_netif esp_http_server esp_wifi esp_https_ota app_update mbedtls espressif__esp_tinyusb espressif__tinyusb
                       PRIV_REQUIRES nvs_flash wear_levelling esp_partition console)

