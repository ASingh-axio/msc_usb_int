idf_component_register(SRCS "OTA.c" "hid_host.c"
                       REQUIRES log nvs_flash lwip esp_netif esp_http_server esp_wifi esp_https_ota app_update mbedtls
                       PRIV_REQUIRES usb
                       INCLUDE_DIRS "include"
                       EMBED_TXTFILES ${project_dir}/server_certs/ca_cert.pem)
