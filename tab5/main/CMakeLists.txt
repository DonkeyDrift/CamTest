idf_component_register(
    SRCS "main.c"
         "app_config.c"
         "board_power.c"
         "source_if.c"
         "source_usb.c"
         "source_dvp.c"
         "h264_pipeline.c"
         "frame_ring.c"
         "yuv_osd.c"
         "wifi_net.c"
         "stream_server.c"
         "http_server.c"
         "metrics.c"
         "web_ui.c"
    INCLUDE_DIRS "."
    EMBED_FILES "fs_web/index.html"
                "certs/tab5_cert.pem"
                "certs/tab5_key.pem"
    REQUIRES esp_wifi nvs_flash esp_http_server esp_https_server cjson mdns esp_psram mbedtls
             usb usb_host_uvc esp_driver_jpeg esp_driver_i2c esp_driver_ppa esp_h264
             esp_video esp_cam_sensor
)
