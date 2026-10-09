idf_component_register(
    SRCS "main.c"
         "app_config.c"
         "camera_pipeline.c"
         "source_if.c"
         "source_dvp.c"
         "source_usb.c"
         "frame_ring.c"
         "yuv_osd.c"
         "wifi_net.c"
         "stream_server.c"
         "http_server.c"
         "metrics.c"
         "scan_ctrl.c"
         "web_ui.c"
         "lcd_ui.c"
    INCLUDE_DIRS "."
    PRIV_INCLUDE_DIRS "${CMAKE_CURRENT_LIST_DIR}/../managed_components/espressif__esp_video/private_include"
    EMBED_FILES "fs_web/index.html"
    REQUIRES esp_wifi nvs_flash esp_http_server cjson mdns esp_psram mbedtls
             usb usb_host_uvc esp_driver_jpeg esp_lcd_touch
)

# esp_video 私有 API（esp_video_get_dvp_video_device_sensor）依赖 esp_cam_sensor；
# BSP 提供板级摄像头上电（bsp_camera_start）。
