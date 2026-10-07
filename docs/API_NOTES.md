# esp_video / BSP API 侦察笔记（S31 实测依据，非猜测）

> 对象版本：espressif/esp_video **2.2.0**（BSP 1.0.1 钉死 ~2.2）、espressif/esp_cam_sensor 2.2.x、
> espressif/esp32_s31_korvo_1 **1.0.1**、ESP-IDF release/v6.1（v6.1.0，commit 0d92878）。
> 以下结论全部读自本地 managed_components 源码。

## 1. 设备节点（esp_video_device.h）

| 设备 | 节点 | 说明 |
|---|---|---|
| DVP 摄像头 | `/dev/video2`（`ESP_VIDEO_DVP_DEVICE_NAME`） | BSP 默认用这个 |
| ISP-DVP | `/dev/video1` | S31 无 ISP（SOC_ISP 不支持），不适用 |
| 硬件 JPEG 编码器 | `/dev/video10`（`ESP_VIDEO_JPEG_DEVICE_NAME`） | 需 `CONFIG_ESP_VIDEO_ENABLE_HW_JPEG_VIDEO_DEVICE=y`（依赖 `SOC_JPEG_CODEC_SUPPORTED`，S31=y） |

## 2. SoC 能力（soc_caps.h @ esp32s31）

- `SOC_JPEG_CODEC_SUPPORTED=1`、`SOC_LCDCAM_CAM_SUPPORTED=1`
- 无 ISP（MIPI/ISP 设备不可用）→ RAW 传感器不能直接出 YUV；本板两款 sensor 都自带 ISP 输出 YUV/RGB。

## 3. JPEG 编码设备（esp_video_jpeg_device.c）

- 走 `driver/jpeg_encode.h`（S31 与 P4 同款驱动），引擎单帧超时 **40 ms**（`timeout_ms=40`，硬编码）。
- V4L2 M2M 语义：
  - `V4L2_BUF_TYPE_VIDEO_OUTPUT` = 输入（原始图）；支持格式：**UYVY / RGB565 / RGB24 / GREY**（YUV420/444 需开 Kconfig，默认无 YUYV！）
  - `V4L2_BUF_TYPE_VIDEO_CAPTURE` = 输出 JPEG（`V4L2_PIX_FMT_JPEG`）
  - 要求两侧 width/height 一致（`jpeg_video_start` 校验）。
  - **触发方式**：`esp_video_recv_element()`（即 `ioctl(VIDIOC_DQBUF)`）时若为 M2M 设备，同步调用 `esp_video_m2m_process()`——**编码发生在 DQBUF(CAPTURE) 调用者任务里**，因此可精确计时；两侧队列都必须保持有货，否则报 "no valid buffer"。
- 质量：`VIDIOC_S_EXT_CTRLS`，id=`V4L2_CID_JPEG_COMPRESSION_QUALITY`，**1~100，越大画质越高、码率越大**（默认 80）。
  色度抽样：`V4L2_CID_JPEG_CHROMA_SUBSAMPLING`（默认 3 = YUV422）。
- JPEG capture 缓冲按 `JPEG_MAX_COMP_RATE=0.75` 预留，PSRAM（`MALLOC_CAP_SPIRAM|CACHE_ALIGNED`）。
- 传感器 YUYV（OV3660）与编码器 UYVY 输入不匹配 → 拷贝进编码器输入缓冲时顺带做 YUYV→UYVY 字节重排
  （YUYV= Y0 U0 Y1 V0 → 每双字节交换后 = U0 Y0 V0 Y1 = UYVY，数学上等价于"按 16-bit 字交换字节序"）。

## 4. DVP 摄像头设备（esp_video_dvp_device.c）

- 缓冲自动落 PSRAM（`DVP_MEM_CAPS = 8BIT|SPIRAM|CACHE_ALIGNED`，SPIRAM 开启时）。
- `VIDIOC_S_FMT` **只接受与当前 sensor 格式一致的宽高**（无硬件缩放！）；改分辨率必须先
  `VIDIOC_S_SENSOR_FMT`（自定义 ioctl，参数 `esp_cam_sensor_format_t`）切换 sensor 格式，再 `S_FMT`。
- 自定义 ioctl（esp_video_ioctl.h）：`VIDIOC_S_SENSOR_FMT / G_SENSOR_FMT / SET_OWNER（仅引用计数）/ S_DQBUF_TIMEOUT`。
- `VIDIOC_ENUM_FRAMESIZES` 只报当前格式一档 → **支持分辨率列表改从 sensor 设备句柄
  `esp_cam_sensor_query_format()` 获取**（句柄经私有 API `esp_video_get_dvp_video_device_sensor()`）。

## 5. 传感器（esp_cam_sensor 2.2.x，Kconfig 门控格式表）

| sensor | 可用格式（需在 Kconfig 打开） | 备注 |
|---|---|---|
| SC101IOT | UYVY/YUYV 1280x720 @15/25fps | 只有一档分辨率；UYVY 可直喂 JPEG |
| OV3660 | YUYV 240x240@24 / YUYV 640x480@10 / RGB565 240x240@24 / RGB565 640x480@10 / JPEG 1280x720@12(sensor 内编码) | 无 160x120/320x240/800x600 档位 |

⇒ 任务书里的 6 档分辨率矩阵在本组件版本下大部分为 unsupported（自动跳过并标记）；
   160×120~320×240 低延迟档需要后续组件版本增加 sensor windowing 支持或 DVP 硬件缩放
   （README「待核实清单」）。运行时以 `esp_cam_sensor_query_format()` 实际枚举为准。

- sensor 自动侦测：`esp_video_init()` 遍历链接进来的 detect 函数（Kconfig 使能的 sensor），
  I2C 探测成功即用。sensor 名：`dev->name`（"ov3660"/"SC101IOT"）。

## 6. BSP（esp32_s31_korvo_1 1.0.1）

- `bsp_camera_start(NULL)` = `bsp_i2c_init()` + LEDC XCLK(20MHz, TIMER1, CH=CONFIG_BSP_CAMERA_XCLK_LEDC_CH 默认 2) + `esp_video_init(dvp_cfg)`。
- 摄像头电源为板上固定 LDO，代码无电源 GPIO；I2C 与 ES8389 共总线（本任务不初始化音频，无争抢）。
- 引脚：XCLK=55 PCLK=54 VSYNC=56 HSYNC(DE)=57 D0..D7=46..53；I2C SCL=1 SDA=0。
- LCD 子板为 RGB 并口 + GT1151（BSP_CAPS_DISPLAY），本任务未接子板，LCD 叠加留 Kconfig 开关 TODO。

## 7. 其他

- `esp_cam_sensor_format_array_t{count; format_array}` + `esp_cam_sensor_query_format(dev,&arr)` 枚举全部格式。
- MJPEG 每帧元数据随 multipart 头下发；浏览器用 fetch+ReadableStream 解析（README 有协议说明）。
- IDF v6.1.0 预览目标：`idf.py --preview set-target esp32s31`（--preview 是全局选项，放动作前）。
