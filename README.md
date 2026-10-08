# CamTest — ESP32-S31-Korvo-1 Wi-Fi 图传性能闭环测试系统

驱动 Korvo-1 板载 DVP 摄像头 → S31 硬件 JPEG 编码 → Wi-Fi 推 MJPEG 流 → 浏览器实时显示画面与
帧率/端到端时延，并一键自动遍历「分辨率 × JPEG 质量 × 网络模式」输出可导出的性能矩阵。
**最终用途**：评估 ESP32-S31 作为 DonkeyCar 图传前端（FPV 低延迟优先）。

## 一、验证环境（请勿随意变更后引用性能数据）

| 项 | 值 |
|---|---|
| 板卡 | ESP32-S31-Korvo-1 V1.1（模组 ESP32-S31-WROOM-3：16MB flash + 16MB PSRAM） |
| ESP-IDF | **v6.1.0**（release/v6.1 分支头，commit `0d9287800812c95662921c2c5e812023939e3d58`，2026-10-07 验证） |
| 目标 | `esp32s31`（preview，需 `idf.py --preview set-target esp32s31`；bootloader 位于 0x2000） |
| 摄像头组件 | espressif/esp_video **2.2.0** + espressif/esp_cam_sensor 2.2.x（V4L2 风格 API） |
| 板级支持 | espressif/esp32_s31_korvo_1 **1.0.1**（其清单钉死 esp_video ~2.2，故锁定 2.2.0） |
| USB UVC 组件 | espressif/usb_host_uvc **2.5.2** + espressif/usb **1.5.0**（IDF v6 起 USB Host 库入组件仓库；esp_video 2.2.0 对 p4/s3/s31 目标本身就依赖 usb_host_uvc ~2.5，版本由 lock 统一） |
| 摄像头 | **以运行时侦测为准**（OV3660 或 SC101IOT，上电自动探测并打印型号，不硬编码） |
| 组件版本注记 | registry API 已出现 esp_video 2.5.0，但组件存储索引滞后不可下载；2.5.0 的 m2m 示例（S31 默认 SC101IOT+MJPEG）对应更新版驱动，待可用后建议升级复测 |

### 编译与烧录

```bash
# 一次性：获取 IDF（中国大陆可用 jihulab 镜像）
git clone -b release/v6.1 --depth 1 --recursive --shallow-submodules \
  https://jihulab.com/esp-mirror/espressif/esp-idf.git ~/esp/esp-idf
~/esp/esp-idf/install.sh esp32s31

# 每个终端：
. ~/esp/esp-idf/export.sh

# 本工程（Wi-Fi 凭据在 menuconfig: CamTest Configuration）
idf.py --preview set-target esp32s31
idf.py menuconfig        # 配置 SSID/密码/模式
idf.py build
idf.py -p /dev/cu.usbserial-1110 flash monitor   # macOS 串口名以实际为准
```

> 进入下载模式（若自动复位失败）：按住 **Boot** → 点按 **Reset** → 松开 **Boot**。
> 供电：默认 USB(UART) 口；高负载（720p/多客户端/**USB 摄像头推流**）**必须**同时从
> **USB Type-C (Power)** 口补电——Korvo-1 的 Type-A 座经 TPS2051C 限流 **500 mA**，
> USB 摄像头（尤其 640×480 MJPEG 高帧率档）很容易撞限，典型症状是"枚举正常、一推流就掉线/
> 重启/反复打开失败"。固件已实现掉线计数（`usb_disconnect_count`）与自动重连，UI 也有供电警告。

### 烧录后串口日志应看到（验收清单）

```
IDF: v6.1.0 | build: ...
===== sensor detect: <OV3660|SC101IOT>（共 N 种格式） =====
  [0] DVP_8bit_20Minput_YUV422_UYVY_1280x720_25fps ...
PSRAM: 16384 kB
STA got IP: 192.168.x.x
mDNS: http://korvo-s31.local
pipeline: <sensor> <W>x<H> <fmt> q20 ...
采集源就绪：DVP（默认）+ USB
USB Host 就绪，未发现 UVC 设备（热插拔监听中；注意 Korvo-1 Type-A 口限流 500 mA）
```

插入 USB 摄像头后应看到（验收要求：完整描述符档位清单）：

```
===== UVC 设备已连接：name=1bcf:28c4 VID/PID=1bcf/28c4 addr=1 stream_idx=0 =====
===== UVC 支持的 (格式, 分辨率, 帧率) 完整列表 =====
  [0] MJPEG 640x480 default=30fps interval_type=3
      interval[0]=333333 → 30fps ...
===== 档位汇总：MJPEG=支持 YUY2=支持，共 N 档；默认模式=passthrough =====
UVC 推流开始：MJPEG 640x480 quality=0 fps_limit=0
```

## 二、网络与 Web 界面

| 入口 | 地址 | 说明 |
|---|---|---|
| Web UI | `http://korvo-s31.local`（或 IP 直连） | 单页离线面板（无 CDN，资源内嵌固件） |
| MJPEG 流 | `http://<host>:81/stream` | `multipart/x-mixed-replace`；part 头带每帧元数据 |
| WebSocket | `ws://<host>:81/ws` | 二进制帧（36B 小端头 + JPEG），协议开销对比用 |
| UDP 分片 | 设备 UDP `:9100` | 用 `tools/udp_receiver.py`（浏览器收不了 UDP）；可能花屏 |
| 光学校验 | `http://<host>/overlay` | 画面毫秒叠加开关 + 校验操作说明 |

**为什么 /stream、/ws 在 81 端口**：esp_http_server 单 task 分发，长连接会阻塞其他会话；
81 端口用原生 socket 每客户端独立任务，支持 2~4 个并发观看端并分别统计（:80 的 API 不受影响）。

### 每帧元数据（/stream part 头）

```
X-Frame-Id / X-Capture-Us / X-Encode-Us / X-Jpeg-Len / X-Sensor / X-Res / X-Quality
X-Source（dvp|usb）/ X-Scaled（0|1）/ X-Ts-Meaning（sensor_out|frame_arrival）
```

`X-Ts-Meaning` 标注 `X-Capture-Us` 的语义（U4，两源严格可比的前提）：
DVP 源 = 传感器输出时刻；USB 源 = **完整一帧到达 ESP32 的时刻**（已含摄像头内部延迟）。

网页用 `fetch()` + `ReadableStream` 手动解析（不用 `<img src>`），在 part 头解析完成瞬间打
`t_arrive_browser`，才能算端到端时延。

### WebSocket 二进制头（小端，36B）

`MJP1 | u32 frame_id | u64 t_capture_us | u64 t_encode_us | u16 w | u16 h | u8 quality | u8 source(0=dvp 1=usb) | u32 jpeg_len | u16 rsv`

### UDP 分片头（小端，24B，≤1200 B/片）

`u32 'UJPG' | u32 frame_id | u64 t_capture_us | u16 frag_idx | u16 frag_total | u16 frag_len | u8 flags(bit0=首片 bit1=末片) | u8 rsv`
（无重传，可能花屏——用于探索时延下限。）

## 三、HTTP API

| 端点 | 方法 | 说明 |
|---|---|---|
| `/api/sync?t_client=<us>` | GET | `{"t_dev_us","t_wall_ms"}`；设备在**即将写响应前**采样 esp_timer |
| `/api/status` | GET | 设备侧实时指标 + 扫描进度/结果行（JSON） |
| `/api/config` | POST | `{"source":"dvp\|usb","res":"640x480","quality":20,"fps_limit":0,"overlay":true,"target_mbps":0,"usb_mode":"passthrough\|reencode","usb_inherent_ms":12.3}`；切源失败自动回滚并返回 `ok:false`；仅改质量走轻量路径 |
| `/api/scan/start?modes=sta,ap&scope=dvp\|usb\|both` | GET | 启动扫描（scope 默认 both；USB 不在线时 USB 组自动跳过并在状态中标注） |
| `/api/scan/stop` | GET | 停止扫描 |
| `/api/scan/result` | GET | 进度 + 全部结果行 |
| `/api/scan/csv` | GET | 导出 CSV（表头见下） |
| `/api/scan/report` | POST | 扫描期间浏览器侧每秒回传 `{arrival_fps,bitrate_mbps,latency_*}`；UDP 行由 `tools/udp_receiver.py --scan` 回传（另含 `udp_loss_rate,udp_incomplete_rate`） |

CSV 表头（U6 完整版；旧列全部保留、旧数据可继续按列名读取）：

```
timestamp,video_source,sensor,usb_device_name,resolution,scaled,quality,usb_mode,protocol,
tcp_nodelay,lcd_on,wifi_mode,bandwidth_mhz,rssi,capture_ts_meaning,
capture_fps,encode_fps,arrival_fps,jpeg_avg_bytes,bitrate_mbps,
latency_mean_ms,latency_p95_ms,latency_max_ms,latency_std_ms,encode_ms,
usb_cam_inherent_latency_ms,usb_disconnect_count,
cpu0_pct,cpu1_pct,free_heap,free_psram,drop_frames,udp_loss_rate,udp_incomplete_rate
```

字段口径（防伪造约定）：
- DVP 行：`video_source=dvp`、`usb_mode=n/a`、`capture_ts_meaning=sensor_out`、`usb_device_name` 空；
- USB passthrough 行：`quality=passthrough`（**绝不伪造质量数值**，以 `jpeg_avg_bytes` 作等效画质指标）、
  `capture_ts_meaning=frame_arrival`；
- `usb_cam_inherent_latency_ms`：**只能**由光学闭环人工标定后经 `/api/config` 写入，未标定为空——设备侧不存在能直接测得该值的 API；
- `protocol` 为扫描矩阵维度（http/ws/udp），`tcp_nodelay` 恒 1（流服务已设 TCP_NODELAY）、`lcd_on` 恒 0（本工程无 LCD 子板）。

串口每秒输出聚合 CSV（无浏览器可用；末尾字段只增不改，旧脚本兼容）：
`CSV,<t_us>,cap_fps,out_fps,send_fps,mbps,jpeg_avg,drop_cap,drop_out,heap,psram,mode,rssi,WxH,source,usb_mode`；
扫描每组完成再输出一行 `SCAN,...` 摘要。

## 四、测试方法学与指标定义（可信度透明）

### 时延测量（核心）

**主方法：设备时间戳 + 浏览器时钟同步（Christian 算法，抗抖动）**

1. 设备侧打点（`esp_timer_get_time()`，单调，不受 SNTP 跳变影响）：
   - `t_capture_us`：DVP `ioctl(VIDIOC_DQBUF)` 返回瞬间；
   - `t_encode_done_us`：编码器 `DQBUF(CAPTURE)` 返回瞬间（esp_video 对 M2M 设备在该 ioctl 内**同步**完成硬件编码，故该值即硬件编码完成时刻）。
2. 元数据随每帧下发（见上），浏览器在 part 头解析完成瞬间取 `t_arrive_browser`。
3. 时钟同步：浏览器向 `/api/sync` 发 ≥8 次请求，第 i 次记 `t0/t1`（客户端钟）与设备读数 `t_dev`：
   - `rtt = t1 − t0`；设备在响应前一刻采样 `t_dev`，故 `t_dev` 对应客户端时刻 ≈ `t0 + rtt/2`（对称假设）；
   - `offset_i = t_dev − (t0 + rtt/2)`；取 **RTT 最小 3 次**的 offset **中位数**为最终 offset；
   - 换算关系：**任意客户端时刻 tc 的设备钟读数 = tc + offset**；
   - **端到端时延 = t_arrive_client − (t_capture + offset)**（两端时间域对齐后相减，offset 常量误差相消，仅残剩 30 s 内漂移）；
   - 页面每 30 s 自动重同步并显示 offset 与 RTT（RTT 是网络可信度的直观参考）。
4. 拆分展示：
   - **编码耗时 = t_encode_done − t_capture**（设备侧，精确实测）；
   - **网络+浏览器传输 = 端到端 − 编码耗时**（**估算**，唯一含估计成分的项，UI 明确标注）。
5. NTP 交叉验证（可选）：menuconfig 打开 SNTP 后 `/api/sync` 返回 `t_wall_ms`，脚本对比两端系统钟差（`tools/pi_compare.py` 输出 `wall_diff`）。注意**时延计算不用系统钟**（防跳变），此值仅作旁证。

**金标准校验：光学闭环**（`/overlay` 页面）

在 YUV 帧上直接烧录设备毫秒计数 → 摄像头对准另一块屏的毫秒计时器 → 连拍 ≥10 张流画面，
画面内两个读数之差 = 全链路系统偏差（含编码、网络、浏览器渲染、屏幕刷新、快门）。
用该偏差校准主方法读数。实测偏差记录在 `RESULTS_TEMPLATE.md`。

### 指标可信度分级

| 指标 | 分级 | 说明 |
|---|---|---|
| 采集/编码/发送 FPS、编码耗时、单帧大小、码率、CPU、内存、丢帧 | **设备侧精确实测** | esp_timer + FreeRTOS runtime stats |
| 到达 FPS（浏览器） | **精确实测** | 浏览器侧 |
| 端到端时延（时钟同步法） | **含估算** | offset 残差一般 ±1~5 ms（同网段、RTT<30 ms 时更小）；务必先做光学标定 |
| 网络+浏览器段 | **估算** | = 端到端 − 编码耗时 |
| Wi-Fi RSSI/信道/PHY | 设备侧实测 | `esp_wifi_sta_get_ap_info`；**无公开 API 拿当前链路速率**（见待核实清单） |

### 自动扫描（闭环，按采集源分组）

矩阵按源分化（U8，两源的可控维度不同）：

| 源 | 分辨率 | 质量 | 协议 | 组数 | 说明 |
|---|---|---|---|---|---|
| DVP | 160×120 / 320×240 / 640×480 | 10 / 20 / 30 | http / ws / udp | 27 | 质量→码率严格可控 |
| USB 直通 | 摄像头 MJPEG 原生档（动态枚举） | passthrough（不可控） | http / ws / udp | 档数×3 | 以 jpeg_avg_bytes 作等效画质指标 |
| USB 重编码 | 摄像头 YUY2 原生档（动态枚举） | 10 / 20 / 30 | http / ws / udp | 档数×9 | 与 DVP 同质量档严格可比 |

- 范围可选 `scope=dvp / usb / both`（UI 下拉，默认 both）；USB 不在线时 USB 组整组跳过并在状态标注 `usb_skipped`；
- USB 组扫描前先从 UVC 描述符枚举档位，不支持的档不进矩阵；DVP 组不支持的档照旧标 `unsupported`；
- **协议是扫描维度**：浏览器根据当前行自动切换 http/ws（`/api/status` 的 `scan.cur_protocol` 驱动）；
  udp 行由 `tools/udp_receiver.py --scan` 回传（浏览器无法收 UDP）；
- 扫描全程 LCD 默认关闭（本工程无 LCD 子板，`lcd_on=0`）；
- UI 显示两级进度（当前源 / 当前组），结束后自动汇总「DVP vs USB」对比表。

DVP 分辨率档位与虚拟分辨率机制（软件抽取/裁剪）：

**虚拟分辨率（软件抽取/裁剪）**：sensor 驱动原生档位有限（OV3660 只有 240×240/640×480），
本固件在"采集→编码"之间加入软件映射，使低分辨率档可用：
- **整数抽取**（保全视场）：640×480 → 320×240（2:1）；
- **中心裁剪**（视野变小、帧率更高）：240×240@25fps → 160×120；
- 480×320 = 640×480 中心裁剪；640×480 原生直通。
选择策略：候选按"帧率为主"评分自动选最优（原生直通 > 整数抽取 > 裁剪，同分保视场）；
UI 下拉列表自动列出全部可达成档位，`fmt` 字段标注来源。

**高帧率档（hifps，60~93fps）**：OV3660 在组件驱动里最高只有 25fps（YUV）/30fps（sensor JPEG）档。
本固件用两步数据驱动手段突破（非盲改，出处可查）：
1. **时钟树移植**：esp_cam_sensor 2.4.x 官方 `240x240 JPEG 30fps` 档与 YUV 24fps 档**窗口/时序完全相同**，
   仅时钟树不同（0x303b=0x1e, 0x303d=0x30, 0x3824=0x0a）。把这套时钟寄存器移植到 YUYV 模式 →
   实测 25→31fps，全视场无失真；
2. **垂直读出窗口裁剪**：该窗口是全高 1548 行（2x binning 读出 774 行≈VTS783，故单减 VTS 会失步）。
   中心裁剪 V 窗口 → 读出行数变少 → 帧率按比例上升（PCLK 保持 40MHz 安全值不变）。
   注意：水平窗口/HTS 改动实测会失步（缩放管线硬约束），不可用。

档位与代价（160x120/240x240 生效，menuconfig 或 UI「高帧率档」选择，L2 为默认）：

| 档 | 实测帧率 | 垂直视场 | 画面纵向拉伸 | 适用 |
|---|---|---|---|---|
| 关 | 25 fps | 100% | 无 | 画质优先 |
| L1 | 31 fps | 100% | 无 | 平衡 |
| **L2（默认）** | **60 fps** | 52% | ×1.9 | FPV 遥控 |
| L3 | 74.6 fps | 41% | ×2.5 | 极限低延迟 |
| L4 | 93 fps | 33% | ×3.1 | 帧率极限实验 |

- 拉伸原理：窗口宽高比变宽（如 1568×800≈1.96:1）被 sensor ISP 压进 240×240 输出。对 DonkeyCar
  训练数据这是**一致性形变**（模型可学习），人工目视会觉物体拉高；
- 60fps 时曝光上限 16.7ms（L4 约 10.7ms），室内暗光需补光；
- 60 秒浸泡：59.9~60.0fps 零丢帧、零重启、内存平稳。
× JPEG 质量 {10,20,30,40}（**方向：越大越清晰、码率越大**，源自 `V4L2_CID_JPEG_COMPRESSION_QUALITY`，UI 已注明）
× 网络模式（当前模式；可勾选同时跑 STA+SoftAP——AP 阶段浏览器会断流并自动重连，拿不到浏览器数据的组该字段留空）。
每组 warmup 2 s + 采样 10 s；设备侧指标自采，浏览器侧时延每秒回传挂到当前组。
**本组件版本（esp_cam_sensor 2.2.x）驱动格式表较窄**：SC101IOT 仅 720p@15/25（UYVY/YUYV），
OV3660 仅 240×240@24 与 640×480@10（YUYV/RGB565）——矩阵中其余档位会如实标 unsupported（见待核实清单）。

## 四点五、USB UVC 采集通路（第二条采集链路）

### 1. 可行性验证结论（先行验证，结论如下）

按任务书第七节要求，先做源码级可行性侦察再动手（2026-10-07，全部结论读自本地
`managed_components` 实际源码，非官方文档转述）：

| 验证项 | 结论 |
|---|---|
| `usb_host_uvc` 组件与 S31 | ✅ espressif/usb_host_uvc **2.5.2**（esp-usb 仓库 `host/class/uvc/usb_host_uvc`），targets 显式含 `esp32s31`；esp_video 2.2.0 的组件清单对 esp32p4/s3/s31 固定依赖 `usb_host_uvc 2.5.*`，`dependencies.lock` 已解析 2.5.2 + espressif/usb 1.5.0（`usb_host_install` 的 `peripheral_map=0` 在 HS capable 目标默认走 High-Speed 外设） |
| esp_video 的 `/dev/video40` V4L2 封装 | ⚠️ **存在但未采用**。esp_video 2.2.0 确有 `ESP_VIDEO_ENABLE_USB_UVC_VIDEO_DEVICE`（设备号 40..49，`uvc_to_v4l2_format` 支持 MJPEG/YUY2），但其断线语义有缺陷：设备拔出后 `uvc_video_stop()`/`uvc_video_deinit()` 因 `dev_addr==0` **提前返回 `ESP_ERR_NOT_FOUND`** → `esp_video_close()` 不清 `inited` 标志、`uvc_host_stream_close()` 永不被调用 → **每次拔插泄漏一条 stream（URB 为内部 RAM）**；且 open 时会同步阻塞等待枚举（默认 10 s）。不满足"拔插 3 次不卡死、自动重连"的验收标准 |
| 最终路线 | ✅ **按任务书预留的降级路线：直接使用 usb_host_uvc 原生 API**，封装进与 DVP 完全一致的采集源接口（`source_usb.c`）。其 `uvc_host_stream_close()` 对死设备容错（stop 的控制传输错误被忽略，全部帧归还后 `uvc_device_remove()` 释放 stream+URB），断线可干净回收。esp_video 的 UVC Kconfig 保持关闭（`sdkconfig.defaults` 显式 `is not set`），避免双重 `usb_host_install` |
| **真机验证（2026-10-07，0bda:1376 摄像头）** | ✅ 枚举→12 档清单打印→两模式推流→UI 热切换全部通过。USB passthrough 640×480 实测 **54.8fps**（@60fps 档，零丢帧）/13~15fps（@实际曝光）；reencode YUY2→硬件 JPEG **5fps**（YUY2@10fps 档）quality 可控。调试中定位并修复 4 个集成坑（见下「工程实录」） |
| 设备识别 | `usb_host_device_open()` + `usb_host_get_device_descriptor()` 取 VID/PID（UI/CSV 显示 `vid:pid`，如 `1bcf:28c4`）。usb 1.5.0 **无字符串描述符公共 API**，iProduct 产品名暂不取（见待核实清单 #9） |
| 档位枚举 | `uvc_host_get_frame_list(dev_addr, stream_idx, ...)` 直接返回描述符解析后的 (格式， 分辨率， 帧间隔) 全表——启动/插入即打印完整清单并生成 UI 档位列表 |

> 真机最小验证（枚举/取帧/帧长/实测帧率）需在板上进行：烧录后插 USB 摄像头，
> 串口即打印上述清单与 `UVC 推流开始`，Web UI 显示画面即为通过。接口层已按
> 官方 `basic_uvc_stream` 示例的调用序列编写（open→format_select(fps=0 默认)→start）。

### 1.5 工程实录：USB 通路集成踩坑与修复（真机调试结论，全部已修复）

| # | 症状 | 根因 | 修复 |
|---|---|---|---|
| 1 | 插入摄像头枚举失败：`Configuration descriptor larger than control transfer max length` | `CONFIG_USB_HOST_CONTROL_TRANSFER_MAX_SIZE` 默认 256 < UVC 配置描述符（本机 799B） | sdkconfig 提到 **1024** |
| 2 | stream_open 的 VS Probe 控制传输 5s 超时（连带一切控制传输卡死） | `usb_host_lib_handle_events()` 必须常驻独立任务泵送——IDF v6 usbh 的传输完成事件经 **lib 事件队列**派发；并进状态机任务后，任何一次阻塞（如 stream_open 等待）就冻结全总线派发 | 独立 `usb_lib_task`（官方示例同款） |
| 3 | reencode 模式第 3 帧起编码器 QBUF 全部 EINVAL（`element not free`） | M2M 编码器每帧必须 `DQBUF(OUTPUT)` 取回被消费的输入槽（元素才回 FREE 态）——漏写即永久占用 | worker 补上（与 camera_pipeline 的 encode_task 同款） |
| 4 | USB 推流时 /api/status 挂死（小端点正常、大端点超时） | 两个档位查询函数加锁补丁半途失败：一处 **return 未还锁**（锁永久泄漏）+ 一处 **无 take 却 give**（互斥量破坏） | 重写两函数，take/give 全路径配平 |
| 5 | 摄像头 720p@60 档持续 `frame error`（摄像头自带错误帧标记） | 该档 isoc 带宽吃紧（dwMaxVideoFrameSize 虚标 1.8MB），属摄像头 quirks | 默认档选 640×480（@60 实测稳定零丢帧）；720p 可用但建议 fps_limit 限 30 |
| 6 | **分辨率切换一次失败后 USB 永久打不开**（get_frame_list 持续 `ESP_ERR_INVALID_ARG`） | teardown 在 close 成功前置 `stream=NULL` → worker 停止态排水捞到 pause 前在途回调的竞态帧时句柄已空 → 帧被丢弃永不归还 → 驱动 "Not all frames are returned" → close 永久失败 → 流对象泄漏占住接口 | 句柄生命周期延长到 close 成功；close 重试（10 轮）每轮前重新 drain，配合 worker 排水把竞态帧归还驱动；修复后 30 次连续切换（320/640/720/176/352 混合 + 源往返）零失败 |
| 7 | 720p@60 持续码率 ~12Mbps 时 **httpd 被挤死**（POST 超时/连接 reset），TCP 到达仅 24fps | 20MHz Wi-Fi 链路过载 + LWIP TCP 窗口 23KB 卡带宽延迟积 | 大于 640×480 的档位自动限 30fps（fps 传值与驱动**同源浮点计算**，避开 FLOAT_EQUAL 精度坑：整数 60.0f 对 166667→59.99988f 差 2.4e-4 不匹配）；TCP SND/WND 23392→49152 |
| 8 | 真实 UI 环境（多浏览器 tab 持流 + 500ms 状态轮询）下切换分辨率仍可能触发 #6 的流泄漏 | `stream_start`/`encoder_open` 失败路径原为裸 `uvc_host_stream_close`（无 drain）——start 已 unpause 后失败必踩"帧未归还→close 失败→泄漏"；teardown_req 事件丢失时流悬挂无自愈 | 抽 `safe_close_stream()`（多轮 drain+close）统一所有关闭路径；monitor 检测 `want=0 且流悬挂` 自动补 teardown；帧环 3→5 槽（多客户端各持槽引用时 3 槽会 publish 全失败，实测 out_drops 4661）。加固后 18 次快速连切（2 个限速客户端 + 轮询，含 720p）**零失败零丢帧** |
| 9 | **UI 切分辨率"看似无效"**：下拉选完应用后立即弹回 1280x720；之后再点"应用配置"或动质量/帧率任意控件，设备被悄悄切到 720p | applyConfig 成功后 `buildResOptions()` 用 `innerHTML` 重建 `<select>` 选项，浏览器把选中项重置回**第一项**（supported_res 首项恰为 1280x720）；且源切换瞬间 S.status.supported_res 还是旧源档位表 | buildResOptions 重建后恢复选中项（当前档不在表内时追加"（当前）"选项兜底）；applyConfig 先 pollStatus 刷新档位表再锚定设备实际生效的 j.res；renderStatus 检测档位表变化自动重建。浏览器实测：USB/DVP 往返 + 176/352/640 连切，下拉保持、横幅一致、原始流真实帧尺寸全部吻合 |
| 10 | **切分辨率/模式后 USB 永久打不开**（res=0x0、usb.state=error，重插不恢复） | 四因叠加：① 开流窗口（encoder_open + rebuild_tiers 串口灌 38 行约 200~600ms）排队帧被 `xQueueReset` **静默丢弃** → 帧计数泄漏 → close 永久 "Not all frames are returned" → 流对象占住接口，此后一切 open 恒 `INVALID_ARG`；② teardown close 失败后置 `stream=NULL` 弃句柄，尾帧再也无法归还；③ monitor 自愈在**锁外**读 want/stream，残余 teardown_req 在新流 start 后 12~16ms 误杀之（串口实证 "want=1" teardown）；④ `set_mode` 先翻 mode 再 teardown，worker 把旧格式帧误路由 | ① 开流收尾改 `drain_work_queue` 归还排队帧（日志实证 "开流收尾归还 N 个排队帧"）；② `ret_frame()` 统一归还+记账（`frames_held`，close 前必须归零）；③ close 失败保留句柄交 monitor 重试；④ monitor teardown 改**锁内二次求值**；⑤ set_mode 先 teardown 后翻 mode。验证：14 步复现矩阵全绿 + 20 次模式往返（reencode↔passthrough × 4 档）零失败，串口 0 "Not all frames"、0 误杀 |

另：显式指定帧率（如 60.0f）在 `uvc_claim_interface` 的描述符匹配中失败过一次（驱动按 `10e6/interval` 的浮点值匹配），故统一传 fps=0 用设备默认帧间隔。

### 2. 两种工作模式（U2，数据中必须区分）

| | 模式 A：passthrough（默认，主推） | 模式 B：reencode |
|---|---|---|
| 链路 | 摄像头 MJPEG → ESP32 **原样转发** | 摄像头 YUY2 → ESP32 换序/缩放/OSD → **S31 硬件 JPEG** → 转发 |
| 画质可控性 | ❌ 由摄像头固件决定，ESP32 不可调。CSV `quality=passthrough`，**不伪造数值**，以 `jpeg_avg_bytes` 作等效画质指标 | ✅ `quality` 1~100 可控，与 DVP 同档严格可比 |
| 预期延迟/CPU | 最低（省整个编码环节） | 略高（多一次编码 + YUY2 大流量） |
| OSD 光学闭环 | ❌ 不可用（不解码无法烧录计数器） | ✅ 叠加在"帧到达后、编码前"，光学校验测的是真实端到端 |
| CSV `usb_mode` | `passthrough` | `reencode` |

启动/切换时自动读描述符：优先 MJPEG（→passthrough 可用）；无 MJPEG 档的摄像头 fallback
YUY2（→reencode），日志与 UI 均明确当前模式。模式切换 `POST /api/config {"usb_mode":...}`，
失败自动回滚原模式。

### 3. 时间戳语义（U4，本次度量学核心）

**USB 源的 `t_capture_us` = 完整一帧到达 ESP32 的时刻**（`frame_cb` 打点，即 UVC 帧边界
EAV 处理完成时刻），它已经包含摄像头内部不透明延迟：

```
[传感器曝光 + 摄像头内部 ISP + 摄像头内部 JPEG 编码(passthrough 才有) + USB 传输]  ← 不可见不可控，数 ms~数十 ms
+ [ESP32 处理：直通=拷贝转发(≈0)；重编码=换序/缩放/硬件编码（精确计时 enc_us = t_encode − t_capture）]
+ [网络传输 + 浏览器解码渲染（时钟同步法，估算）]
```

- CSV 用 `capture_ts_meaning` 区分：DVP=`sensor_out`，USB=`frame_arrival`。**两源数据同列可比的前提
  是承认这个语义差**——对比表中 USB 的延迟天然偏大摄像头内部那一段，不是链路劣化；
- UI 三段拆分展示（哪段是估算已标注）：① 摄像头内部（估算/标定值） ② ESP32 处理（精确实测） ③ 网络+浏览器（估算 = 端到端 − ②，①未标定时该项含①）。

**摄像头内部固有延迟（`usb_cam_inherent_latency_ms`）为何只能估**：设备侧任何 API 都拿不到
"传感器曝光完成时刻"，唯一入口是光学闭环——

```
标定流程（reencode 模式，≥10 次采样）：
  1. /overlay 开启画面毫秒叠加（叠加发生在"帧到达 ESP32 之后"）
  2. 摄像头对准另一块屏的毫秒计时器，连拍 ≥10 张流画面
  3. 光学端到端 = 照片中（屏幕计时 − 画面内设备计数）
  4. 摄像头内部延迟 ≈ 光学端到端 − 同条件软件法端到端（时钟同步法）
  5. POST /api/config {"usb_inherent_ms": <均值>} 写入 → CSV 填该值，UI 三段拆分启用
误差：光学法本身 ±17 ms（60 Hz 屏幕刷新半格 + 读数），建议 20+ 张取均值；
      passthrough 与 reencode 的内部延迟差 ≈ 摄像头内部 JPEG 编码耗时，可用两模式各自标定后差分
```

### 4. 分辨率/帧率档位（U3）

- 描述符枚举的**全部 (格式, 分辨率, 帧率)** 在插入时打印到串口，UI 的 USB 面板列表展示（全部为"原生"档）；
- passthrough 只接受**原生精确档**（不解码所以不能缩放）；reencode 可用 YUY2 原生档向下的
  虚拟档（整数抽取/中心裁剪，UI 标"（缩放）"，帧级 `X-Scaled`/CSV `scaled` 同步标注，源分辨率见 `/api/status` 的 `native`）；
- 扫描矩阵只包含原生档（DVP 侧虚拟档机制保持不变）；`fps=0` 表示用摄像头默认帧间隔
  （描述符 `default_interval`），实测帧率以到达 FPS 为准。

### 5. 热插拔 / 掉线 / 供电（U5）

- 自建 USB Host client 收 `NEW_DEV`/`DEV_GONE`（广播事件）+ 开机补扫地址列表（client 注册
  不回放已连接设备）→ 掉线即 `usb_disconnect_count++`；
- 断线清理顺序（防泄漏）：暂停流 → 归还全部帧缓冲（`uvc_host_frame_return`）→ `uvc_host_stream_close`
  （死设备容错）→ 关编码器。重新插入后若 USB 是当前源则**自动重连**（1 s 退避重试）；
- 打开/协商反复失败进入 `error` 状态——**首先检查供电**（Type-C 口补电），固件日志与 UI 均提示。

## 五、软件架构

```
main/
├── main.c            初始化顺序 + 看护（Wi-Fi 断连重启 / DVP 编码停滞重建 / USB 状态播报）
├── source_if.c       ★ 统一采集源调度（U1）：vtable 分发 + 热切换（失败回滚）+ USB 查询转发
├── source_dvp.c      DVP 后端 = camera_pipeline 的薄适配（原通路零改动）
├── source_usb.c      USB 后端：usb_host_uvc 原生 API + 热插拔自动重连 + 两模式（见「USB UVC 采集通路」）
├── camera_pipeline.c DVP 采集 → 硬件 JPEG(M2M) → 帧环；分辨率切换重建；三任务流水
├── frame_ring.c      最新帧环形发布/订阅（PSRAM + 引用计数；丢帧式；槽带 source/scaled/ts_meaning 元数据）
├── yuv_osd.c         5x7 毫秒计数器叠加（光学闭环；USB 仅 reencode 模式可叠加）
├── wifi_net.c        STA/SoftAP + mDNS + RSSI/PHY + SNTP(可选)
├── stream_server.c   :81 原生 socket 流服务（/stream、/ws、UDP 分片），每客户端独立任务，TCP_NODELAY
├── http_server.c     :80 管理 API（esp_http_server；含 USB 状态/档位/标定值）
├── metrics.c         每秒聚合（FPS/码率/CPU/内存/栈水位）+ 串口 CSV + 码率自适应降质（直通模式仅告警）
├── scan_ctrl.c       扫描状态机（按源分组矩阵 / 两级进度 / 完整 CSV 表头）
├── web_ui.c / fs_web/index.html   内嵌单页 UI（EMBED_FILES，离线可用；源切换/USB 面板/三段时延/对比视图）
docs/API_NOTES.md     esp_video/BSP 源码侦察笔记（API 均有出处）
tools/pi_compare.py   第三方对照测量（HTTP 流，时钟同步法，同口径）
tools/udp_receiver.py UDP 分片接收端（--scan 模式供扫描 UDP 行回传）
RESULTS_TEMPLATE.md   实测数据回填模板（Markdown 表 + CSV 表头）
```

上层（编码/传输/度量/扫描/UI）只依赖 `source_if.h`；新增采集源只需实现 vtable 并在
`source_if.c` 注册，不动任何上层逻辑。帧环跨源共享且永不销毁——切换采集源时流客户端
持续收帧（fid 连续），无断流。

并发设计要点：采集（core1 高优先级）/编码（core1 中）/网络发送（core0 低）三级流水；
发送端只取最新帧（丢帧保时延）；帧缓冲/编码输出/客户端发送缓冲全 PSRAM；
`/stream` 每客户端独立任务（最多 `CAMTEST_MAX_STREAM_CLIENTS`）+ 环形引用计数防撕裂；
长时间发送靠 socket 超时 + 每帧让出，任务 WDT 放宽到 10 s（理由：大帧 PSRAM→socket 拷贝可超 1 s）。

## 六、精度与误差分析（如实）

1. **设备侧指标**：微秒级单调钟，误差 = 打点执行时间（≪1 ms），可信。
2. **时钟同步法残差来源**：
   - RTT 不对称（上行/下行路径不同）→ 用 min-RTT 样本抑制，典型 ±1~3 ms；
   - offset 漂移（两端晶振频差 ~20 ppm → 30 s 漂移 ~0.6 ms）；
   - 浏览器 `t_arrive` 在"part 头解析完成"时打点，比真实首字节到达晚一个解析周期（<1 ms）；
   - JS 事件循环延迟（页面卡顿时 t_arrive 偏大，重负载时可达数十 ms —— 用 RTT/offset 波动判别）。
   综合：**软件法绝对误差通常 ±5 ms 内**；以光学闭环标定值为准。
3. **光学闭环自身误差**：屏幕刷新（60 Hz 一格 16.7 ms）+ 人为读数，取 10+ 张照片均值。
4. **到达 FPS vs 采集 FPS**：浏览器掉帧（解码/渲染慢、WS 头解析）会低估链路能力，故同时展示设备侧 FPS。
5. S31 **无 H.264 硬编码**（仅 JPEG codec），MJPEG 码率天然偏高（720p q40 约 8~15 Mbps 量级），
   多车并发 2.4 GHz 拥塞时这是硬约束——结论章节如实记录。
6. 编码器引擎单帧超时 40 ms（esp_video 硬编码）：720p 超时会以错误帧呈现（`drop_encode` 计数），见实测。

## 七、待核实清单（不编造 API，逐条列明）

| # | 事项 | 现状 |
|---|---|---|
| 1 | esp_video **2.5.0**（含 m2m 示例与更全 sensor 格式表）何时在组件存储索引可下载 | registry API 已列 2.5.0，存储 403；升级后低分辨率档位可能直接可用 |
| 2 | esp32s31 是否会出 IDF **v6.1.1** 正式 tag | 目前 release/v6.1 分支头为 v6.1.0；README 已锁 commit |
| 3 | sensor 帧内**硬件 PCLK 频率**无直接 API | 格式表仅有 `xclk`（20 MHz 输入时钟）；日志打印 xclk 代替，已在 UI 标注 |
| 4 | Wi-Fi **当前链路 PHY 速率**（Mbps）公开 API | v6.1 未见公开接口（仅 RSSI/PHY 模式）；如需可用 debug 接口或测包估计 |
| 5 | OV3660 RGB565_BE 经字节交换喂编码器的画质 | 已实现 BE→LE 交换路径，但默认主路径为 YUYV→UYVY，RGB565 路径未逐像素验证 |
| 6 | LCD 子板（ESP32-S3-LCD-EV-Board-SUB3）本地叠加 | 未接硬件；BSP 支持 display（esp_lvgl_port），接口已预留，暂不默认编译 |
| 7 | 编码器 40 ms 单帧超时在 720p 下的真实余量 | 实测见 RESULTS_TEMPLATE（若 720p 频繁错误帧，属驱动常量限制） |
| 8 | `esp_video_get_dvp_video_device_sensor()` 为私有 API | 2.2.0 固定版本下稳定；升级组件时需复查（CMake 已隔离 include 路径） |
| 9 | UVC 摄像头 **iProduct 产品名** | espressif/usb 1.5.0 无字符串描述符公共 API（`usb_host_get_string_descriptor` 不存在）；现以 `vid:pid` 代替（如 `1bcf:28c4`）。若需要真名须自行提交控制传输 GET_DESCRIPTOR(String)，见 `source_usb.c` TODO |
| 10 | USB HS DMA 与 DVP GDMA/Wi-Fi/LCD 的 **PSRAM 总线争抢** | 架构上确有共存路径（USB 帧缓冲/帧环/编码缓冲全 PSRAM），量化数据待真机：对比 USB 推流前后 Wi-Fi 空闲 RTT 与 DVP 编码耗时即可判定；结果记入 RESULTS_TEMPLATE |
| 11 | USB 摄像头**内部固有延迟标定值** | 需按「USB UVC 采集通路 §3」流程真机标定（≥10 采样），写入后 CSV 才有值；设备侧绝不自动生成 |
| 12 | 各型号 UVC 摄像头兼容性（MJPEG 档位真伪、dwMaxVideoFrameSize 虚标） | usb_host_uvc 按协商值分配帧缓冲可容忍虚标；但个别摄像头只报 YUY2 或帧率虚标——上电串口清单即判真伪 |
| 13 | esp_video 2.5.x 的 UVC V4L2 封装是否修复了断线缺陷 | 未验证（2.5.0 组件存储不可用）；修复后可评估切回 V4L2 统一通路（设备节点 `/dev/video40` 代码路径已侦察完毕） |

## 七点五、USB UVC 采集源（第二通路，source_if 统一源架构）

固件内置统一采集源抽象（`main/source_if.*`）：上层（传输/度量/扫描/UI）不感知底层实现，
DVP 与 USB UVC 可运行时热切换（`POST /api/config {"source":"usb"|"dvp"}`，切换失败自动回滚原源，
绝不处于双源皆停状态）。

- **硬件**：Korvo-1 的 USB Type-A 座（USB 2.0 HS 480 Mbps，Host-only，经 TPS2051C 限流 500 mA）。
  推流时建议同时从 Type-C (Power) 口补供电。
- **驱动选型**：`usb_host_uvc 2.5.2` 原生 API。**不走** esp_video 的 `/dev/video40` V4L2 封装——
  源码侦察发现其断线去初始化路径存在资源泄漏与状态卡死（`uvc_video_deinit` 在 dev_addr==0 时
  提前返回，`uvc_host_stream_close` 永不被调用；open 还会同步阻塞等待枚举 10 s），不满足
  "拔插 3 次不卡死 + 自动重连"；原生 API 对死设备容错（帧全归还后 remove 释放全部资源）。
- **两种工作模式**（UI「USB 模式」或 `/api/config usb_mode`）：
  - `passthrough`：摄像头内部 MJPEG 直通（不解不编，延迟/CPU 最优；**画质不可控**，quality 显示 0）；
  - `reencode`：YUY2 → S31 硬件 JPEG（质量可控，带宽压力更大）。
- **时延语义**（`capture_ts_meaning` 字段，UI/CSV 均标注）：DVP 的 `t_capture` = 传感器输出时刻；
  USB 的 `t_capture` = **完整一帧到达 ESP32 时刻**（含摄像头内部曝光/ISP/编码/USB 传输的不可见延迟）。
  该"摄像头内部固有延迟"用光学闭环标定后经 `/api/config {"usb_inherent_ms":…}` 写入，
  UI 时延拆分栏按 ①摄像头内部(估算) ②ESP32 处理(精确) ③网络+浏览器(估算) 三段展示。
- **状态机**：`no_device / device_ready / streaming / error`（典型 error 诱因：供电不足），
  掉线自动重连并计数（`usb_disconnect_count`）。
- **扫描矩阵**已扩展：CSV 新增 `video_source/sensor/usb_device_name/scaled/usb_mode/protocol/
  tcp_nodelay/lcd_on/capture_ts_meaning/usb_cam_inherent_latency_ms/usb_disconnect_count/
  udp_loss_rate/udp_incomplete_rate` 等列（DVP 行 usb 列填 n/a/0）。

## 八、已知限制

- 本板 **Type-A USB 为 Host-only**：作为 **Host 接 UVC 摄像头**（本工程 USB 通路）没有问题；
  只是做不了 UVC Device（把画面喂给 PC）。
- USB **passthrough 模式无法叠加 OSD**（不解码）——光学校验请在 reencode 模式做，
  两种模式的摄像头内部延迟差即摄像头内部 JPEG 编码耗时（可各自标定后差分）。
- USB 摄像头多数**不支持 160×120**（通常最低 320×240 或 176×144）：DonkeyCar 若以 160×120
  为推理输入，DVP 的软件抽取档可直接给，USB 直通档位以描述符枚举为准（reencode 可经缩放给到 160×120）。
- USB passthrough 的画质/帧率/曝光策略由摄像头固件决定，本系统**无法干预也无法如实上报
  其"质量值"**（CSV 以 passthrough+jpeg_avg_bytes 表达，属设计约束而非缺陷）。
- DVP 无硬件缩放、sensor 驱动（2.2.x）无 windowing 配置 → 低分辨率通过**软件抽取/裁剪**实现；
  高帧率（60~93fps）通过**时钟树移植 + V 窗口裁剪**实现（推导与实测见上节）。代价均为视场/形变，
  无失真全视场的帧率上限是 31fps（L1）——再往上必须裁 V 窗口，这是 OV3660+DVP 的物理约束。
- 多台小车并发：架构已支持多观看端与 AP 模式，但 2.4 GHz 同频拥塞的量化测试未包含在本轮（预留 RSSI/信道记录）。
- UDP 分片模式无重传（探索下限用，花屏属预期）。

## 九、结果

见 `RESULTS_TEMPLATE.md`（回填模板 + 指标口径）。扫描产物一键导出 `/api/scan/csv`。

### USB UVC 通路实测（2026-10-07 首轮真机，摄像头 0bda:1376，仅枚举/直通/重编码功能验证；完整 DVP vs USB 对比待按模板复测）

| 项 | 值 | 口径 |
|---|---|---|
| 枚举 | 12 档：MJPEG 1280×720/800×600/640×480/352×288/320×240/176×144 @60/30/25fps；YUY2 同分辨率 @10fps | 描述符实测（串口完整打印） |
| USB passthrough 320×240@60 | 设备 cap **59.8~60.0fps**；HTTP 到达 **46~54fps**（间隔 p95=1 帧） | 实测 2026-10-08 |
| USB passthrough 640×480@60 | 设备 cap **60.0fps** 满帧（URB 扩容 4→8 后由 54.8 提升）；HTTP 到达 **40.7~45.5fps**（TCP 窗口 48KB 后由 24.7 提升；间隔 p95=2 帧）；室内弱光自动曝光降至 13~15fps 属摄像头行为 | 实测 2026-10-08 |
| USB passthrough 1280×720@30 | 设备 cap 24.9~30.0fps（大档自动限 30fps，见工程实录 #7）；HTTP 到达 22fps；60fps 档会挤死 httpd 不再使用 | 实测 2026-10-08 |
| 分辨率切换压力 | 30 次连续切换（混合档位 + 源往返）零失败；加固后再压 18 次快速连切（2 个限速流客户端 + 500ms 状态轮询并发，模拟真实 UI 双 tab）**零失败、全程零丢帧** | 实测 2026-10-08 |
| USB reencode 640×480 q20/q30 | 5.0fps 编码发布（YUY2@10fps 档）、9.2~10.5 kB/帧、CPU 5~7/24%、零丢帧 | 设备侧，实测 |
| UI 热切换 | DVP↔USB 下拉即切、失败回滚、流客户端不断流；连续 6 次往返无重启 | 实测 |
| 摄像头内部固有延迟 | **未标定**（需按 §四点五.3 流程光学校准，≥10 采样） | 待测 |
| DVP vs USB 时延对比 | **待数据**（需上述标定 + scope=both 扫描，模板已就绪） | 待测 |

### 已实测参考数据（2026-10-07，本仓库固件，HUAWEI-DKC 2.4G STA，11ax ch1 20MHz，RSSI −34~−40 dBm）

板上 sensor 实测为 **OV3660**（上电自动侦测，日志打印 3 种格式）。
软件默认凭据为测试网络（HUAWEI-DKC / dkc@2026），请在 menuconfig → CamTest Configuration 修改。

**设备侧（精确实测，串口 CSV / 扫描导出）**

| 分辨率（来源） | 质量 | 采集 FPS | 编码 FPS | 编码段耗时 | 单帧均值 | 码率 | 丢帧 |
|---|---|---|---|---|---|---|---|
| **160x120 hifps-L2**（默认） | 20 | **60.0** | **60.0** | ~2 ms | 1.2 kB | 0.58 Mbps | 0 |
| **160x120 hifps-L3** | 20 | 74.6 | 74.6 | ~2 ms | 1.2 kB | ~0.7 Mbps | 0 |
| **160x120 hifps-L4** | 20 | **93.0** | 93.0 | ~2 ms | 1.2 kB | ~0.9 Mbps | 0 |
| 160x120 hifps-L1（全视场） | 20 | 31.0 | 31.0 | ~2 ms | 1.2 kB | 0.30 Mbps | 0 |
| 160x120 原生（240x240 裁剪@25fps） | 10~40 | 25.0 | 25.0 | 2.6 ms | 1.1~1.4 kB | 0.23~0.28 Mbps | 0 |
| 240x240（原生） | 25 | 25.0 | 25.0 | ~10 ms | ~4 kB | ~0.9 Mbps | 0 |
| 320x240（640x480 抽取 2:1） | 10~40 | 11.1 | 11.1 | 9.0 ms | 3.5~4.3 kB | 0.32~0.39 Mbps | 0 |
| 480x320（640x480 裁剪） | 10~40 | 11.1 | 11.1 | 15.0 ms | 4.4~5.7 kB | 0.40~0.52 Mbps | 0 |
| 640x480（原生） | 10 | 11.1 | 11.1 | 26.3 ms | 8.5 kB | 0.755 Mbps | 0 |
| 640x480（原生） | 20 | 11.1 | 11.1 | 26.2 ms | 10.0 kB | 0.885 Mbps | 0 |
| 640x480（原生） | 30 | 11.1 | 11.1 | 26.2 ms | 10.9 kB | 0.972 Mbps | 0 |
| 640x480（原生） | 40 | 11.1 | 11.2 | 26.2 ms | 11.7 kB | 1.048 Mbps | 0 |

- 编码段耗时（t_encode − t_capture）含"拷贝+YUYV→UYVY 换序 + 编码"全程（S31 DVP→JPEG 无零拷贝路径），
  硬件编码本身占比小于一半；这也解释了它对质量不敏感。
- 240×240 档 25 fps（sensor 标称 24fps）——在线切换无需重启，画面/指标即时生效。
- 关闭 Wi-Fi modem 省电（`esp_wifi_set_ps(WIFI_PS_NONE)`，本固件默认关闭）后局域网 ICMP RTT
  从 ~76 ms 降至 ~11 ms，对端到端时延影响显著，FPV 场景务必保持关闭。

**端到端时延（时钟同步法；浏览器实测，扫描期在线回传）**：160×120 **11~16 ms**（P95 15~54）、
320×240 19~30 ms、480×320 26~40 ms、640×480 41~66 ms（第三方脚本 pi_compare 同口径 36.0 ms）。

**端到端时延（时钟同步法；pi_compare.py 第三方脚本与浏览器同口径）**

| 传输 | 条件 | mean | P95 | max | σ |
|---|---|---|---|---|---|
| HTTP MJPEG | 640x480 q20，LAN | **36.0 ms** | 55.9 ms | 95.8 ms | 9.7 |
| UDP 分片 | 同上 | 42.7 ms | 68.5 ms | 135.4 ms | 14.7 |
| 浏览器面板 | 同上（含 JS 解码渲染） | 41~66 ms | 50~126 ms | — | 10~37 |

- 端到端 ≈ 编码段（26~27 ms，精确实测）+ 网络+浏览器段（估算，10~30 ms，随 Wi-Fi 抖动）。
- 扫描矩阵结果：160×120/320×240/480×320/640×480 十六组全部可跑（虚拟分辨率路径，零丢帧）；
  800×600/1280×720 两档在 OV3660 上如实标记 unsupported（需要放大或更高原生传感器；
  插 SC101IOT 后 1280×720 原生 UYVY 可用）。

### DonkeyCar 选型建议（DVP vs USB UVC）

> 结论分两层：**结构性判断**（现在就能给，基于链路结构与已实测 DVP 数据）与
> **数据确认项**（USB 组真机数据回填后复核，模板见 RESULTS_TEMPLATE.md 的 USB 节）。
> 未标注"实测"的数字一律是结构推导，不是测量值——不接受把估算包装成实测。

**结构性判断（当前证据下成立）**

1. **延迟结构**：USB 通路把"摄像头内部延迟"（曝光+ISP+摄像头内编码+USB 传输，估计 15~50 ms 量级，
   随型号差异大）折进了端到端；DVP 的 t_capture 是传感器输出时刻，这段不受控因素为 0。
   即使 USB passthrough 省掉 S31 侧编码（DVP 在 160×120 时编码段仅 ~2 ms，实测），省掉的
   2 ms 很难抵消摄像头内部十几 ms 的不透明延迟——**低延迟 FPV 首选 DVP 的结构性理由**。
   是否反超，待 USB 组光学标定数据回填（RESULTS_TEMPLATE USB 节）后复核。
2. **分辨率下限**：DonkeyCar 常用 160×120 推理输入。DVP 可由 240×240/640×480 软件抽取/裁剪
   直接给到 160×120（实测 60fps@L2 甚至 93fps@L4）；USB 摄像头绝大多数最低 320×240，
   直通模式拿不到 160×120（reencode 经缩放可以，但多付一次编码）。
3. **体积/安装/供电/成本**：DVP 摄像头是板上排线一体（OV3660，0 元成本、零额外安装、
   由主口供电即可）；USB 摄像头需外加（¥30~100+）、占 Type-A 座、**大概率需要 Power 口
   补电**（500 mA 限流），车模布线与配重都要考虑。
4. **画质与码率**：DVP 的 quality 严格可控且已实测各档码率（160×120 q20 ≈ 0.58 Mbps）；
   USB passthrough 画质/曝光策略是黑盒（CSV 以 passthrough+jpeg_avg_bytes 表达），
   对需要一致训练数据的 DonkeyCar 是额外变量。

**USB 通路的真实优势（什么场景该用它）**

- 需要**更高分辨率/更高画质**的记录视角（720p/1080p MJPEG 摄像头直通，S31 侧近零开销）；
- DVP 排线损坏/换装不便的现成 USB 摄像头复用；
- 需要摄像头端变焦/自动对焦等 DVP 方案没有的能力。

**最终建议**：DonkeyCar FPV/推理主链路用**板载 DVP**（延迟结构最优、160×120 原生可达、
零增量成本与供电风险）；USB UVC 作为**高分辨率记录/备选链路**保留——本工程已把两路
纳入同一度量与扫描体系，插上摄像头跑一次 `scope=both` 扫描即可得到本车环境下的
逐项对比数据，替换选型无需改代码。
