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
> 供电：默认 USB(UART) 口；高负载（720p/多客户端）建议同时从 **USB Type-C (Power)** 口补电，
> 避免 500 mA 限流掉电重启。**Type-A 口是 Host-only，本系统不使用**（S31 无 UVC Device 方案）。

### 烧录后串口日志应看到（验收清单）

```
IDF: v6.1.0 | build: ...
===== sensor detect: <OV3660|SC101IOT>（共 N 种格式） =====
  [0] DVP_8bit_20Minput_YUV422_UYVY_1280x720_25fps ...
PSRAM: 16384 kB
STA got IP: 192.168.x.x
mDNS: http://korvo-s31.local
pipeline: <sensor> <W>x<H> <fmt> q20 ...
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
```

网页用 `fetch()` + `ReadableStream` 手动解析（不用 `<img src>`），在 part 头解析完成瞬间打
`t_arrive_browser`，才能算端到端时延。

### WebSocket 二进制头（小端，36B）

`MJP1 | u32 frame_id | u64 t_capture_us | u64 t_encode_us | u16 w | u16 h | u8 quality | u8 rsv | u32 jpeg_len | u16 rsv`

### UDP 分片头（小端，24B，≤1200 B/片）

`u32 'UJPG' | u32 frame_id | u64 t_capture_us | u16 frag_idx | u16 frag_total | u16 frag_len | u8 flags(bit0=首片 bit1=末片) | u8 rsv`
（无重传，可能花屏——用于探索时延下限。）

## 三、HTTP API

| 端点 | 方法 | 说明 |
|---|---|---|
| `/api/sync?t_client=<us>` | GET | `{"t_dev_us","t_wall_ms"}`；设备在**即将写响应前**采样 esp_timer |
| `/api/status` | GET | 设备侧实时指标 + 扫描进度/结果行（JSON） |
| `/api/config` | POST | `{"res":"640x480","quality":20,"fps_limit":30,"overlay":true,"target_mbps":0}`；切分辨率重建链路，仅改质量走轻量路径 |
| `/api/scan/start?modes=sta,ap` | GET | 启动扫描（默认仅当前网络模式） |
| `/api/scan/stop` | GET | 停止扫描 |
| `/api/scan/result` | GET | 进度 + 全部结果行 |
| `/api/scan/csv` | GET | 导出 CSV（表头见下） |
| `/api/scan/report` | POST | 扫描期间浏览器侧每秒回传 `{arrival_fps,bitrate_mbps,latency_mean_ms,latency_p95_ms,latency_max_ms,latency_std_ms}` |

CSV 表头（与任务书一致）：

```
timestamp,sensor,resolution,quality,wifi_mode,bandwidth_mhz,rssi,capture_fps,encode_fps,arrival_fps,jpeg_avg_bytes,bitrate_mbps,latency_mean_ms,latency_p95_ms,latency_max_ms,latency_std_ms,encode_ms,cpu0_pct,cpu1_pct,free_heap,free_psram,drop_frames
```

串口每秒输出聚合 CSV（无浏览器可用）：`CSV,<t_us>,cap_fps,enc_fps,send_fps,mbps,jpeg_avg,drop_cap,drop_enc,heap,psram,mode,rssi,WxH`；
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

### 自动扫描（闭环）

矩阵 = 6 档分辨率（160×120 ~ 1280×720，sensor 不支持的标 `unsupported` 并跳过）
× JPEG 质量 {10,20,30,40}（**方向：越大越清晰、码率越大**，源自 `V4L2_CID_JPEG_COMPRESSION_QUALITY`，UI 已注明）
× 网络模式（当前模式；可勾选同时跑 STA+SoftAP——AP 阶段浏览器会断流并自动重连，拿不到浏览器数据的组该字段留空）。
每组 warmup 2 s + 采样 10 s；设备侧指标自采，浏览器侧时延每秒回传挂到当前组。
**本组件版本（esp_cam_sensor 2.2.x）驱动格式表较窄**：SC101IOT 仅 720p@15/25（UYVY/YUYV），
OV3660 仅 240×240@24 与 640×480@10（YUYV/RGB565）——矩阵中其余档位会如实标 unsupported（见待核实清单）。

## 五、软件架构

```
main/
├── main.c            初始化顺序 + 看护（Wi-Fi 断连重启 / 编码停滞自动重建链路）
├── camera_pipeline.c DVP 采集 → 硬件 JPEG(M2M) → 帧环；分辨率切换重建；三任务流水
├── frame_ring.c      最新帧环形发布/订阅（PSRAM + 引用计数；丢帧式，不堆积）
├── yuv_osd.c         5x7 毫秒计数器叠加（光学闭环）
├── wifi_net.c        STA/SoftAP + mDNS + RSSI/PHY + SNTP(可选)
├── stream_server.c   :81 原生 socket 流服务（/stream、/ws、UDP 分片），每客户端独立任务
├── http_server.c     :80 管理 API（esp_http_server）
├── metrics.c         每秒聚合（FPS/码率/CPU/内存/栈水位）+ 串口 CSV + 码率自适应降质
├── scan_ctrl.c       扫描状态机（warmup/采样/结果行/CSV）
├── web_ui.c / fs_web/index.html   内嵌单页 UI（EMBED_FILES，离线可用）
docs/API_NOTES.md     esp_video/BSP 源码侦察笔记（API 均有出处）
tools/pi_compare.py   第三方对照测量（HTTP 流，时钟同步法，同口径）
tools/udp_receiver.py UDP 分片接收端
RESULTS_TEMPLATE.md   实测数据回填模板（Markdown 表 + CSV 表头）
```

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

## 八、已知限制

- 本板 **Type-A USB 为 Host-only**，无法做 UVC Device → 纯 Wi-Fi 方案（任务书约束）。
- DVP 无硬件缩放、sensor 驱动（2.2.x）无 windowing 配置 → 低分辨率档位（160×120/320×240）不可用，
  DonkeyCar 关注的低延迟小分辨率结论需等「待核实清单 #1」升级后复测或驱动侧增加窗口模式。
- 多台小车并发：架构已支持多观看端与 AP 模式，但 2.4 GHz 同频拥塞的量化测试未包含在本轮（预留 RSSI/信道记录）。
- UDP 分片模式无重传（探索下限用，花屏属预期）。

## 九、结果

见 `RESULTS_TEMPLATE.md`（回填模板 + 指标口径）。扫描产物一键导出 `/api/scan/csv`。

### 已实测参考数据（2026-10-07，本仓库固件，HUAWEI-DKC 2.4G STA，11ax ch1 20MHz，RSSI −34~−40 dBm）

板上 sensor 实测为 **OV3660**（上电自动侦测，日志打印 3 种格式）。
软件默认凭据为测试网络（HUAWEI-DKC / dkc@2026），请在 menuconfig → CamTest Configuration 修改。

**设备侧（精确实测，串口 CSV / 扫描导出）**

| 分辨率 | 质量 | 采集 FPS | 编码 FPS | 编码段耗时 | 单帧均值 | 码率 | 丢帧 |
|---|---|---|---|---|---|---|---|
| 640x480 YUYV | 10 | 11.1 | 11.1 | 26.3 ms | 8.5 kB | 0.755 Mbps | 0 |
| 640x480 YUYV | 20 | 11.1 | 11.1 | 26.2 ms | 10.0 kB | 0.885 Mbps | 0 |
| 640x480 YUYV | 30 | 11.1 | 11.1 | 26.2 ms | 10.9 kB | 0.972 Mbps | 0 |
| 640x480 YUYV | 40 | 11.1 | 11.2 | 26.2 ms | 11.7 kB | 1.048 Mbps | 0 |
| 240x240 YUYV | 25 | 25.0 | 25.0 | ~10 ms | ~4 kB | ~0.9 Mbps | 0 |

- 编码段耗时（t_encode − t_capture）含"拷贝+YUYV→UYVY 换序 + 编码"全程（S31 DVP→JPEG 无零拷贝路径），
  硬件编码本身占比小于一半；这也解释了它对质量不敏感。
- 240×240 档 25 fps（sensor 标称 24fps）——在线切换无需重启，画面/指标即时生效。
- 关闭 Wi-Fi modem 省电（`esp_wifi_set_ps(WIFI_PS_NONE)`，本固件默认关闭）后局域网 ICMP RTT
  从 ~76 ms 降至 ~11 ms，对端到端时延影响显著，FPV 场景务必保持关闭。

**端到端时延（时钟同步法；pi_compare.py 第三方脚本与浏览器同口径）**

| 传输 | 条件 | mean | P95 | max | σ |
|---|---|---|---|---|---|
| HTTP MJPEG | 640x480 q20，LAN | **36.0 ms** | 55.9 ms | 95.8 ms | 9.7 |
| UDP 分片 | 同上 | 42.7 ms | 68.5 ms | 135.4 ms | 14.7 |
| 浏览器面板 | 同上（含 JS 解码渲染） | 41~66 ms | 50~126 ms | — | 10~37 |

- 端到端 ≈ 编码段（26~27 ms，精确实测）+ 网络+浏览器段（估算，10~30 ms，随 Wi-Fi 抖动）。
- 扫描矩阵结果：本组件版本下 160×120/320×240/480×320/800×600/1280×720 五档被如实标记
  unsupported（见待核实清单 #1/#8 与第八节限制）。
