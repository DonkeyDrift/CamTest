# CamTest Tab5 — M5Stack Tab5（ESP32-P4 + C6）硬件 H.264 图传

> 本文件是 `tab5-h264` 分支的交付说明。S31 主线（仓库根目录）不受影响；Tab5 是
> 独立 IDF 工程（`tab5/`），复用 S31 的架构骨架（source_if 抽象 / frame_ring /
> WS 时钟同步 / 三层看门狗 / NVS 持久化 / 光学标定），重写了编码与平台层。

## 成果摘要（2026-10-09 夜间真机实测）

| 指标 | USB-UVC H.264（主模式） | USB-UVC MJPEG 直通 | USB-UVC JPEG 重编码 | 板载 SC202CS（MIPI） |
|---|---|---|---|---|
| 分辨率@帧率 | **640x480@15fps** | 720p@14.8（全档≈15） | 640x480@15 | **640x480@27~30fps**（720p@19） |
| 端到端时延(到达) | **mean 50.3 / p50 44.8 / p95 88.9 ms** | — | 52.9 / 45.5 / 105.9 | 65.2 / 61.6 / 94.6 |
| 设备处理②（精确） | 26.5ms（解码5.9+重排14.3+编码5.9） | ≈0（拷贝） | 23.2ms | 39.1ms |
| 码率 | **0.37 Mbps** | 7.3 Mbps@720p | 1.05 Mbps | 1.43 Mbps |
| 负值帧（时钟同步质量） | 0/179 | — | 0/177 | 0/305 |

S31 对比：320x240@60 端到端 53.1ms。Tab5 在 **4 倍像素**下做到 50ms 级时延，
码率只有 MJPEG 的 **1/20**（H.264 的带宽红利），且三条通路全部 0 负值帧。

## 关键硬件事实（真机 + 原理图验证，花了一晚上踩出来的）

1. **Tab5 的 Type-A 主机口是全速 USB（12 Mbps），不是高速**。原理图：
   Type-A ← USB2_OTG（GPIO19/20，FS/LS PHY）；HS UTMI PHY 引脚（GPIO26/27）
   被 I2S 音频占用；Type-C（GPIO24/25）= USB-Serial-JTAG（烧录/串口）。
   ⇒ UVC 摄像头带宽天花板 ≈1 MB/s，且相机固件在全速下**所有档位锁 15fps**
   （实测 176x144 也只有 15fps，与分辨率无关）。720p MJPEG（7.3Mbps）恰好
   贴着全速上限能跑 15fps，但几乎没有余量给多客户端。
2. **P4 rev<3.0 的硬件约束**（本板 P4 为 rev1.3）：
   - 硬件 H.264 编码器（HW v3）**只接受 `O_UYY_E_VYY`**（隔行 UYY/VYY 的
     YUV420 打包，fourcc `OUYY_EVYY`）；UYVY 等多格式输入是 rev≥3.0 才有。
   - JPEG 解码器**禁止 422→420 重采样**（4:2:0 采样源可直出 420）。
   - PPA（缩放/旋转）**不收 YUV422 输入**（UYVY 只在 rev≥3.0 支持），但
     YUV420→YUV420 缩放可用。
   ⇒ 本工程的 H.264 管线：MJPEG → jpeg 硬解(YUV422) → **软件重排**为
   O_UYY_E_VYY（无损，422 色度逐行都有；24→32bit 重打包，14.3ms@640x480）
   → 硬件 H.264。缩放场景：先重排再 PPA（YUV420 域）。
3. **Wi-Fi 在 C6 协处理器上**：esp_hosted 3.0.x（SDIO 4bit 40MHz，Tab5 板级
   选项一键配好引脚）。C6 出厂固件与 host 3.0.7 大版本号不符（报 0.0.0），
   但兼容模式可用（STA 实测 OK）；若吞吐不达标可刷 C6 固件对齐版本。
4. esp_video 2.x 全系列在 components-file 存储下载失败（S3 AccessDenied，
   2026-10-09），故 `tab5/components/` 里放了 esp_video 2.2.0 + esp_cam_sensor
   2.2 本地副本（复制自 S31 工程已验证的 managed_components）。

## 构建 / 烧录 / 使用

```bash
. ~/esp/esp-idf/export.sh          # IDF v6.1（与 S31 同一套）
cd tab5
idf.py set-target esp32p4          # 首次
idf.py build
idf.py -p /dev/cu.usbmodem11201 flash   # Tab5 USB-C 口（USB-Serial-JTAG）
```

- Wi-Fi 凭据：`tab5/main/Kconfig.projbuild`（默认 HUAWEI-DKC）。
- 上电后：`http://tab5-cam.local`（或串口日志里的 IP）。**H.264 模式请用
  Chrome/Edge**（WebCodecs；Safari/ZCode 内置浏览器无 WebCodecs，页面会提示
  并自动保留指标面板，画面可切 passthrough+HTTP MJPEG 查看）。
- 探针工具（`tab5/tools/`）：
  - `tab5_ws_probe.py <ip> [秒] [out.h264]`：抓 WS v2 流、校验头、存 Annex-B
    （用 PyAV/ffplay 离线解码验证）；
  - `tab5_latency.py <ip> [秒]`：P85 时钟同步 + 端到端时延统计；
  - `tab5_uart.py <port> <秒> <out>`：抓串口（注意：**打开即复位板子**，
    USB-Serial-JTAG 的 DTR/RTS 行为，抓运行态日志要用长窗口覆盖重启）。

## Web UI（单文件内嵌，无外部依赖）

实时画面（H264=WebCodecs / JPEG=bitmap）· 时延拆分（①摄像头内部②设备处理
③网络+浏览器 + H264 解码耗时）· 时钟同步 P85 窗口 · 到达/采集/编码 FPS ·
码率与自适应降码率 governor（H264 模式调码率而非 JPEG 质量）· UVC 档位表 ·
三层看门狗状态 · 光学标定向导（照片冻结双钟法，H264/reencode 模式可叠加
毫秒计数器 OSD）。

## 架构

```
[UVC 0bda:1376] usb_host_uvc 原生 API ─┐
                                       ├─ source_if（热切换/回滚，帧环终身持有）
[板载 SC202CS] esp_video CSI+ISP ──────┘        │
                                            frame_ring（5×1.5MB PSRAM）
                                                │
                 passthrough: MJPEG 原样 ───────┤
                 h264:  硬解→软件重排 O_UYY_E_VYY→esp_h264 硬编 →├→ WS :81/ws（AVC1/MJP1 头）
                 reencode: 硬解→esp_driver_jpeg ─┘
                                                └→ HTTP :81/stream（仅 JPEG 帧）
C6 Wi-Fi（esp_hosted SDIO）←— esp_wifi_remote（IDF v6.1 内置）
```

可靠性继承（S31 真机坑位全数移植）：帧归还记账（frames_held）+ safe_close
多轮排水、monitor 帧停滞看门狗（5s teardown→重开、6 轮不愈重启）、打开失败
60s 重启（USB 重枚举自愈）、cam_watch 流冻结 20s 兜底、NVS 按字段持久化
（CTM2 布局）、开流收尾归还竞态帧。

## 观看方式与 WebCodecs 的 Secure Context 陷阱

**WebCodecs（VideoDecoder）只在 Secure Context 暴露**：`http://192.168.x.x` 直连时，
**连 Chrome/Edge 也没有 VideoDecoder**（实测 UA Chrome/146，`isSecureContext=false`）。
因此固件在 h264 模式下**并行发布一条 JPEG 预览流**（与 H264 同源同刻、独立
pv_* 计数不污染主指标）：无 WebCodecs 的页面自动渲染预览流，有 WebCodecs 的
自动忽略它——所有 http 观看端都能出画面。

吃到**真 H.264 硬解低延迟路径**的三种方式（均已真机验证）：

| 方式 | 做法 | 证书摩擦 |
|---|---|---|
| ★ localhost 转发（推荐） | `python3 tools/tab5_secure_proxy.py [设备IP]` → 打开 `http://localhost:8080/` | 无（localhost 天然 secure context） |
| https 直连 | 打开 `https://<设备IP>/`（或 `https://tab5-cam.local/`），流走 wss://…:8443 | 需分别对 :443 与 :8443 各点一次「高级→继续前往」（端口独立例外；WS 连不上时页面会给出引导链接） |
| 信任证书 | 把 `main/certs/tab5_cert.pem` 导入系统信任 | 一次到位 |

- 设备侧：`:443` https（esp_https_server，全功能 API/UI）、`:8443` wss/https
  流（裸 mbedtls TLS，限 3 客户端，每连接 ~40KB 内部 RAM）；`:80/:81` 明文
  全保留。证书为构建期生成的 10 年自签（SAN 含 tab5-cam.local/localhost/
  127.0.0.1/192.168.3.44，换网段 IP 需重新生成或用 mDNS 域名访问）。
- 实测（localhost 路径）：WebCodecs H264 硬解，320x240@44fps e2e≈43ms、
  640x480 处理②≈26ms；wss 探针双流 44.5fps。

## 已知限制 / 后续工作

- **720p H264 只有 8fps**：解码+重排+编码在 720p 下超帧预算（估计重排 ~32ms+
  编码 ~14ms+解码 ~13ms）。优化方向：重排挪到 core0 双缓冲流水、SIMD 化。
- WebCodecs 起播等首个 IDR（GOP 30 帧 ≈2s）；已做开流强制 IDR（连上即有
  SPS/PPS）+ I 帧作 key 块。
- DVP 切源/切档偶发 `STREAMOFF errno=88`（ENOTSOCK，无害噪音，流正常）。
- esp_video 2.2.0 的 `stream->started` 标志跨 close/open 持久（已在 start 里
  幂等防护 + REQBUFS 补救）。
- C6 出厂固件与 esp_hosted 3.0.7 版本握手报 mismatch（兼容模式可用）；吞吐
  标定与 C6 固件对齐是后续优化点（SDIO 40MHz 理论余量很大）。
- 全速 USB 是 UVC 帧率的物理天花板；若需 >15fps UVC，需外接 HS hub/PHY 硬件
  改造，或换用板载 MIPI 相机（27~30fps 已验证）。

## 实测记录（关键日志摘录）

```
src_usb: 剖面 ms：解码 5.9 重排 14.3 H264编码 5.9        （640x480@15fps）
tab5_latency.py: e2e mean=50.3 p50=44.8 p95=88.9 negative=0/179
tab5_ws_probe.py: frames=150 fps=15.01 idr=4 annexb 正常，PyAV 解码
                  Constrained Baseline 640x480 yuv420p
切换矩阵：DVP↔USB 往返、同源修复、20+ 次模式/分辨率切换零失败
```
