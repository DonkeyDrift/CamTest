# 测试结果记录模板 — ESP32-S31-Korvo-1 Wi-Fi MJPEG 图传

> 使用方法：网页「开始扫描」跑完后用 `/api/scan/csv` 导出 CSV，或把本表逐行手工回填。
> 每行 = 一组「分辨率 × JPEG 质量 × 网络模式」，warmup 2 s + 采样 10 s。
> 表中 **到** 字段（到达 fps / 时延）来自浏览器测量；无浏览器时为空。

## 环境记录（每次批次必填）

| 项 | 值 |
|---|---|
| 日期时间 | 2026-10-__ __:__ |
| 固件版本 / IDF | （串口启动横幅，如 `IDF v6.1.0, build __`） |
| Sensor 侦测结果 | （OV3660 / SC101IOT，串口打印） |
| 路由器型号 / 信道 / 带宽 | |
| STA 距离 / 障碍 | （如 1 m 视距 / 穿 1 堵墙） |
| SoftAP 直连距离 | |
| PC 网卡 / 浏览器 | |
| 供电方式 | （UART 口 / Power 口补电） |
| 光学闭环偏差 | （对准毫秒计时器，画面读数 − 实际读数 = ____ ms，见 README 校验步骤） |

## 主表（与 CSV 字段一致）

```csv
timestamp,sensor,resolution,quality,wifi_mode,bandwidth_mhz,rssi,capture_fps,encode_fps,arrival_fps,jpeg_avg_bytes,bitrate_mbps,latency_mean_ms,latency_p95_ms,latency_max_ms,latency_std_ms,encode_ms,cpu0_pct,cpu1_pct,free_heap,free_psram,drop_frames
```

| timestamp | sensor | resolution | quality | wifi_mode | MHz | RSSI | 采集fps | 编码fps | 到达fps | kB/帧 | Mbps | 时延mean | P95 | max | σ | 编码ms | CPU0/1% | 丢帧 |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
|  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |

## 结论区（测完填写）

> 参考：2026-10-07 首轮真机实测（OV3660 / 640x480 / STA 11ax / LAN）已录入
> README「已实测参考数据」：采集/编码 11.1fps（q10~40 恒定）、编码段 26.2~27.2ms、
> 端到端（时钟同步法）mean 36.0ms / P95 55.9ms、UDP 42.7ms、浏览器面板 41~66ms。
> 本轮实测验证了全链路功能；请在你的目标场景（距离/遮挡/多车）下按本模板复测。

- DonkeyCar 推荐档（160×120~320×240）实测：
  - 160×120 q__ STA：端到端时延 mean __ ms / P95 __ ms，帧率 __ fps
  - 320×240 q__ STA：端到端时延 mean __ ms / P95 __ ms，帧率 __ fps
- 多客户端并发（2 台 / 3 台）到达帧率衰减：
- STA vs SoftAP 时延差：
- HTTP vs WebSocket vs UDP 时延差：
- 编码耗时随分辨率的变化：
- 已知异常 / 丢帧记录：

## 指标口径（防止误读）

| 字段 | 精确度 | 说明 |
|---|---|---|
| capture_fps / encode_fps / encode_ms / 码率 / CPU / 内存 | 精确实测 | 设备侧 esp_timer + FreeRTOS runtime stats |
| arrival_fps | 精确实测 | 浏览器侧 |
| latency_*（端到端时延） | **含估算** | 时钟同步法：offset 由 min-RTT 3 次取中位，残差一般 ±1~5 ms（同网段），务必先做光学闭环校准记录系统偏差 |
| 网络+浏览器传输段 = 端到端 − 编码 | **估算** | 唯一含估计成分的项 |
