# CamTest — ESP32-S31-Korvo-1 Wi-Fi MJPEG 图传性能闭环测试系统
#
# 验证环境（请勿随意改动后再投稿性能数据）：
#   ESP-IDF: release/v6.1 分支头, commit 0d9287800812c95662921c2c5e812023939e3d58
#            （S31 支持为 preview；本文档锁定的分支在 2026-10 验证通过。
#              用户如使用正式 tag v6.1.1，请先重新编译并确认无 Kconfig/编译警告。）
#   目标芯片: esp32s31
#
# 编译：idf.py set-target esp32s31 && idf.py build
cmake_minimum_required(VERSION 3.16)
include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(camtest)
