# The following lines of boilerplate have to be in your project's CMakeLists
# in this exact order for cmake to work correctly
cmake_minimum_required(VERSION 3.16)

include($ENV{IDF_PATH}/tools/cmake/project.cmake)

#
# 产品固件版本必须与 BLE GET_INFO 返回值一致。阶段开发期间工作树通常位于上一阶段
# Git 标签之后；若完全依赖 git describe，ESP-IDF 启动日志会把 0.9.0 错报成旧阶段
# 标签。这里显式固定本阶段版本，下一阶段升级时应与 exo_ble.c 一起修改。
#
set(PROJECT_VER "0.11.23")
project(twai_network_master)
