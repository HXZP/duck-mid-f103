# SPDX-License-Identifier: Apache-2.0

cmake_minimum_required(VERSION 3.20.0)
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(duck_mid)

file(GLOB_RECURSE APP_SOURCES CONFIGURE_DEPENDS
    "src/*.c"
    "src/*.cpp"
)

target_sources(app PRIVATE
    ${APP_SOURCES})

# 将上层模块目录添加到Zephyr的额外模块搜索路径中**
list(APPEND ZEPHYR_EXTRA_MODULES
    ${CMAKE_CURRENT_SOURCE_DIR}/..  
)

# 添加设备树绑定文件的搜索路径
list(APPEND DTS_ROOT ${CMAKE_CURRENT_SOURCE_DIR}/../boards) 
