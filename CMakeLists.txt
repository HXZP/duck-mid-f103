# SPDX-License-Identifier: Apache-2.0

cmake_minimum_required(VERSION 3.20.0)

list(APPEND ZEPHYR_EXTRA_MODULES
    ${CMAKE_CURRENT_SOURCE_DIR}/../../zephyr-hxzp
)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(duck_mid)

file(GLOB_RECURSE APP_SOURCES CONFIGURE_DEPENDS
    "src/*.c"
    "src/*.cpp"
)

list(FILTER APP_SOURCES EXCLUDE REGEX ".*/src/can_protocol\\.c$")
target_sources(app PRIVATE
    ${APP_SOURCES})
