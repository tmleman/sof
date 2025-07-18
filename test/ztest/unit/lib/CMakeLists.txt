cmake_minimum_required(VERSION 3.20.0)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(test_lib)

set(SOF_ROOT "${PROJECT_SOURCE_DIR}/../../../..")

target_include_directories(app PRIVATE ${SOF_ROOT}/zephyr/include)

target_sources(app PRIVATE test_lib_ztest.c)
