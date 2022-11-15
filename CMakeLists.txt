# SPDX-License-Identifier: Apache-2.0

cmake_minimum_required(VERSION 3.20.0)


find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(hello_world)

# add_subdirectory(decadriver)
include_directories(decadriver platform protocol inc ranging baca)

target_sources(app PRIVATE 
    src/main.c
    src/uwb_transport.c
    platform/deca_spi.c
    platform/deca_mutex.c
    platform/deca_sleep.c
    platform/platform.c
    decadriver/deca_device.c
    decadriver/deca_params_init.c
    protocol/data_msg.pb.c
    ranging/ranging.cpp
    protocol/mac.c
    baca/baca.c

)

set_property(TARGET app PROPERTY CXX_STANDARD 20)

# target_link_libraries(app PUBLIC decadriver)

