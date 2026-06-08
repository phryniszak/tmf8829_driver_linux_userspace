# ************************************************************************************
# Copyright (c) [2025] ams-OSRAM AG
#
# SPDX-License-Identifier: GPL-2.0 OR MIT
#
# For full license texts, see LICENSES-GPL-2.0.txt or LICENSES-MIT.TXT.
# ************************************************************************************

cmake_minimum_required(VERSION 3.10)
project(tmf8829 C)

set(CMAKE_C_STANDARD 99)
set(CMAKE_C_STANDARD_REQUIRED ON)

# Optional features (mirrors the previous Makefile switches)
option(ENABLE_JSON_LOGGING "Enable JSON logging" ON)
option(ENABLE_HISTOGRAM "Enable histogram parsing and storage (saves ~600KB RAM if disabled)" ON)
option(ENABLE_KEYSTONE "Enable keystone angle calculation" ON)

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()
set(CMAKE_C_FLAGS_RELEASE "-O2")

add_compile_options(-Wall -Wextra)

set(SOURCES
    main.c
    tmf8829.c
    tmf8829_driver.c
    tmf8829_frameparser.c
    tmf8829_shim.c
)

set(LINK_LIBS m pthread)

if(ENABLE_JSON_LOGGING)
    list(APPEND SOURCES tmf8829_json.c)
    list(APPEND LINK_LIBS z)
endif()

if(ENABLE_KEYSTONE)
    list(APPEND SOURCES tmf8829_keystone.c)
endif()

add_executable(tmf8829 ${SOURCES})

target_compile_definitions(tmf8829 PRIVATE
    $<$<BOOL:${ENABLE_JSON_LOGGING}>:ENABLE_JSON_LOGGING>
    $<$<BOOL:${ENABLE_HISTOGRAM}>:ENABLE_HISTOGRAM>
    $<$<BOOL:${ENABLE_KEYSTONE}>:ENABLE_KEYSTONE>
)

target_link_libraries(tmf8829 PRIVATE ${LINK_LIBS})

include(GNUInstallDirs)
install(TARGETS tmf8829 RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR})
