cmake_minimum_required(VERSION 3.20.0)

find_package(Zephyr)
project(cc1101sniffer)

target_sources(app PRIVATE
        src/main.c
        )

target_include_directories(app PRIVATE ${PROJECT_SOURCE_DIR}/../drivers/cc1101)