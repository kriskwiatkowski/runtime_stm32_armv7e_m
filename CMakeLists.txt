cmake_minimum_required(VERSION 3.10)
include(FetchContent)

project(stm32f4_runtime LANGUAGES C ASM)
set(LINKER_SCRIPT ${PROJECT_SOURCE_DIR}/src/stm32f405x6.ld)
set(MAIN_TARGET stm32f4_runtime)
set(CMAKE_VERBOSE_MAKEFILE ON)
set(CMAKE_C_STANDARD 99)
set(OUTPUT ${CMAKE_BINARY_DIR}/pack)
set(CMAKE_INSTALL_LIBDIR ${OUTPUT}/lib)
set(CMAKE_INSTALL_SRCDIR ${OUTPUT}/src)
set(CMAKE_INSTALL_INCLUDEDIR ${OUTPUT}/include/platform)

if (CMAKE_BUILD_TYPE MATCHES "Debug")
    string(APPEND CMAKE_C_FLAGS " -O0")
endif()

FetchContent_Declare(libopencm3
  GIT_REPOSITORY             https://github.com/libopencm3/libopencm3.git
  GIT_TAG                    32a169207775d6c53c536d46b78ecf8eca3fdd18
  TLS_VERIFY                 TRUE
  DOWNLOAD_EXTRACT_TIMESTAMP TRUE
  SOURCE_DIR                 ${CMAKE_BINARY_DIR}/3rd/libopencm3
)
FetchContent_MakeAvailable(libopencm3)

add_library(
    ${MAIN_TARGET} STATIC
    src/init.c
    src/rng-stm32f4.c
    src/printf.c)

target_sources(
    ${MAIN_TARGET}
    PUBLIC
    FILE_SET HEADERS
    FILES
        ${CMAKE_SOURCE_DIR}/inc/platform/printf.h
        ${CMAKE_SOURCE_DIR}/inc/platform/stm32f4.h
    BASE_DIRS
        ${CMAKE_SOURCE_DIR}/inc/platform)

target_compile_definitions(
    ${MAIN_TARGET} PRIVATE
    -DSTM32F4)

target_include_directories(
    ${MAIN_TARGET}
    PRIVATE
        ${CMAKE_BINARY_DIR}/3rd/libopencm3/include
    PUBLIC
        ${CMAKE_SOURCE_DIR}/inc)

add_custom_target(libopencm3
    COMMAND FP_FLAGS="-mfloat-abi=soft"
    CFLAGS='-fno-asynchronous-unwind-tables' CC=${CMAKE_C_COMPILER} make V=1 TARGETS='stm32/f4'
    WORKING_DIRECTORY ${CMAKE_BINARY_DIR}/3rd/libopencm3)

add_dependencies(${MAIN_TARGET} libopencm3)

install(
    TARGETS ${MAIN_TARGET}
    ARCHIVE
    PUBLIC_HEADER
    FILE_SET HEADERS)

install(
    FILES
        ${CMAKE_BINARY_DIR}/3rd/libopencm3/lib/libopencm3_stm32f4.a
        ${CMAKE_SOURCE_DIR}/src/stm32f405x6.ld
    DESTINATION
        ${CMAKE_INSTALL_LIBDIR})

install(
    FILES
        ${CMAKE_SOURCE_DIR}/src/handlers.c
    DESTINATION
        ${CMAKE_INSTALL_SRCDIR})

install(
    DIRECTORY
        ${CMAKE_BINARY_DIR}/3rd/libopencm3/include/libopencm3
        ${CMAKE_BINARY_DIR}/3rd/libopencm3/include/libopencmsis
    DESTINATION
        ${CMAKE_INSTALL_INCLUDEDIR})
