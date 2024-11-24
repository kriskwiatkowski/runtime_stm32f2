cmake_minimum_required(VERSION 3.10)
include(FetchContent)

project(stm32l4_runtime LANGUAGES C ASM)
set(MAIN_TARGET stm32l4_runtime)
set(CMAKE_VERBOSE_MAKEFILE ON)
set(CMAKE_C_STANDARD 99)
set(OUTPUT ${CMAKE_BINARY_DIR}/pack)
set(CMAKE_INSTALL_BINDIR ${OUTPUT}/bin)
set(CMAKE_INSTALL_LIBDIR ${OUTPUT}/lib)
set(CMAKE_INSTALL_SRCDIR ${OUTPUT}/src)
set(CMAKE_INSTALL_INCLUDEDIR ${OUTPUT}/include/platform)
set(LINKER_SCRIPT ${PROJECT_SOURCE_DIR}/src/stm32f207zg.ld)
set(CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}")

include(postbuild)

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
    src/rng.c
    src/printf.c)

target_sources(
    ${MAIN_TARGET}
    PUBLIC
    FILE_SET HEADERS
    FILES
        ${CMAKE_SOURCE_DIR}/inc/platform/printf.h
        ${CMAKE_SOURCE_DIR}/inc/platform/platform.h
    BASE_DIRS
        ${CMAKE_SOURCE_DIR}/inc/platform)

target_compile_definitions(
    ${MAIN_TARGET} PRIVATE
    ${EXTRA_C_FLAGS})

target_include_directories(
    ${MAIN_TARGET}
    PRIVATE
        ${CMAKE_BINARY_DIR}/3rd/libopencm3/include
    PUBLIC
        ${CMAKE_SOURCE_DIR}/inc)

add_custom_target(libopencm3
    COMMAND FP_FLAGS="-mfloat-abi=soft" CFLAGS='-fno-asynchronous-unwind-tables' CC=${CMAKE_C_COMPILER} make V=1 TARGETS='stm32/f2'
    WORKING_DIRECTORY ${CMAKE_BINARY_DIR}/3rd/libopencm3)

add_dependencies(${MAIN_TARGET} libopencm3)

add_executable(
    hello
    app/hello.c
    src/handlers.c)
target_include_directories(
        hello PRIVATE ${CMAKE_BINARY_DIR}/3rd/libopencm3/include)
target_compile_definitions(hello PRIVATE -DSTM32F2)
target_link_libraries(
    hello
    ${MAIN_TARGET}
    -L${CMAKE_BINARY_DIR}/3rd/libopencm3/lib -lopencm3_stm32f2)
target_link_options(
    hello PRIVATE
    "-T${LINKER_SCRIPT}" -nostartfiles)
target_postbuild_executable(hello ${CMAKE_INSTALL_BINDIR})

install(
    TARGETS ${MAIN_TARGET}
    ARCHIVE
    PUBLIC_HEADER
    FILE_SET HEADERS)

install(
    FILES
        ${CMAKE_CURRENT_BINARY_DIR}/hello.bin
    DESTINATION ${CMAKE_INSTALL_BINDIR})

install(
    FILES
        ${CMAKE_BINARY_DIR}/3rd/libopencm3/lib/libopencm3_stm32f2.a
        ${CMAKE_SOURCE_DIR}/src/stm32f207zg.ld
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
