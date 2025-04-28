cmake_minimum_required(VERSION 3.10)
include(FetchContent)

set(RUNNER_TARGET stm32f2)
project(
    ${RUNNER_TARGET}_runtime
    VERSION 0.0.2
    LANGUAGES C ASM)

set(CMAKE_VERBOSE_MAKEFILE ON)
set(CMAKE_C_STANDARD 99)
set(CMAKE_INSTALL_BINDIR bin)
set(CMAKE_INSTALL_LIBDIR lib)
set(CMAKE_INSTALL_SRCDIR src)
set(CMAKE_INSTALL_INCLUDEDIR include/platform)
set(LINKER_SCRIPT ${PROJECT_SOURCE_DIR}/src/stm32f207zg.ld)
set(CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}")

include(GNUInstallDirs)
include(cmake/postbuild)

if (CMAKE_BUILD_TYPE MATCHES "Debug")
    string(APPEND CMAKE_C_FLAGS " -O0")
endif()

# ---- Build libopenCM3 as a dependency ----
FetchContent_Declare(libopencm3
  GIT_REPOSITORY             https://github.com/libopencm3/libopencm3.git
  GIT_TAG                    32a169207775d6c53c536d46b78ecf8eca3fdd18
  TLS_VERIFY                 TRUE
  DOWNLOAD_EXTRACT_TIMESTAMP TRUE
  SOURCE_DIR                 ${CMAKE_BINARY_DIR}/3rd/libopencm3
)
FetchContent_MakeAvailable(libopencm3)
add_custom_target(libopencm3
    COMMAND FP_FLAGS="-mfloat-abi=soft" CFLAGS='-fno-asynchronous-unwind-tables' CC=${CMAKE_C_COMPILER} make V=1 TARGETS='stm32/f2'
    WORKING_DIRECTORY ${CMAKE_BINARY_DIR}/3rd/libopencm3)

# ---- Declare target library ----
add_library(
    ${RUNNER_TARGET} STATIC
    src/handlers.c
    src/init.c
    src/rng.c
    src/printf.c)

target_sources(
    ${RUNNER_TARGET}
    PUBLIC
    FILE_SET HEADERS
    FILES
        ${CMAKE_SOURCE_DIR}/inc/platform/printf.h
        ${CMAKE_SOURCE_DIR}/inc/platform/platform.h
        ${CMAKE_SOURCE_DIR}/inc/platform/utils.h
        ${CMAKE_SOURCE_DIR}/inc/platform/stack_usage.h
    BASE_DIRS
        ${CMAKE_SOURCE_DIR}/inc/platform)

target_compile_definitions(
    ${RUNNER_TARGET} PRIVATE
    ${EXTRA_C_FLAGS})

target_include_directories(
    ${RUNNER_TARGET}
    PUBLIC
        $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/inc>
        $<INSTALL_INTERFACE:include>
    PRIVATE
        $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/3rd/libopencm3/include>
        $<BUILD_INTERFACE:${CMAKE_BINARY_DIR}/3rd/libopencm3/include>)

add_dependencies(
    ${RUNNER_TARGET} libopencm3)

include(GenerateExportHeader)
generate_export_header(
    ${RUNNER_TARGET}
    BASE_NAME ${RUNNER_TARGET}
    EXPORT_FILE_NAME export/${RUNNER_TARGET}/${RUNNER_TARGET}_export.h)

# ---- Hello world application ----
add_executable(
    hello
    app/hello.c)

target_include_directories(
    hello
    PUBLIC
    ${CMAKE_BINARY_DIR}/3rd/libopencm3/include)

target_compile_definitions(hello PRIVATE -DSTM32F2)

target_link_libraries(
    hello PRIVATE
    ${RUNNER_TARGET}
    -L${CMAKE_BINARY_DIR}/3rd/libopencm3/lib -lopencm3_stm32f2)

target_link_options(
    hello PRIVATE
    "-T${LINKER_SCRIPT}" -nostartfiles
    -Wl,--whole-archive $<TARGET_FILE_NAME:${RUNNER_TARGET}> -Wl,--no-whole-archive)

target_postbuild_executable(hello ${CMAKE_INSTALL_BINDIR})

include(cmake/install-rules.cmake)

# ---- CPack ----
string(TOLOWER "${CMAKE_BUILD_TYPE}" CMAKE_BUILD_TYPE_LOW)
set(CPACK_GENERATOR "TGZ")
set(CPACK_PACKAGE_VERSION ${PROJECT_VERSION})
set(CPACK_PACKAGE_FILE_NAME "${RUNNER_TARGET}_${PROJECT_VERSION}_${CMAKE_BUILD_TYPE_LOW}")
include(CPack)
