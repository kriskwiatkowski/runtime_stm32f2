# ---- Install runtime lib ----
set(EXPORT_TARGET ${RUNNER_TARGET}_target)
include(CMakePackageConfigHelpers)
write_basic_package_version_file(
    "${CMAKE_CURRENT_BINARY_DIR}/${RUNNER_TARGET}ConfigVersion.cmake"
    COMPATIBILITY SameMajorVersion)

install(
    TARGETS ${RUNNER_TARGET}
    ARCHIVE
    PUBLIC_HEADER
    FILE_SET HEADERS)

install(
    FILES ${LINKER_SCRIPT}
    DESTINATION ${CMAKE_INSTALL_LIBDIR})

install(
    FILES
        ${CMAKE_BINARY_DIR}/3rd/libopencm3/lib/libopencm3_stm32f2.a
        ${CMAKE_SOURCE_DIR}/src/stm32f207zg.ld
    DESTINATION
        ${CMAKE_INSTALL_LIBDIR})

install(
    TARGETS ${RUNNER_TARGET}
    EXPORT ${EXPORT_TARGET}
    ARCHIVE
    PUBLIC_HEADER
    FILE_SET HEADERS)

install(
    EXPORT ${EXPORT_TARGET}
    NAMESPACE ${RUNNER_TARGET}::
    DESTINATION "cmake")

# ---- Install sample app ----
install(
    FILES
        ${CMAKE_CURRENT_BINARY_DIR}/hello.bin
    DESTINATION ${CMAKE_INSTALL_BINDIR})
