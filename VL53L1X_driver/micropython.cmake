# modules/vl53l1x/micropython.cmake
# MicroPython user C module for VL53L1X

set(MICROPY_VL53L1X_SRC
    ${CMAKE_CURRENT_LIST_DIR}/modvl53l1x.c
    ${CMAKE_CURRENT_LIST_DIR}/vl53l1x_driver.c

    # ST core driver
    ${CMAKE_CURRENT_LIST_DIR}/core/src/vl53l1_api.c
    ${CMAKE_CURRENT_LIST_DIR}/core/src/vl53l1_api_calibration.c
    ${CMAKE_CURRENT_LIST_DIR}/core/src/vl53l1_api_core.c
    ${CMAKE_CURRENT_LIST_DIR}/core/src/vl53l1_api_debug.c
    ${CMAKE_CURRENT_LIST_DIR}/core/src/vl53l1_api_preset_modes.c
    ${CMAKE_CURRENT_LIST_DIR}/core/src/vl53l1_api_strings.c
    ${CMAKE_CURRENT_LIST_DIR}/core/src/vl53l1_core.c
    ${CMAKE_CURRENT_LIST_DIR}/core/src/vl53l1_core_support.c
    ${CMAKE_CURRENT_LIST_DIR}/core/src/vl53l1_error_strings.c
    ${CMAKE_CURRENT_LIST_DIR}/core/src/vl53l1_register_funcs.c
    ${CMAKE_CURRENT_LIST_DIR}/core/src/vl53l1_silicon_core.c
    ${CMAKE_CURRENT_LIST_DIR}/core/src/vl53l1_wait.c

    # Platform layer
    #${CMAKE_CURRENT_LIST_DIR}/platform/src/vl53l1_platform.c
    #${CMAKE_CURRENT_LIST_DIR}/platform/src/vl53l1_platform_init.c
    #${CMAKE_CURRENT_LIST_DIR}/platform/src/vl53l1_platform_log.c
)

add_library(usermod_vl53l1x INTERFACE)

target_sources(usermod_vl53l1x INTERFACE ${MICROPY_VL53L1X_SRC})

target_include_directories(usermod_vl53l1x INTERFACE
    ${CMAKE_CURRENT_LIST_DIR}/core/src
    ${CMAKE_CURRENT_LIST_DIR}/core/inc
    ${CMAKE_CURRENT_LIST_DIR}/platform/src
    ${CMAKE_CURRENT_LIST_DIR}/platform/inc
)

# Link this module into MicroPython
target_link_libraries(usermod INTERFACE usermod_vl53l1x)
