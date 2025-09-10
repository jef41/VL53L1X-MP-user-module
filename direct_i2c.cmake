# modules/vl53l1x/micropython.cmake
# MicroPython user C module for VL53L1X

set(MICROPY_MY_I2C_SRC
    ${CMAKE_CURRENT_LIST_DIR}/direct_i2c.c

)

add_library(usermod_myi2c INTERFACE)

target_sources(usermod_myi2c INTERFACE ${MICROPY_MY_I2C_SRC})


# Link this module into MicroPython
target_link_libraries(usermod INTERFACE usermod_myi2c)