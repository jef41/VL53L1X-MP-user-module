// modules/vl53l1x/vl53l1x_driver.h
#pragma once
#include <stdint.h>
#include <stddef.h>

#include "vl53l1_platform.h"  // for VL53L1_Dev_t, VL53L1_Error
#include "py/obj.h"      // for mp_obj_t


// Initialize the VL53L1X driver
VL53L1_Error vl53l1x_driver_init(VL53L1_Dev_t *dev, mp_obj_t i2c_obj, uint8_t addr);
