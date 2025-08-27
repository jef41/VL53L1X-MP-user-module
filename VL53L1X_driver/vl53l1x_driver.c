// modules/vl53l1x/vl53l1x_driver.c
#include "vl53l1x_driver.h"
#include "vl53l1_api.h" // Provided by ST 
#include "py/obj.h"
#include "py/runtime.h"
#include "py/misc.h"

// Our private wrapper, defined to hold a MicroPython I2C object + address
typedef struct {
    mp_obj_t i2c_obj;   // machine.I2C object from Python
    uint8_t  i2c_addr;  // 7-bit I2C address (default 0x29)
} vl53l1x_dev_t;

// Helper macros for cleaner code
#define VL53L1X_ADDR(dev) (((vl53l1x_dev_t *)(dev)->platform_private)->i2c_addr)
#define VL53L1X_I2C(dev)  (((vl53l1x_dev_t *)(dev)->platform_private)->i2c_obj)

// Example init wrapper
VL53L1_Error vl53l1x_driver_init(VL53L1_Dev_t *dev, mp_obj_t i2c_obj, uint8_t addr) {
    // Allocate platform-private struct
    vl53l1x_dev_t *pdev = m_new_obj(vl53l1x_dev_t);
    pdev->i2c_obj = i2c_obj;
    pdev->i2c_addr = addr;

    dev->platform_private = pdev;

    // Call ST API init function
    return VL53L1_software_init(dev);
}

