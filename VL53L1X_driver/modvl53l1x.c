// modules/vl53l1x/modvl53l1x.c
#include "py/obj.h"
#include "py/runtime.h"
#include "py/mperrno.h"
#include "py/mphal.h"

//#include "py/obj.h"
//#include "py/runtime.h"
//#include "py/mperrno.h"

//#include "py/misc.h"
#include "vl53l1_api.h"
#include "vl53l1_platform.h"

// Platform-private struct comes from vl53l1_platform.c
typedef struct {
    mp_obj_t i2c_obj;   // machine.I2C instance
    uint8_t  i2c_addr;  // 7-bit address
} vl53l1x_dev_t;

// Wrapper object exposed to Python
typedef struct _mp_obj_vl53l1x_t {
    mp_obj_base_t base;
    VL53L1_Dev_t dev;         // ST driver struct
    bool initialised;
} mp_obj_vl53l1x_t;

// Constructor: VL53L1X(i2c, addr=0x29)
static mp_obj_t vl53l1x_make_new(const mp_obj_type_t *type,
                                 size_t n_args, size_t n_kw,
                                 const mp_obj_t *all_args) {
    enum { ARG_i2c, ARG_addr };
    static const mp_arg_t allowed_args[] = {
        { MP_QSTR_i2c,  MP_ARG_REQUIRED | MP_ARG_OBJ, {.u_obj = MP_OBJ_NULL} },
        { MP_QSTR_addr, MP_ARG_INT, {.u_int = 0x29} },
    };
    mp_arg_val_t args[MP_ARRAY_SIZE(allowed_args)];
    mp_arg_parse_all(n_args, n_kw, all_args,
                     MP_ARRAY_SIZE(allowed_args), allowed_args, args);

    // Allocate Python-visible object
    mp_obj_vl53l1x_t *self = m_new_obj(mp_obj_vl53l1x_t);
    self->base.type = type;
    self->initialised = false;

    // Allocate platform-private struct and attach to ST device
    vl53l1x_dev_t *pdev = m_new_obj(vl53l1x_dev_t);
    pdev->i2c_obj = args[ARG_i2c].u_obj;
    pdev->i2c_addr = (uint8_t)args[ARG_addr].u_int;

    memset(&self->dev, 0, sizeof(VL53L1_Dev_t));
    self->dev.platform_private = pdev;

    return MP_OBJ_FROM_PTR(self);
}


// init()
static mp_obj_t vl53l1x_init(mp_obj_t self_in) {
    mp_obj_vl53l1x_t *self = MP_OBJ_TO_PTR(self_in);
    if (!self->initialised) {
        int status = vl53l1x_init(&self->dev, NULL, NULL, NULL);
        if (status != 0) {
            mp_raise_msg_varg(&mp_type_RuntimeError,
                              MP_ERROR_TEXT("VL53L1X init failed: %d"),
                              status);
        }
        self->initialised = true;
    }
    return mp_const_none;
}
static MP_DEFINE_CONST_FUN_OBJ_1(vl53l1x_init_obj, vl53l1x_init);

// deinit()
static mp_obj_t vl53l1x_deinit(mp_obj_t self_in) {
    mp_obj_vl53l1x_t *self = MP_OBJ_TO_PTR(self_in);
    if (self->initialised) {
        vl53l1x_deinit(&self->dev);
        self->initialised = false;
    }
    return mp_const_none;
}
static MP_DEFINE_CONST_FUN_OBJ_1(vl53l1x_deinit_obj, vl53l1x_deinit);

// start_ranging()
static mp_obj_t vl53l1x_start_ranging(mp_obj_t self_in, mp_obj_t mode_in) {
    mp_obj_vl53l1x_t *self = MP_OBJ_TO_PTR(self_in);
    int mode = mp_obj_get_int(mode_in);
    int status = vl53l1x_start_ranging(&self->dev, mode);
    if (status != 0) {
        mp_raise_msg_varg(&mp_type_RuntimeError,
                          MP_ERROR_TEXT("start_ranging failed: %d"), status);
    }
    return mp_const_none;
}
static MP_DEFINE_CONST_FUN_OBJ_2(vl53l1x_start_ranging_obj, vl53l1x_start_ranging);

// stop_ranging()
static mp_obj_t vl53l1x_stop_ranging(mp_obj_t self_in) {
    mp_obj_vl53l1x_t *self = MP_OBJ_TO_PTR(self_in);
    vl53l1x_stop_ranging(&self->dev);
    return mp_const_none;
}
static MP_DEFINE_CONST_FUN_OBJ_1(vl53l1x_stop_ranging_obj, vl53l1x_stop_ranging);

// read_mm()
static mp_obj_t vl53l1x_read_mm(mp_obj_t self_in) {
    mp_obj_vl53l1x_t *self = MP_OBJ_TO_PTR(self_in);
    uint16_t mm = 0;
    int status = vl53l1x_get_distance_mm(&self->dev, &mm);
    if (status != 0) {
        mp_raise_msg(&mp_type_RuntimeError, MP_ERROR_TEXT("read failed"));
    }
    return mp_obj_new_int(mm);
}
static MP_DEFINE_CONST_FUN_OBJ_1(vl53l1x_read_mm_obj, vl53l1x_read_mm);

// ---- Example config API: set_timing_budget ----
static mp_obj_t vl53l1x_set_timing_budget(mp_obj_t self_in, mp_obj_t us_in) {
    mp_obj_vl53l1x_t *self = MP_OBJ_TO_PTR(self_in);
    int budget = mp_obj_get_int(us_in);
    int status = VL53L1X_SetTimingBudgetInMs(&self->dev, budget);
    if (status != 0) {
        mp_raise_msg_varg(&mp_type_RuntimeError,
                          MP_ERROR_TEXT("set_timing_budget failed: %d"),
                          status);
    }
    return mp_const_none;
}
static MP_DEFINE_CONST_FUN_OBJ_2(vl53l1x_set_timing_budget_obj, vl53l1x_set_timing_budget);

// ---- Example: get_timing_budget ----
static mp_obj_t vl53l1x_get_timing_budget(mp_obj_t self_in) {
    mp_obj_vl53l1x_t *self = MP_OBJ_TO_PTR(self_in);
    uint16_t budget = 0;
    VL53L1X_GetTimingBudgetInMs(&self->dev, &budget);
    return mp_obj_new_int(budget);
}
static MP_DEFINE_CONST_FUN_OBJ_1(vl53l1x_get_timing_budget_obj, vl53l1x_get_timing_budget);

// TODO: Add similar wrappers for distance_mode, inter_measurement, ROI, offset, xtalk, etc.

// locals_dict
static const mp_rom_map_elem_t vl53l1x_locals_dict_table[] = {
    { MP_ROM_QSTR(MP_QSTR_init), MP_ROM_PTR(&vl53l1x_init_obj) },
    { MP_ROM_QSTR(MP_QSTR_deinit), MP_ROM_PTR(&vl53l1x_deinit_obj) },
    { MP_ROM_QSTR(MP_QSTR_start_ranging), MP_ROM_PTR(&vl53l1x_start_ranging_obj) },
    { MP_ROM_QSTR(MP_QSTR_stop_ranging), MP_ROM_PTR(&vl53l1x_stop_ranging_obj) },
    { MP_ROM_QSTR(MP_QSTR_read_mm), MP_ROM_PTR(&vl53l1x_read_mm_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_timing_budget), MP_ROM_PTR(&vl53l1x_set_timing_budget_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_timing_budget), MP_ROM_PTR(&vl53l1x_get_timing_budget_obj) },
    // add other bindings here...
};
static MP_DEFINE_CONST_DICT(vl53l1x_locals_dict, vl53l1x_locals_dict_table);

static const mp_obj_type_t vl53l1x_type = {
    { &mp_type_type },
    .name = MP_QSTR_VL53L1X,
    .make_new = vl53l1x_make_new,
    .locals_dict = (mp_obj_dict_t*)&vl53l1x_locals_dict,
};

static const mp_rom_map_elem_t mp_module_vl53l1x_globals_table[] = {
    { MP_ROM_QSTR(MP_QSTR___name__), MP_ROM_QSTR(MP_QSTR_vl53l1x) },
    { MP_ROM_QSTR(MP_QSTR_VL53L1X), MP_ROM_PTR(&vl53l1x_type) },
};
static MP_DEFINE_CONST_DICT(mp_module_vl53l1x_globals, mp_module_vl53l1x_globals_table);

const mp_obj_module_t mp_module_vl53l1x = {
    .base = { &mp_type_module },
    .globals = (mp_obj_dict_t*)&mp_module_vl53l1x_globals,
};

MP_REGISTER_MODULE(MP_QSTR_vl53l1x, mp_module_vl53l1x);

#include "py/obj.h"
#include "py/runtime.h"
#include "vl53l1x_driver.h"

typedef struct {
    mp_obj_base_t base;
    machine_i2c_obj_t *i2c;
    uint8_t address;
    VL53L1X_Dev_t dev;  // hypothetical driver struct
} mp_obj_vl53l1x_t;

static mp_obj_t vl53l1x_make_new(...) {
    // Parse args: i2c, address
    // Allocate and initialize mp_obj_vl53l1x_t
}

static mp_obj_t vl53l1x_init(mp_obj_t self_in) {
    // Call driver's init with proper I2C glue
}

static mp_obj_t vl53l1x_start_ranging(mp_obj_t self_in, mp_obj_t mode_in) { ... }
static mp_obj_t vl53l1x_read_mm(mp_obj_t self_in) { ... }
static mp_obj_t vl53l1x_stop_ranging(mp_obj_t self_in) { ... }
static mp_obj_t vl53l1x_deinit(mp_obj_t self_in) { ... }

// Define method table, type object, etc.
