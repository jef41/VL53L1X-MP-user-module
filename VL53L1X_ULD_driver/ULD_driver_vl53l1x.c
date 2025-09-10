// ===== file: mp_driver_vl53l1x.c =====
// MicroPython wrapper for VL53L1X ULD (STSW-IMG009 v3.5.4)

#include "py/runtime.h"
#include "py/obj.h"
#include "py/mphal.h"
#include "extmod/modmachine.h"

#include "VL53L1X_api.h"
#include "vl53l1_platform.h"


// forward decl from platform glue
void vl53l1_platform_bind_i2c(mp_obj_t i2c_obj);

// Object structure
typedef struct _vl53l1x_obj_t {
    mp_obj_base_t base;
    mp_obj_t i2c_obj;  // keep Python object
    uint8_t i2c_addr;      // device address
} vl53l1x_obj_t;


static VL53L1_Error vl53l1x_driver_init(vl53l1x_obj_t *self) {
    vl53l1_platform_bind_i2c(self->i2c_obj);


    return VL53L1_ERROR_NONE;
}

// Constructor: MyI2C(i2c, addr)
static mp_obj_t vl53l1x_make_new(const mp_obj_type_t *type,
                               size_t n_args, size_t n_kw,
                               const mp_obj_t *args) {
    mp_arg_check_num(n_args, n_kw, 2, 2, false);

    vl53l1x_obj_t *self = mp_obj_malloc(vl53l1x_obj_t, type);

    // Convert Python I2C object to C-level mp_machine_i2c_obj_t
    mp_obj_t i2c_in = args[0];
    if (!mp_obj_is_type(i2c_in, &machine_i2c_type)) {
        mp_raise_TypeError(MP_ERROR_TEXT("expected I2C object"));
    }
    self->i2c_obj = MP_OBJ_TO_PTR(i2c_in);

    self->i2c_addr = mp_obj_get_int(args[1]);

    VL53L1_Error err = vl53l1x_driver_init(self);
    if (err != VL53L1_ERROR_NONE) {
        mp_raise_msg(&mp_type_OSError, MP_ERROR_TEXT("VL53L1X init failed"));
    }
    return MP_OBJ_FROM_PTR(self);
}

// Write(bytes)
static mp_obj_t vl53l1x_write(mp_obj_t self_in, mp_obj_t buf_in) {
    vl53l1x_obj_t *self = MP_OBJ_TO_PTR(self_in);

    mp_buffer_info_t bufinfo;
    mp_get_buffer_raise(buf_in, &bufinfo, MP_BUFFER_READ);

    mp_machine_i2c_buf_t i2c_buf;
    i2c_buf.buf = bufinfo.buf;
    i2c_buf.len = bufinfo.len;
    //i2c_buf.flags = MP_MACHINE_I2C_FLAG_STOP;

    int ret = mp_machine_i2c_transfer_adaptor(
        //(mp_machine_i2c_obj_t*)self->i2c_obj, 
        self->i2c_obj,       // cast is optional since it's void*
        self->i2c_addr, 
        1, 
        &i2c_buf, 
        MP_MACHINE_I2C_FLAG_STOP
    );

    if (ret < 0) {
        mp_raise_msg_varg(&mp_type_OSError, MP_ERROR_TEXT("I2C write failed, code %d"), ret);
    }

    //return mp_obj_new_int(ret);  // return number of buffers successfully transferred
    // Success: return number of bytes written
    return mp_obj_new_int(bufinfo.len);
}
static MP_DEFINE_CONST_FUN_OBJ_2(vl53l1x_write_obj, vl53l1x_write);

// Read(n)
static mp_obj_t vl53l1x_read(mp_obj_t self_in, mp_obj_t n_in) {
    vl53l1x_obj_t *self = MP_OBJ_TO_PTR(self_in);
    mp_int_t n = mp_obj_get_int(n_in);

    // Allocate a buffer for reading
    vstr_t vstr;
    vstr_init_len(&vstr, n);

    mp_machine_i2c_buf_t i2c_buf;
    i2c_buf.buf = (uint8_t*)vstr.buf;
    i2c_buf.len = n;
    //i2c_buf.flags = MP_MACHINE_I2C_FLAG_STOP | MP_MACHINE_I2C_FLAG_READ;

    int ret = mp_machine_i2c_transfer_adaptor(
        //(mp_machine_i2c_obj_t*)self->i2c_obj, 
        self->i2c_obj,       // cast is optional since it's void*
        self->i2c_addr, 
        1, 
        &i2c_buf, 
        MP_MACHINE_I2C_FLAG_STOP | MP_MACHINE_I2C_FLAG_READ
    );

    if (ret < 0) {
        vstr_clear(&vstr);
        mp_raise_msg_varg(&mp_type_OSError, MP_ERROR_TEXT("I2C read failed, code %d"), ret);
    }

    // Return the bytes object directly
    return mp_obj_new_bytes((const byte *)vstr.buf, vstr.len);
}
static MP_DEFINE_CONST_FUN_OBJ_2(vl53l1x_read_obj, vl53l1x_read);



// Methods
// 8a. GetSensorID
static mp_obj_t vl53l1x_get_sensor_id(mp_obj_t self_in) {
    vl53l1x_obj_t *self = MP_OBJ_TO_PTR(self_in);
    uint16_t sensorid=0;
    VL53L1X_GetSensorId(self->i2c_addr, &sensorid);
    return mp_obj_new_int(sensorid);
}
static MP_DEFINE_CONST_FUN_OBJ_1(vl53l1x_get_sensor_id_obj, vl53l1x_get_sensor_id);


// Method table
static const mp_rom_map_elem_t vl53l1x_locals_dict_table[] = {
    { MP_ROM_QSTR(MP_QSTR_write), MP_ROM_PTR(&vl53l1x_write_obj) },
    { MP_ROM_QSTR(MP_QSTR_read),  MP_ROM_PTR(&vl53l1x_read_obj)  },
    { MP_ROM_QSTR(MP_QSTR_get_sensor_id), MP_ROM_PTR(&vl53l1x_get_sensor_id_obj) },
};
static MP_DEFINE_CONST_DICT(vl53l1x_locals_dict, vl53l1x_locals_dict_table);

// Define the type
MP_DEFINE_CONST_OBJ_TYPE(
    vl53l1x_type,
    MP_QSTR_VL53L1X,
    MP_TYPE_FLAG_NONE,
    make_new, vl53l1x_make_new,
    locals_dict, &vl53l1x_locals_dict
);

// Module globals
static const mp_rom_map_elem_t vl53l1x_module_globals_table[] = {
    { MP_ROM_QSTR(MP_QSTR___name__), MP_ROM_QSTR(MP_QSTR_vl53l1x) },
    { MP_ROM_QSTR(MP_QSTR_VL53L1X), MP_ROM_PTR(&vl53l1x_type) },
};
static MP_DEFINE_CONST_DICT(vl53l1x_module_globals, vl53l1x_module_globals_table);

// Module definition
const mp_obj_module_t vl53l1x_user_cmodule = {
    .base = { &mp_type_module },
    .globals = (mp_obj_dict_t*)&vl53l1x_module_globals,
};

// Register module
MP_REGISTER_MODULE(MP_QSTR_vl53l1x, vl53l1x_user_cmodule);
