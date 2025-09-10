// going through the Python object layer, slower, but works 
/*
from machine import I2C, Pin
import myi2c

i2c = I2C(0, scl=Pin(17), sda=Pin(16), freq=100_000)
dev = myi2c.MyI2C(i2c, 0x29)

# Write bytes
dev.write(b'\x01\x0f')

# Read 4 bytes
print(dev.read(2))
*/
#include "py/runtime.h"
#include "py/obj.h"
#include "py/mphal.h"
#include "extmod/modmachine.h"

// Object structure
typedef struct _myi2c_obj_t {
    mp_obj_base_t base;
    mp_obj_t i2c_obj;  // Python I2C instance
    uint8_t addr;      // device address
} myi2c_obj_t;

// Constructor: MyI2C(i2c, addr)
static mp_obj_t myi2c_make_new(const mp_obj_type_t *type,
                               size_t n_args, size_t n_kw,
                               const mp_obj_t *args) {
    mp_arg_check_num(n_args, n_kw, 2, 2, false);

    myi2c_obj_t *self = mp_obj_malloc(myi2c_obj_t, type);
    self->i2c_obj = args[0];
    self->addr = mp_obj_get_int(args[1]);
    return MP_OBJ_FROM_PTR(self);
}

// Write(bytes)
static mp_obj_t myi2c_write(mp_obj_t self_in, mp_obj_t buf_in) {
    myi2c_obj_t *self = MP_OBJ_TO_PTR(self_in);

    // Load the 'writeto' method from the Python I2C object
    mp_obj_t method;
    mp_load_method_maybe(self->i2c_obj, MP_QSTR_writeto, &method);

    // Prepare arguments: addr, buf
    mp_obj_t args[3];          // self + addr + buf
    args[0] = self->i2c_obj;   // Python I2C instance (self)
    args[1] = mp_obj_new_int(self->addr);
    args[2] = buf_in;

    // Call the method
    return mp_call_function_n_kw(method, 3, 0, args);
}
static MP_DEFINE_CONST_FUN_OBJ_2(myi2c_write_obj, myi2c_write);

// Read(n)
static mp_obj_t myi2c_read(mp_obj_t self_in, mp_obj_t n_in) {
    myi2c_obj_t *self = MP_OBJ_TO_PTR(self_in);
    mp_int_t n = mp_obj_get_int(n_in);

    // Load the 'readfrom' method from the Python I2C object
    mp_obj_t method;
    mp_load_method_maybe(self->i2c_obj, MP_QSTR_readfrom, &method);

    // Prepare arguments: addr, n
    mp_obj_t args[3];
    args[0] = self->i2c_obj;
    args[1] = mp_obj_new_int(self->addr);
    args[2] = mp_obj_new_int(n);

    // Call the method
    return mp_call_function_n_kw(method, 3, 0, args);
}
static MP_DEFINE_CONST_FUN_OBJ_2(myi2c_read_obj, myi2c_read);

// Method table
static const mp_rom_map_elem_t myi2c_locals_dict_table[] = {
    { MP_ROM_QSTR(MP_QSTR_write), MP_ROM_PTR(&myi2c_write_obj) },
    { MP_ROM_QSTR(MP_QSTR_read),  MP_ROM_PTR(&myi2c_read_obj)  },
};
static MP_DEFINE_CONST_DICT(myi2c_locals_dict, myi2c_locals_dict_table);

// Define the type
MP_DEFINE_CONST_OBJ_TYPE(
    myi2c_type,
    MP_QSTR_MyI2C,
    MP_TYPE_FLAG_NONE,
    make_new, myi2c_make_new,
    locals_dict, &myi2c_locals_dict
);

// Module globals
static const mp_rom_map_elem_t myi2c_module_globals_table[] = {
    { MP_ROM_QSTR(MP_QSTR___name__), MP_ROM_QSTR(MP_QSTR_myi2c) },
    { MP_ROM_QSTR(MP_QSTR_MyI2C), MP_ROM_PTR(&myi2c_type) },
};
static MP_DEFINE_CONST_DICT(myi2c_module_globals, myi2c_module_globals_table);

// Module definition
const mp_obj_module_t myi2c_user_cmodule = {
    .base = { &mp_type_module },
    .globals = (mp_obj_dict_t*)&myi2c_module_globals,
};

// Register module
MP_REGISTER_MODULE(MP_QSTR_myi2c, myi2c_user_cmodule);
