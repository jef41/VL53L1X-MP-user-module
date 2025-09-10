#include "py/runtime.h"
#include "py/obj.h"
#include "py/mphal.h"
#include "extmod/modmachine.h"

// Object structure
typedef struct _myi2c_obj_t {
    mp_obj_base_t base;
    //mp_machine_i2c_obj_t *i2c_obj;  // Direct C-level I2C object
    mp_obj_t i2c_obj;  // keep Python object
    uint8_t addr;                    // device address
} myi2c_obj_t;

// Constructor: MyI2C(i2c, addr)
static mp_obj_t myi2c_make_new(const mp_obj_type_t *type,
                               size_t n_args, size_t n_kw,
                               const mp_obj_t *args) {
    mp_arg_check_num(n_args, n_kw, 2, 2, false);

    myi2c_obj_t *self = mp_obj_malloc(myi2c_obj_t, type);

    // Convert Python I2C object to C-level mp_machine_i2c_obj_t
    mp_obj_t i2c_in = args[0];
    if (!mp_obj_is_type(i2c_in, &machine_i2c_type)) {
        mp_raise_TypeError(MP_ERROR_TEXT("expected I2C object"));
    }
    self->i2c_obj = MP_OBJ_TO_PTR(i2c_in);

    self->addr = mp_obj_get_int(args[1]);
    return MP_OBJ_FROM_PTR(self);
}

// Write(bytes)
static mp_obj_t myi2c_write(mp_obj_t self_in, mp_obj_t buf_in) {
    myi2c_obj_t *self = MP_OBJ_TO_PTR(self_in);

    mp_buffer_info_t bufinfo;
    mp_get_buffer_raise(buf_in, &bufinfo, MP_BUFFER_READ);

    mp_machine_i2c_buf_t i2c_buf;
    i2c_buf.buf = bufinfo.buf;
    i2c_buf.len = bufinfo.len;
    //i2c_buf.flags = MP_MACHINE_I2C_FLAG_STOP;

    int ret = mp_machine_i2c_transfer_adaptor(
        //(mp_machine_i2c_obj_t*)self->i2c_obj, 
        self->i2c_obj,       // cast is optional since it's void*
        self->addr, 
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
static MP_DEFINE_CONST_FUN_OBJ_2(myi2c_write_obj, myi2c_write);

// Read(n)
static mp_obj_t myi2c_read(mp_obj_t self_in, mp_obj_t n_in) {
    myi2c_obj_t *self = MP_OBJ_TO_PTR(self_in);
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
        self->addr, 
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
static MP_DEFINE_CONST_FUN_OBJ_2(myi2c_read_obj, myi2c_read);

// Write then Read (repeated start)
static mp_obj_t myi2c_writeread(mp_obj_t self_in, mp_obj_t wbuf_in, mp_obj_t n_in) {
    myi2c_obj_t *self = MP_OBJ_TO_PTR(self_in);
    mp_int_t n = mp_obj_get_int(n_in);

    // --- Write part ---
    mp_buffer_info_t wbufinfo;
    mp_get_buffer_raise(wbuf_in, &wbufinfo, MP_BUFFER_READ);
    mp_machine_i2c_buf_t wbuf;
    wbuf.buf = wbufinfo.buf;
    wbuf.len = wbufinfo.len;
    int ret = mp_machine_i2c_transfer_adaptor(self->i2c_obj, self->addr, 1, &wbuf,
                                              MP_MACHINE_I2C_FLAG_STOP);
    if (ret < 0) {
        mp_raise_msg_varg(&mp_type_OSError, MP_ERROR_TEXT("I2C write failed, code %d"), ret);
    }

    // --- Read part ---
    vstr_t rbuf;
    vstr_init_len(&rbuf, n);
    mp_machine_i2c_buf_t rbuf_struct;
    rbuf_struct.buf = (uint8_t*)rbuf.buf;
    rbuf_struct.len = n;
    ret = mp_machine_i2c_transfer_adaptor(self->i2c_obj, self->addr, 1, &rbuf_struct,
                                          MP_MACHINE_I2C_FLAG_READ | MP_MACHINE_I2C_FLAG_STOP);
    if (ret < 0) {
        vstr_clear(&rbuf);
        mp_raise_msg_varg(&mp_type_OSError, MP_ERROR_TEXT("I2C read failed, code %d"), ret);
    }

    return mp_obj_new_bytes((const byte *)rbuf.buf, rbuf.len);
}
static MP_DEFINE_CONST_FUN_OBJ_3(myi2c_writeread_obj, myi2c_writeread);


// Method table
static const mp_rom_map_elem_t myi2c_locals_dict_table[] = {
    { MP_ROM_QSTR(MP_QSTR_write), MP_ROM_PTR(&myi2c_write_obj) },
    { MP_ROM_QSTR(MP_QSTR_read),  MP_ROM_PTR(&myi2c_read_obj)  },
    { MP_ROM_QSTR(MP_QSTR_writeread), MP_ROM_PTR(&myi2c_writeread_obj) },
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
