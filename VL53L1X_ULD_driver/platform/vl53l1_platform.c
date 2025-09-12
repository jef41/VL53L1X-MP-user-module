/* 

*/
// ===== file: vl53l1_platform.c =====
// MicroPython I2C glue for ST VL53L1X ULD (STSW-IMG009)
// Hardware-agnostic: uses MicroPython's machine.I2C protocol from C.

#include <string.h>
#include "py/obj.h"
#include "py/runtime.h"
#include "py/mphal.h"
#include "extmod/modmachine.h" //"extmod/machine_i2c.h"

#include "vl53l1_platform.h"
#include "vl53l1_error_codes.h"

//uint8_t _I2CBuffer[256];

// MicroPython I2C object currently bound to the driver
static mp_obj_base_t *s_i2c = NULL;

// Bind a machine.I2C (or SoftI2C) object to be used by platform I/O.
// Call this from your module's constructor/init.
void vl53l1_platform_bind_i2c(mp_obj_t i2c_obj) {
    // We don't strictly need to check the type; any object implementing the
    // I2C protocol will work with mp_machine_i2c_transfer_adaptor.
    s_i2c = (mp_obj_base_t *)MP_OBJ_TO_PTR(i2c_obj);
    // Type check (important!)
    if (s_i2c->type != &machine_i2c_type) {
        mp_raise_TypeError(MP_ERROR_TEXT("expected an I2C object"));
    }
}

// Internal helper to perform I2C transfers. Returns 0 on success, -1 on error.
static int i2c_transfer_write(uint16_t addr, const uint8_t *wbuf, size_t wlen) {
    if (s_i2c == NULL) {
        mp_raise_msg(&mp_type_OSError, MP_ERROR_TEXT("VL53L1X: I2C not bound"));
    }

    /*mp_printf(&mp_plat_print, "I2C WRITE addr=0x%02x, len=%u: ", addr, (unsigned)wlen);
    for (size_t i = 0; i < wlen; i++) {
        mp_printf(&mp_plat_print, "%02x ", wbuf[i]);
    }
    mp_printf(&mp_plat_print, "\n");*/

    mp_machine_i2c_buf_t bufs[1] = {
        {.len = wlen, .buf = (uint8_t *)wbuf},
    };
    int ret = mp_machine_i2c_transfer_adaptor(s_i2c, addr, 1, bufs, MP_MACHINE_I2C_FLAG_STOP);
    return (ret >= 0) ? 0 : -1;
}

// Internal helper to perform I2C transfers. Returns 0 on success, -1 on error.
static int i2c_transfer_read(uint16_t addr, uint8_t *rbuf, size_t rlen) {
    //mp_printf(&mp_plat_print, "I2C READ addr=0x%02x, len=%u\n", addr, (unsigned)rlen);

    mp_machine_i2c_buf_t bufs[1] = {
        {.len = rlen, .buf = rbuf},
    };
    int ret = mp_machine_i2c_transfer_adaptor(s_i2c, addr, 1, bufs, MP_MACHINE_I2C_FLAG_READ | MP_MACHINE_I2C_FLAG_STOP);

    /*if (ret >= 0) {
        mp_printf(&mp_plat_print, "  DATA: ");
        for (size_t i = 0; i < rlen; i++) {
            mp_printf(&mp_plat_print, "%02x ", rbuf[i]);
        }
        mp_printf(&mp_plat_print, "\n");
    }*/

    return (ret >= 0) ? 0 : -1;
}

// Internal helper to perform I2C transfers. Returns 0 on success, -1 on error.
/*static int i2c_transfer_wr(uint16_t addr, const uint8_t *wbuf, size_t wlen, uint8_t *rbuf, size_t rlen) {
    if (s_i2c == NULL) {
        mp_raise_msg(&mp_type_OSError, MP_ERROR_TEXT("VL53L1X: I2C not bound"));
    }

    if (rlen == 0) {
        // Simple write

        mp_printf(&mp_plat_print, "I2C WRITE addr=0x%02x, len=%u: ", addr, (unsigned)wlen);
        for (size_t i = 0; i < wlen; i++) {
            mp_printf(&mp_plat_print, "%02x ", wbuf[i]);
        }
        mp_printf(&mp_plat_print, "\n");

        mp_machine_i2c_buf_t bufs[1] = {
            {.len = wlen, .buf = (uint8_t *)wbuf},
        };
        int ret = mp_machine_i2c_transfer_adaptor(s_i2c, addr, 1, bufs, MP_MACHINE_I2C_FLAG_STOP);
        return (ret >= 0) ? 0 : -1;
    } else if (wlen == 0) {
        // Simple read

        mp_printf(&mp_plat_print, "I2C READ addr=0x%02x, len=%u\n", addr, (unsigned)rlen);

        mp_machine_i2c_buf_t bufs[1] = {
            {.len = rlen, .buf = rbuf},
        };
        int ret = mp_machine_i2c_transfer_adaptor(s_i2c, addr, 1, bufs, MP_MACHINE_I2C_FLAG_READ | MP_MACHINE_I2C_FLAG_STOP);

        if (ret >= 0) {
            mp_printf(&mp_plat_print, "  DATA: ");
            for (size_t i = 0; i < rlen; i++) {
                mp_printf(&mp_plat_print, "%02x ", rbuf[i]);
            }
            mp_printf(&mp_plat_print, "\n");
        }

        return (ret >= 0) ? 0 : -1;
    } else {
        // Combined write (no stop) then read (with stop)
        / *mp_machine_i2c_buf_t bufs[2] = {
            {.len = wlen, .buf = (uint8_t *)wbuf},
            {.len = rlen, .buf = rbuf},
        };* /
        mp_machine_i2c_buf_t wbufs[1] = {
            {.len = wlen, .buf = (uint8_t *)wbuf},
        };
        mp_machine_i2c_buf_t rbufs[1] = {
            {.len = rlen, .buf = rbuf},
        };

        mp_printf(&mp_plat_print, "I2C WR+RD addr=0x%02x, wlen=%u, rlen=%u\n",
                  addr, (unsigned)wlen, (unsigned)rlen);
        mp_printf(&mp_plat_print, "  WRITE: ");
        for (size_t i = 0; i < wlen; i++) {
            mp_printf(&mp_plat_print, "%02x ", wbuf[i]);
        }
        mp_printf(&mp_plat_print, "\n");

        //int ret = mp_machine_i2c_transfer_adaptor(s_i2c, addr, 2, bufs, MP_MACHINE_I2C_FLAG_READ | MP_MACHINE_I2C_FLAG_STOP);
        int ret = mp_machine_i2c_transfer_adaptor(s_i2c, addr, 1, wbufs, MP_MACHINE_I2C_FLAG_STOP);

        if (ret < 0) {
            mp_raise_msg_varg(&mp_type_OSError, MP_ERROR_TEXT("I2C write failed, code %d"), ret);
        }

        ret = mp_machine_i2c_transfer_adaptor(s_i2c, addr, 1, rbufs,
                                            MP_MACHINE_I2C_FLAG_READ | MP_MACHINE_I2C_FLAG_STOP);
        if (ret < 0) {
            //vstr_clear(&rbuf);
            mp_raise_msg_varg(&mp_type_OSError, MP_ERROR_TEXT("I2C read failed, code %d"), ret);
        }


        if (ret >= 0) {
            mp_printf(&mp_plat_print, "  READ: ");
            for (size_t i = 0; i < rlen; i++) {
                mp_printf(&mp_plat_print, "%02x ", rbuf[i]);
            }
            mp_printf(&mp_plat_print, "\n");
        }

        return (ret >= 0) ? 0 : -1;
    }
}*/

int8_t VL53L1_WriteMulti(uint16_t dev, uint16_t index, uint8_t *pdata, uint32_t count) {
    int status_int;
    uint8_t hdr[2];
    VL53L1_Error Status = VL53L1_ERROR_NONE;
    /*(if (count > sizeof(wbuf) - 1) {
        return VL53L1_ERROR_INVALID_PARAMS;
    }*/
    // Allocate a temporary buffer to hold the payload in big-endian
    /*uint8_t buf[count * 2];
    for (uint32_t i = 0; i < count; i++) {
    buf[i] = pdata[i]>>8;  // convert each 16-bit word to big-endian
    buf[i+1] = pdata[i]&0xFF;
    }*/
    //mp_printf(&mp_plat_print, "VL53L1_WriteMulti called\n");
    hdr[0] = index>>8;
    hdr[1] = index&0xFF;

    // Write header (index) + data in a single transaction
    mp_machine_i2c_buf_t bufs[2] = {
    {.len = sizeof hdr, .buf = hdr},
    {.len = count, .buf = pdata},
    };
    status_int = mp_machine_i2c_transfer_adaptor(s_i2c, dev, 2, bufs, MP_MACHINE_I2C_FLAG_STOP);
    if (status_int != 0) {
    Status = VL53L1_ERROR_CONTROL_INTERFACE;
    }
    return Status;
}

int8_t VL53L1_ReadMulti(uint16_t dev, uint16_t index, uint8_t *pdata, uint32_t count) {
    //uint8_t hdr[2];
    //(hdr, index);
    // Write index, then read data
    //int ret = i2c_transfer_wr(dev, hdr, sizeof hdr, pdata, count);
    //return 0; //(ret == 0) ? 0 : -1;
    //mp_printf(&mp_plat_print, "VL53L1_ReadMulti called\n");
    VL53L1_Error Status = VL53L1_ERROR_NONE;
    int32_t status_int;
    uint8_t wbuf[2]; //, rbuf[1];

    wbuf[0] = index>>8;
	wbuf[1] = index&0xFF;
    //status_int = i2c_transfer_wr(dev, wbuf, sizeof wbuf, 0, 0);
    status_int = i2c_transfer_write(dev, wbuf, sizeof wbuf);

    if( status_int ){
        Status = VL53L1_ERROR_CONTROL_INTERFACE;
        goto done;
    }
    //status_int = i2c_transfer_wr(dev, wbuf, 0, rbuf, sizeof rbuf);
    status_int = i2c_transfer_read(dev, pdata, count);
    if (status_int != 0) {
        Status = VL53L1_ERROR_CONTROL_INTERFACE;
        goto done;
    }
done:
    //VL53L1_PutI2cBus();
    return Status;
}

int8_t VL53L1_WrByte(uint16_t dev, uint16_t index, uint8_t data) {
    //uint8_t buf[3];
    //put_be16(buf, index); buf[2] = data;
    //return 0; //return (i2c_transfer_wr(dev, buf, sizeof buf, NULL, 0) == 0) ? 0 : -1;
    VL53L1_Error Status = VL53L1_ERROR_NONE;
    int32_t status_int;
    uint8_t wbuf[3];

    wbuf[0] = index>>8;
    wbuf[1] = index&0xFF;
    wbuf[2] = data;

    status_int = i2c_transfer_write(dev, wbuf, sizeof wbuf);
    if (status_int != 0) {
        Status = VL53L1_ERROR_CONTROL_INTERFACE;
    }
    return Status;
}

int8_t VL53L1_WrWord(uint16_t dev, uint16_t index, uint16_t data) {
    //uint8_t buf[4];
    //put_be16(buf, index);      // header/index: big-endian
    //put_be16(buf + 2, data);   // payload: little-endian
    //return 0; //return (i2c_transfer_wr(dev, buf, sizeof buf, NULL, 0) == 0) ? 0 : -1;
    VL53L1_Error Status = VL53L1_ERROR_NONE;
    int32_t status_int;
    uint8_t wbuf[4];

    wbuf[0] = index>>8;
    wbuf[1] = index&0xFF;
    wbuf[2] = data >> 8;
    wbuf[3] = data & 0x00FF;

    status_int = i2c_transfer_write(dev, wbuf, sizeof wbuf);
    if (status_int != 0) {
        Status = VL53L1_ERROR_CONTROL_INTERFACE;
    }
    return Status;
}

int8_t VL53L1_WrDWord(uint16_t dev, uint16_t index, uint32_t data) {
    //uint8_t buf[6];
    //put_be16(buf, index);      // header/index: big-endian
    //put_be32(buf + 2, data);   // payload: little-endian
    //return 0; //return (i2c_transfer_wr(dev, buf, sizeof buf, NULL, 0) == 0) ? 0 : -1;
    VL53L1_Error Status = VL53L1_ERROR_NONE;
    int32_t status_int;
    uint8_t wbuf[6];

    wbuf[0] = index>>8;
    wbuf[1] = index&0xFF;
    wbuf[2] = (data >> 24) & 0xFF;
    wbuf[3] = (data >> 16) & 0xFF;
    wbuf[4] = (data >> 8)  & 0xFF;
    wbuf[5] = (data >> 0 ) & 0xFF;

    status_int = i2c_transfer_write(dev, wbuf, sizeof wbuf);
    if (status_int != 0) {
        Status = VL53L1_ERROR_CONTROL_INTERFACE;
    }
    return Status;
}

int8_t VL53L1_RdByte(uint16_t dev, uint16_t index, uint8_t *data) {
    //uint8_t hdr[2];
    //put_be16(hdr, index);
    //return 0; //return (i2c_transfer_wr(dev, hdr, sizeof hdr, data, 1) == 0) ? 0 : -1;
    VL53L1_Error Status = VL53L1_ERROR_NONE;
    int32_t status_int;
    uint8_t wbuf[2], rbuf[1];

    wbuf[0] = index>>8;
	wbuf[1] = index&0xFF;
    //status_int = i2c_transfer_wr(dev, wbuf, sizeof wbuf, 0, 0);
    status_int = i2c_transfer_write(dev, wbuf, sizeof wbuf);

    if( status_int ){
        Status = VL53L1_ERROR_CONTROL_INTERFACE;
        goto done;
    }
    //status_int = i2c_transfer_wr(dev, wbuf, 0, rbuf, sizeof rbuf);
    status_int = i2c_transfer_read(dev, rbuf, sizeof rbuf);
    if (status_int != 0) {
        Status = VL53L1_ERROR_CONTROL_INTERFACE;
        goto done;
    }

    *data = (uint8_t)rbuf[0];
done:
    //VL53L1_PutI2cBus();
    return Status;
}

int8_t VL53L1_RdWord(uint16_t dev, uint16_t index, uint16_t *data) {
    /*uint8_t r[2], hdr[2];
    hdr[0] = (index >> 8) & 0xFF;  / / MSB
    hdr[1] = index & 0xFF;         / / LSB

    int ret = i2c_transfer_wr(dev, hdr, sizeof hdr, r, sizeof r);
    if (ret == 0) { *data = ((uint16_t)r[0] << 8) | r[1];; return 0; }
    return -1;*/
    VL53L1_Error Status = VL53L1_ERROR_NONE;
    int32_t status_int;
    uint8_t wbuf[2], rbuf[2];

    wbuf[0] = index>>8;
	wbuf[1] = index&0xFF;
    //status_int = i2c_transfer_wr(dev, wbuf, sizeof wbuf, 0, 0);
    status_int = i2c_transfer_write(dev, wbuf, sizeof wbuf);

    if( status_int ){
        Status = VL53L1_ERROR_CONTROL_INTERFACE;
        goto done;
    }
    //status_int = i2c_transfer_wr(dev, wbuf, 0, rbuf, sizeof rbuf);
    status_int = i2c_transfer_read(dev, rbuf, sizeof rbuf);
    if (status_int != 0) {
        Status = VL53L1_ERROR_CONTROL_INTERFACE;
        goto done;
    }

    *data = ((uint16_t)rbuf[0]<<8) + (uint16_t)rbuf[1];
done:
    //VL53L1_PutI2cBus();
    return Status;
}

int8_t VL53L1_RdDWord(uint16_t dev, uint16_t index, uint32_t *data) {
    //uint8_t hdr[2], r[4];
    //put_be16(hdr, index);       // header/index: big-endian
    //int ret = i2c_transfer_wr(dev, hdr, sizeof hdr, r, sizeof r);
    //if (ret == 0) { *data = get_le32(r); return 0; }
    //return -1;
    VL53L1_Error Status = VL53L1_ERROR_NONE;
    int32_t status_int;
    uint8_t wbuf[2], rbuf[4];

    wbuf[0] = index>>8;
	wbuf[1] = index&0xFF;
    //status_int = i2c_transfer_wr(dev, wbuf, sizeof wbuf, 0, 0);
    status_int = i2c_transfer_write(dev, wbuf, sizeof wbuf);

    if( status_int ){
        Status = VL53L1_ERROR_CONTROL_INTERFACE;
        goto done;
    }
    //status_int = i2c_transfer_wr(dev, wbuf, 0, rbuf, sizeof rbuf);
    status_int = i2c_transfer_read(dev, rbuf, sizeof rbuf);
    if (status_int != 0) {
        Status = VL53L1_ERROR_CONTROL_INTERFACE;
        goto done;
    }

    *data = ((uint32_t)rbuf[0]<<24) +
            ((uint32_t)rbuf[1]<<16) + 
            ((uint32_t)rbuf[2]<<8) +
            (uint32_t)rbuf[3];
done:
    //VL53L1_PutI2cBus();
    return Status;
}

int8_t VL53L1_WaitMs(uint16_t dev, int32_t wait_ms) {
    (void)dev;
    if (wait_ms < 0) wait_ms = 0;
    mp_hal_delay_ms((mp_uint_t)wait_ms);
    return 0;
}

// ===== end of vl53l1_platform.c =====