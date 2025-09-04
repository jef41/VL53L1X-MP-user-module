// ===== file: mp_driver_vl53l1x.c =====
// MicroPython wrapper for VL53L1X ULD (STSW-IMG009 v3.5.4)

#include "py/obj.h"
#include "py/objfun.h"
#include "py/runtime.h"
#include "py/mphal.h"

#include "VL53L1X_api.h"
#include "vl53l1_platform.h"


// forward decl from platform glue
void vl53l1_platform_bind_i2c(mp_obj_t i2c_obj);

// ---- Object ----
typedef struct _vl53l1x_obj_t {
    mp_obj_base_t base;
    mp_obj_t i2c_obj;   // machine.I2C object
    uint8_t i2c_addr;   // I2C 7-bit address (default 0x29)

    // track values for APIs that have setters but no getters
    uint16_t last_sigma_thresh;
    uint16_t last_min_count_rate;
    uint16_t last_roi_w, last_roi_h;
} vl53l1x_obj_t;

// ---- Helpers ----
static VL53L1_Error vl53l1x_driver_init(vl53l1x_obj_t *self) {
    vl53l1_platform_bind_i2c(self->i2c_obj);

    // Wait for boot
    uint8_t booted = 0;
    for (int i = 0; i < 100 && !booted; ++i) {
        (void)VL53L1X_BootState(self->i2c_addr, &booted);
        if (!booted) mp_hal_delay_ms(2);
    }
    if (!booted) return VL53L1_ERROR_TIME_OUT;

    // Basic sensor init
    VL53L1_Error err = VL53L1X_SensorInit(self->i2c_addr);
    if (err != VL53L1_ERROR_NONE) return err;

    // Default configuration
    (void)VL53L1X_SetDistanceMode(self->i2c_addr, 2 /*Long*/);
    (void)VL53L1X_SetTimingBudgetInMs(self->i2c_addr, 50);
    (void)VL53L1X_SetInterMeasurementInMs(self->i2c_addr, 50);
    (void)VL53L1X_StartRanging(self->i2c_addr);

    return VL53L1_ERROR_NONE;
}

static int vl53l1x_driver_read_mm(vl53l1x_obj_t *self) {
    uint8_t ready = 0;
    for (int i = 0; i < 50; ++i) {
        (void)VL53L1X_CheckForDataReady(self->i2c_addr, &ready);
        if (ready) break;
        mp_hal_delay_ms(2);
    }
    if (!ready) return -1;

    uint16_t dist = 0;
    (void)VL53L1X_GetDistance(self->i2c_addr, &dist);
    (void)VL53L1X_ClearInterrupt(self->i2c_addr);
    return (int)dist;
}

// ---- Constructor ----
static mp_obj_t vl53l1x_make_new(const mp_obj_type_t *type,
                                 size_t n_args, size_t n_kw,
                                 const mp_obj_t *args) {
    if (n_kw != 0) {
        mp_raise_TypeError(MP_ERROR_TEXT("keyword arguments not supported"));
    }
    if (n_args < 1 || n_args > 2) {
        mp_raise_TypeError(MP_ERROR_TEXT("expected 1 or 2 positional arguments"));
    }

    vl53l1x_obj_t *self = mp_obj_malloc(vl53l1x_obj_t, type);
    self->i2c_obj = args[0];
    self->i2c_addr = (n_args == 2) ? (uint8_t)mp_obj_get_int(args[1]) : 0x29;

    self->last_sigma_thresh = 0;
    self->last_min_count_rate = 0;
    self->last_roi_w = self->last_roi_h = 16; // default ROI 16x16

    VL53L1_Error err = vl53l1x_driver_init(self);
    if (err != VL53L1_ERROR_NONE) {
        mp_raise_msg(&mp_type_OSError, MP_ERROR_TEXT("VL53L1X init failed"));
    }
    return MP_OBJ_FROM_PTR(self);
}

// ---- Methods ----
// =======================================================
// VL53L1X ULD MicroPython Bindings (38 functions)
// =======================================================

// distance_mm()
static mp_obj_t vl53l1x_read_mm(mp_obj_t self_in) {
    vl53l1x_obj_t *self = MP_OBJ_TO_PTR(self_in);
    int mm = vl53l1x_driver_read_mm(self);
    if (mm < 0) mp_raise_msg(&mp_type_OSError, MP_ERROR_TEXT("no data"));
    return mp_obj_new_int(mm);
}
static MP_DEFINE_CONST_FUN_OBJ_1(vl53l1x_read_mm_obj, vl53l1x_read_mm);

// 1. BootState
static mp_obj_t vl53l1x_boot_state(mp_obj_t self_in) {
    vl53l1x_obj_t *self = MP_OBJ_TO_PTR(self_in);
    uint8_t state;
    VL53L1X_BootState(self->i2c_addr, &state);
    return mp_obj_new_int(state);
}
static MP_DEFINE_CONST_FUN_OBJ_1(vl53l1x_boot_state_obj, vl53l1x_boot_state);

// 2. SensorInit
static mp_obj_t vl53l1x_sensor_init(mp_obj_t self_in) {
    vl53l1x_obj_t *self = MP_OBJ_TO_PTR(self_in);
    VL53L1X_SensorInit(self->i2c_addr);
    return mp_const_none;
}
static MP_DEFINE_CONST_FUN_OBJ_1(vl53l1x_sensor_init_obj, vl53l1x_sensor_init);

// 3. StartRanging
static mp_obj_t vl53l1x_start_ranging(mp_obj_t self_in) {
    vl53l1x_obj_t *self = MP_OBJ_TO_PTR(self_in);
    VL53L1X_StartRanging(self->i2c_addr);
    return mp_const_none;
}
static MP_DEFINE_CONST_FUN_OBJ_1(vl53l1x_start_ranging_obj, vl53l1x_start_ranging);

// 4. StopRanging
static mp_obj_t vl53l1x_stop_ranging(mp_obj_t self_in) {
    vl53l1x_obj_t *self = MP_OBJ_TO_PTR(self_in);
    VL53L1X_StopRanging(self->i2c_addr);
    return mp_const_none;
}
static MP_DEFINE_CONST_FUN_OBJ_1(vl53l1x_stop_ranging_obj, vl53l1x_stop_ranging);

// 5. CheckForDataReady
static mp_obj_t vl53l1x_check_data_ready(mp_obj_t self_in) {
    vl53l1x_obj_t *self = MP_OBJ_TO_PTR(self_in);
    uint8_t ready;
    VL53L1X_CheckForDataReady(self->i2c_addr, &ready);
    return mp_obj_new_bool(ready);
}
static MP_DEFINE_CONST_FUN_OBJ_1(vl53l1x_check_data_ready_obj, vl53l1x_check_data_ready);

// 6. ClearInterrupt
static mp_obj_t vl53l1x_clear_interrupt(mp_obj_t self_in) {
    vl53l1x_obj_t *self = MP_OBJ_TO_PTR(self_in);
    VL53L1X_ClearInterrupt(self->i2c_addr);
    return mp_const_none;
}
static MP_DEFINE_CONST_FUN_OBJ_1(vl53l1x_clear_interrupt_obj, vl53l1x_clear_interrupt);

// 7. GetInterruptPolarity
static mp_obj_t vl53l1x_get_interrupt_polarity(mp_obj_t self_in) {
    vl53l1x_obj_t *self = MP_OBJ_TO_PTR(self_in);
    uint8_t pol;
    VL53L1X_GetInterruptPolarity(self->i2c_addr, &pol);
    return mp_obj_new_int(pol);
}
static MP_DEFINE_CONST_FUN_OBJ_1(vl53l1x_get_interrupt_polarity_obj, vl53l1x_get_interrupt_polarity);

// 8. GetDistance
static mp_obj_t vl53l1x_get_distance(mp_obj_t self_in) {
    vl53l1x_obj_t *self = MP_OBJ_TO_PTR(self_in);
    uint16_t dist;
    VL53L1X_GetDistance(self->i2c_addr, &dist);
    return mp_obj_new_int(dist);
}
static MP_DEFINE_CONST_FUN_OBJ_1(vl53l1x_get_distance_obj, vl53l1x_get_distance);

// 9. GetSignalRate
static mp_obj_t vl53l1x_get_signal_rate(mp_obj_t self_in) {
    vl53l1x_obj_t *self = MP_OBJ_TO_PTR(self_in);
    uint16_t rate;
    VL53L1X_GetSignalRate(self->i2c_addr, &rate);
    return mp_obj_new_int(rate);
}
static MP_DEFINE_CONST_FUN_OBJ_1(vl53l1x_get_signal_rate_obj, vl53l1x_get_signal_rate);

// 10. GetAmbientRate
static mp_obj_t vl53l1x_get_ambient_rate(mp_obj_t self_in) {
    vl53l1x_obj_t *self = MP_OBJ_TO_PTR(self_in);
    uint16_t rate;
    VL53L1X_GetAmbientRate(self->i2c_addr, &rate);
    return mp_obj_new_int(rate);
}
static MP_DEFINE_CONST_FUN_OBJ_1(vl53l1x_get_ambient_rate_obj, vl53l1x_get_ambient_rate);

// 11. GetSpadNb
static mp_obj_t vl53l1x_get_spad_nb(mp_obj_t self_in) {
    vl53l1x_obj_t *self = MP_OBJ_TO_PTR(self_in);
    uint16_t spad;
    VL53L1X_GetSpadNb(self->i2c_addr, &spad);
    return mp_obj_new_int(spad);
}
static MP_DEFINE_CONST_FUN_OBJ_1(vl53l1x_get_spad_nb_obj, vl53l1x_get_spad_nb);

// 12. GetRangeStatus
static mp_obj_t vl53l1x_get_range_status(mp_obj_t self_in) {
    vl53l1x_obj_t *self = MP_OBJ_TO_PTR(self_in);
    uint8_t status;
    VL53L1X_GetRangeStatus(self->i2c_addr, &status);
    return mp_obj_new_int(status);
}
static MP_DEFINE_CONST_FUN_OBJ_1(vl53l1x_get_range_status_obj, vl53l1x_get_range_status);

// 13. GetResult
static mp_obj_t vl53l1x_get_result(mp_obj_t self_in) {
    vl53l1x_obj_t *self = MP_OBJ_TO_PTR(self_in);
    VL53L1X_Result_t res;
    VL53L1X_GetResult(self->i2c_addr, &res);
    mp_obj_t tuple[4] = {
        mp_obj_new_int(res.Status),
        mp_obj_new_int(res.Distance),
        mp_obj_new_int(res.Ambient),
        mp_obj_new_int(res.SigPerSPAD),
    };
    return mp_obj_new_tuple(4, tuple);
}
static MP_DEFINE_CONST_FUN_OBJ_1(vl53l1x_get_result_obj, vl53l1x_get_result);

// 14. SetTimingBudgetInMs
static mp_obj_t vl53l1x_set_timing_budget(mp_obj_t self_in, mp_obj_t budget_in) {
    vl53l1x_obj_t *self = MP_OBJ_TO_PTR(self_in);
    uint16_t budget = mp_obj_get_int(budget_in);
    VL53L1X_SetTimingBudgetInMs(self->i2c_addr, budget);
    return mp_const_none;
}
static MP_DEFINE_CONST_FUN_OBJ_2(vl53l1x_set_timing_budget_obj, vl53l1x_set_timing_budget);

// 15. GetTimingBudgetInMs
static mp_obj_t vl53l1x_get_timing_budget(mp_obj_t self_in) {
    vl53l1x_obj_t *self = MP_OBJ_TO_PTR(self_in);
    uint16_t budget;
    VL53L1X_GetTimingBudgetInMs(self->i2c_addr, &budget);
    return mp_obj_new_int(budget);
}
static MP_DEFINE_CONST_FUN_OBJ_1(vl53l1x_get_timing_budget_obj, vl53l1x_get_timing_budget);

// 16. SetInterMeasurementInMs
static mp_obj_t vl53l1x_set_inter_measurement(mp_obj_t self_in, mp_obj_t period_in) {
    vl53l1x_obj_t *self = MP_OBJ_TO_PTR(self_in);
    uint16_t period = mp_obj_get_int(period_in);
    VL53L1X_SetInterMeasurementInMs(self->i2c_addr, period);
    return mp_const_none;
}
static MP_DEFINE_CONST_FUN_OBJ_2(vl53l1x_set_inter_measurement_obj, vl53l1x_set_inter_measurement);

// 17. GetInterMeasurementInMs
static mp_obj_t vl53l1x_get_inter_measurement(mp_obj_t self_in) {
    vl53l1x_obj_t *self = MP_OBJ_TO_PTR(self_in);
    uint16_t period;
    VL53L1X_GetInterMeasurementInMs(self->i2c_addr, &period);
    return mp_obj_new_int(period);
}
static MP_DEFINE_CONST_FUN_OBJ_1(vl53l1x_get_inter_measurement_obj, vl53l1x_get_inter_measurement);

// 18. SetDistanceMode
static mp_obj_t vl53l1x_set_distance_mode(mp_obj_t self_in, mp_obj_t mode_in) {
    vl53l1x_obj_t *self = MP_OBJ_TO_PTR(self_in);
    uint16_t mode = mp_obj_get_int(mode_in);
    VL53L1X_SetDistanceMode(self->i2c_addr, mode);
    return mp_const_none;
}
static MP_DEFINE_CONST_FUN_OBJ_2(vl53l1x_set_distance_mode_obj, vl53l1x_set_distance_mode);

// 19. GetDistanceMode
static mp_obj_t vl53l1x_get_distance_mode(mp_obj_t self_in) {
    vl53l1x_obj_t *self = MP_OBJ_TO_PTR(self_in);
    uint16_t mode;
    VL53L1X_GetDistanceMode(self->i2c_addr, &mode);
    return mp_obj_new_int(mode);
}
static MP_DEFINE_CONST_FUN_OBJ_1(vl53l1x_get_distance_mode_obj, vl53l1x_get_distance_mode);

// 20. SetROI
static mp_obj_t vl53l1x_set_roi(mp_obj_t self_in, mp_obj_t x_in, mp_obj_t y_in) {
    vl53l1x_obj_t *self = MP_OBJ_TO_PTR(self_in);
    uint16_t x = mp_obj_get_int(x_in);
    uint16_t y = mp_obj_get_int(y_in);
    VL53L1X_SetROI(self->i2c_addr, x, y);
    return mp_const_none;
}
static MP_DEFINE_CONST_FUN_OBJ_3(vl53l1x_set_roi_obj, vl53l1x_set_roi);

// 21. GetROI_XY
static mp_obj_t vl53l1x_get_roi(mp_obj_t self_in) {
    vl53l1x_obj_t *self = MP_OBJ_TO_PTR(self_in);
    uint16_t x, y;
    VL53L1X_GetROI_XY(self->i2c_addr, &x, &y);
    mp_obj_t tuple[2] = { mp_obj_new_int(x), mp_obj_new_int(y) };
    return mp_obj_new_tuple(2, tuple);
}
static MP_DEFINE_CONST_FUN_OBJ_1(vl53l1x_get_roi_obj, vl53l1x_get_roi);

// 22. SetROICenter
static mp_obj_t vl53l1x_set_roi_center(mp_obj_t self_in, mp_obj_t spad_in) {
    vl53l1x_obj_t *self = MP_OBJ_TO_PTR(self_in);
    uint8_t spad = mp_obj_get_int(spad_in);
    VL53L1X_SetROICenter(self->i2c_addr, spad);
    return mp_const_none;
}
static MP_DEFINE_CONST_FUN_OBJ_2(vl53l1x_set_roi_center_obj, vl53l1x_set_roi_center);

// 23. GetROICenter
static mp_obj_t vl53l1x_get_roi_center(mp_obj_t self_in) {
    vl53l1x_obj_t *self = MP_OBJ_TO_PTR(self_in);
    uint8_t spad;
    VL53L1X_GetROICenter(self->i2c_addr, &spad);
    return mp_obj_new_int(spad);
}
static MP_DEFINE_CONST_FUN_OBJ_1(vl53l1x_get_roi_center_obj, vl53l1x_get_roi_center);

// 24. SetDistanceThreshold
// 4 is the max of variables passed with MP_DEFINE_CONST_FUN_OBJ_ but 5th variable is always 0
/*static mp_obj_t vl53l1x_set_distance_threshold(mp_obj_t self_in,
                                               mp_obj_t low_in,
                                               mp_obj_t high_in,
                                               mp_obj_t window_in,
                                               mp_obj_t mode_in) {
    vl53l1x_obj_t *self = MP_OBJ_TO_PTR(self_in);
    uint16_t low = mp_obj_get_int(low_in);
    uint16_t high = mp_obj_get_int(high_in);
    uint8_t window = mp_obj_get_int(window_in);
    uint8_t mode = mp_obj_get_int(mode_in);*/

static mp_obj_t vl53l1x_set_distance_threshold(size_t n_args, const mp_obj_t *args) {
    if (n_args != 5) {
        mp_raise_TypeError(MP_ERROR_TEXT("vl53l1x_set_distance_threshold requires exactly 5 arguments"));
    }
    vl53l1x_obj_t *self = MP_OBJ_TO_PTR(args[0]);
    uint16_t low = mp_obj_get_int(args[1]);
    uint16_t high = mp_obj_get_int(args[2]);
    uint8_t window = mp_obj_get_int(args[3]);
    uint8_t mode = mp_obj_get_int(args[4]);
    VL53L1X_SetDistanceThreshold(self->i2c_addr, low, high, window, mode);
    return mp_const_none;
}
//static MP_DEFINE_CONST_FUN_OBJ_5(vl53l1x_set_distance_threshold_obj, vl53l1x_set_distance_threshold);
static MP_DEFINE_CONST_FUN_OBJ_VAR(vl53l1x_set_distance_threshold_obj, 5, vl53l1x_set_distance_threshold);


// 25. GetDistanceThresholdLow
static mp_obj_t vl53l1x_get_distance_threshold_low(mp_obj_t self_in) {
    vl53l1x_obj_t *self = MP_OBJ_TO_PTR(self_in);
    uint16_t low;
    VL53L1X_GetDistanceThresholdLow(self->i2c_addr, &low);
    return mp_obj_new_int(low);
}
static MP_DEFINE_CONST_FUN_OBJ_1(vl53l1x_get_distance_threshold_low_obj, vl53l1x_get_distance_threshold_low);

// 26. GetDistanceThresholdHigh
static mp_obj_t vl53l1x_get_distance_threshold_high(mp_obj_t self_in) {
    vl53l1x_obj_t *self = MP_OBJ_TO_PTR(self_in);
    uint16_t high;
    VL53L1X_GetDistanceThresholdHigh(self->i2c_addr, &high);
    return mp_obj_new_int(high);
}
static MP_DEFINE_CONST_FUN_OBJ_1(vl53l1x_get_distance_threshold_high_obj, vl53l1x_get_distance_threshold_high);

// 27. GetDistanceThresholdWindow
static mp_obj_t vl53l1x_get_distance_threshold_window(mp_obj_t self_in) {
    vl53l1x_obj_t *self = MP_OBJ_TO_PTR(self_in);
    uint16_t window;
    VL53L1X_GetDistanceThresholdWindow(self->i2c_addr, &window);
    return mp_obj_new_int(window);
}
static MP_DEFINE_CONST_FUN_OBJ_1(vl53l1x_get_distance_threshold_window_obj, vl53l1x_get_distance_threshold_window);

// 28. GetDistanceThresholdMode
/*static mp_obj_t vl53l1x_get_distance_threshold_mode(mp_obj_t self_in) {
    vl53l1x_obj_t *self = MP_OBJ_TO_PTR(self_in);
    uint8_t mode;
    VL53L1X_GetDistanceThresholdMode(self->i2c_addr, &mode);
    return mp_obj_new_int(mode);
}
static MP_DEFINE_CONST_FUN_OBJ_1(vl53l1x_get_distance_threshold_mode_obj, vl53l1x_get_distance_threshold_mode);*/

// 29. SetSignalThreshold
static mp_obj_t vl53l1x_set_signal_threshold(mp_obj_t self_in, mp_obj_t threshold_in) {
    vl53l1x_obj_t *self = MP_OBJ_TO_PTR(self_in);
    uint16_t threshold = mp_obj_get_int(threshold_in);
    VL53L1X_SetSignalThreshold(self->i2c_addr, threshold);
    return mp_const_none;
}
static MP_DEFINE_CONST_FUN_OBJ_2(vl53l1x_set_signal_threshold_obj, vl53l1x_set_signal_threshold);

// 30. GetSignalThreshold
static mp_obj_t vl53l1x_get_signal_threshold(mp_obj_t self_in) {
    vl53l1x_obj_t *self = MP_OBJ_TO_PTR(self_in);
    uint16_t threshold;
    VL53L1X_GetSignalThreshold(self->i2c_addr, &threshold);
    return mp_obj_new_int(threshold);
}
static MP_DEFINE_CONST_FUN_OBJ_1(vl53l1x_get_signal_threshold_obj, vl53l1x_get_signal_threshold);

// 31. SetSigmaThreshold
static mp_obj_t vl53l1x_set_sigma_threshold(mp_obj_t self_in, mp_obj_t threshold_in) {
    vl53l1x_obj_t *self = MP_OBJ_TO_PTR(self_in);
    uint16_t threshold = mp_obj_get_int(threshold_in);
    VL53L1X_SetSigmaThreshold(self->i2c_addr, threshold);
    return mp_const_none;
}
static MP_DEFINE_CONST_FUN_OBJ_2(vl53l1x_set_sigma_threshold_obj, vl53l1x_set_sigma_threshold);

// 32. GetSigmaThreshold
static mp_obj_t vl53l1x_get_sigma_threshold(mp_obj_t self_in) {
    vl53l1x_obj_t *self = MP_OBJ_TO_PTR(self_in);
    uint16_t threshold;
    VL53L1X_GetSigmaThreshold(self->i2c_addr, &threshold);
    return mp_obj_new_int(threshold);
}
static MP_DEFINE_CONST_FUN_OBJ_1(vl53l1x_get_sigma_threshold_obj, vl53l1x_get_sigma_threshold);

// 33. StartTemperatureUpdate
static mp_obj_t vl53l1x_start_temp_update(mp_obj_t self_in) {
    vl53l1x_obj_t *self = MP_OBJ_TO_PTR(self_in);
    VL53L1X_StartTemperatureUpdate(self->i2c_addr);
    return mp_const_none;
}
static MP_DEFINE_CONST_FUN_OBJ_1(vl53l1x_start_temp_update_obj, vl53l1x_start_temp_update);

// 34. PerformRefCalibration
/*static mp_obj_t vl53l1x_perform_ref_calibration(mp_obj_t self_in, mp_obj_t vhv_in, mp_obj_t phase_in) {
    vl53l1x_obj_t *self = MP_OBJ_TO_PTR(self_in);
    uint8_t vhv = mp_obj_get_int(vhv_in);
    uint8_t phase = mp_obj_get_int(phase_in);
    VL53L1X_PerformRefCalibration(self->i2c_addr, &vhv, &phase);
    mp_obj_t tuple[2] = { mp_obj_new_int(vhv), mp_obj_new_int(phase) };
    return mp_obj_new_tuple(2, tuple);
}
static MP_DEFINE_CONST_FUN_OBJ_3(vl53l1x_perform_ref_calibration_obj, vl53l1x_perform_ref_calibration);*/

// 35. PerformXTalkCalibration
/*static mp_obj_t vl53l1x_perform_xtalk_calibration(mp_obj_t self_in, mp_obj_t target_in) {
    vl53l1x_obj_t *self = MP_OBJ_TO_PTR(self_in);
    uint16_t target = mp_obj_get_int(target_in);
    VL53L1X_PerformXTalkCalibration(self->i2c_addr, target);
    return mp_const_none;
}
static MP_DEFINE_CONST_FUN_OBJ_2(vl53l1x_perform_xtalk_calibration_obj, vl53l1x_perform_xtalk_calibration);*/

// 36. GetDeviceInfo
/*static mp_obj_t vl53l1x_get_device_info(mp_obj_t self_in) {
    vl53l1x_obj_t *self = MP_OBJ_TO_PTR(self_in);
    VL53L1X_DeviceInfo_t info;
    VL53L1X_GetDeviceInfo(self->i2c_addr, &info);
    mp_obj_t tuple[5] = {
        mp_obj_new_str(info.Name, strlen(info.Name)),
        mp_obj_new_str(info.Type, strlen(info.Type)),
        mp_obj_new_str(info.ProductId, strlen(info.ProductId)),
        mp_obj_new_int(info.ProductType),
        mp_obj_new_int(info.ProductRevisionMajor * 10 + info.ProductRevisionMinor)
    };
    return mp_obj_new_tuple(5, tuple);
}
static MP_DEFINE_CONST_FUN_OBJ_1(vl53l1x_get_device_info_obj, vl53l1x_get_device_info);*/

// 37. GetSWVersion
static mp_obj_t vl53l1x_get_sw_version(mp_obj_t self_in) {
    //vl53l1x_obj_t *self = MP_OBJ_TO_PTR(self_in);
    VL53L1X_Version_t ver;
    VL53L1X_GetSWVersion(&ver);
    mp_obj_t tuple[4] = {
        mp_obj_new_int(ver.major),
        mp_obj_new_int(ver.minor),
        mp_obj_new_int(ver.build),
        mp_obj_new_int(ver.revision)
    };
    return mp_obj_new_tuple(4, tuple);
}
static MP_DEFINE_CONST_FUN_OBJ_1(vl53l1x_get_sw_version_obj, vl53l1x_get_sw_version);

// 38. SetI2CAddress
static mp_obj_t vl53l1x_set_i2c_address(mp_obj_t self_in, mp_obj_t addr_in) {
    vl53l1x_obj_t *self = MP_OBJ_TO_PTR(self_in);
    uint8_t new_addr = mp_obj_get_int(addr_in);
    VL53L1X_SetI2CAddress(self->i2c_addr, new_addr);
    self->i2c_addr = new_addr; // update local copy
    return mp_const_none;
}
static MP_DEFINE_CONST_FUN_OBJ_2(vl53l1x_set_i2c_address_obj, vl53l1x_set_i2c_address);



// ---- Locals dict ----
static const mp_rom_map_elem_t vl53l1x_locals_dict_table[] = {
    { MP_ROM_QSTR(MP_QSTR_distance_mm), MP_ROM_PTR(&vl53l1x_read_mm_obj) },
    // 1. BootState
    { MP_ROM_QSTR(MP_QSTR_boot_state), MP_ROM_PTR(&vl53l1x_boot_state_obj) },
    // 2. SensorInit
    { MP_ROM_QSTR(MP_QSTR_sensor_init), MP_ROM_PTR(&vl53l1x_sensor_init_obj) },
    // 3. StartRanging
    { MP_ROM_QSTR(MP_QSTR_start_ranging), MP_ROM_PTR(&vl53l1x_start_ranging_obj) },
    // 4. StopRanging
    { MP_ROM_QSTR(MP_QSTR_stop_ranging), MP_ROM_PTR(&vl53l1x_stop_ranging_obj) },
    // 5. CheckForDataReady
    { MP_ROM_QSTR(MP_QSTR_check_data_ready), MP_ROM_PTR(&vl53l1x_check_data_ready_obj) },
    // 6. ClearInterrupt
    { MP_ROM_QSTR(MP_QSTR_clear_interrupt), MP_ROM_PTR(&vl53l1x_clear_interrupt_obj) },
    // 7. GetInterruptPolarity
    { MP_ROM_QSTR(MP_QSTR_get_interrupt_polarity), MP_ROM_PTR(&vl53l1x_get_interrupt_polarity_obj) },
    // 8. GetDistance
    { MP_ROM_QSTR(MP_QSTR_get_distance), MP_ROM_PTR(&vl53l1x_get_distance_obj) },
    // 9. GetSignalRate
    { MP_ROM_QSTR(MP_QSTR_get_signal_rate), MP_ROM_PTR(&vl53l1x_get_signal_rate_obj) },
    // 10. GetAmbientRate
    { MP_ROM_QSTR(MP_QSTR_get_ambient_rate), MP_ROM_PTR(&vl53l1x_get_ambient_rate_obj) },
    // 11. GetSpadNb
    { MP_ROM_QSTR(MP_QSTR_get_spad_nb), MP_ROM_PTR(&vl53l1x_get_spad_nb_obj) },
    // 12. GetRangeStatus
    { MP_ROM_QSTR(MP_QSTR_get_range_status), MP_ROM_PTR(&vl53l1x_get_range_status_obj) },
    // 13. GetResult
    { MP_ROM_QSTR(MP_QSTR_get_result), MP_ROM_PTR(&vl53l1x_get_result_obj) },
    // 14. SetTimingBudgetInMs
    { MP_ROM_QSTR(MP_QSTR_set_timing_budget), MP_ROM_PTR(&vl53l1x_set_timing_budget_obj) },
    // 15. GetTimingBudgetInMs
    { MP_ROM_QSTR(MP_QSTR_get_timing_budget), MP_ROM_PTR(&vl53l1x_get_timing_budget_obj) },
    // 16. SetInterMeasurementInMs
    { MP_ROM_QSTR(MP_QSTR_set_inter_measurement), MP_ROM_PTR(&vl53l1x_set_inter_measurement_obj) },
    // 17. GetInterMeasurementInMs
    { MP_ROM_QSTR(MP_QSTR_get_inter_measurement), MP_ROM_PTR(&vl53l1x_get_inter_measurement_obj) },
    // 18. SetDistanceMode
    { MP_ROM_QSTR(MP_QSTR_set_distance_mode), MP_ROM_PTR(&vl53l1x_set_distance_mode_obj) },
    // 19. GetDistanceMode
    { MP_ROM_QSTR(MP_QSTR_get_distance_mode), MP_ROM_PTR(&vl53l1x_get_distance_mode_obj) },   
    // 20. SetROI
    { MP_ROM_QSTR(MP_QSTR_set_roi), MP_ROM_PTR(&vl53l1x_set_roi_obj) },
    // 21. GetROI
    { MP_ROM_QSTR(MP_QSTR_get_roi), MP_ROM_PTR(&vl53l1x_get_roi_obj) },
    // 22. SetROICenter
    { MP_ROM_QSTR(MP_QSTR_set_roi_center), MP_ROM_PTR(&vl53l1x_set_roi_center_obj) },
    // 23. GetROICenter
    { MP_ROM_QSTR(MP_QSTR_get_roi_center), MP_ROM_PTR(&vl53l1x_get_roi_center_obj) },
    // 24. SetDistanceThreshold
    { MP_ROM_QSTR(MP_QSTR_set_distance_threshold), MP_ROM_PTR(&vl53l1x_set_distance_threshold_obj) },
    // 25. GetDistanceThresholdLow
    { MP_ROM_QSTR(MP_QSTR_get_distance_threshold_low), MP_ROM_PTR(&vl53l1x_get_distance_threshold_low_obj) },
    // 26. GetDistanceThresholdHigh
    { MP_ROM_QSTR(MP_QSTR_get_distance_threshold_high), MP_ROM_PTR(&vl53l1x_get_distance_threshold_high_obj) },
    // 27. GetDistanceThresholdWindow
    { MP_ROM_QSTR(MP_QSTR_get_distance_threshold_window), MP_ROM_PTR(&vl53l1x_get_distance_threshold_window_obj) },
    // 28. GetDistanceThresholdMode
    //{ MP_ROM_QSTR(MP_QSTR_get_distance_threshold_mode), MP_ROM_PTR(&vl53l1x_get_distance_threshold_mode_obj) },
    // 29. SetSignalThreshold
    { MP_ROM_QSTR(MP_QSTR_set_signal_threshold), MP_ROM_PTR(&vl53l1x_set_signal_threshold_obj) },
    // 30. GetSignalThreshold
    { MP_ROM_QSTR(MP_QSTR_get_signal_threshold), MP_ROM_PTR(&vl53l1x_get_signal_threshold_obj) },
    // 31. SetSigmaThreshold
    { MP_ROM_QSTR(MP_QSTR_set_sigma_threshold), MP_ROM_PTR(&vl53l1x_set_sigma_threshold_obj) },
    // 32. GetSigmaThreshold
    { MP_ROM_QSTR(MP_QSTR_get_sigma_threshold), MP_ROM_PTR(&vl53l1x_get_sigma_threshold_obj) },
    // 33. StartTemperatureUpdate
    { MP_ROM_QSTR(MP_QSTR_start_temp_update), MP_ROM_PTR(&vl53l1x_start_temp_update_obj) },
    // 34. PerformRefCalibration
    //{ MP_ROM_QSTR(MP_QSTR_perform_ref_calibration), MP_ROM_PTR(&vl53l1x_perform_ref_calibration_obj) },
    // 35. PerformXTalkCalibration
    //{ MP_ROM_QSTR(MP_QSTR_perform_xtalk_calibration), MP_ROM_PTR(&vl53l1x_perform_xtalk_calibration_obj) },
    // 36. GetDeviceInfo
    //{ MP_ROM_QSTR(MP_QSTR_get_device_info), MP_ROM_PTR(&vl53l1x_get_device_info_obj) },
    // 37. GetSWVersion
    { MP_ROM_QSTR(MP_QSTR_get_sw_version), MP_ROM_PTR(&vl53l1x_get_sw_version_obj) },
    // 38. SetI2CAddress
    { MP_ROM_QSTR(MP_QSTR_set_i2c_address), MP_ROM_PTR(&vl53l1x_set_i2c_address_obj) },

};
static MP_DEFINE_CONST_DICT(vl53l1x_locals_dict, vl53l1x_locals_dict_table);

// ---- Type ----
static MP_DEFINE_CONST_OBJ_TYPE(
    vl53l1x_type,
    MP_QSTR_VL53L1X,
    MP_TYPE_FLAG_NONE,
    make_new, vl53l1x_make_new,
    locals_dict, &vl53l1x_locals_dict
);

// ---- Module ----
static const mp_rom_map_elem_t vl53l1x_module_globals_table[] = {
    { MP_ROM_QSTR(MP_QSTR___name__), MP_ROM_QSTR(MP_QSTR_vl53l1x) },
    { MP_ROM_QSTR(MP_QSTR_VL53L1X), MP_ROM_PTR(&vl53l1x_type) },
};
static MP_DEFINE_CONST_DICT(vl53l1x_module_globals, vl53l1x_module_globals_table);

const mp_obj_module_t mp_module_vl53l1x = {
    .base = { &mp_type_module },
    .globals = (mp_obj_dict_t *)&vl53l1x_module_globals,
};

MP_REGISTER_MODULE(MP_QSTR_vl53l1x, mp_module_vl53l1x);

// ===== end of mp_driver_vl53l1x.c =====
