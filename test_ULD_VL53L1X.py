''' A test script for ULD driver functionality
    all get commands are tested, some set commands are commented out
    because I have not studied their use
    
    sensor read & write can be used to read addresses directly for debug/test
    e.g.
    sensor.write(b'\x01\x0f')
    should return 2 (2 bytes written), subsequently
    sensor.read(2)
    should return b'\xea\xcc' ie the sensorID
    
    sensor.write_to_addr will write a single byte to a register
    note that these settings (may not) are not applied until a start ranging operation is applied
    
    
    
    using firmware from 
    \GitHub\VL53L1X-MP-user-module\build\build-RPI_PICO
    /GitHub/VL53L1X-MP-user-module/VL53L1X_ULD_driver/micropython.cmake
    
    BASE_DIR="/GitHub/VL53L1X-MP-user-module/build" &&
    MODULES_PATH="/GitHub/VL53L1X-MP-user-module/VL53L1X_ULD_driver/micropython.cmake" &&
    PORT_PATH="/GitHub/micropython/ports/rp2" &&
    cd "$BASE_DIR/build-RPI_PICO" &&
    cmake -DMICROPY_BOARD=RPI_PICO -DUSER_C_MODULES=$MODULES_PATH -S $PORT_PATH &&
    make -j$(nproc) &&
    picotool info -a firmware.uf2
    
    Table of SPAD locations. Each SPAD has a number which is not obvious.
    Pin 1 *
    128,136,144,152,160,168,176,184, 192,200,208,216,224,232,240,248
    129,137,145,153,161,169,177,185, 193,201,209,217,225,233,241,249
    130,138,146,154,162,170,178,186, 194,202,210,218,226,234,242,250
    131,139,147,155,163,171,179,187, 195,203,211,219,227,235,243,251
    132,140,148,156,164,172,180,188, 196,204,212,220,228,236,244,252
    133,141,149,157,165,173,181,189, 197,205,213,221,229,237,245,253
    134,142,150,158,166,174,182,190, 198,206,214,222,230,238,246,254
    135,143,151,159,167,175,183,191, 199,207,215,223,231,239,247,255
    127,119,111,103, 95, 87, 79, 71, 63, 55, 47, 39, 31, 23, 15, 7
    126,118,110,102, 94, 86, 78, 70, 62, 54, 46, 38, 30, 22, 14, 6
    125,117,109,101, 93, 85, 77, 69, 61, 53, 45, 37, 29, 21, 13, 5
    124,116,108,100, 92, 84, 76, 68, 60, 52, 44, 36, 28, 20, 12, 4
    123,115,107, 99, 91, 83, 75, 67, 59, 51, 43, 35, 27, 19, 11, 3
    122,114,106, 98, 90, 82, 74, 66, 58, 50, 42, 34, 26, 18, 10, 2
    121,113,105, 97, 89, 81, 73, 65, 57, 49, 41, 33, 25, 17, 9, 1
    120,112,104, 96, 88, 80, 72, 64, 56, 48, 40, 32, 24, 16, 8, 0
    
    the ROI SPAD as seen when viewing from behind the device looking toward
    the target

'''
from machine import I2C, Pin
import vl53l1x

# Create sensor object
i2c = I2C(0, sda=Pin(16), scl=Pin(17), freq=400_000)
sensor = vl53l1x.VL53L1X(i2c, 0x29)

print("=== VL53L1X ULD Method Test ===\n")

# 1. GetSWVersion
print("1. SWVersion:", sensor.get_sw_version())

# 2. SetI2CAddress
#sensor.set_i2c_address(0x30)
#print("2. SetI2CAddress: 0x30")

# 3. SensorInit
#sensor.sensor_init()
#print("3. SensorInit completed")

# 4. ClearInterrupt
sensor.clear_interrupt()
print("4. ClearInterrupt done")

# 5. SetInterruptPolarity
sensor.set_interrupt_polarity(1)
print("5. SetInterruptPolarity to 1")

# 6. GetInterruptPolarity
print("6. InterruptPolarity:", sensor.get_interrupt_polarity())

# 7. StartRanging
sensor.start_ranging()
print("7. StartRanging called")

# 8. StopRanging
sensor.stop_ranging()
print("8. StopRanging called")

# 9. CheckForDataReady
print("9. DataReady:", sensor.check_data_ready())

# 10. SetTimingBudgetInMs
sensor.set_timing_budget(50)
print("10. TimingBudget set to 50ms")

# 11. GetTimingBudgetInMs
print("11. TimingBudget:", sensor.get_timing_budget())

# 12. SetDistanceMode
sensor.set_distance_mode(2)
print("12. DistanceMode set to 2")

# 13. GetDistanceMode
print("13. DistanceMode:", sensor.get_distance_mode())

# 14. SetInterMeasurementInMs
sensor.set_inter_measurement(100)
print("14. InterMeasurement set to 100ms")

# 15. GetInterMeasurementInMs
print("15. InterMeasurement:", sensor.get_inter_measurement())

# 16. BootState
print("16. BootState:", sensor.boot_state())

# 17. GetSensorId
print("17. SensorId:", hex(sensor.get_sensor_id()))

# 18. GetDistance
print("18. Distance:", sensor.get_distance())

# 19. GetSignalPerSpad
print("19. SignalPerSpad:", sensor.get_signal_per_spad())

# 20. GetAmbientPerSpad
print("20. AmbientPerSpad:", sensor.get_ambient_per_spad())

# 21. GetSignalRate
print("21. SignalRate:", sensor.get_signal_rate())

# 22. GetSpadNb
print("22. SpadNb:", sensor.get_spad_nb())

# 23. GetAmbientRate
print("23. AmbientRate:", sensor.get_ambient_rate())

# 24. GetRangeStatus
print("24. RangeStatus:", sensor.get_range_status())

# 25. SetOffset
#sensor.set_offset(5)
#print("25. Offset set to 5")

# 26. GetOffset
print("26. Offset:", sensor.get_offset())

# 27. SetXtalk
#sensor.set_xtalk(10)
#print("27. Xtalk set to 10")

# 28. GetXtalk
print("28. Xtalk:", sensor.get_xtalk())

# 29. SetDistanceThreshold
sensor.set_distance_threshold(50, 200, 1, 2)
print("29. DistanceThreshold set")

# 30. GetDistanceThresholdWindow
print("30. DistanceThresholdWindow:", sensor.get_distance_threshold_window())

# 31. GetDistanceThresholdLow
print("31. DistanceThresholdLow:", sensor.get_distance_threshold_low())

# 32. GetDistanceThresholdHigh
print("32. DistanceThresholdHigh:", sensor.get_distance_threshold_high())

# 33. SetROI
sensor.set_roi(16, 16)
print("33. ROI set to 16x16")

# 34. GetROI_XY
print("34. ROI_XY:", sensor.get_roi_xy())

# 35. SetROICenter
sensor.set_roi_center(199)
print("35. ROICenter set to 199")

# 36. GetROICenter
print("36. ROICenter:", sensor.get_roi_center())

# 37. SetSignalThreshold
sensor.set_signal_threshold(1024)
print("37. SignalThreshold set to 1024")

# 38. GetSignalThreshold
print("38. SignalThreshold:", sensor.get_signal_threshold())

# 39. SetSigmaThreshold
#sensor.set_sigma_threshold(200)
#print("39. SigmaThreshold set to 200")

# 40. GetSigmaThreshold
print("40. SigmaThreshold:", sensor.get_sigma_threshold())

# 41. StartTemperatureUpdate
sensor.start_temp_update()
print("41. StartTemperatureUpdate called")

# 42. GetResult
print("42. Result:", sensor.get_result())

# 43. Return to Regular Ranging
sensor.return_to_ranging_mode()
print("43. Returned to regular ranging mode")

# 44. Calibrate Offset
# print("44. Calibrated Offset:", sensor.calibrate_offset(1000))

# 45. Calibrate cross talk
# print("45. Calibrated Xtalk:", sensor.calibrate_xtalk(1000))
