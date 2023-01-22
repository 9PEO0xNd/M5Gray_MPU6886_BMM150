#include <Arduino.h>
#include <Wire.h>
#include "Preferences.h"
//#include "math.h"
#include "bmm150.h"
#include "bmm150_defs.h"

class BMM150class
{
public:
    Preferences prefs;
    struct bmm150_dev dev;
    bmm150_mag_data mag_offset;
    bmm150_mag_data mag_scale; // for Soft iron distortion

private:
//    int8_t i2c_read(uint8_t dev_id, uint8_t reg_addr, uint8_t *read_data, uint16_t len);
//    int8_t i2c_write(uint8_t dev_id, uint8_t reg_addr, uint8_t *read_data, uint16_t len);
    int8_t bmm150_initialization();
    void bmm150_offset_save();
    void bmm150_offset_load();
    bmm150_mag_data mag_max;
    bmm150_mag_data mag_min;
    bmm150_mag_data mag_chord; // for Soft iron distortion

public:
    BMM150class();
    int8_t Init(void);
    void bmm150_calibrate(uint32_t calibrate_time);
    void getMagnetData(float *mx, float *my, float *mz);
    void getMagnetOffset(float *mx, float *my, float *mz); // hard iron
    void getMagnetScale(float *mx, float *my, float *mz);  // soft iron
};
