#include "BMM150class.h"

BMM150class::BMM150class()
{
}

int8_t i2c_read(uint8_t dev_id, uint8_t reg_addr, uint8_t *read_data, uint16_t len)
{
    if (M5.I2C.readBytes(dev_id, reg_addr, len, read_data))
    {
        return BMM150_OK;
    }
    else
    {
        return BMM150_E_DEV_NOT_FOUND;
    }
}

int8_t i2c_write(uint8_t dev_id, uint8_t reg_addr, uint8_t *read_data, uint16_t len)
{
    if (M5.I2C.writeBytes(dev_id, reg_addr, read_data, len))
    {
        return BMM150_OK;
    }
    else
    {
        return BMM150_E_DEV_NOT_FOUND;
    }
}

int8_t BMM150class::bmm150_initialization()
{
    int8_t rslt = BMM150_OK;

    /* Sensor interface over SPI with native chip select line */
    dev.dev_id = 0x10;
    dev.intf = BMM150_I2C_INTF;
    dev.read = i2c_read;
    dev.write = i2c_write;
    dev.delay_ms = delay;

    /* make sure max < mag data first  */
    mag_max.x = -2000;
    mag_max.y = -2000;
    mag_max.z = -2000;

    /* make sure min > mag data first  */
    mag_min.x = 2000;
    mag_min.y = 2000;
    mag_min.z = 2000;

    rslt = bmm150_init(&dev);
    dev.settings.pwr_mode = BMM150_NORMAL_MODE;
    rslt |= bmm150_set_op_mode(&dev);
    dev.settings.preset_mode = BMM150_PRESETMODE_ENHANCED;
    rslt |= bmm150_set_presetmode(&dev);
    return rslt;
}

void BMM150class::bmm150_offset_save()
{
    prefs.begin("bmm150", false);
    prefs.putBytes("offset", (uint8_t *)&mag_offset, sizeof(bmm150_mag_data));
    // added soft-iron scale
    prefs.putBytes("scale", (uint8_t *)&mag_scale, sizeof(bmm150_mag_data));
    prefs.end();
}

void BMM150class::bmm150_offset_load()
{
    if (prefs.begin("bmm150", true))
    {
        prefs.getBytes("offset", (uint8_t *)&mag_offset, sizeof(bmm150_mag_data));
        // added soft iron distortion scaling
        prefs.getBytes("scale", (uint8_t *)&mag_scale, sizeof(bmm150_mag_data));
        prefs.end();
        Serial.printf("bmm150 load offset finish.... \r\n");
    }
    else
    {
        Serial.printf("bmm150 load offset failed.... \r\n");
    }
}
/*
void BMM150class::setup()
{
    M5.begin(true, false, true, false);
    Wire.begin(21, 22, 400000);

    M5.Lcd.fillScreen(BLACK);
    M5.Lcd.setTextColor(WHITE);

    if (bmm150_initialization() != BMM150_OK)
    {
        // img.fillSprite(0);
        // img.drawCentreString("BMM150 init failed", 160, 110, 4);
        // img.pushSprite(0, 0);
        M5.Lcd.setCursor(0, 10);
        M5.Lcd.print("BMM150 init failed");
        for (;;)
        {
            delay(100);
        }
    }

    bmm150_offset_load();
}
*/
void BMM150class::bmm150_calibrate(uint32_t calibrate_time)
{
    uint32_t calibrate_timeout = 0;
    float avg_chord;

    calibrate_timeout = millis() + calibrate_time;
    Serial.printf("Go calibrate, use %d ms \r\n", calibrate_time);
    Serial.printf("running ...");

    while (calibrate_timeout > millis())
    {
        bmm150_read_mag_data(&dev);
        if (dev.data.x)
        {
            mag_min.x = (dev.data.x < mag_min.x) ? dev.data.x : mag_min.x;
            mag_max.x = (dev.data.x > mag_max.x) ? dev.data.x : mag_max.x;
        }

        if (dev.data.y)
        {
            mag_max.y = (dev.data.y > mag_max.y) ? dev.data.y : mag_max.y;
            mag_min.y = (dev.data.y < mag_min.y) ? dev.data.y : mag_min.y;
        }

        if (dev.data.z)
        {
            mag_min.z = (dev.data.z < mag_min.z) ? dev.data.z : mag_min.z;
            mag_max.z = (dev.data.z > mag_max.z) ? dev.data.z : mag_max.z;
        }
        delay(100);
    }

    mag_offset.x = (mag_max.x + mag_min.x) / 2;
    mag_offset.y = (mag_max.y + mag_min.y) / 2;
    mag_offset.z = (mag_max.z + mag_min.z) / 2;
    // added soft iron calibration
    mag_chord.x = (mag_max.x - mag_min.x) / 2;
    mag_chord.y = (mag_max.y - mag_min.y) / 2;
    mag_chord.z = (mag_max.z - mag_min.z) / 2;
    avg_chord = (mag_chord.x + mag_chord.y + mag_chord.z)/3;
    mag_scale.x = avg_chord / mag_chord.x;
    mag_scale.y = avg_chord / mag_chord.y;
    mag_scale.z = avg_chord / mag_chord.z;
    bmm150_offset_save();

    Serial.printf("\n calibrate finish ... \r\n");
//    Serial.printf("mag_max.x: %.2f x_min: %.2f \t", mag_max.x, mag_min.x);
//    Serial.printf("y_max: %.2f y_min: %.2f \t", mag_max.y, mag_min.y);
//    Serial.printf("z_max: %.2f z_min: %.2f \r\n", mag_max.z, mag_min.z);
}

void BMM150class::Init(void)
{
    Wire.begin(21, 22, 400000);
    
    M5.Lcd.fillScreen(BLACK);
    M5.Lcd.setTextColor(WHITE);

    if (bmm150_initialization() != BMM150_OK)
    {
        M5.Lcd.setCursor(0, 10);
        M5.Lcd.print("BMM150 init failed");
        for (;;)
        {
            delay(100);
        }
    }
    
    bmm150_offset_load();

    Serial.printf("MID X : %.2f \t MID Y : %.2f \t MID Z : %.2f \n", mag_offset.x, mag_offset.y, mag_offset.z);
    Serial.printf("SCALE X : %.2f \t SCALE Y : %.2f \t SCALE Z : %.2f \n", mag_scale.x, mag_scale.y, mag_scale.z);
    M5.Lcd.setCursor(0, 10);
    M5.Lcd.printf("offset X, Y, Z: %5.4f %5.4f %5.4f \n", mag_offset.x, mag_offset.y, mag_offset.z);
    M5.Lcd.setCursor(0, 20);
    M5.Lcd.printf("scale  X, Y, Z: %5.4f %5.4f %5.4f \n", mag_scale.x, mag_scale.y, mag_scale.z);

    delay(3000); // see preset data 3 sec.
}

void BMM150class::getMagnetData(float *mx, float *my, float *mz)
{
    bmm150_read_mag_data(&dev);
//    float head_dir = atan2(dev.data.x - mag_offset.x, dev.data.y - mag_offset.y) * 180.0 / M_PI;
    *mx = dev.data.x;
    *my = dev.data.y;
    *mz = dev.data.z;
}

void BMM150class::getMagnetOffset(float *mx, float *my, float *mz)
{
    bmm150_offset_load();
    *mx = mag_offset.x;
    *my = mag_offset.y;
    *mz = mag_offset.z;
}

void BMM150class::getMagnetScale(float *mx, float *my, float *mz)
{
    bmm150_offset_load();
    *mx = mag_scale.x;
    *my = mag_scale.y;
    *mz = mag_scale.z;
}
