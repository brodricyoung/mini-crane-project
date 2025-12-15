#include "swing_sensor.h"
#include "main.h"
#include <math.h>

SwingSensor_t SwingSensor;

static I2C_HandleTypeDef *IMU_I2C;

// Adjust if your module uses another address
#define BNO055_ADDR        (0x28 << 1)

// Register for pitch (you can change to roll if needed)
#define REG_EULER_PITCH_L   0x1E   // pitch LSB
#define REG_EULER_PITCH_M   0x1F   // pitch MSB

// Filter coefficient (0.1–0.3 works well)
#define FILTER_ALPHA        0.15f

// Swing detection threshold
#define SWING_DELTA_THRESH  0.35f  // degrees per sample

// -----------------------
// Low-level IMU read
// -----------------------
static float IMU_ReadPitch(void)
{
    uint8_t buf[2];

    if (HAL_I2C_Mem_Read(IMU_I2C, BNO055_ADDR, REG_EULER_PITCH_L,
                         I2C_MEMADD_SIZE_8BIT, buf, 2, 10) != HAL_OK)
    {
        return SwingSensor.raw_angle;  // return last known if error
    }

    int16_t raw = (int16_t)((buf[1] << 8) | buf[0]);
    return (float)raw / 16.0f;  // convert to degrees
}

// -----------------------
// API IMPLEMENTATION
// -----------------------
void SwingSensor_Init(I2C_HandleTypeDef *hi2c)
{
    IMU_I2C = hi2c;
    SwingSensor.angle = 0.0f;
    SwingSensor.raw_angle = 0.0f;
    SwingSensor.angle_offset = 0.0f;
    SwingSensor.offset_valid = false;
}

void SwingSensor_CalibrateZero(void)
{
    SwingSensor.raw_angle = IMU_ReadPitch();
    SwingSensor.angle_offset = SwingSensor.raw_angle;
    SwingSensor.offset_valid = true;
}

void SwingSensor_Update(void)
{
    float new_raw = IMU_ReadPitch();
    SwingSensor.raw_angle = new_raw;

    float angle_rel = new_raw;

    if (SwingSensor.offset_valid)
        angle_rel -= SwingSensor.angle_offset;

    // EMA filtering
    SwingSensor.angle =
        (FILTER_ALPHA * angle_rel) + ((1.0f - FILTER_ALPHA) * SwingSensor.angle);
}

float SwingSensor_GetAngle(void)
{
    return SwingSensor.angle;
}

bool SwingSensor_IsSwinging(void)
{
    static float prev = 0;
    float now = SwingSensor.angle;

    float delta = fabsf(now - prev);
    prev = now;

    return (delta > SWING_DELTA_THRESH);
}
