#ifndef __BMG160_SUPPORT_H__
#define __BMG160_SUPPORT_H__


#include "Arduino.h"
// Arduino Data Type Edited by Tao

#define BMA_SelectPin 53
#define BMG_SelectPin 49
#include <stdint.h>

typedef uint8_t u8;
typedef uint16_t u16;
typedef uint32_t u32;
typedef uint64_t u64;
typedef int8_t s8;
typedef int16_t s16;
typedef int32_t s32;
typedef int64_t s64;

s8 BMA_SPI_READ_WRITE_STRING(u8 *data, u8 cnt);
s8 BMA_SPI_WRITE(u8 *data, u8 cnt);

s8 BMG_SPI_READ_WRITE_STRING(u8 *data, u8 cnt);
s8 BMG_SPI_WRITE(u8 *data, u8 cnt);

s8 BMI055_ReadOut(float *acc_data, float *realtime_acc, float *gyro_data, float *imu_temp, u8 iscal);


#endif
