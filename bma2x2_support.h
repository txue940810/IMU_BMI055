#ifndef __BMA2x2_SUPPORT_H__
#define __BMA2x2_SUPPORT_H__
#include "spi_support.h"
//#include <stdint.h>
//
//typedef uint8_t u8;
//typedef uint16_t u16;
//typedef uint32_t u32;
//typedef uint64_t u64;
//typedef int8_t s8;
//typedef int16_t s16;
//typedef int32_t s32;
//typedef int64_t s64;

/*----------------------------------------------------------------------------*
*  The following functions are used for reading and writing of
* sensor data using I2C or SPI communication
*----------------------------------------------------------------------------*/
s8 BMA2x2_SPI_bus_write(u8 dev_addr, u8 reg_addr, u8 *reg_data, u8 cnt);
/* \Brief: The function is used as SPI bus read
 * \Return : Status of the SPI read
 * \param dev_addr : The device address of the sensor
 * \param reg_addr : Address of the first register,
 *   will data is going to be read
 * \param reg_data : This data read from the sensor, which is hold in an array
 * \param cnt : The no of byte of data to be read */
s8 BMA2x2_SPI_bus_read(u8 dev_addr, u8 reg_addr, u8 *reg_data, u8 cnt);
/*
 * \Brief: SPI/I2C init routine
*/
s8 bma2x2_SPI_routine(void);
/********************End of I2C/SPI function declarations*******************/
/*  Brief : The delay routine
 *  \param : delay in ms
 */
void BMA2x2_delay_msek(u32 msek);
/*!
 *  @brief This function is an example for delay
 *  @param : None
 *  @return : communication result
 */
s32 bma2x2_acc_readout_config(void);
/*----------------------------------------------------------------------------*
*  struct bma2x2_t parameters can be accessed by using bma2x2
 *  bma2x2_t having the following parameters
 *  Bus write function pointer: BMA2x2_WR_FUNC_PTR
 *  Bus read function pointer: BMA2x2_RD_FUNC_PTR
 *  Burst read function pointer: BMA2x2_BRD_FUNC_PTR
 *  Delay function pointer: delay_msec
 *  I2C address: dev_addr
 *  Chip id of the sensor: chip_id
 *---------------------------------------------------------------------------*/

s32 bma2x2_acc_readout(struct bma2x2_accel_data_temp *sample_xyzt);


#endif
