#ifndef __BMG160_SUPPORT_H__
#define __BMG160_SUPPORT_H__

#include "spi_support.h"
//typedef unsigned char u8;
//typedef unsigned int u16;
//typedef unsigned long u32;
//typedef unsigned long long u64;
//typedef signed char s8;
//typedef signed int s16;
//typedef signed long s32;
//typedef signed long long s64;


s8 bmg160_SPI_routine(void);
 
s8 BMG160_SPI_bus_write(u8 dev_addr, u8 reg_addr, u8 *reg_data, u8 cnt);

/*  \Brief: The function is used as SPI bus read
 *  \Return : Status of the SPI read
 *  \param dev_addr : The device address of the sensor
 *  \param reg_addr : Address of the first register, will data is going to be read
 *  \param reg_data : This data read from the sensor, which is hold in an array
 *  \param cnt : The no of byte of data to be read
 */
s8 BMG160_SPI_bus_read(u8 dev_addr, u8 reg_addr, u8 *reg_data, u8 cnt);

/*
 * \Brief: SPI/I2C init routine
 */



/********************End of I2C/SPI function declarations***********************/

/*---------------------------------------------------------------------------*
*  The following function is used to provide necessary delay wait time in the API
*---------------------------------------------------------------------------*/

/*  Brief : The delay routine
 *  \param : delay in ms
 */
void BMG160_delay_msek(u32 msek);

s32 bmg160_gyro_readout_config(void);

/* This function is an example for reading sensor data
 *  \param: None
 *  \return: communication result
 */
s32 bmg160_gyro_readout(struct bmg160_data_t *gyro_xyz_data);
#endif
