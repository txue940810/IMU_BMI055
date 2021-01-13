/**
* Copyright (c) 2020 Bosch Sensortec GmbH. All rights reserved.
*
* BSD-3-Clause
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions are met:
*
* 1. Redistributions of source code must retain the above copyright
*    notice, this list of conditions and the following disclaimer.
*
* 2. Redistributions in binary form must reproduce the above copyright
*    notice, this list of conditions and the following disclaimer in the
*    documentation and/or other materials provided with the distribution.
*
* 3. Neither the name of the copyright holder nor the names of its
*    contributors may be used to endorse or promote products derived from
*    this software without specific prior written permission.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
* "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
* LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
* FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
* COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
* INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
* (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
* SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
* HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
* STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING
* IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
* POSSIBILITY OF SUCH DAMAGE.
*
* @file bma2x2_support.c
* @date 29/01/2020
* @version  2.0.8
*
*/
/*---------------------------------------------------------------------------*/
/* Includes*/
/*---------------------------------------------------------------------------*/

#include "bma2x2_support.h"
#include "bma2x2.h"
#include "bmg160.h"
#include "spi_support.h"

struct bma2x2_t bma2x2;
/*----------------------------------------------------------------------------*
*  V_BMA2x2RESOLUTION_u8R used for selecting the accelerometer resolution
 *  12 bit
 *  14 bit
 *  10 bit
*----------------------------------------------------------------------------*/
extern u8 V_BMA2x2RESOLUTION_u8R;
/* This function is an example for reading sensor data
 *  \param: None
 *  \return: communication result
 */
s8 BMA2x2_SPI_bus_write(u8 dev_addr, u8 reg_addr, u8 *reg_data, u8 cnt);
s8 BMA2x2_SPI_bus_read(u8 dev_addr, u8 reg_addr, u8 *reg_data, u8 cnt);

s32 bma2x2_acc_readout_config(void)
{
	/* Local variable used to assign the bandwidth value*/
	u8 bw_value_u8 = BMA2x2_INIT_VALUE;
	/* Local variable used to set the bandwidth value*/
	u8 banwid = BMA2x2_INIT_VALUE;
	/* status of communication*/
	s32 com_rslt = ERROR;


/*********************** START INITIALIZATION ************************
  *	Based on the user need configure I2C or SPI interface.
  *	It is example code to explain how to use the bma2x2 API*/
	bma2x2_SPI_routine(); 
 /*--------------------------------------------------------------------------*
 *  This function used to assign the value/reference of
 *	the following parameters
 *	I2C address
 *	Bus Write
 *	Bus read
 *	Chip id
 *-------------------------------------------------------------------------*/
	com_rslt = bma2x2_init(&bma2x2);
  Serial.println("Acc Initialized");

/*	For initialization it is required to set the mode of
 *	the sensor as "NORMAL"
 *	NORMAL mode is set from the register 0x11 and 0x12
 *	0x11 -> bit 5,6,7 -> set value as 0
 *	0x12 -> bit 5,6 -> set value as 0
 *	data acquisition/read/write is possible in this mode
 *	by using the below API able to set the power mode as NORMAL
 *	For the Normal/standby/Low power 2 mode Idle time
		of at least 2us(micro seconds)
 *	required for read/write operations*/
	/* Set the power mode as NORMAL*/
	com_rslt += bma2x2_set_power_mode(BMA2x2_MODE_NORMAL);
  u8 acc_mode=0xFF;
  bma2x2_get_power_mode(&acc_mode);
  if (acc_mode==BMA2x2_MODE_NORMAL)
  {
    Serial.println("Acc Power Mode Set Normal");
  }
  else
  {
    Serial.print("Acc Power Mode Set Failed");Serial.println(acc_mode,HEX);
  }
  delay(100);
/*	Note:
	* For the Suspend/Low power1 mode Idle time of
		at least 450us(micro seconds)
	* required for read/write operations*/

/************************* END INITIALIZATION *************************/

/*------------------------------------------------------------------------*
************************* START GET and SET FUNCTIONS DATA ****************
*---------------------------------------------------------------------------*/
	/* This API used to Write the bandwidth of the sensor input
	value have to be given

	bandwidth is set from the register 0x10 bits from 1 to 4*/
	//+-8g range
  u8 acc_range=BMA2x2_RANGE_8G;
  com_rslt += bma2x2_set_range(acc_range);
  Serial.println("Acc Set Range 8G");
  u8 ret=0x00;
  com_rslt += bma2x2_get_range(&ret);
  if (ret==acc_range){
    Serial.println("Acc Range Set Sucess");
  }
  else{
    Serial.print("Acc Range Set Failed");Serial.println(ret,HEX);
    while(1);
  }
  
	bw_value_u8 = BMA2x2_BW_62_50HZ;/* set bandwidth of 62.5Hz and update 125Hz*/
	com_rslt += bma2x2_set_bw(bw_value_u8);
  Serial.println("Acc BW Set 62.5Hz and Update Set 125Hz");

	/* This API used to read back the written value of bandwidth*/
	com_rslt += bma2x2_get_bw(&banwid);
  if (banwid==bw_value_u8){
    Serial.println("Acc BW Set Sucess");
  }
  else{
    Serial.print("Acc BW Set Failed");Serial.println(banwid,HEX);
    while(1);
  }
  
  u8 acc_temp=0x00;
  com_rslt += bma2x2_get_intr_output_type(0x00,&acc_temp);
  com_rslt += bma2x2_set_new_data(0x00,0x01);
  com_rslt += bma2x2_set_intr_enable(0x04,0x01);

/*-----------------------------------------------------------------*
************************* END GET and SET FUNCTIONS ****************
*-------------------------------------------------------------------*/
return com_rslt;
}

s32 bma2x2_acc_readout(struct bma2x2_accel_data_temp *sample_xyzt)
{
  s32 com_rslt=0;
  com_rslt += bma2x2_read_accel_xyzt(sample_xyzt);

return com_rslt;
}


/*---------------------------------------------------------------------------*
 * The following function is used to map the SPI bus read, write and delay
 * with global structure bma2x2_t
 *--------------------------------------------------------------------------*/
s8 bma2x2_SPI_routine(void)
{
/*--------------------------------------------------------------------------*
 *  By using bma2x2 the following structure parameter can be accessed
 *	Bus write function pointer: BMA2x2_WR_FUNC_PTR
 *	Bus read function pointer: BMA2x2_RD_FUNC_PTR
 *	Delay function pointer: delay_msec
 *--------------------------------------------------------------------------*/

	bma2x2.bus_write = BMA2x2_SPI_bus_write;
	bma2x2.bus_read = BMA2x2_SPI_bus_read;
	bma2x2.delay_msec = BMA2x2_delay_msek;

	return BMA2x2_INIT_VALUE;
}

/************** I2C/SPI buffer length ******/
#define	I2C_BUFFER_LEN 8
#define SPI_BUFFER_LEN 20
#define BMA2x2_BUS_READ_WRITE_ARRAY_INDEX	1
#define BMA2x2_SPI_BUS_WRITE_CONTROL_BYTE	0x7F
#define BMA2x2_SPI_BUS_READ_CONTROL_BYTE	0x80

/*	\Brief: The function is used as SPI bus read
 *	\Return : Status of the SPI read
 *	\param dev_addr : The device address of the sensor
 *	\param reg_addr : Address of the first register,
 *          will data is going to be read
 *	\param reg_data : This data read from the sensor,
 *          which is hold in an array
 *	\param cnt : The no of byte of data to be read */
s8 BMA2x2_SPI_bus_read(u8 dev_addr, u8 reg_addr, u8 *reg_data, u8 cnt)
{
	s32 iError = BMA2x2_INIT_VALUE;
	u8 array[SPI_BUFFER_LEN] = {0xFF};
	u8 stringpos;
	/*	For the SPI mode only 7 bits of register addresses are used.
	The MSB of register address is declared the bit what functionality it is
	read/write (read as 1/write as 0)*/
	array[BMA2x2_INIT_VALUE] = reg_addr|BMA2x2_SPI_BUS_READ_CONTROL_BYTE;
	/*read routine is initiated register address is mask with 0x80*/

	iError = BMA_SPI_READ_WRITE_STRING(array, cnt+1); 

	for (stringpos = BMA2x2_INIT_VALUE; stringpos < cnt; stringpos++) {
		*(reg_data + stringpos) = array[stringpos +
		BMA2x2_BUS_READ_WRITE_ARRAY_INDEX];
	}
	return (s8)iError;
}



s8 BMA2x2_SPI_bus_write(u8 dev_addr, u8 reg_addr, u8 *reg_data, u8 cnt)
{
	s32 iError = BMA2x2_INIT_VALUE;
	u8 array[SPI_BUFFER_LEN * 2];
	u8 stringpos = BMA2x2_INIT_VALUE;

	for (stringpos = BMA2x2_INIT_VALUE; stringpos < cnt; stringpos++) {
		/* the operation of (reg_addr++)&0x7F done:
		because it ensure the
		0 and 1 of the given value
		It is done only for 8bit operation*/
		array[stringpos * 2] = (reg_addr++) &
		BMA2x2_SPI_BUS_WRITE_CONTROL_BYTE;
		array[stringpos * 2 + BMA2x2_BUS_READ_WRITE_ARRAY_INDEX] =
		*(reg_data + stringpos);
	}
   
	iError = BMA_SPI_WRITE(array, cnt);
  
	return (s8)iError;
}




/*	Brief : The delay routine
 *	\param : delay in ms
*/
void BMA2x2_delay_msek(u32 msek)
{
	/*Here you can write your own delay routine*/
 delay(msek);
}
