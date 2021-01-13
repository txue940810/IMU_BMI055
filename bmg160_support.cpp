#include "spi_support.h"
#include "bmg160_support.h"
#include "bma2x2_support.h"
#include "bmg160.h"
#include "bma2x2.h"

/*---------------------------------------------------------------------------*
 *  struct bmg160_t parameters can be accessed by using bmg160
 *  bmg160_t having the following parameters
 *  Bus write function pointer: BMG160_WR_FUNC_PTR
 *  Bus read function pointer: BMG160_RD_FUNC_PTR
 *  Burst read function pointer: BMG160_BRD_FUNC_PTR
 *  Delay function pointer: delay_msec
 *  I2C address: dev_addr
 *  Chip id of the sensor: chip_id
 *-------------------------------------------------------------------------*/
struct bmg160_t bmg160;
float acc_init_data[3];
extern float gyro_init_bias[3];
extern float init_temp;

s32 bmg160_gyro_readout(struct bmg160_data_t *gyro_xyz_data);

/*---------------------------------------------------------------------------*
 *  The following function is used to map the SPI bus read, write and delay
 *  with global structure bmg160_t
 *--------------------------------------------------------------------------*/

/************** I2C/SPI buffer length ******/
#define I2C_BUFFER_LEN 8
#define SPI_BUFFER_LEN 20
#define MASK_DATA1     0xFF
#define MASK_DATA2     0x80
#define MASK_DATA3     0x7F


s8 BMG160_SPI_bus_read(u8 dev_addr, u8 reg_addr, u8 *reg_data, u8 cnt)
{
    s32 iError = BMG160_INIT_VALUE;
    u8 array[SPI_BUFFER_LEN] = { MASK_DATA1 };
    u8 stringpos;

    /*  For the SPI mode only 7 bits of register addresses are used.
     * The MSB of register address is declared the bit what functionality it is
     * read/write (read as 1/write as BMG160_INIT_VALUE)
     */

    /*read routine is initiated register address is mask with 0x80*/
    array[BMG160_INIT_VALUE] = reg_addr | MASK_DATA2;
    
    iError = BMG_SPI_READ_WRITE_STRING(array, cnt+1);
     

    for (stringpos = BMG160_INIT_VALUE; stringpos < cnt; stringpos++)
    {
        *(reg_data + stringpos) = array[stringpos + BMG160_GEN_READ_WRITE_DATA_LENGTH];
    }

    return (s8)iError;
}

/*  \Brief: The function is used as SPI bus write
 *  \Return : Status of the SPI write
 *  \param dev_addr : The device address of the sensor
 *  \param reg_addr : Address of the first register, will data is going to be written
 *  \param reg_data : It is a value hold in the array,
 *      will be used for write the value into the register
 *  \param cnt : The no of byte of data to be write
 */
s8 BMG160_SPI_bus_write(u8 dev_addr, u8 reg_addr, u8 *reg_data, u8 cnt)
{
    s32 iError = BMG160_INIT_VALUE;
    u8 array[SPI_BUFFER_LEN * C_BMG160_TWO_U8X];
    u8 stringpos = BMG160_INIT_VALUE;

    for (stringpos = BMG160_INIT_VALUE; stringpos < cnt; stringpos++)
    {
        /* the operation of (reg_addr++)&0x7F done: because it ensure the
         * BMG160_INIT_VALUE and 1 of the given value
         * It is done only for 8bit operation
         */
        array[stringpos * C_BMG160_TWO_U8X] = (reg_addr++) & MASK_DATA3;
        array[stringpos * C_BMG160_TWO_U8X + BMG160_GEN_READ_WRITE_DATA_LENGTH] = *(reg_data + stringpos);
    }


    iError = BMG_SPI_WRITE(array, cnt); //the lowerest API for SPI

    return (s8)iError;
}

/*  Brief : The delay routine
 *  \param : delay in ms
 */
void BMG160_delay_msek(u32 msek)
{
    /*Here you can write your own delay routine*/
    delay(msek);
}

/*--------------------------------------------------------------------------*
 *  By using bmg160 the following structure parameter can be accessed
 *  Bus write function pointer: BMG160_WR_FUNC_PTR
 *  Bus read function pointer: BMG160_RD_FUNC_PTR
 *  Delay function pointer: delay_msec
 *-------------------------------------------------------------------------*/
s8 bmg160_SPI_routine(void)
{
    bmg160.bus_write = BMG160_SPI_bus_write;
    bmg160.bus_read = BMG160_SPI_bus_read;
    bmg160.delay_msec = BMG160_delay_msek;

    return BMG160_INIT_VALUE;
}

/* This function is an example for reading sensor data
 *  \param: None
 *  \return: communication result
 */
s32 bmg160_gyro_readout_config(void)
{
    /* Gyro */
    /* variable used for read the sensor data*/
    s16 v_gyro_datax_s16, v_gyro_datay_s16, v_gyro_dataz_s16 = BMG160_INIT_VALUE;

    /* structure used for read the sensor data - xyz*/
    struct bmg160_data_t data_gyro;

    /* structure used for read the sensor data - xyz and interrupt status*/
    struct bmg160_data_t gyro_xyz_data;

    /* variable used for read the gyro bandwidth data*/
    u8 v_gyro_value_u8 = BMG160_INIT_VALUE;

    /* variable used for set the gyro bandwidth data*/
    u8 v_bw_u8;

    /* variable used for set the gyro range*/
    u8 v_range_u8;

    /* result of communication results*/
    s32 com_rslt;

    /*-------------------------------------------------------------------------*
    *********************** START INITIALIZATION ***********************
    *-------------------------------------------------------------------------*/

    /*  Based on the user need configure I2C or SPI interface.
     *  It is example code to explain how to use the bmg160 API
     */
    bmg160_SPI_routine(); 
    /*--------------------------------------------------------------------------*
     *  This function used to assign the value/reference of
     *  the following parameters
     *  Gyro I2C address
     *  Bus Write
     *  Bus read
     *  Gyro Chip id
     *----------------------------------------------------------------------------*/
    com_rslt = bmg160_init(&bmg160); 
    Serial.println("Gyro Initialized");
    /*----------------------------------------------------------------------------*/

    /*  For initialization it is required to set the mode of the sensor as "NORMAL"
     *  data acquisition/read/write is possible in this mode
     *  by using the below API able to set the power mode as NORMAL
     *  NORMAL mode set from the register 0x11 and 0x12
     *  While sensor in the NORMAL mode idle time of at least 2us(micro seconds)
     *  is required to write/read operations
     *  0x11 -> bit 5,7 -> set value as BMG160_INIT_VALUE
     *  0x12 -> bit 6,7 -> set value as BMG160_INIT_VALUE
     *  Note:
     *      If the sensor is in the fast power up mode idle time of least
     *      450us(micro seconds) required for write/read operations
     */

    /*-------------------------------------------------------------------------*/
    /* Set the gyro power mode as NORMAL*/
   // com_rslt += bmg160_set_power_mode(BMG160_MODE_NORMAL);
    u8 powerset= 0x00;
    BMG160_SPI_bus_write(0x00,0x11,&powerset,1);
    BMG160_SPI_bus_write(0x00,0x12,&powerset,1);
    
    u8 v_power_mode=0xFF;
    com_rslt += bmg160_get_power_mode(&v_power_mode);
    if (v_power_mode==BMG160_MODE_NORMAL)
    {
      Serial.println("Gyro Power Mode Set Normal");
    }
    else
    {
      Serial.print("Gyro Power Mode Set Failed");Serial.println(v_power_mode,HEX);
      while(1);
    }
    delay(10);
    /*--------------------------------------------------------------------------*
    ************************* END INITIALIZATION ******************************
    *--------------------------------------------------------------------------*/

    /*------------------------------------------------------------------------*
     ************************* START GET and SET FUNCTIONS DATA ***************
     *--------------------------------------------------------------------------*/

    /* This API used to Write the bandwidth of the gyro sensor
     * input value have to be give 0x10 bit BMG160_INIT_VALUE to 3
     * The bandwidth set from the register
     */
    v_range_u8=BMG160_RANGE_2000 ;
    com_rslt += bmg160_set_range_reg(v_range_u8);
    Serial.println("Gyro Set Range 2000 dps");
    
    com_rslt += bmg160_get_range_reg(&v_gyro_value_u8);
    if (v_gyro_value_u8==v_range_u8)
      Serial.println("Gyro Range Set Success");
    else{
      Serial.print("Gyro Range Set Failed: ");Serial.println(v_gyro_value_u8,HEX);
      while(1);
      }
      
    v_bw_u8 = BMG160_BW_32_HZ ; /* set gyro bandwidth of 116Hz update rate 1000Hz*/
    com_rslt += bmg160_set_bw(v_bw_u8);
    Serial.println("Gyro BW Set 32Hz and Update Set 100Hz");

    /* This API used to read back the written value of bandwidth for gyro*/
    com_rslt += bmg160_get_bw(&v_gyro_value_u8);
    if (v_gyro_value_u8==v_bw_u8){
      Serial.println("Gyro BW Set Success");
      }
    else{
      Serial.print("Gyro BW Set Failed: ");Serial.println(v_gyro_value_u8,HEX);
      while(1);
      }

    u8 temp=0x01;
    bmg160_get_intr_output_type(0x00, &temp);
    bmg160_set_intr_data(0x00, 0x01);
    com_rslt += bmg160_set_data_enable(0x01); // enable the interrupt

//    com_rslt += bmg160_set_slow_offset_thres(0x02);// set slow compensation threshold as 0.5dps
//    com_rslt += bmg160_set_slow_offset_durn(0x03);// 320ms
//    com_rslt += bmg160_set_slow_offset_enable_axis(0x00, 0x01);
//    com_rslt += bmg160_set_slow_offset_enable_axis(0x01, 0x01);
//    com_rslt += bmg160_set_slow_offset_enable_axis(0x02, 0x01);
//    temp=0xDF;
//    com_rslt += BMG160_SPI_bus_write(0x00,0x31,&temp,1);
//    temp=0x00;
//    com_rslt += BMG160_SPI_bus_read(0x00,0x31,&temp,1);
//    Serial.print("Slow Comepensation Config");Serial.println(temp,HEX);

    float mean_gx=0.0,mean_gy=0.0,mean_gz=0.0,mean_temp=0.0;
    float mean_ax=0.0,mean_ay=0.0,mean_az=0.0;
    for (int count=0;count<300;count++)
    {
      float acc[3],gyro[3],temp,rt_acc[3];
      BMI055_ReadOut(acc, rt_acc, gyro, &temp,0x01);

      mean_temp+=temp;
      
      mean_gx+=gyro[0];
      mean_gy+=gyro[1];
      mean_gz+=gyro[2];

      mean_ax+=acc[0];
      mean_ay+=acc[1];
      mean_az+=acc[2];
      
      delay(10);
    }  
    mean_gx/=300.0;mean_gy/=300.0;mean_gz/=300.0;mean_temp/=300.0;
    mean_ax/=300.0;mean_ay/=300.0;mean_az/=300.0;
    
    if(sqrt(mean_gx*mean_gx+mean_gy*mean_gy+mean_gz*mean_gz)>0.20)
    {
      Serial.print("Gyro is not still");
      Serial.println("Gyro calibration is not completed!");
      Serial.print(mean_gx);Serial.print(':');
      Serial.print(mean_gy);Serial.print(':');
      Serial.print(mean_gz);Serial.println(':');
      while(1);
    }
    else
    {
      gyro_init_bias[0]=mean_gx;
      gyro_init_bias[1]=mean_gy;
      gyro_init_bias[2]=mean_gz;
      
      acc_init_data[0]=mean_ax;
      acc_init_data[1]=mean_ay;
      acc_init_data[2]=mean_az;
      
      init_temp=mean_temp;
      
      Serial.println("Gyro calibration completed!");
      Serial.print(mean_gx);Serial.print(':');
      Serial.print(mean_gy);Serial.print(':');
      Serial.print(mean_gz);Serial.println(':');
    }
    
    return com_rslt;
}

s32 bmg160_gyro_readout(struct bmg160_data_t *gyro_xyz_data)
{
     s32 com_rslt = bmg160_get_data_xyz(gyro_xyz_data); 
     return 0;
  }
  
