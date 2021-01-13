#include "spi_support.h"
#include "spi.h"
#include "Arduino.h"
#include "bmg160.h"
#include "bma2x2.h"
#include "bma2x2_support.h"
#include "bmg160_support.h"

s8 BMA_SPI_READ_WRITE_STRING(u8 *data, u8 cnt)
{
  SPI.beginTransaction(SPISettings(8000000, MSBFIRST, SPI_MODE0));
  digitalWrite(BMA_SelectPin, LOW);
  delayMicroseconds(1);
  
  for (u8 i=0;i<cnt;i++)
  {
     *data=SPI.transfer(*data);
     data++;
  } 

  digitalWrite(BMA_SelectPin, HIGH);
  SPI.endTransaction();

  return 0;
}

s8 BMA_SPI_WRITE(u8 *data, u8 cnt)
{

  for (u8 i=0;i<cnt;i++)
  {
    SPI.beginTransaction(SPISettings(8000000, MSBFIRST, SPI_MODE0));
    digitalWrite(BMA_SelectPin, LOW);
    delayMicroseconds(1);
    
    SPI.transfer(data[i*2]);
    SPI.transfer(data[i*2+1]);

    digitalWrite(BMA_SelectPin, HIGH);
    SPI.endTransaction();
    delayMicroseconds(2);
  }

  return 0;
}

s8 BMG_SPI_READ_WRITE_STRING(u8 *data, u8 cnt)
{
  SPI.beginTransaction(SPISettings(8000000, MSBFIRST, SPI_MODE0));
  digitalWrite(BMG_SelectPin, LOW);
  delayMicroseconds(1);
  
  for (u8 i=0;i<cnt;i++)
  {
     *data=SPI.transfer(*data);
     data++;
  } 

  digitalWrite(BMG_SelectPin, HIGH);
  SPI.endTransaction();

  return 0;
}

s8 BMG_SPI_WRITE(u8 *data, u8 cnt)
{

  for (u8 i=0;i<cnt;i++)
  {
    SPI.beginTransaction(SPISettings(8000000, MSBFIRST, SPI_MODE0));
    digitalWrite(BMG_SelectPin, LOW);
    delayMicroseconds(1);
    
    SPI.transfer(data[i*2]);
    SPI.transfer(data[i*2+1]);

    digitalWrite(BMG_SelectPin, HIGH);
    SPI.endTransaction();
    delayMicroseconds(2);
  }

  return 0;
}

struct bma2x2_accel_data_temp acc_xyzt;
struct bmg160_data_t gyro_xyz;
s32 bmg160_gyro_readout(struct bmg160_data_t *gyro_xyz_data);

double filter_a[4]={-2.36951301,2.31398841,-1.05466541,0.18737949};
double filter_b[5]={0.00482434,0.01929737,0.02894606,0.01929737,0.00482434}; //10Hz 4  order
double filter_axi[5]={0.0},filter_axo[5]={0.0};
double filter_ayi[5]={0.0},filter_ayo[5]={0.0};
double filter_azi[5]={0.0},filter_azo[5]={0.0};

float unit_acc=8.0/(4096.0/2.0);
float unit_gyro=2000.0/(32767.0);

float gyro_init_bias[3]={0.0};
float acc_init_bias[3]={0.02,0.077,-0.015}; 
float init_temp=0.0;

s8 BMI055_ReadOut(float *acc_data, float *realtime_acc, float *gyro_data, float *imu_temp, u8 iscal)
{
  s32 rs=bmg160_gyro_readout(&gyro_xyz);
  rs=bma2x2_acc_readout(&acc_xyzt);

  filter_axi[4]=filter_axi[3]; 
  filter_axi[3]=filter_axi[2];
  filter_axi[2]=filter_axi[1];
  filter_axi[1]=filter_axi[0];
  filter_axi[0]=acc_xyzt.x*unit_acc;
  filter_ayi[4]=filter_ayi[3]; 
  filter_ayi[3]=filter_ayi[2];
  filter_ayi[2]=filter_ayi[1];
  filter_ayi[1]=filter_ayi[0];
  filter_ayi[0]=acc_xyzt.y*unit_acc;
  filter_azi[4]=filter_azi[3]; 
  filter_azi[3]=filter_azi[2];
  filter_azi[2]=filter_azi[1];
  filter_azi[1]=filter_azi[0];
  filter_azi[0]=acc_xyzt.z*unit_acc;
  
  float temp_x=0.0,temp_y=0.0,temp_z=0.0;
  for(unsigned char k=0;k<5;k++)
  {
    temp_x+=filter_axi[k]*filter_b[k];
    temp_y+=filter_ayi[k]*filter_b[k];
    temp_z+=filter_azi[k]*filter_b[k];
  } 
  for (unsigned char k=0;k<4;k++)
  {
    temp_x-=filter_axo[k+1]*filter_a[k];
    temp_y-=filter_ayo[k+1]*filter_a[k];
    temp_z-=filter_azo[k+1]*filter_a[k];
  }
  
  filter_axo[4]=filter_axo[3];
  filter_axo[3]=filter_axo[2];
  filter_axo[2]=filter_axo[1];
  filter_axo[1]=temp_x;
  filter_ayo[4]=filter_ayo[3];
  filter_ayo[3]=filter_ayo[2];
  filter_ayo[2]=filter_ayo[1];
  filter_ayo[1]=temp_y;
  filter_azo[4]=filter_azo[3];
  filter_azo[3]=filter_azo[2];
  filter_azo[2]=filter_azo[1];
  filter_azo[1]=temp_z;
  
  *imu_temp=acc_xyzt.temp*0.5+23.0;

  if (iscal==0x01)
  {
    acc_data[0]=acc_xyzt.x*unit_acc-acc_init_bias[0]-(*imu_temp-25.0)*0.00013;
    acc_data[1]=acc_xyzt.y*unit_acc-acc_init_bias[1]+(*imu_temp-25.0)*0.00035 ;
    acc_data[2]=acc_xyzt.z*unit_acc-acc_init_bias[2]+(*imu_temp-25.0)*0.00020;

    realtime_acc[0]=acc_data[0];
    realtime_acc[1]=acc_data[1];
    realtime_acc[2]=acc_data[2];

    gyro_data[0]=gyro_xyz.datax*unit_gyro;
    gyro_data[1]=gyro_xyz.datay*unit_gyro;
    gyro_data[2]=gyro_xyz.dataz*unit_gyro;
    
    }
  else
  { 
    acc_data[0]=temp_x-acc_init_bias[0]-(*imu_temp-25.0)*0.00013;
    acc_data[1]=temp_y-acc_init_bias[1]+(*imu_temp-25.0)*0.00040;
    acc_data[2]=temp_z-acc_init_bias[2]+(*imu_temp-25.0)*0.00020;

    realtime_acc[0]=acc_xyzt.x*unit_acc-acc_init_bias[0]-(*imu_temp-25.0)*0.00013;
    realtime_acc[1]=acc_xyzt.y*unit_acc-acc_init_bias[1]+(*imu_temp-25.0)*0.00035 ;
    realtime_acc[2]=acc_xyzt.z*unit_acc-acc_init_bias[2]+(*imu_temp-25.0)*0.00020;
    
    gyro_data[0]=(gyro_xyz.datax*unit_gyro-gyro_init_bias[0])*1.00 +(*imu_temp-init_temp)*(0.16/30.0);  //0.9974
    gyro_data[1]=(gyro_xyz.datay*unit_gyro-gyro_init_bias[1])*1.00 +(*imu_temp-init_temp)*(0.25/30.0);  //0.9963
    gyro_data[2]=(gyro_xyz.dataz*unit_gyro-gyro_init_bias[2])*1.00 +(*imu_temp-init_temp)*(0.05/30.0);  //1.0012

    
    if (gyro_data[0]>0)
      gyro_data[0]*=0.997348;  //1.000594
    if (gyro_data[0]<0)
      gyro_data[0]*=0.996987; //0.999446
    
    if (gyro_data[1]>0)
      gyro_data[1]*=0.995049;  //1.000215
    if (gyro_data[1]<0)
      gyro_data[1]*=0.994937; //0.999102
    
    if (gyro_data[2]>0)
      gyro_data[2]*=1.001565 ;  //1.000940
    if (gyro_data[2]<0)
      gyro_data[2]*=1.001496;  //0.999744 
    }

  return 0;
}
