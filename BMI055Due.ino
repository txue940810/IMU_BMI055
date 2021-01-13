#include <Arduino.h>
#include <SPI.h>
#include <arduino-timer.h>
#include "spi_support.h"
#include "bma2x2.h"
#include "bma2x2_support.h"
#include "bmg160.h"
#include "bmg160_support.h"
#include "datatransfer.h"
#include "quatfusion.h"
#include <stdint.h>

#define BMA_SelectPin 53
#define BMG_SelectPin 49
auto mytimer=timer_create_default();

//struct bma2x2_accel_data_temp acc_xyzt;
//struct bmg160_data_t gyro_xyz;
float acc_data[4],gyro_data[3],imu_temp,rt_acc_data[3];
float quat[4]={1,0,0,0};
float eular[3];
u8 data_tran[92];

extern float acc_init_data[3];

bool timer(void *);
s32 bmg160_gyro_readout_config(void);

void setup() {
  Serial.begin(460800);
  SPI.begin();
  Serial.println("Hello World!");
  pinMode(BMA_SelectPin, OUTPUT);
  pinMode(BMG_SelectPin, OUTPUT);
  pinMode(10,OUTPUT);
  digitalWrite(BMA_SelectPin, HIGH);
  digitalWrite(BMG_SelectPin, HIGH);
  
  bma2x2_acc_readout_config();
  bmg160_gyro_readout_config();
  quatfusion_para_init(acc_init_data, quat);
  
  mytimer.every(10,timer);
  delay(500);
}


void loop() 
{
    mytimer.tick();
}

uint32_t pack_cnt=0;

bool timer(void *)
{
  digitalWrite(10, HIGH);
  
//  s32 rs=bmg160_gyro_readout(&gyro_xyz);
//  rs=bma2x2_acc_readout(&acc_xyzt);

//  filter_axi[4]=filter_axi[3]; 
//  filter_axi[3]=filter_axi[2];
//  filter_axi[2]=filter_axi[1];
//  filter_axi[1]=filter_axi[0];
//  filter_axi[0]=acc_xyzt.x*unit_acc;
//  filter_ayi[4]=filter_ayi[3]; 
//  filter_ayi[3]=filter_ayi[2];
//  filter_ayi[2]=filter_ayi[1];
//  filter_ayi[1]=filter_ayi[0];
//  filter_ayi[0]=acc_xyzt.y*unit_acc;
//  filter_azi[4]=filter_azi[3]; 
//  filter_azi[3]=filter_azi[2];
//  filter_azi[2]=filter_azi[1];
//  filter_azi[1]=filter_azi[0];
//  filter_azi[0]=acc_xyzt.z*unit_acc;
//  
//  float temp_x=0.0,temp_y=0.0,temp_z=0.0;
//  for(unsigned char k=0;k<5;k++)
//  {
//    temp_x+=filter_axi[k]*filter_b[k];
//    temp_y+=filter_ayi[k]*filter_b[k];
//    temp_z+=filter_azi[k]*filter_b[k];
//  } 
//  for (unsigned char k=0;k<4;k++)
//  {
//    temp_x-=filter_axo[k+1]*filter_a[k];
//    temp_y-=filter_ayo[k+1]*filter_a[k];
//    temp_z-=filter_azo[k+1]*filter_a[k];
//  }
//  
//  filter_axo[4]=filter_axo[3];
//  filter_axo[3]=filter_axo[2];
//  filter_axo[2]=filter_axo[1];
//  filter_axo[1]=temp_x;
//  filter_ayo[4]=filter_ayo[3];
//  filter_ayo[3]=filter_ayo[2];
//  filter_ayo[2]=filter_ayo[1];
//  filter_ayo[1]=temp_y;
//  filter_azo[4]=filter_azo[3];
//  filter_azo[3]=filter_azo[2];
//  filter_azo[2]=filter_azo[1];
//  filter_azo[1]=temp_z;
// 
//  acc_data[0]=filter_axo[1]*acc_cal_a[0]+acc_cal_b[0];
//  acc_data[1]=filter_ayo[1]*acc_cal_a[1]+acc_cal_b[1];
//  acc_data[2]=filter_azo[1]*acc_cal_a[2]+acc_cal_b[2];
//  acc_data[3]=acc_xyzt.temp*0.5+23.0;
//
////  gyro_cal_b[0]=3.867e-5*acc_data[3]*acc_data[3]-0.006137*acc_data[3]+0.00579;
////  gyro_cal_b[1]=9.673e-5*acc_data[3]*acc_data[3]-0.01099*acc_data[3]+0.2518;
////  gyro_cal_b[2]=7.139e-5*acc_data[3]*acc_data[3]-0.005547*acc_data[3]-0.003029;
//  
////  filter_gxi[2]=filter_gxi[1];
////  filter_gxi[1]=filter_gxi[0];
////  filter_gxi[0]=gyro_xyz.datax*unit_gyro-gyro_cal_b[0];
////  filter_gyi[2]=filter_gyi[1];
////  filter_gyi[1]=filter_gyi[0];
////  filter_gyi[0]=gyro_xyz.datay*unit_gyro-gyro_cal_b[1];
////  filter_gzi[2]=filter_gzi[1];
////  filter_gzi[1]=filter_gzi[0];
////  filter_gzi[0]=gyro_xyz.dataz*unit_gyro-gyro_cal_b[2];
//
////  float gyro_buff_x_last=gyro_buff_x[99];
////  float gyro_buff_y_last=gyro_buff_y[99];
////  float gyro_buff_z_last=gyro_buff_z[99];
////  
////  for (u8 k=99;k>0;k--)
////  {
////    gyro_buff_x[k]=gyro_buff_x[k-1];
////    gyro_buff_y[k]=gyro_buff_y[k-1];
////    gyro_buff_z[k]=gyro_buff_z[k-1];
////  }
////  gyro_buff_x[0]=gyro_xyz.datax*unit_gyro;
////  gyro_buff_y[0]=gyro_xyz.datay*unit_gyro;
////  gyro_buff_z[0]=gyro_xyz.dataz*unit_gyro;
////
////  gyro_buff_x_mean+=(gyro_buff_x[0]-gyro_buff_x_last)/100.0;
////  gyro_buff_y_mean+=(gyro_buff_y[0]-gyro_buff_y_last)/100.0;
////  gyro_buff_z_mean+=(gyro_buff_z[0]-gyro_buff_z_last)/100.0;
//
////  if (fabs(gyro_buff_x_mean)<0.5)
////  {
////    gyro_bias[0]=gyro_buff_x_mean;
////  }
////  
////  if (fabs(gyro_buff_y_mean)<0.5)
////  {
////    gyro_bias[1]=gyro_buff_y_mean;
////  }
////
////  if (fabs(gyro_buff_z_mean)<0.5)
////  {
////    gyro_bias[2]=gyro_buff_z_mean;
////    }
//    
//  gyro_data[0]=(gyro_xyz.datax*unit_gyro-gyro_bias[0])*gyro_cal_a[0]+(acc_data[3]-init_temp)*(0.16/30.0);
//  gyro_data[1]=(gyro_xyz.datay*unit_gyro-gyro_bias[1])*gyro_cal_a[1]+(acc_data[3]-init_temp)*(0.25/30.0);
//  gyro_data[2]=(gyro_xyz.dataz *unit_gyro-gyro_bias[2])*gyro_cal_a[2]+(acc_data[3]-init_temp)*(0.05/30.0);
//     
////  temp_x=0.0;temp_y=0.0;temp_z=0.0;
////  for(unsigned char k=0;k<3;k++)
////  {
////    temp_x+=filter_gxi[k]*gyro_filter_b[k];
////    temp_y+=filter_gyi[k]*gyro_filter_b[k];
////    temp_z+=filter_gzi[k]*gyro_filter_b[k];
////  } 
////  for (unsigned char k=0;k<2;k++)
////  {
////    temp_x-=filter_gxo[k]*gyro_filter_a[k];
////    temp_y-=filter_gyo[k]*gyro_filter_a[k];
////    temp_z-=filter_gzo[k]*gyro_filter_a[k];
////  }
////  filter_gxo[1]=filter_gxo[0];filter_gxo[0]=temp_x; 
////  filter_gyo[1]=filter_gyo[0];filter_gyo[0]=temp_y; 
////  filter_gzo[1]=filter_gzo[0];filter_gzo[0]=temp_z;
//  
////  gyro_data[0]=filter_gxi[0]; //
////  gyro_data[1]=filter_gyi[0]; //
////  gyro_data[2]=filter_gzi[0]; //

  BMI055_ReadOut(acc_data, rt_acc_data, gyro_data, &imu_temp, 0x00);
  quaternion_update(acc_data,gyro_data,quat,acc_init_data,rt_acc_data);
  eular_angle(quat, eular); 

//  data_tran[0]=0x41;
//  data_tran[1]=0x78;
//  data_tran[2]=0xFF;
//  data_tran[3]=0x06;
//  data_tran[4]=0x81;
//  data_tran[5]=0x54;
//  data_tran[6]=0x01;
//  data_tran[7]=0x88;
//  data_tran[8]=0x0C;
//
//  u8 *ptr=(u8 *)&acc_data[0];
//  data_tran[9]=*(ptr++);
//  data_tran[10]=*(ptr++);
//  data_tran[11]=*(ptr++);
//  data_tran[12]=*(ptr++);
//  data_tran[13]=*(ptr++);
//  data_tran[14]=*(ptr++);
//  data_tran[15]=*(ptr++);
//  data_tran[16]=*(ptr++);
//  data_tran[17]=*(ptr++);
//
//  data_tran[18]=*(ptr++);
//  data_tran[19]=*(ptr++);
//  data_tran[20]=*(ptr++);
//  data_tran[21]=0x00;
//  data_tran[22]=0x8C;
//  data_tran[23]=0x0C;
//  ptr=(u8 *)&gyro_data[0];
//  data_tran[24]=*(ptr++);
//  data_tran[25]=*(ptr++);
//  data_tran[26]=*(ptr++);
//
//  data_tran[27]=*(ptr++);
//  data_tran[28]=*(ptr++);
//  data_tran[29]=*(ptr++);
//  data_tran[30]=*(ptr++);
//  data_tran[31]=*(ptr++);
//  data_tran[32]=*(ptr++);
//  data_tran[33]=*(ptr++);
//  data_tran[34]=*(ptr++);
//  data_tran[35]=*(ptr++);
//
//  data_tran[36]=0x00;
//  data_tran[37]=0xB0;
//  data_tran[38]=0x10;
//  ptr=(u8 *)&quat[0];
//  data_tran[39]=*(ptr++);
//  data_tran[40]=*(ptr++);
//  data_tran[41]=*(ptr++);
//  data_tran[42]=*(ptr++);
//  data_tran[43]=*(ptr++);
//  data_tran[44]=*(ptr++);
//
//  data_tran[45]=*(ptr++);
//  data_tran[46]=*(ptr++);
//  data_tran[47]=*(ptr++);
//  data_tran[48]=*(ptr++);
//  data_tran[49]=*(ptr++);
//  data_tran[50]=*(ptr++);
//  data_tran[51]=*(ptr++);
//  data_tran[52]=*(ptr++);
//  data_tran[53]=*(ptr++);
//  
//  data_tran[54]=*(ptr++);
//  data_tran[55]=0x01;
//  data_tran[56]=0xB0;
//  data_tran[57]=0x10;
//  ptr=(u8 *)&eular[0];
//  data_tran[58]=*(ptr++);
//  data_tran[59]=*(ptr++);
//  data_tran[60]=*(ptr++);
//  data_tran[61]=*(ptr++);
//  data_tran[62]=*(ptr++);
//
//  data_tran[63]=*(ptr++);
//  data_tran[64]=*(ptr++);
//  data_tran[65]=*(ptr++);
//  data_tran[66]=*(ptr++);
//  data_tran[67]=*(ptr++);
//  data_tran[68]=*(ptr++);
//  data_tran[69]=*(ptr++);
//  data_tran[70]=*(ptr++);
//  data_tran[71]=*(ptr++);
//
//  data_tran[72]=*(ptr++);
//  data_tran[73]=*(ptr++);
//  data_tran[74]=0x00;
//  data_tran[75]=0xC0;
//  data_tran[76]=0x04;
//  ptr=(u8 *)&imu_temp;
//  data_tran[77]=*(ptr++);
//  data_tran[78]=*(ptr++);
//  data_tran[79]=*(ptr++);
//  data_tran[80]=*(ptr++);
//
//  data_tran[81]=0x02;
//  data_tran[82]=0xC0;
//  data_tran[83]=0x00;
//  data_tran[84]=0x00;
//  data_tran[85]=0x00;
//  data_tran[86]=0x00;
//  data_tran[87]=0x00;
//  data_tran[88]=0x00;
//  data_tran[89]=0x00;
//
//  data_tran[90]=0x00;
//  data_tran[91]=0x00;

//  for (u8 k=0;k<92;k++)
//    Serial.write(data_tran[k]);

    data_tran[0]=0x41;
    data_tran[1]=0x78;
    data_tran[2]=0xFF;
    data_tran[3]=0x06;
    data_tran[4]=0x81;
    data_tran[5]=0x38;
    
    data_tran[6]=0x01;
    data_tran[7]=0x88;
    data_tran[8]=0x0C;

    u8 *ptr=(u8 *)&acc_data[0];
    data_tran[9]=*(ptr++);
    data_tran[10]=*(ptr++);
    data_tran[11]=*(ptr++);
    data_tran[12]=*(ptr++);
    data_tran[13]=*(ptr++);
    data_tran[14]=*(ptr++);
    data_tran[15]=*(ptr++);
    data_tran[16]=*(ptr++);
    data_tran[17]=*(ptr++);
  
    data_tran[18]=*(ptr++);
    data_tran[19]=*(ptr++);
    data_tran[20]=*(ptr++);
    data_tran[21]=0x00;
    data_tran[22]=0x8C;
    data_tran[23]=0x0C;
    ptr=(u8 *)&gyro_data[0];
    data_tran[24]=*(ptr++);
    data_tran[25]=*(ptr++);
    data_tran[26]=*(ptr++);
  
    data_tran[27]=*(ptr++);
    data_tran[28]=*(ptr++);
    data_tran[29]=*(ptr++);
    data_tran[30]=*(ptr++);
    data_tran[31]=*(ptr++);
    data_tran[32]=*(ptr++);
    data_tran[33]=*(ptr++);
    data_tran[34]=*(ptr++);
    data_tran[35]=*(ptr++);
  
    data_tran[36]=0x00;
    data_tran[37]=0xB0;
    data_tran[38]=0x10;
    ptr=(u8 *)&quat[0];
    data_tran[39]=*(ptr++);
    data_tran[40]=*(ptr++);
    data_tran[41]=*(ptr++);
    data_tran[42]=*(ptr++);
    data_tran[43]=*(ptr++);
    data_tran[44]=*(ptr++);
  
    data_tran[45]=*(ptr++);
    data_tran[46]=*(ptr++);
    data_tran[47]=*(ptr++);
    data_tran[48]=*(ptr++);
    data_tran[49]=*(ptr++);
    data_tran[50]=*(ptr++);
    data_tran[51]=*(ptr++);
    data_tran[52]=*(ptr++);
    data_tran[53]=*(ptr++);
    
    data_tran[54]=*(ptr++);
    data_tran[55]=0x00;
    data_tran[56]=0xC0;
    data_tran[57]=0x04;
    
    ptr=(u8 *)&pack_cnt;
    data_tran[58]=*(ptr++);
    data_tran[59]=*(ptr++);
    data_tran[60]=*(ptr++);
    data_tran[61]=*(ptr++);
    data_tran[62]=0x5A;
    data_tran[63]=0x6D;
  
  for (u8 k=0;k<64;k++)
    Serial.write(data_tran[k]);

  pack_cnt++;
  digitalWrite(10, LOW);
  return true;

}
