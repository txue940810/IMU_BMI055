#ifndef _QUATFUSION_
#define _QUATFUSION_

unsigned char quatfusion_para_init(float *acc,float *quat);
unsigned char quaternion_update(float *acc,float *gyro,float *quat,float *init_acc,float *realtime_acc);
unsigned char eular_angle(float *quat, float *eular);
float saturation(float x,float limit);

#endif
