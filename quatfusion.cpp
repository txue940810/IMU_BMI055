 #include "quatfusion.h"
#include "math.h"
#include "arduino.h"
#include "MatrixMath.h"
#include "spi_support.h"

mtx_type A[4][4];
mtx_type H[3][4];
mtx_type P[4][4];
mtx_type K[4][1];
float Dt=0.01;
float Q=1e-4;
float Cnt=0.0;

MatrixMath MatrixDue;      // Pre-instantiate

unsigned char quatfusion_para_init(float *acc, float *quat)
{
  float ax=acc[0];float ay=acc[1];float az=acc[2];
  float norm_acc=sqrt(ax*ax+ay*ay+az*az);
  ax/=norm_acc;ay/=norm_acc;az/=norm_acc;
  
  float pitch=asin(ay);
  float roll=-atan2(ax,az);
  float yaw=0;
  float RotA[3][3]={{cos(yaw)*cos(roll)-sin(roll)*sin(yaw)*sin(pitch),sin(yaw)*cos(roll)+sin(roll)*cos(yaw)*sin(pitch),-cos(pitch)*sin(roll)},
  {-sin(yaw)*cos(pitch),cos(yaw)*cos(pitch),sin(pitch)},
  {cos(yaw)*sin(roll)+cos(roll)*sin(yaw)*sin(pitch),sin(yaw)*sin(roll)-cos(roll)*cos(yaw)*sin(pitch),cos(pitch)*cos(roll)}};
    
  quat[0]=0.5*sqrt(1+RotA[0][0]+RotA[1][1]+RotA[2][2]);
  quat[1]=(RotA[1][2]-RotA[2][1])/4.0/quat[0];
  quat[2]=(RotA[2][0]-RotA[0][2])/4.0/quat[0];
  quat[3]=(RotA[0][1]-RotA[1][0])/4.0/quat[0];
        
  P[0][0]=1;P[0][1]=0;P[0][2]=0;P[0][3]=0;
  P[1][0]=0;P[1][1]=1;P[1][2]=0;P[1][3]=0;
  P[2][0]=0;P[2][1]=0;P[2][2]=1;P[2][3]=0;
  P[3][0]=0;P[3][1]=0;P[3][2]=0;P[3][3]=1e-4;
  
  return 1;
}

float acc_norm_buff[50]={1.0};

unsigned char quaternion_update(float *acc,float *gyro,float *quat,float *init_acc, float *realtime_acc)
{
//  Serial.print("Epo:");Serial.print(Cnt);Serial.print(' ');  

  float temp=3.141592653/180.0;
  float ax=acc[0];float ay=acc[1];float az=acc[2];
  float gx=gyro[0]*temp;float gy=gyro[1]*temp;float gz=gyro[2]*temp;
  float q0=quat[0];float q1=quat[1];float q2=quat[2];float q3=quat[3];
  
  float norm_acc=sqrt(ax*ax+ay*ay+az*az);
  ax/=norm_acc;ay/=norm_acc;az/=norm_acc;
  
  float init_norm_acc=sqrt(init_acc[0]*init_acc[0]+init_acc[1]*init_acc[1]+init_acc[2]*init_acc[2]);
  
  for (u8 k=49;k>0;k--)
  {
    acc_norm_buff[k]=acc_norm_buff[k-1];    
  }
//  acc_norm_buff[0]=sqrt(realtime_acc[0]*realtime_acc[0]+realtime_acc[1]*realtime_acc[1]+realtime_acc[2]*realtime_acc[2]);
  acc_norm_buff[0]=norm_acc;
  
  int acc_static_index=0;
  for (u8 k=0;k<50;k++)
  {
    if (fabs(acc_norm_buff[k] - init_norm_acc)>0.05 || sqrt(gx*gx+gy*gy+gz*gz)>2.0*temp) 
    {
      acc_static_index+=1;
    }     
  }
   
  float k_acc=1e-1;
  if (fabs(gx)<1.0*temp)
    gx=0.0;
  if (fabs(gy)<1.0*temp)
    gy=0.0;
  if (fabs(gz)<1.0*temp)
    gz=0.0 ;

  A[0][0]=1;A[0][1]=-0.5*Dt*gx;A[0][2]=-0.5*Dt*gy;A[0][3]=-0.5*Dt*gz;
  A[1][0]=0.5*Dt*gx;A[1][1]=1;A[1][2]=0.5*Dt*gz;A[1][3]=-0.5*Dt*gy;
  A[2][0]=0.5*Dt*gy;A[2][1]=-0.5*Dt*gz;A[2][2]=1;A[2][3]=0.5*Dt*gx;
  A[3][0]=0.5*Dt*gz;A[3][1]=0.5*Dt*gy;A[3][2]=-0.5*Dt*gx;A[3][3]=1;

  H[0][0]=-2*q2;H[0][1]=2*q3;H[0][2]=-2*q0;H[0][3]=2*q1;
  H[1][0]=2*q1;H[1][1]=2*q0;H[1][2]=2*q3;H[1][3]=2*q2;
  H[2][0]=2*q0;H[2][1]=-2*q1;H[2][2]=-2*q2;H[2][3]=2*q3;

//  MatrixDue.Print((mtx_type*)P,4,4,"P_Start");
  mtx_type Temp0[4][4];
  MatrixDue.Multiply((mtx_type*)A, (mtx_type*)P, 4, 4, 4, (mtx_type*)Temp0); // AP
  mtx_type Temp1[4][4];
  MatrixDue.Transpose((mtx_type*)A, 4, 4, (mtx_type*)Temp1); //A'
  MatrixDue.Multiply((mtx_type*)Temp0,(mtx_type*)Temp1,4, 4, 4, (mtx_type*)P);//APA'
  P[0][0]+=Q;P[1][1]+=Q;P[2][2]+=Q;P[3][3]+=Q*0.0;//APA'+Q
 // Matrix.Print((mtx_type*)P,4,4,"P^1");

  mtx_type Temp2[3][4];
  MatrixDue.Multiply((mtx_type*)H, (mtx_type*)P, 3, 4, 4, (mtx_type*)Temp2); //HP
  mtx_type Temp3[4][3];
  MatrixDue.Transpose((mtx_type*)H, 3, 4, (mtx_type*)Temp3); //H'
  mtx_type Temp4[3][3];
  MatrixDue.Multiply((mtx_type*)Temp2,(mtx_type*)Temp3,3,4,3,(mtx_type*)Temp4); //HPH'
//  Matrix.Print((mtx_type*)Temp4,3,3,"HPH'");
  Temp4[0][0]+=k_acc;Temp4[1][1]+=k_acc;Temp4[2][2]+=k_acc; //HPH'+R
//  Matrix.Print((mtx_type*)Temp4,3,3,"HPH'+R");
  
  mtx_type Temp5[4][3];
  MatrixDue.Multiply((mtx_type*)P,(mtx_type*)Temp3,4,4,3,(mtx_type*)Temp5); //PH'
  MatrixDue.Invert((mtx_type*)Temp4, 3);// (HPH'+R)^-1
//  Matrix.Print((mtx_type*)Temp4, 3, 3, "(HPH'+R)^-1");
  mtx_type Temp6[4][3];
  MatrixDue.Multiply((mtx_type*)Temp5,(mtx_type*)Temp4,4,3,3,(mtx_type*)Temp6);//K
//  Matrix.Print((mtx_type*)Temp6, 4, 3, "K");

  float d0=ax-(2*q1*q3-2*q0*q2);float d1=ay-(2*q1*q0+2*q2*q3);float d2=az-(q0*q0-q1*q1-q2*q2+q3*q3);
  float k00=Temp6[0][0];float k01=Temp6[0][1];float k02=Temp6[0][2];
  float k10=Temp6[1][0];float k11=Temp6[1][1];float k12=Temp6[1][2];
  float k20=Temp6[2][0];float k21=Temp6[2][1];float k22=Temp6[2][2];
  float k30=Temp6[3][0];float k31=Temp6[3][1];float k32=Temp6[3][2];
 

//  Serial.print("q0:");Serial.print(q0);Serial.print(' ');
//  Serial.print("q1:");Serial.print(q1);Serial.print(' ');
//  Serial.print("q2:");Serial.print(q2);Serial.print(' ');
//  Serial.print("q3:");Serial.println(q3);

  float h1=2*q1*q3-2*q0*q2,h2=2*q1*q0+2*q2*q3,h3=q0*q0-q1*q1-q2*q2+q3*q3;
  float cov1=ay*h3-az*h2,cov2=az*h1-ax*h3,cov3=ax*h2-ay*h1;
  float norm_cov=sqrt(cov1*cov1+cov2*cov2+cov3*cov3);
  float theta=asin(norm_cov);
  float qx0=1.0,qx1=0.0,qx2=0.0,qx3=0.0;
  if (norm_cov!=0)
  {
    qx0=cos(theta/2*0.1);
    qx1=sin(theta/2*0.1)*cov1/norm_cov;
    qx2=sin(theta/2*0.1)*cov2/norm_cov;
    qx3=sin(theta/2*0.1)*cov3/norm_cov;
  }
  
  float tq0=q0,tq1=q1,tq2=q2,tq3=q3; 
    
  if (acc_static_index==0)
  {
//    q0+=k00*d0+k01*d1+k02*d2;
//    q1+=k10*d0+k11*d1+k12*d2;
//    q2+=k20*d0+k21*d1+k22*d2;
//    q3+=k30*d0+k31*d1+k32*d2;

  q0=qx0*tq0-qx1*tq1-qx2*tq2-qx3*tq3;
  q1=qx1*tq0+qx0*tq1+qx3*tq2-qx2*tq3;
  q2=qx2*tq0-qx3*tq1+qx0*tq2+qx1*tq3;
  q3=qx3*tq0+qx2*tq1-qx1*tq2+qx0*tq3;
  }

  
    
  mtx_type Temp7[4][4];
  MatrixDue.Multiply((mtx_type*)Temp6,(mtx_type*)H, 4, 3, 4,(mtx_type*)Temp7); //KH
//  Matrix.Print((mtx_type*)Temp7,4,4,"KH");
  mtx_type Temp8[4][4];
  
  Temp8[0][0]=1.0;Temp8[0][1]=0.0;Temp8[0][2]=0.0;Temp8[0][3]=0.0;
  Temp8[1][0]=0.0;Temp8[1][1]=1.0;Temp8[1][2]=0.0;Temp8[1][3]=0.0;
  Temp8[2][0]=0.0;Temp8[2][1]=0.0;Temp8[2][2]=1.0;Temp8[2][3]=0.0;
  Temp8[3][0]=0.0;Temp8[3][1]=0.0;Temp8[3][2]=0.0;Temp8[3][3]=1.0;//I
  
//  Matrix.Print((mtx_type*)Temp8,4,4,"I");
  mtx_type Temp9[4][4];
  MatrixDue.Subtract((mtx_type*)Temp8,(mtx_type*)Temp7,4,4,(mtx_type*)Temp9);//I-KH
//  Matrix.Print((mtx_type*)Temp9,4,4,"I-KH");
  mtx_type Temp10[4][4];
  MatrixDue.Multiply((mtx_type*)Temp9,(mtx_type*)P, 4, 4, 4, (mtx_type*)Temp10);
  
//  Matrix.Print((mtx_type*)Temp10,4,4,"(I-KH)P");
  MatrixDue.Copy((mtx_type*)Temp10,4,4,(mtx_type*)P);
//  Matrix.Print((mtx_type*)P,4,4,"P_End");

  tq0=q0;tq1=q1;tq2=q2;tq3=q3;
  q0+=0.5*(-gx*tq1-gy*tq2-gz*tq3)*Dt;
  q1+=0.5*(gx*tq0+gz*tq2-gy*tq3)*Dt;
  q2+=0.5*(gy*tq0-gz*tq1+gx*tq3)*Dt;
  q3+=0.5*(gz*tq0+gy*tq1-gx*tq2)*Dt;

  float norm_q=sqrt(q0*q0+q1*q1+q2*q2+q3*q3);
  q0/=norm_q;q1/=norm_q;q2/=norm_q;q3/=norm_q; 
  quat[0]=q0;quat[1]=q1;quat[2]=q2;quat[3]=q3;

  Cnt++;
  return 1;
}

unsigned char eular_angle(float *quat, float *eular)
{
  float q0=quat[0];float q1=quat[1];float q2=quat[2];float q3=quat[3];
  float temp=180/3.141592653;
  //yaw
  eular[2]=-atan2(2*(q1*q2-q0*q3),q0*q0-q1*q1+q2*q2-q3*q3)*temp;
  //pitch
  eular[1]=asin(saturation(2*(q2*q3+q0*q1),1))*temp;
  //roll
  eular[0]=atan2(-2*(q1*q3-q0*q2),q0*q0-q1*q1-q2*q2+q3*q3)*temp;
  
  return 1;
  }

 float saturation(float x,float limit)
{
  float out=0.0;
  if (x>limit)
  {
    out=limit;
  } 
   else if (x<-limit)
   {
     out=-limit; 
   }
  else
  {
    out=x;
  }
  return out;
}
