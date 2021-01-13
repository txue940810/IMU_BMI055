#include "datatransfer.h"
#include "Arduino.h"

u8 QTSO_Send(u8 len,float * dat)
{
  Serial.write(0x03);
  Serial.write(0xFC);
  for (u8 k=0;k<len;k++)
  {
   float temp=dat[k];
   u8 *dataptr=(u8 *)&temp;
   Serial.write(*dataptr);
   dataptr++;
   Serial.write(*dataptr);
   dataptr++;
   Serial.write(*dataptr);
   dataptr++;
   Serial.write(*dataptr);
  }
  Serial.write(0xFC);
  Serial.write(0x03);
  return 1;
}
