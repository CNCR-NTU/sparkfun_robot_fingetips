#include "SoftwareWire.h"
SoftwareWire myWire( A1, A0);   // A1=SDA, A2=SCL
SoftwareWire my2Wire( A2, A0);  // A3=SDA, A2=SCL
SoftwareWire my3Wire( A3, A0);

//   I2C address is LPS25HB
#define addr1 0x5D
//   I2C address for VCNL4040
#define addr2 0x60

void setup() 
{
  Serial.begin(9600);                
  
  //turn on  LPS25HB for fingertip nr.1
  myWire.begin();
  myWire.beginTransmission(addr1); 
  myWire.write(0x20);
  myWire.write(0xC0);
  myWire.endTransmission();

  //turn on  LPS25HB for fingertip nr.2
  my2Wire.begin();                   
  my2Wire.beginTransmission(addr1); 
  my2Wire.write(0x20);
  my2Wire.write(0xC0);
  my2Wire.endTransmission();

  //turn on  LPS25HB for fingertip nr.3
  my3Wire.begin();
  my3Wire.beginTransmission(addr1);  
  my3Wire.write(0x20);
  my3Wire.write(0xC0);
  my3Wire.endTransmission();

  //turn on  VCNL4040 for fingertip nr.1
  myWire.beginTransmission(addr2);   
  myWire.write((byte)0x00);
  myWire.write((byte)0x00);
  myWire.write((byte)0x00);
  myWire.endTransmission();
  //configuring  VCNL4040 rate for fingertip nr.1 
  myWire.beginTransmission(addr2);   
  myWire.write(0x03);
  myWire.write(byte(0x00));
  myWire.write(byte(0x09));
  myWire.endTransmission();

  //turn on  VCNL4040 for fingertip nr.2
  my2Wire.beginTransmission(addr2);  
  my2Wire.write((byte)0x00);
  my2Wire.write((byte)0x00);
  my2Wire.write((byte)0x00);
  my2Wire.endTransmission();
  //configuring  VCNL4040 rate for fingertip nr.2
  my2Wire.beginTransmission(addr2);  
  my2Wire.write(0x03);
  my2Wire.write(byte(0x00));
  my2Wire.write(byte(0x09));
  my2Wire.endTransmission();

  //turn on  VCNL4040 for fingertip nr.3
  my3Wire.beginTransmission(addr2);   
  my3Wire.write((byte)0x00);
  my3Wire.write((byte)0x00);
  my3Wire.write((byte)0x00);
  my3Wire.endTransmission();
  //configuring  VCNL4040 rate for fingertip nr.3 
  my3Wire.beginTransmission(addr2);   
  my3Wire.write(0x03);
  my3Wire.write(byte(0x00));
  my3Wire.write(byte(0x09));
  my3Wire.endTransmission();
}

void loop()
{
 int32_t pressure1_2, pressure1_1, pressure1_0;                //variables for fingertip nr.1
 int16_t temp1_1, temp1_0, proximity1_1, proximity1_0, light1_1 , light1_0;
 
 int32_t pressure2_2, pressure2_1, pressure2_0;               //variables for fingertip nr.2
 int16_t temp2_1, temp2_0, proximity2_1, proximity2_0, light2_1 , light2_0;  

 int32_t pressure3_2, pressure3_1, pressure3_0;               //variables for fingertip nr.3
 int16_t temp3_1, temp3_0, proximity3_1, proximity3_0, light3_1 , light3_0;  

   myWire.beginTransmission(addr2);     //retrieving data from VCNL4040 registers (fingertip nr.1) for proximity bytes 
   myWire.write(0x08);
   myWire.endTransmission(false); 
   myWire.requestFrom(addr2, 2);
  
    if(myWire.available() == 2)
    {
      proximity1_0 = myWire.read();
      proximity1_1 = myWire.read();
    }
    
   myWire.beginTransmission(addr2);     //retrieving data from VCNL4040 registers (fingertip nr.1) for ambient light bytes
   myWire.write(0x09);
   myWire.endTransmission(false);
   myWire.requestFrom(addr2, 2);
    if(myWire.available() == 2)
     {
      light1_0 = myWire.read();
      light1_1 = myWire.read();
     }
     
   my2Wire.beginTransmission(addr2);     //retrieving data from VCNL4040 registers (fingertip nr.2) for proximity bytes 
   my2Wire.write(0x08);
   my2Wire.endTransmission(false); 
   my2Wire.requestFrom(addr2, 2);
    if(my2Wire.available() == 2)
    {
      proximity2_0 = my2Wire.read();
      proximity2_1 = my2Wire.read();
    }
    
   my2Wire.beginTransmission(addr2);      //retrieving data from VCNL4040 registers (fingertip nr.2) for ambient light bytes
   my2Wire.write(0x09);
   my2Wire.endTransmission(false);
   my2Wire.requestFrom(addr2, 2);
    if(my2Wire.available() == 2)
     {
      light2_0 = my2Wire.read();
      light2_1 = my2Wire.read();
     }

   my3Wire.beginTransmission(addr2);     //retrieving data from VCNL4040 registers (fingertip nr.2) for proximity bytes 
   my3Wire.write(0x08);
   my3Wire.endTransmission(false); 
   my3Wire.requestFrom(addr2, 2);
    if(my3Wire.available() == 2)
    {
      proximity3_0 = my3Wire.read();
      proximity3_1 = my3Wire.read();
    }
    
   my3Wire.beginTransmission(addr2);      //retrieving data from VCNL4040 registers (fingertip nr.2) for ambient light bytes
   my3Wire.write(0x09);
   my3Wire.endTransmission(false);
   my3Wire.requestFrom(addr2, 2);
    if(my3Wire.available() == 2)
     {
      light3_0 = my3Wire.read();
      light3_1 = my3Wire.read();
     }

     
   myWire.beginTransmission(addr1);        //retrieving data from LPS25HB registers (fingertip nr.1) for pressure bytes 
   myWire.write(0x28 | 0x80);
   myWire.endTransmission(false); 
   myWire.requestFrom(addr1, 3);
    if(myWire.available() == 3)
     {
      pressure1_0 = myWire.read();
      pressure1_1 = myWire.read();
      pressure1_2 = myWire.read();
     }

   myWire.beginTransmission(addr1);        //retrieving data from LPS25HB registers (fingertip nr.1) for temperature bytes
   myWire.write(0x2B | 0x80);
   myWire.endTransmission(false); 
   myWire.requestFrom(addr1, 2);
     if(myWire.available() == 2)
     {
       temp1_0 = myWire.read();
       temp1_1 = myWire.read();
     }
 
   my2Wire.beginTransmission(addr1);      //retrieving data from LPS25HB registers (fingertip nr.2) for pressure bytes 
   my2Wire.write(0x28 | 0x80);
   my2Wire.endTransmission(false); 
   my2Wire.requestFrom(addr1, 3);
     if(my2Wire.available() == 3)
     {
       pressure2_0 = my2Wire.read();
       pressure2_1 = my2Wire.read();
       pressure2_2 = my2Wire.read();
     }
 
   my2Wire.beginTransmission(addr1);
   my2Wire.write(0x2B | 0x80);      //retrieving data from LPS25HB registers (fingertip nr.2) for temperature bytes    
   my2Wire.endTransmission(false); 
   my2Wire.requestFrom(addr1, 2);
     if(my2Wire.available() == 2)
     {
       temp2_0 = my2Wire.read();
       temp2_1 = my2Wire.read();
     }
   
   my3Wire.beginTransmission(addr1);      //retrieving data from LPS25HB registers (fingertip nr.2) for pressure bytes 
   my3Wire.write(0x28 | 0x80);
   my3Wire.endTransmission(false); 
   my3Wire.requestFrom(addr1, 3);
     if(my3Wire.available() == 3)
     {
       pressure3_0 = my3Wire.read();
       pressure3_1 = my3Wire.read();
       pressure3_2 = my3Wire.read();
     }
 
   my3Wire.beginTransmission(addr1);     //retrieving data from LPS25HB registers (fingertip nr.2) for temperature bytes    
   my3Wire.write(0x2B | 0x80);
   my3Wire.endTransmission(false); 
   my3Wire.requestFrom(addr1, 2);
     if(my3Wire.available() == 2)
     {
       temp3_0 = my3Wire.read();
       temp3_1 = my3Wire.read();
     }

  // Converting data for first fingertip
  
  float proximity1 = get_proximity(proximity1_0, proximity1_1);
  unsigned int light1= get_light(light1_0, light1_1);
  float pressure1 = get_pressure(pressure1_0, pressure1_1, pressure1_2);
  float temp1 = get_temperature(temp1_0, temp1_1);

  // Converting data for second fingertip

  float proximity2 = get_proximity(proximity2_0, proximity2_1);
  unsigned int light2 = get_light(light2_0, light2_1);
  float pressure2 = get_pressure(pressure2_0, pressure2_1, pressure2_2);
  float temp2 = get_temperature(temp2_0, temp2_1);

  // Converting data for third fingertip

  float proximity3 = get_proximity(proximity3_0, proximity3_1);
  unsigned int light3= get_light(light3_0, light3_1);
  float pressure3 = get_pressure(pressure3_0, pressure3_1, pressure3_2);
  float temp3 = get_temperature(temp3_0, temp3_1);

  
  
  // Output data to serial monitor
  Serial.print(pressure1);
  Serial.print(","); 
  Serial.print(temp1);
  Serial.print(",");
  Serial.print(proximity1);
  Serial.print(","); 
  Serial.print(light1);
  Serial.print(","); 
  Serial.print(pressure2);
  Serial.print(","); 
  Serial.print(temp2);
  Serial.print(",");  
  Serial.print(proximity2);
  Serial.print(","); 
  Serial.print(light2);
  Serial.print(",");
  Serial.print(pressure3);
  Serial.print(","); 
  Serial.print(temp3);
  Serial.print(",");  
  Serial.print(proximity3);
  Serial.print(","); 
  Serial.println(light3);
 //delay(10);
}

float get_pressure(int32_t pre0, int32_t pre1, int32_t pre2) {
 float pressure_t = (pre2<<16 | pre1<<8 | pre0);
 pressure_t=pressure_t/4096;
 return pressure_t;
}

float get_temperature(int16_t temp0, int16_t temp1) { 
 float temp = (temp1<<8 | temp0);
 temp=42.5 + (temp/480);
 return temp;
}  

float get_proximity(int16_t pro0, int16_t pro1) { 
 float prox_t = (pro1<<8 | pro0);
  return prox_t;
}  


 int get_light(int16_t light0, int16_t light1) { 
  long light_t = (light1<<8 | light0);
  //light_t=(light_t/65536)*983;
  return light_t;
}  
