/*
 * BMP085.cpp
 *
 *  Created on: 13 juil. 2013
 *      Author: Vincent
 */

#include "BMP085.h"
#include <new>

BMP085::BMP085(PinName sda, PinName scl) : i2c_(*reinterpret_cast<I2C*>(i2cRaw))
{
    // Placement new to avoid additional heap memory allocation.
    new(i2cRaw) I2C(sda, scl);

    pressure = 101300;
    temperature = 298;
}

BMP085::~BMP085()
{
    // If the I2C object is initialized in the buffer in this object, call destructor of it.
    if(&i2c_ == reinterpret_cast<I2C*>(&i2cRaw))
        reinterpret_cast<I2C*>(&i2cRaw)->~I2C();
}

// oversampling setting
// 0 = ultra low power
// 1 = standard
// 2 = high
// 3 = ultra high resolution
const unsigned char oversampling_setting = 3; //oversampling for measurement
const unsigned char pressure_conversiontime[4] = {4.5, 7.5, 13.5, 25.5 };  // delays for oversampling settings 0, 1, 2 and 3

// sensor registers from the BOSCH BMP085 datasheet
short ac1;
short ac2;
short ac3;
unsigned short ac4;
unsigned short ac5;
unsigned short ac6;
short b1;
short b2;
short mb;
short mc;
short md;

void BMP085::writeRegister(unsigned char r, unsigned char v)
{
  char cmd1[2];
  cmd1[0] = r;
  cmd1[1] = v;
  i2c_.write(I2C_ADDRESS,cmd1, 2);
}

// read a 16 bit register
int BMP085::readIntRegister(unsigned char r)
{
  char cmd1[2];
  char cmd2[1];
  //unsigned char msb, lsb;
  cmd2[0] = r;
  i2c_.write(I2C_ADDRESS,cmd2, 1);
  i2c_.read(I2C_ADDRESS, cmd1, 2);
  return (((int)cmd1[0]<<8) | ((int)cmd1[1]));
}

// read uncompensated temperature value
unsigned int BMP085::readUT() {
  writeRegister(0xf4,0x2e);
  wait(0.0045); // the datasheet suggests 4.5 ms
  return readIntRegister(0xf6);

}

// read uncompensated pressure value
long BMP085::readUP() {
  writeRegister(0xf4,0x34+(oversampling_setting<<6));
  wait(pressure_conversiontime[oversampling_setting]*0.001);

  //unsigned char msb, lsb, xlsb;
  char cmd1[3];
  char cmd0[1];
  cmd0[0] = 0xf6;
  i2c_.write(I2C_ADDRESS,cmd0, 1);
  i2c_.read(I2C_ADDRESS, cmd1, 3);
  return (((long)cmd1[0]<<16) | ((long)cmd1[1]<<8) | ((long)cmd1[2])) >>(8-oversampling_setting);

}

// Below there are the utility functions to get data from the sensor.

// read temperature and pressure from sensor
void BMP085::readSensor() {

  long ut= readUT();
  long up = readUP();


  int x1, x2, x3, b3, b5, b6, p;
  unsigned int b4, b7;
  //calculate true temperature
  x1 = ((long)ut - ac6) * ac5 >> 15;
  x2 = ((long) mc << 11) / (x1 + md);
  b5 = x1 + x2;
  temperature = (b5 + 8) >> 4;

  //calculate true pressure
  b6 = b5 - 4000;
  x1 = (b2 * (b6 * b6 >> 12)) >> 11;
  x2 = ac2 * b6 >> 11;
  x3 = x1 + x2;

  if (oversampling_setting == 3) b3 = ((int32_t) ac1 * 4 + x3 + 2) << 1;
  if (oversampling_setting == 2) b3 = ((int32_t) ac1 * 4 + x3 + 2);
  if (oversampling_setting == 1) b3 = ((int32_t) ac1 * 4 + x3 + 2) >> 1;
  if (oversampling_setting == 0) b3 = ((int32_t) ac1 * 4 + x3 + 2) >> 2;

  x1 = ac3 * b6 >> 13;
  x2 = (b1 * (b6 * b6 >> 12)) >> 16;
  x3 = ((x1 + x2) + 2) >> 2;
  b4 = (ac4 * (unsigned long) (x3 + 32768)) >> 15;
  b7 = ((unsigned long) up - b3) * (50000 >> oversampling_setting);
  p = b7 < 0x80000000 ? (b7 * 2) / b4 : (b7 / b4) * 2;

  x1 = (p >> 8) * (p >> 8);
  x1 = (x1 * 3038) >> 16;
  x2 = (-7357 * p) >> 16;
  pressure = p + ((x1 + x2 + 3791) >> 4);
}

void  BMP085::getCalibrationData() {
  ac1 = readIntRegister(0xAA);
  ac2 = readIntRegister(0xAC);
  ac3 = readIntRegister(0xAE);
  ac4 = readIntRegister(0xB0);
  ac5 = readIntRegister(0xB2);
  ac6 = readIntRegister(0xB4);
  b1 = readIntRegister(0xB6);
  b2 = readIntRegister(0xB8);
  mb = readIntRegister(0xBA);
  mc = readIntRegister(0xBC);
  md = readIntRegister(0xBE);
}

float BMP085::press(void)
{
    return(pressure);
}

float BMP085::temp(void)
{
    return(temperature);
}

float BMP085::altitud(void)
{
    //press();
   // temp();
    float To=298;
    float ho=0;
    float Po=101325;
    //ecuacion
    float c=(To-0.0065*ho);
    float e=(pressure/Po);
    float d=exp(0.19082*log(e));
    float b=c*d;
    float alt=153.84615*(To-b);
    return(alt);
}

