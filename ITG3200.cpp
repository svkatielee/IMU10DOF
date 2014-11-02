/**
 * Copyright (c) 2011 Pololu Corporation.  For more information, see
 * 
 * http://www.pololu.com/
 * http://forum.pololu.com/
 * 
 * Permission is hereby granted, free of charge, to any person
 * obtaining a copy of this software and associated documentation
 * files (the "Software"), to deal in the Software without
 * restriction, including without limitation the rights to use,
 * copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the
 * Software is furnished to do so, subject to the following
 * conditions:
 * 
 * The above copyright notice and this permission notice shall be
 * included in all copies or substantial portions of the Software.
 * 
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES
 * OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
 * NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT
 * HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY,
 * WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR
 * OTHER DEALINGS IN THE SOFTWARE.
 */
 
#include "mbed.h"
#include "ITG3200.h"
#include <math.h>

// Defines ////////////////////////////////////////////////////////////////

// The Arduino two-wire interface uses a 7-bit number for the address, 
// and sets the last bit correctly based on reads and writes
// mbed I2C libraries take the 7-bit address shifted left 1 bit
// #define GYR_ADDRESS (0xD2 >> 1)
#define GYR_ADDRESS (0x68 << 1) //ITG3200

// Public Methods //////////////////////////////////////////////////////////////

// Constructor
ITG3200::ITG3200(PinName sda, PinName scl):
    _device(sda, scl)
{
    _device.frequency(100000);
   
    //Set FS_SEL to 0x03 for proper operation.
    writeReg(SMPLRT_DIV_REG, 0x07);   // samplr rte 1000hz 7sample low pass filter
    writeReg(DLPF_FS_REG, 0x03 << 3);
   // writeReg(L3G4200D_CTRL_REG4, 0x20); // 2000 dps full scale default for ITG3200
    writeReg(PWR_MGM_REG, 0x01); // CLK_SEL to x gyro

    setGains(2.0,2.0,2.0);
    setOffsets(0.0,0.0,0.0);

}

// Initialize gyro again
void ITG3200::init(void)
{
	//Set FS_SEL to 0x03 for proper operation.
    writeReg(SMPLRT_DIV_REG, 0x07);   // samplr rte 1000hz 7sample low pass filter
    writeReg(DLPF_FS_REG, 0x03 << 3);
   // writeReg(L3G4200D_CTRL_REG4, 0x20); // 2000 dps full scale default for ITG3200
    writeReg(PWR_MGM_REG, 0x01); // CLK_SEL to x gyro

    // initialize varaibles
    setRevPolarity(false, false, false);
    setGains(1.0,1.0,1.0);
    setOffsets(0.0,0.0,0.0);
}

// read status registers of ITG3200
void ITG3200::status(byte *s)
{
	s[0]=readReg(WHO_AM_I_REG);
	s[1]=readReg(SMPLRT_DIV_REG);
	s[2]=readReg(DLPF_FS_REG);
	s[3]=readReg(PWR_MGM_REG);
}

// Writes a gyro register
void ITG3200::writeReg(byte reg, byte value)
{
    data[0] = reg;
    data[1] = value;
    
    _device.write(GYR_ADDRESS, data, 2);
}

// Reads a gyro register
byte ITG3200::readReg(byte reg)
{
    byte value = 0;
    
    _device.write(GYR_ADDRESS, &reg, 1);
    _device.read(GYR_ADDRESS, &value, 1);

    return value;
}

void ITG3200::setGains(float _Xgain, float _Ygain, float _Zgain) {
  gains[0] = _Xgain;
  gains[1] = _Ygain;
  gains[2] = _Zgain;
}

void ITG3200::setOffsets(int _Xoffset, int _Yoffset, int _Zoffset) {
  offsets[0] = _Xoffset;
  offsets[1] = _Yoffset;
  offsets[2] = _Zoffset;
}

void ITG3200::setRevPolarity(bool _Xpol, bool _Ypol, bool _Zpol) {
  polarities[0] = _Xpol ? -1 : 1;
  polarities[1] = _Ypol ? -1 : 1;
  polarities[2] = _Zpol ? -1 : 1;
}


void ITG3200::zeroCalibrate(unsigned int totSamples, unsigned int sampleDelayMS) {
  int xyz[3]; 
  float tmpOffsets[] = {0,0,0};

  for (int i = 0;i < totSamples;i++){
    wait_ms(sampleDelayMS);
    read(xyz);
    tmpOffsets[0] += xyz[0];
    tmpOffsets[1] += xyz[1];
    tmpOffsets[2] += xyz[2];  
  }
  setOffsets(-tmpOffsets[0] / totSamples, -tmpOffsets[1] / totSamples, -tmpOffsets[2] / totSamples);
}


// Reads the 3 gyro channels and stores them in vector g
void ITG3200::read(int *g)
{
    // assert the MSB of the address to get the gyro 
    // to do slave-transmit subaddress updating. (ITG default)
    data[0] = GYRO_XOUT_H_REG;
    _device.write(GYR_ADDRESS, data, 1); 

//    Wire.requestFrom(GYR_ADDRESS, 6);
//    while (Wire.available() < 6);

    _device.read(GYR_ADDRESS, data, 6); 

    uint8_t xha = data[0];
    uint8_t xla = data[1];
    uint8_t yha = data[2];
    uint8_t yla = data[3];
    uint8_t zha = data[4];
    uint8_t zla = data[5];

    g[0] = (short) (yha << 8 | yla);
    g[1] = (short) (xha << 8 | xla);
    g[2] = (short) (zha << 8 | zla);
}

void ITG3200::readRawCal(int *_GyroXYZ) {
  read(_GyroXYZ);
  _GyroXYZ[0] += offsets[0];
  _GyroXYZ[1] += offsets[1];
  _GyroXYZ[2] += offsets[2];
}



void ITG3200::read3(int x, int y, int z) {
  int* readings;
  read(readings);

  x = readings[0];  
  y = readings[1]; 
  z = readings[2]; 
}

void ITG3200::readFin(float *_GyroXYZ){
  int xyz[3];
  
  readRawCal(xyz); // x,y,z will contain calibrated integer values from the sensor
  _GyroXYZ[0] =  (float)(xyz[0]) / (14.375 * polarities[0] * gains[0]);
  _GyroXYZ[1] =  (float)(xyz[1]) / (14.375 * polarities[1] * gains[1]);
  _GyroXYZ[2] =  (float)(xyz[2]) / (14.375 * polarities[2] * gains[2]);
}
