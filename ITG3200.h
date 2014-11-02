/**
 * @author Uwe Gartmann
 * @author Used ITG3200 library developed Aaron Berk as template
 *
 * @section LICENSE
 *
 * Copyright (c) 2010 ARM Limited
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 *
 * @section DESCRIPTION
 *
 * ITG-3200 triple axis, digital interface, gyroscope.
 *
 * Datasheet:
 *
 * http://invensense.com/mems/gyro/documents/PS-ITG-3200-00-01.4.pdf
 */

#ifndef ITG3200_H
#define ITG3200_H

/**
 * Includes
 */
#include "mbed.h"

/** Interface library for the ITG3200 3-axis gyro
 *
 * Ported from Pololu L3G4200D library for Arduino by
 * Michael Shimniok http://bot-thoughts.com
 * Then ported to ITG3200 by
 * Larry Littlefield http://kb7kmo.blogspot.com
 *
 * @code
 * #include "mbed.h"
 * #include "ITG3200.h"
 * ITG3200 gyro(p28, p27);
 * ...
 * int g[3];
 * gyro.read(g);
 * @endcode
 */typedef char byte;
/**
 * Defines
 */
#define ITG3200_I2C_ADDRESS 0x68 //7-bit address.

//-----------
// Registers
//-----------
#define WHO_AM_I_REG    0x00
#define SMPLRT_DIV_REG  0x15
#define DLPF_FS_REG     0x16
#define INT_CFG_REG     0x17
#define INT_STATUS      0x1A
#define TEMP_OUT_H_REG  0x1B
#define TEMP_OUT_L_REG  0x1C
#define GYRO_XOUT_H_REG 0x1D
#define GYRO_XOUT_L_REG 0x1E
#define GYRO_YOUT_H_REG 0x1F
#define GYRO_YOUT_L_REG 0x20
#define GYRO_ZOUT_H_REG 0x21
#define GYRO_ZOUT_L_REG 0x22
#define PWR_MGM_REG     0x3E

//----------------------------
// Low Pass Filter Bandwidths
//----------------------------
#define LPFBW_256HZ 0x00
#define LPFBW_188HZ 0x01
#define LPFBW_98HZ  0x02
#define LPFBW_42HZ  0x03
#define LPFBW_20HZ  0x04
#define LPFBW_10HZ  0x05
#define LPFBW_5HZ   0x06

/**
 * ITG-3200 triple axis digital gyroscope.
 */

class ITG3200
{
    public:
        /** Create a new ITG3200 I2C interface
         * @param sda is the pin for the I2C SDA line
         * @param scl is the pin for the I2C SCL line
         */
        ITG3200(PinName sda, PinName scl);
        
        /** Read gyro values
         * @param g Array containing x, y, and z gyro values
         * @return g Array containing x, y, and z gyro values
         */
        void init(void);
        void status(byte *s);
        void read(int *g);
        void read3(int x, int y, int z);
        void zeroCalibrate(unsigned int totSamples, unsigned int sampleDelayMS);
        void readRawCal(int *_GyroXYZ);
        void readFin(float *_GyroXYZ); // includes gain and offset
        
    private:
        volatile float gains[3]; 
        volatile int offsets[3];
        volatile float polarities[3];
        
        void setGains(float _Xgain, float _Ygain, float _Zgain);
        void setOffsets(int _Xoffset, int _Yoffset, int _Zoffset);
        void setRevPolarity(bool _Xpol, bool _Ypol, bool _Zpol);  // true = Reversed  false = default
    
    
        byte data[6];
        int _rates[3];
        I2C _device;
        void writeReg(byte reg, byte value);
        byte readReg(byte reg);
        void enableDefault(void);
};

#endif
