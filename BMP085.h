/*
 * BMP085.h
 *
 *  Created on: 13 juil. 2013
 *      Author: Vincent
 */

#ifndef BMP085_H_
#define BMP085_H_

#include "mbed.h"

class BMP085{

public :

    static const int I2C_ADDRESS = 0xEE;  // sensor address

    BMP085(PinName sda, PinName scl);


    BMP085(I2C &i2c) : i2c_(i2c) {
        //init();
    }

    ~BMP085();

    void writeRegister(unsigned char r, unsigned char v);

    // read a 16 bit register
    int readIntRegister(unsigned char r);

    // read uncompensated temperature value
    unsigned int readUT();


    // read uncompensated pressure value
    long readUP();

    // Below there are the utility functions to get data from the sensor.

    // read temperature and pressure from sensor
    void readSensor();

    void  getCalibrationData();

    float press(void);

    float temp(void);

    float altitud(void);


private :

    I2C &i2c_;
    // variables to keep the values
    float temperature;
    float pressure;


    char i2cRaw[sizeof(I2C)];
};


#endif /* BMP085_H_ */

