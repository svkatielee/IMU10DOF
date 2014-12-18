IMU10DOF
=========

Update Dec 18 2014
------------------
Tested and fixed minor issues by downloading to the updated gcc4mbed (with rev 92 mbed)
environment off line. compiled and run fine. Removed old files and added my deploy to 
the makefile.

------------------
mbed lib and example for 10dof MEMS IMU from DFRobot.com for use on ST's NUCLEOF401RE.
http://www.dfrobot.com/wiki/index.php/10_DOF_Sensor_%28SKU:SEN0140%29
    Adxl345 accelerometer
    ITG3200 gyro
    HMC5883L Compass
    BMP085 pressure sensor

I started with the code from Aloïs Wolff's HK10DOF on mbed.com
URL=http://developer.mbed.org/users/pommzorz/code/HK10DOF/

His IMU from Hobby King uses most of the same sensors as the DFRobot,
only the Gyro is different, L3G4200D instead of my ITG3200.

I modified the L3G4200D code to use the same calls and syntax but manage
the IGT3200.

I also corrected a few discrepancies. I2C bus frequency, initialize variables and 
data types. It seems either mbed or the NUCLEOF401RE are very picky about type mismatch.

The main.cpp is a demo program to exercise the library. It was compiled off-line from mbed 
using a makefile. I setup locally by following instructions from Adam Green here:
https://github.com/adamgreen/gcc4mbed  and here: https://github.com/adamgreen/gcc4mbed/blob/master/notes/porting.creole#readme

I used information on this page http://blog.bitify.co.uk/2013/11/using-complementary-filter-to-combine.html
to understand and debug this code. The gunplot was especially helpful.
