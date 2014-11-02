10dof_IMU
=========

mbed lib and example for 10dof MEMS IMU from DFRobot.com

I started with the code from Aloïs Wolff's HK10DOF on mbed.com
URL=http://developer.mbed.org/users/pommzorz/code/HK10DOF/

His IMU from Hobby King uses most of the same sensors as the DFRobot,
except for the Gyro, L3G4200D instead of my ITG3200.

I modified the L3G4200D code to use the same calls and syntax but manage
the IGT3200.

I also corrected a few discrepencies.

So I have initialized this repository with the orignal core from mbed and 
then commited my changes.
