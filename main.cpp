
#include "mbed.h"
//#include "math.h"

#include "HMC5883L.h"
#include "ADXL345_I2C.h"
#include "L3G4200D.h"
#include "BMP085.h"
#include "HK10DOF.h"

//#include "PwmIn.h"

 /*
PwmOut rled(LED_RED);
PwmOut gled(LED_GREEN);
PwmOut bled(LED_BLUE);
 */
HK10DOF imu(PB_9, PB_8);
float ypr[3];
float values[9];
double head;
int x[3];
byte s[4];
int main()
{
 //   rled=1;
 //   gled=1;

    imu.pc.printf("Ceci est un Test\r\n");
    imu.init(true);
    //imu.gyro.status(s);imu.pc.printf(" GYR: %04i %04i %04i %04i \n\r",s[0],s[1],s[2],s[3]);
    //imu.gyro.init();
   // wait(1);
    imu.gyro.status(s);imu.pc.printf(" GYR2: %x %x %x %x \n\r",s[0],s[1],s[2],s[3]);
    imu.pc.printf("Test passe\r\n");
    float alt=0;

    wait(3);
    imu.pc.printf("\n\r");
    
    while(1) {
        imu.getValues(values);
        imu.getEuler(ypr);
       // imu.gyro.readFin(ypr);  //imu.pc.printf(" GYR: %04i %04i %04i \n\r",x[0],x[1],x[2]);

        imu.pc.printf("YAW: %+5.2f PITCH: %+5.2f ROLL: %+5.2f ",ypr[0],ypr[1],ypr[2]);
        alt=imu.getBaroAlt();
        imu.pc.printf("ALT : %03.0f ",alt);
		head = imu.magn.getHeadingXY(); imu.pc.printf("HDG: %+4.1f ",(head * 180/M_PI));
		//imu.magn.getXYZ
       // imu.pc.printf("ACC: %04.0f %04.0f %04.0f GYR: %04.0f %04.0f %04.0f MAG: %04.0f %04.0f %04.0f \n\r",values[0], values[1], values[2], values[3], values[4], values[5], values[6], values[7], values[8] );
        wait_ms(1);
   //     bled=!bled;

        imu.pc.printf("\n\r");

    }





}
