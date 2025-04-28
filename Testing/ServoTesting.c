// servo_set.c : Usage  sudo ./servo_set <angle_deg> [servo#]
// servo# 1 → GPIO18 (default) , 2 → GPIO19
#include <stdio.h>
#include <stdlib.h>
#include <pigpio.h>

#define SERVO1_PIN 18  // PWM0
#define SERVO2_PIN 19  // PWM1
#define SERVO_MIN_US 1000
#define SERVO_MAX_US 2000

void die(const char*s){perror(s);}

int main(int argc,char*argv[])
{
    if(argc<2)
    {
        fprintf(stderr,"usage: %s <angle 0-180> [1|2]\n",argv[0]);
        return 1;
    }
    double angle=atof(argv[1]);
    if(angle<0||angle>180)
    {
        fprintf(stderr,"angle out of range\n");
        return 1;
    }

    int which=(argc>2) ? atoi(argv[2]) : 1;
    int pin=(which==2) ? SERVO2_PIN : SERVO1_PIN;

    if(gpioInitialise()<0)
    {
        die("pigpio");
    } 

    int pulse = SERVO_MIN_US + (int)((angle/180.0)*(SERVO_MAX_US-SERVO_MIN_US));
    gpioSetMode(pin,PI_OUTPUT);
    if(gpioServo(pin,pulse))
    {
        die("gpioServo");
    } 
    printf("Set servo %d (GPIO %d) to %.1f° (pulse %d µs)\n",
           which,pin,angle,pulse);
    gpioDelay(500000); // give servo time to move
    gpioTerminate();
    return 0;
}
