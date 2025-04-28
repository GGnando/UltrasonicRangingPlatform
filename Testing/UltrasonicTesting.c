// UltrasonicTesting.c : Fires 3 HC-SR04 sensors and prints distances in cm.
#include <stdio.h>
#include <stdlib.h>
#include <pigpio.h>

#define US1_TRIG 23
#define US1_ECHO 24
#define US2_TRIG 25
#define US2_ECHO  8
#define US3_TRIG  7
#define US3_ECHO  1

double pulse_us(int trig,int echo)
{
    gpioWrite(trig,0); gpioDelay(2);
    gpioWrite(trig,1); gpioDelay(10);
    gpioWrite(trig,0);

    uint32_t t0=gpioTick();
    while(!gpioRead(echo))
    {
        if(gpioTick()-t0>60000)
        {
            return -1;
        }
    }  
    uint32_t start=gpioTick();
    while(gpioRead(echo))
    {
        if(gpioTick()-start>30000)
        {
            return -1;
        }
    }   
    return (double)(gpioTick()-start);
}
static inline double us2cm(double us){ return us/58.0; }

int main(void)
{
    if(gpioInitialise()<0)
    {
        perror("pigpio");
        return 1;
    }

    int trig[3]={US1_TRIG,US2_TRIG,US3_TRIG};
    int echo[3]={US1_ECHO,US2_ECHO,US3_ECHO};
    for(int i=0;i<3;i++)
    {
        gpioSetMode(trig[i],PI_OUTPUT);
        gpioWrite(trig[i],0);
        gpioSetMode(echo[i],PI_INPUT);
    }

    while(1)
    {
        for(int i=0;i<3;i++)
        {
            double t=pulse_us(trig[i],echo[i]);
            if(t<0)
            {
                printf("US%d:  ---  ",i+1);
            }
            else
            {
                printf("US%d: %6.1fcm  ",i+1, us2cm(t));
            }    
        }
        printf("\n");
        gpioDelay(1000000); // 1 s
    }
    return 0;
}
