// MpuTesting.c : Reads MPU-6050 (±2 g, ±500 °/s) and prints raw values.
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <pigpio.h>

#define I2C_BUS           1
#define MPU_ADDR        0x68
#define REG_PWR_MGMT_1   0x6B
#define REG_ACCEL_CONFIG 0x1C
#define REG_GYRO_CONFIG  0x1B
#define REG_ACCEL_XOUT_H 0x3B

static int i2c_fd = -1;
void die(const char *s){perror(s);}

void mpu_init(void)
{
    if((i2c_fd=i2cOpen(I2C_BUS,MPU_ADDR,0))<0)
    {
        die("i2cOpen");
    }
    // wake up mpu
    i2cWriteByteData(i2c_fd, REG_PWR_MGMT_1, 0);
    // use +2g configuration 
    i2cWriteByteData(i2c_fd, REG_ACCEL_CONFIG, 0);
    // use ±500 deg/s configuration
    i2cWriteByteData(i2c_fd, REG_GYRO_CONFIG, 1 << 3);
    // 21 Hz DLPF
    i2cWriteByteData(i2c_fd, REG_CONFIG, 4);
}

void read_mpu(double a[3], double g[3])
{
    uint8_t b[14];
    if(i2cReadI2CBlockData(i2c_fd,REG_ACCEL_XOUT_H,b,14)!=14)
    {
        die("i2cRead");
    }
    int16_t ax=(b[0]<<8)|b[1];
    int16_t ay=(b[2]<<8)|b[3];
    int16_t az=(b[4]<<8)|b[5];
    int16_t gx=(b[8]<<8)|b[9];
    int16_t gy=(b[10]<<8)|b[11];
    int16_t gz=(b[12]<<8)|b[13];
    const double ASF=9.80665/16384.0, GSF=500.0/32768.0;
    a[0]=ax*ASF; a[1]=ay*ASF; a[2]=az*ASF;
    g[0]=gx*GSF; g[1]=gy*GSF; g[2]=gz*GSF;
}

int main(void)
{
    if(gpioInitialise()<0)
    {
        die("pigpio");
    } 
    mpu_init();

    double acc[3], gyr[3];
    while(1){
        read_mpu(acc,gyr);
        printf("ACC  %7.3f %7.3f %7.3f  m/s² | "
               "GYR  %6.1f %6.1f %6.1f  °/s\n",
               acc[0],acc[1],acc[2], gyr[0],gyr[1],gyr[2]);
        gpioDelay(200000); // 0.2 s
    }
    return 0;
}
