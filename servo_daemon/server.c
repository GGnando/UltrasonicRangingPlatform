#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>
#include <math.h>
#include <pigpio.h>
#include <pthread.h>
#include <string.h>
#include <unistd.h>
#include <errno.h>
#include <netinet/in.h>
#include <sys/socket.h>

#define SERVER_PORT 5000
#define SAMPLE_HZ 10
#define SERVO_MIN_US 1000
#define SERVO_MAX_US 2000

// ultrasonic sensors
#define SERVO1_PIN 18
#define SERVO2_PIN 19
#define US1_TRIG 23
#define US1_ECHO 24
#define US2_TRIG 25
#define US2_ECHO 8
#define US3_TRIG 7
#define US3_ECHO 1
#define NUMBER_OF_US 3

// MPU-6050
#define I2C_BUS 1
#define MPU_ADDR 0x68
#define REG_PWR_MGMT_1 0x6B
#define REG_ACCEL_CONFIG 0x1C
#define REG_GYRO_CONFIG 0x1B
#define REG_CONFIG 0x1A
#define REG_ACCEL_XOUT_H 0x3B
#define REG_GYRO_XOUT_H 0x43

// servo
#define NUMBER_OF_SERVOS 2

static int i2c_fd;

// telemetry to client
pthread_mutex_t telemtry_mutex = PTHREAD_MUTEX_INITIALIZER;
int client_fd = -1;

struct
{
    double dist[NUMBER_OF_US];
    double acc[3];
    double gyr[3];
    double servo[NUMBER_OF_SERVOS];
    bool cfg[NUMBER_OF_SERVOS];
} T = {{0}, {0}, {0}, {90, 90}, {false, false}};

/* helpers */
static void die(const char *s)
{
    #ifdef DEBUG
    perror(s);
    #endif
}

/* ── MPU init / read ─────────────────────────────────────────────── */
static void mpu_init(void)
{
    if ((i2c_fd = i2cOpen(I2C_BUS, MPU_ADDR, 0)) < 0)
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

static void mpu_read_all(double a[3], double g[3])
{
    uint8_t b[14];
    if (i2cReadI2CBlockData(i2c_fd, REG_ACCEL_XOUT_H, b, 14) != 14)
    {
        die("i2cRead");
    }

    int16_t ax = (b[0] << 8) | b[1];
    int16_t ay = (b[2] << 8) | b[3];
    int16_t az = (b[4] << 8) | b[5];
    int16_t gx = (b[8] << 8) | b[9];
    int16_t gy = (b[10] << 8) | b[11];
    int16_t gz = (b[12] << 8) | b[13];

    // from mpu6050 user manual
    const double ASF = 9.80665 / 16384.0; /* 16 384 LSB ≙ 1 g */
    const double GSF = 500.0 / 32768.0;   /* ±500 °/s */
    // xyz memory layout
    a[0] = ax * ASF;
    a[1] = ay * ASF;
    a[2] = az * ASF;
    g[0] = gx * GSF;
    g[1] = gy * GSF;
    g[2] = gz * GSF;
}

static double pulse_us(int trig, int echo)
{
    // set trigger pin high for 10 microseconds
    gpioWrite(trig, 0);
    gpioDelay(2);
    gpioWrite(trig, 1);
    gpioDelay(10);
    gpioWrite(trig, 0);

    uint32_t t0 = gpioTick();
    while (!gpioRead(echo))
    {
        if (gpioTick() - t0 > 60000)
        {
            return -1;
        }
    }

    uint32_t t1 = gpioTick();
    while (gpioRead(echo))
    {
        if (gpioTick() - t1 > 30000)
        {
            return -1;
        }
    }

    return (double)(gpioTick() - t1);
}

static inline double us2cm(double us)
{
    // speed of sound in one second to cm
    return us / 58.0;
}

void get_ultrasonic_distance(int us_pin[NUMBER_OF_US][2])
{
    for (int i = 0; i < 3; i++)
    {
        double t = pulse_us(us_pin[i][0], us_pin[i][1]);
        T.dist[i] = (t < 0) ? -1 : us2cm(t);
    }
}


// Servo (overrides PWM commands on same GPIO)
static void servo_us(int pin, int us)
{
    // Start/stop servo pulses on a GPIO
    gpioServo(pin, us);
}

static void steer_from_acc_z(double az)
{
    const double stable_threshold_accel = 4.0;      
    const double STEP_DEG = 2.0;
    /* keep running count of consecutive valid samples */
    static int posRun = 0, negRun = 0;

    int dir = 0; /* +1 = CW, -1 = CCW, 0 = idle */

    if (az > stable_threshold_accel)
    {
        ++posRun;
        negRun = 0;
        if (posRun >= 3)
        {
            dir = 1;
        }
    }
    else if (az < -stable_threshold_accel)
    {
        negRun++;
        posRun = 0;
        if (negRun >= 3)
        {
            dir = -1;
        }
    }
    else
    {
        // dont care about measurements between these values. 
        // assume accel is resting or is transitioning
        posRun = 0; 
        negRun = 0;
    }

    if(dir)
    {
        for (int servo = 0; servo < NUMBER_OF_SERVOS; ++servo)
        {
            if (!T.cfg[servo])
            {
                // move only active servo
                continue; 
            }
    
            T.servo[servo] += dir * STEP_DEG;
            if (T.servo[servo] < 0)
            {
                T.servo[servo] = 0;
            }
            if (T.servo[servo] > 180)
            {
                T.servo[servo] = 180;
            }
    
            int pulse = SERVO_MIN_US +
                        (int)((T.servo[servo] / 180.0) * (SERVO_MAX_US - SERVO_MIN_US));
            servo_us(servo ? SERVO2_PIN : SERVO1_PIN, pulse);
        }
    }    
}

static int tcp_listen(void)
{
    int fd = socket(AF_INET, SOCK_STREAM, 0);
    if (fd < 0)
    {
        die("socket");
    }

    int one = 1;
    setsockopt(fd, SOL_SOCKET, SO_REUSEADDR, &one, sizeof(one));
    struct sockaddr_in a = {0};
    a.sin_family = AF_INET;
    a.sin_addr.s_addr = htonl(INADDR_ANY);
    a.sin_port = htons(SERVER_PORT);
    if (bind(fd, (struct sockaddr *)&a, sizeof(a)))
    {
        die("bind");
    }
    if (listen(fd, 1))
    {
        die("listen");
    }
    return fd;
}

void *sampler(void *arg)
{
    const int us_pin[NUMBER_OF_US][2] = {{US1_TRIG, US1_ECHO}, {US2_TRIG, US2_ECHO}, {US3_TRIG, US3_ECHO}};
    const double dt = 1.0 / SAMPLE_HZ;
    double gX = 0, gY = 0, yaw = 0;

    while (1)
    {
        pthread_mutex_lock(&telemtry_mutex);
        
        // handle all logic related to ultrasonic sensors
        get_ultrasonic_distance(us_pin);

        mpu_read_all(T.acc, T.gyr);

        steer_from_acc_z(T.acc[2]);

        pthread_mutex_unlock(&telemtry_mutex);

        /* ---- telemetry ------------------------------------------ */
        if (client_fd >= 0)
        {
            char line[192];
            pthread_mutex_lock(&telemtry_mutex);
            int n = snprintf(line, sizeof(line),
                             "T %.1f %.1f %.1f  %.3f %.3f %.3f  %.1f %.1f %.1f  "
                             "%.1f %.1f %.1f  %.1f %.1f %d %d\n",
                             T.dist[0], T.dist[1], T.dist[2],
                             T.acc[0], T.acc[1], T.acc[2],
                             T.gyr[0], T.gyr[1], T.gyr[2],
                             T.servo[0], T.servo[1],
                             (int)T.cfg[0], (int)T.cfg[1]);
            pthread_mutex_unlock(&telemtry_mutex);
            if (write(client_fd, line, n) <= 0)
            {
                close(client_fd);
                client_fd = -1;
            }
        }
        gpioDelay((uint32_t)(dt * 1e6));
    }
    return NULL;
}

/* ── main ───────────────────────────────────────────────────────── */
int main(void)
{
    if (gpioInitialise() < 0)
    {
        die("pigpio");
    }

    int trig[] = {US1_TRIG, US2_TRIG, US3_TRIG};
    int echo[] = {US1_ECHO, US2_ECHO, US3_ECHO};
    for (int i = 0; i < 3; i++)
    {
        gpioSetMode(trig[i], PI_OUTPUT);
        gpioWrite(trig[i], 0);
        gpioSetMode(echo[i], PI_INPUT);
    }
    gpioSetMode(SERVO1_PIN, PI_OUTPUT);
    gpioSetMode(SERVO2_PIN, PI_OUTPUT);

    mpu_init();
    pthread_t tid;
    pthread_create(&tid, NULL, sampler, NULL);

    int lfd = tcp_listen();
    printf("servo_daemon listening on %d …\n", SERVER_PORT);

    char cmd[32];
    while (1)
    {
        if (client_fd < 0)
        {
            client_fd = accept(lfd, NULL, NULL);
            if (client_fd >= 0)
            {
                puts("Client connected.");
            }
        }
        else
        {
            ssize_t r = read(client_fd, cmd, sizeof(cmd) - 1);
            if (r <= 0)
            {
                close(client_fd);
                client_fd = -1;
                continue;
            }
            cmd[r] = '\0';
            pthread_mutex_lock(&telemtry_mutex);
            if (!strncmp(cmd, "CFG1_START", 10))
            {
                T.cfg[0] = true;
            }
            else if (!strncmp(cmd, "CFG1_STOP", 9))
            {
                T.cfg[0] = false;
            }
            else if (!strncmp(cmd, "CFG2_START", 10))
            {
                T.cfg[1] = true;
            }
            else if (!strncmp(cmd, "CFG2_STOP", 9))
            {
                T.cfg[1] = false;
            }
            else if (!strncmp(cmd, "QUIT", 4))
            {
                close(client_fd);
                client_fd = -1;
            }
            pthread_mutex_unlock(&telemtry_mutex);
        }
    }
    return 0;
}
