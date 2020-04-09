/*
This code is provided under the BSD license.
Copyright (c) 2014, Emlid Limited. All rights reserved.
Written by Igor Vereninov and Mikhail Avkhimenia
twitter.com/emlidtech || www.emlid.com || info@emlid.com

Application: Mahory AHRS algorithm supplied with data from MPU9250 and LSM9DS1.
Outputs roll, pitch and yaw in the console and sends quaternion
over the network - it can be used with 3D IMU visualizer located in
Navio/Applications/3D IMU visualizer.

To run this app navigate to the directory containing it and run following
commands: make sudo ./AHRS -i [sensor name] ipaddress portnumber Sensors names:
mpu is MPU9250, lsm is LSM9DS1. If you want to visualize IMU data on another
machine pass it's address and port For print help:
./AHRS -h

To achieve stable loop you need to run this application with a high priority
on a linux kernel with real-time patch. Raspbian distribution with real-time
kernel is available at emlid.com and priority can be set with chrt command:
chrt -f -p 99 PID
*/

#include <Common/MPU9250.h>
#include <Common/Util.h>
#include <Navio+/RCOutput_Navio.h>
#include <Navio2/LSM9DS1.h>
#include <Navio2/PWM.h>
#include <Navio2/RCOutput_Navio2.h>
#include <arpa/inet.h>
#include <memory>
#include <netinet/in.h>
#include <signal.h>
#include <stdint.h>
#include <stdio.h>
#include <sys/time.h>
#include <unistd.h>

#include "balance.hpp"
#include "pid.h"

#define G_SI 9.80665
#define PI 3.14159

AHRS::AHRS(std::unique_ptr<InertialSensor> imu) {
  sensor = move(imu);
  q0 = 1;
  q1 = 0;
  q2 = 0, q3 = 0;
  twoKi = 0;
  twoKp = 2;
}
void AHRS::update(float dt) {
  float recipNorm;
  float q0q0, q0q1, q0q2, q0q3, q1q1, q1q2, q1q3, q2q2, q2q3, q3q3;
  float hx, hy, bx, bz;
  float halfvx, halfvy, halfvz, halfwx, halfwy, halfwz;
  float halfex, halfey, halfez;
  float qa, qb, qc;

  float ax, ay, az;
  float gx, gy, gz;
  float mx, my, mz;

  sensor->update();
  sensor->read_accelerometer(&ax, &ay, &az);
  sensor->read_gyroscope(&gx, &gy, &gz);
  sensor->read_magnetometer(&mx, &my, &mz);

  // Use IMU algorithm if magnetometer measurement invalid (avoids NaN in
  // magnetometer normalisation)
  if ((mx == 0.0f) && (my == 0.0f) && (mz == 0.0f)) {
    updateIMU(dt);
    return;
  }

  // Compute feedback only if accelerometer measurement valid (avoids NaN in
  // accelerometer normalisation)
  if (!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {

    // Normalise accelerometer measurement
    recipNorm = invSqrt(ax * ax + ay * ay + az * az);
    ax *= recipNorm;
    ay *= recipNorm;
    az *= recipNorm;

    // Normalise magnetometer measurement
    recipNorm = invSqrt(mx * mx + my * my + mz * mz);
    mx *= recipNorm;
    my *= recipNorm;
    mz *= recipNorm;

    // Auxiliary variables to avoid repeated arithmetic
    q0q0 = q0 * q0;
    q0q1 = q0 * q1;
    q0q2 = q0 * q2;
    q0q3 = q0 * q3;
    q1q1 = q1 * q1;
    q1q2 = q1 * q2;
    q1q3 = q1 * q3;
    q2q2 = q2 * q2;
    q2q3 = q2 * q3;
    q3q3 = q3 * q3;

    // Reference direction of Earth's magnetic field
    hx = 2.0f *
         (mx * (0.5f - q2q2 - q3q3) + my * (q1q2 - q0q3) + mz * (q1q3 + q0q2));
    hy = 2.0f *
         (mx * (q1q2 + q0q3) + my * (0.5f - q1q1 - q3q3) + mz * (q2q3 - q0q1));
    bx = sqrt(hx * hx + hy * hy);
    bz = 2.0f *
         (mx * (q1q3 - q0q2) + my * (q2q3 + q0q1) + mz * (0.5f - q1q1 - q2q2));

    // Estimated direction of gravity and magnetic field
    halfvx = q1q3 - q0q2;
    halfvy = q0q1 + q2q3;
    halfvz = q0q0 - 0.5f + q3q3;
    halfwx = bx * (0.5f - q2q2 - q3q3) + bz * (q1q3 - q0q2);
    halfwy = bx * (q1q2 - q0q3) + bz * (q0q1 + q2q3);
    halfwz = bx * (q0q2 + q1q3) + bz * (0.5f - q1q1 - q2q2);

    // Error is sum of cross product between estimated direction and measured
    // direction of field vectors
    halfex = (ay * halfvz - az * halfvy) + (my * halfwz - mz * halfwy);
    halfey = (az * halfvx - ax * halfvz) + (mz * halfwx - mx * halfwz);
    halfez = (ax * halfvy - ay * halfvx) + (mx * halfwy - my * halfwx);

    // Compute and apply integral feedback if enabled
    if (twoKi > 0.0f) {
      integralFBx += twoKi * halfex * dt; // integral error scaled by Ki
      integralFBy += twoKi * halfey * dt;
      integralFBz += twoKi * halfez * dt;
      gx += integralFBx; // apply integral feedback
      gy += integralFBy;
      gz += integralFBz;
    } else {
      integralFBx = 0.0f; // prevent integral windup
      integralFBy = 0.0f;
      integralFBz = 0.0f;
    }

    // Apply proportional feedback
    gx += twoKp * halfex;
    gy += twoKp * halfey;
    gz += twoKp * halfez;
  }

  // Integrate rate of change of quaternion
  gx *= (0.5f * dt); // pre-multiply common factors
  gy *= (0.5f * dt);
  gz *= (0.5f * dt);
  qa = q0;
  qb = q1;
  qc = q2;
  q0 += (-qb * gx - qc * gy - q3 * gz);
  q1 += (qa * gx + qc * gz - q3 * gy);
  q2 += (qa * gy - qb * gz + q3 * gx);
  q3 += (qa * gz + qb * gy - qc * gx);

  // Normalise quaternion
  recipNorm = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
  q0 *= recipNorm;
  q1 *= recipNorm;
  q2 *= recipNorm;
  q3 *= recipNorm;
}

void AHRS::updateIMU(float dt) {
  float recipNorm;
  float halfvx, halfvy, halfvz;
  float halfex, halfey, halfez;
  float qa, qb, qc;

  float ax, ay, az;
  float gx, gy, gz;

  // Accel + gyro.
  sensor->update();
  sensor->read_accelerometer(&ax, &ay, &az);
  sensor->read_gyroscope(&gx, &gy, &gz);

  ax /= G_SI;
  ay /= G_SI;
  az /= G_SI;
  gx *= (180 / PI) * 0.0175;
  gy *= (180 / PI) * 0.0175;
  gz *= (180 / PI) * 0.0175;

  gx -= gyroOffset[0];
  gy -= gyroOffset[1];
  gz -= gyroOffset[2];

  // Compute feedback only if accelerometer measurement valid (avoids NaN in
  // accelerometer normalisation)
  if (!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {

    // Normalise accelerometer measurement
    recipNorm = invSqrt(ax * ax + ay * ay + az * az);
    ax *= recipNorm;
    ay *= recipNorm;
    az *= recipNorm;

    // Estimated direction of gravity and vector perpendicular to magnetic flux
    halfvx = q1 * q3 - q0 * q2;
    halfvy = q0 * q1 + q2 * q3;
    halfvz = q0 * q0 - 0.5f + q3 * q3;

    // Error is sum of cross product between estimated and measured direction of
    // gravity
    halfex = (ay * halfvz - az * halfvy);
    halfey = (az * halfvx - ax * halfvz);
    halfez = (ax * halfvy - ay * halfvx);

    // Compute and apply integral feedback if enabled
    if (twoKi > 0.0f) {
      integralFBx += twoKi * halfex * dt; // integral error scaled by Ki
      integralFBy += twoKi * halfey * dt;
      integralFBz += twoKi * halfez * dt;
      gx += integralFBx; // apply integral feedback
      gy += integralFBy;
      gz += integralFBz;
    } else {
      integralFBx = 0.0f; // prevent integral windup
      integralFBy = 0.0f;
      integralFBz = 0.0f;
    }

    // Apply proportional feedback
    gx += twoKp * halfex;
    gy += twoKp * halfey;
    gz += twoKp * halfez;
  }

  // Integrate rate of change of quaternion
  gx *= (0.5f * dt); // pre-multiply common factors
  gy *= (0.5f * dt);
  gz *= (0.5f * dt);
  qa = q0;
  qb = q1;
  qc = q2;
  q0 += (-qb * gx - qc * gy - q3 * gz);
  q1 += (qa * gx + qc * gz - q3 * gy);
  q2 += (qa * gy - qb * gz + q3 * gx);
  q3 += (qa * gz + qb * gy - qc * gx);

  // Normalise quaternion
  recipNorm = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
  q0 *= recipNorm;
  q1 *= recipNorm;
  q2 *= recipNorm;
  q3 *= recipNorm;
}

void AHRS::setGyroOffset() {
  //---------------------- Calculate the offset -----------------------------

  float offset[3] = {0.0, 0.0, 0.0};
  float gx, gy, gz;

  //----------------------- MPU initialization ------------------------------

  sensor->initialize();

  //-------------------------------------------------------------------------

  printf("Beginning Gyro calibration...\n");
  for (int i = 0; i < 100; i++) {
    sensor->update();
    sensor->read_gyroscope(&gx, &gy, &gz);

    gx *= 180 / PI;
    gy *= 180 / PI;
    gz *= 180 / PI;

    offset[0] += gx * 0.0175;
    offset[1] += gy * 0.0175;
    offset[2] += gz * 0.0175;

    usleep(10000);
  }
  offset[0] /= 100.0;
  offset[1] /= 100.0;
  offset[2] /= 100.0;

  printf("Offsets are: %f %f %f\n", offset[0], offset[1], offset[2]);

  gyroOffset[0] = offset[0];
  gyroOffset[1] = offset[1];
  gyroOffset[2] = offset[2];
}

void AHRS::getEuler(float *roll, float *pitch, float *yaw) {
  *roll = atan2(2 * (q0 * q1 + q2 * q3), 1 - 2 * (q1 * q1 + q2 * q2)) * 180.0 /
          M_PI;
  *pitch = asin(2 * (q0 * q2 - q3 * q1)) * 180.0 / M_PI;
  *yaw = atan2(2 * (q0 * q3 + q1 * q2), 1 - 2 * (q2 * q2 + q3 * q3)) * 180.0 /
         M_PI;
}

float AHRS::invSqrt(float x) {
  float halfx = 0.5f * x;
  float y = x;
  long i = *(long *)&y;
  i = 0x5f3759df - (i >> 1);
  y = *(float *)&i;
  y = y * (1.5f - (halfx * y * y));
  return y;
}

float AHRS::getW() { return q0; }

float AHRS::getX() { return q1; }

float AHRS::getY() { return q2; }

float AHRS::getZ() { return q3; }

class Socket {

public:
  Socket(char *ip, char *port) {
    sockfd = socket(AF_INET, SOCK_DGRAM, 0);
    servaddr.sin_family = AF_INET;
    servaddr.sin_addr.s_addr = inet_addr(ip);
    servaddr.sin_port = htons(atoi(port));
  }

  Socket() {
    sockfd = socket(AF_INET, SOCK_DGRAM, 0);
    servaddr.sin_family = AF_INET;
    servaddr.sin_addr.s_addr = inet_addr("127.0.0.1");
    servaddr.sin_port = htons(7000);
  }

  void output(float W, float X, float Y, float Z, int Hz) {
    sprintf(sendline, "%10f %10f %10f %10f %dHz\n", W, X, Y, Z, Hz);
    sendto(sockfd, sendline, strlen(sendline), 0, (struct sockaddr *)&servaddr,
           sizeof(servaddr));
  }

private:
  int sockfd;
  struct sockaddr_in servaddr = {0};
  char sendline[80];
};

std::unique_ptr<InertialSensor> get_inertial_sensor(std::string sensor_name) {
  if (sensor_name == "mpu") {
    printf("Selected: MPU9250\n");
    auto ptr = std::unique_ptr<InertialSensor>{new MPU9250()};
    return ptr;
  } else if (sensor_name == "lsm") {
    printf("Selected: LSM9DS1\n");
    auto ptr = std::unique_ptr<InertialSensor>{new LSM9DS1()};
    return ptr;
  } else {
    return NULL;
  }
}

void print_help() {
  printf("Possible parameters:\nSensor selection: -i [sensor name]\n");
  printf("Sensors names: mpu is MPU9250, lsm is LSM9DS1\nFor help: -h\n");
  printf("If you want to visualize IMU data on another machine,\n");
  printf("add IP address and port number (by default 7000):\n");
  printf("-i [sensor name] ipaddress portnumber\n");
}

std::string get_sensor_name(int argc, char *argv[]) {
  if (get_navio_version() == NAVIO2) {

    if (argc < 2) {
      printf("Enter parameter\n");
      print_help();
      return std::string();
    }

    // prevent the error message
    opterr = 0;
    int parameter;

    while ((parameter = getopt(argc, argv, "i:h")) != -1) {
      switch (parameter) {
      case 'i':
        return optarg;
      case 'h':
        print_help();
        return "";
      case '?':
        printf("Wrong parameter.\n");
        print_help();
        return std::string();
      }
    }

  } else { // sensor on NAVIO+

    return "mpu";
  }
  return "";
}

struct roll_dt {
  float roll;
  float dt;
};

//============================== Main loop ====================================

roll_dt imuLoop(AHRS *ahrs) {
  // Orientation data

  float roll, pitch, yaw;

  struct timeval tv;
  float dt;
  // Timing data

  static float maxdt;
  static float mindt = 0.01;
  static float dtsumm = 0;
  static int isFirst = 1;
  static unsigned long previoustime, currenttime;

  //----------------------- Calculate delta time ----------------------------

  gettimeofday(&tv, NULL);
  previoustime = currenttime;
  currenttime = 1000000 * tv.tv_sec + tv.tv_usec;
  dt = (currenttime - previoustime) / 1000000.0;
  if (dt < 1 / 1300.0)
    usleep((1 / 1300.0 - dt) * 1000000);
  gettimeofday(&tv, NULL);
  currenttime = 1000000 * tv.tv_sec + tv.tv_usec;
  dt = (currenttime - previoustime) / 1000000.0;

  //-------- Read raw measurements from the MPU and update AHRS --------------

  ahrs->updateIMU(dt);

  //------------------------ Read Euler angles ------------------------------

  ahrs->getEuler(&roll, &pitch, &yaw);

  //------------------- Discard the time of the first cycle -----------------

  if (!isFirst) {
    if (dt > maxdt)
      maxdt = dt;
    if (dt < mindt)
      mindt = dt;
  }
  isFirst = 0;

  return roll_dt{roll, dt};
}

#define SERVO_MIN 1000 /*mS*/
#define SERVO_MID 1500 /*mS*/
#define SERVO_MAX 2000 /*mS*/

#define PWM_OUTPUT_WHEEL_LEFT 1
#define PWM_OUTPUT_WHEEL_RIGHT 2

using namespace Navio;

std::unique_ptr<RCOutput> get_rcout() {
  if (get_navio_version() == NAVIO2) {
    auto ptr = std::unique_ptr<RCOutput>{new RCOutput_Navio2()};
    return ptr;
  } else {
    auto ptr = std::unique_ptr<RCOutput>{new RCOutput_Navio()};
    return ptr;
  }
}

//=============================================================================

void stop_motors_handler(sig_atomic_t s) {
  printf("Terminating greacfully");
  auto pwm = get_rcout();
  pwm->set_duty_cycle(PWM_OUTPUT_WHEEL_LEFT, SERVO_MID);
  pwm->set_duty_cycle(PWM_OUTPUT_WHEEL_RIGHT, SERVO_MID);
  exit(0);
}

int main(int argc, char *argv[]) {
  signal(SIGINT, stop_motors_handler);

  // Check to be the only user
  if (check_apm()) {
    return EXIT_FAILURE;
  }
  // Setup IMU
  auto sensor_name = get_sensor_name(argc, argv);

  if (sensor_name.empty())
    return EXIT_FAILURE;

  auto imu = get_inertial_sensor(sensor_name);

  if (!imu) {
    printf("Wrong sensor name. Select: mpu or lsm\n");
    return EXIT_FAILURE;
  }

  if (!imu->probe()) {
    printf("Sensor not enable\n");
    return EXIT_FAILURE;
  }

  // Setup PWM Output
  auto pwm = get_rcout();

  if (getuid()) {
    fprintf(stderr, "Not root. Please launch like this: sudo %s\n", argv[0]);
    return EXIT_FAILURE;
  }

  if (!(pwm->initialize(PWM_OUTPUT_WHEEL_LEFT))) {
    printf("Could not initialize left wheel\n");
    return EXIT_FAILURE;
  }

  if (!(pwm->initialize(PWM_OUTPUT_WHEEL_RIGHT))) {
    printf("Could not initialize right wheel\n");
    return EXIT_FAILURE;
  }

  pwm->set_frequency(PWM_OUTPUT_WHEEL_LEFT, 50);
  pwm->set_frequency(PWM_OUTPUT_WHEEL_RIGHT, 50);

  if (!(pwm->enable(PWM_OUTPUT_WHEEL_LEFT))) {
    return EXIT_FAILURE;
  }

  if (!(pwm->enable(PWM_OUTPUT_WHEEL_RIGHT))) {
    return EXIT_FAILURE;
  }

  auto ahrs = std::unique_ptr<AHRS>{new AHRS(move(imu))};

  // Setup gyroscope offset
  ahrs->setGyroOffset();

  // Set motors to off
  pwm->set_duty_cycle(PWM_OUTPUT_WHEEL_LEFT, SERVO_MID);
  pwm->set_duty_cycle(PWM_OUTPUT_WHEEL_RIGHT, SERVO_MID);

  PID pid_v = PID(-3, 3, 0.001, 0.0001, 0);

  PID pid_roll =
      PID(SERVO_MIN - SERVO_MID, SERVO_MAX - SERVO_MID, 160, 0.0018, 0.00005);

  float setpoint_roll = 1.50;
  float setpoint_v = 0;
  float dtsumm;

  {
    printf("Waiting...\n");
    roll_dt measurement{100, 0};
    while (measurement.roll > 3 || measurement.roll < -3) {
      measurement = imuLoop(ahrs.get());
    }
    printf("ready! %+05.2f\n", measurement.roll);
  }

  while (true) {
    roll_dt measurement = imuLoop(ahrs.get());
    float increment =
        pid_roll.calculate(setpoint_roll, measurement.roll, measurement.dt);

    setpoint_roll = pid_v.calculate(0, increment, measurement.dt);

    float pwm_target = SERVO_MID + increment;

    dtsumm += measurement.dt;
    if (dtsumm > 0.025) {
      pwm->set_duty_cycle(PWM_OUTPUT_WHEEL_LEFT, pwm_target);
      pwm->set_duty_cycle(PWM_OUTPUT_WHEEL_RIGHT, pwm_target);
      // Console output
      printf(
          "SETPOINT: %+05.2f ROLL: %+05.2f INC: %+05.2f TARGET: %+05.2f PERIOD "
          "%.4fs RATE "
          "%dHz \n",
          setpoint_roll, measurement.roll, increment, pwm_target,
          measurement.dt, int(1 / measurement.dt));
      dtsumm = 0;
    }
  }
}
