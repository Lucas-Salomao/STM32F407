/*
 * MPU9250.h
 *
 *  Created on: Feb 13, 2023
 *      Author: salom
 */

#ifndef INC_MPU9250_H_
#define INC_MPU9250_H_

#include "stm32f4xx_hal.h"

I2C_HandleTypeDef hi2c;

int16_t accelCount[3];
int16_t gyroCount[3];
int16_t magCount[3];

float ax, ay, az;
float gx, gy, gz;
float mx, my, mz;
float ex, ey, ez;
float dt, Kp;

float q[4] = { 1.0f, 0.0f, 0.0f, 0.0f };

float yaw, pitch, roll;
float yawRad, pitchRad, rollRad;

#define MAGNETOMETER_SCALE 0.15f
float magnetometer_scale = MAGNETOMETER_SCALE;

// Scale options for accelerometer
#define MPU9250_ACCEL_SCALE_2G 0x00
#define MPU9250_ACCEL_SCALE_4G 0x08
#define MPU9250_ACCEL_SCALE_8G 0x10
#define MPU9250_ACCEL_SCALE_16G 0x18

// Scale options for gyroscope
#define MPU9250_GYRO_SCALE_250DPS 0x00
#define MPU9250_GYRO_SCALE_500DPS 0x08
#define MPU9250_GYRO_SCALE_1000DPS 0x10
#define MPU9250_GYRO_SCALE_2000DPS 0x18

// Scale options for magnetometer
#define MPU9250_MAG_SCALE_14BITS 0x00
#define MPU9250_MAG_SCALE_16BITS 0x10

// MPU9250 Registers

#define ACCEL_XOUT_H 0x3B
#define ACCEL_XOUT_L 0x3C
#define ACCEL_YOUT_H 0x3D
#define ACCEL_YOUT_L 0x3E
#define ACCEL_ZOUT_H 0x3F
#define ACCEL_ZOUT_L 0x40
#define TEMP_OUT_H 0x41
#define TEMP_OUT_L 0x42
#define GYRO_XOUT_H 0x43
#define GYRO_XOUT_L 0x44
#define GYRO_YOUT_H 0x45
#define GYRO_YOUT_L 0x46
#define GYRO_ZOUT_H 0x47
#define GYRO_ZOUT_L 0x48
#define EXT_SENS_DATA_00 0x49
#define EXT_SENS_DATA_01 0x4A
#define EXT_SENS_DATA_02 0x4B
#define EXT_SENS_DATA_03 0x4C
#define EXT_SENS_DATA_04 0x4D
#define EXT_SENS_DATA_05 0x4E
#define EXT_SENS_DATA_06 0x4F
#define EXT_SENS_DATA_07 0x50
#define EXT_SENS_DATA_08 0x51
#define EXT_SENS_DATA_09 0x52
#define EXT_SENS_DATA_10 0x53
#define EXT_SENS_DATA_11 0x54
#define EXT_SENS_DATA_12 0x55
#define EXT_SENS_DATA_13 0x56
#define EXT_SENS_DATA_14 0x57
#define EXT_SENS_DATA_15 0x58
#define EXT_SENS_DATA_16 0x59
#define EXT_SENS_DATA_17 0x5A
#define EXT_SENS_DATA_18 0x5B
#define EXT_SENS_DATA_19 0x5C
#define EXT_SENS_DATA_20 0x5D
#define EXT_SENS_DATA_21 0x5E
#define EXT_SENS_DATA_22 0x5F
#define EXT_SENS_DATA_23 0x60
#define I2C_SLV0_DO 0x63
#define I2C_SLV1_DO 0x64
#define I2C_SLV2_DO 0x65
#define I2C_SLV3_DO 0x66
#define I2C_MST_DELAY_CTRL 0x67
#define MPU9250_ADDRESS 0x68
#define MOT_DETECT_CTRL 0x69
#define USER_CTRL 0x6A
#define PWR_MGMT_1 0x6B
#define PWR_MGMT_2 0x6C
#define WHO_AM_I_VAL 0x71
#define FIFO_COUNTH 0x72
#define FIFO_COUNTL 0x73
#define FIFO_R_W 0x74
#define WHO_AM_I_REGISTER 0x75
#define XA_OFFSET_H 0x77
#define XA_OFFSET_L 0x78
#define YA_OFFSET_H 0x7A
#define YA_OFFSET_L 0x7B
#define ZA_OFFSET_H 0x7D
#define ZA_OFFSET_L 0x7E

#define GYRO_CONFIG 0x1B
#define ACCEL_CONFIG 0x1C
#define ACCEL_CONFIG_2 0x1D

#define AK8963_ADDRESS 0x0C
#define AK8963_WHO_AM_I 0x00
#define AK8963_WHO_AM_I_VAL 0x48
#define AK8963_INFO 0x01
#define AK8963_ST1 0x02
#define AK8963_XOUT_L 0x03
#define AK8963_XOUT_H 0x04
#define AK8963_YOUT_L 0x05
#define AK8963_YOUT_H 0x06
#define AK8963_ZOUT_L 0x07
#define AK8963_ZOUT_H 0x08
#define AK8963_ST2 0x09
#define AK8963_CNTL1 0x0A
#define AK8963_ASTC 0x0C
#define AK8963_I2CDIS 0x0F
#define AK8963_ASAX 0x10
#define AK8963_ASAY 0x11
#define AK8963_ASAZ 0x12

void MPU9250_init(I2C_HandleTypeDef i2c);
void MPU9250_read_accel_data(int16_t* destination);
void MPU9250_read_gyro_data(int16_t* destination);
void MPU9250_read_mag_data(int16_t* destination);
void MPU9250_read_all_data(int16_t* accel_data, int16_t* gyro_data, int16_t* mag_data);
void updateIMU();
void updateOrientation();
void getEulerAngles_rad(float *roll, float *pitch, float *yaw);
void getEulerAngles_deg(float *roll, float *pitch, float *yaw);


#endif /* INC_MPU9250_H_ */
