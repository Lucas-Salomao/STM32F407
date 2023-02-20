/*
 * MadgwickAHRS.h
 *
 *  Created on: 13 de fev de 2023
 *      Author: salom
 */

#ifndef INC_MADGWICKAHRS_H_
#define INC_MADGWICKAHRS_H_

#include "math.h"
#include "stdint.h"

#define sampleFreq  100.0f          // sample frequency in Hz
#define betaDef     0.1f            // 2 * proportional gain

float beta = betaDef;
float q0 = 1.0f, q1 = 0.0f, q2 = 0.0f, q3 = 0.0f; // quaternion of sensor frame relative to auxiliary frame

void calculate_orientation(int16_t *accel_data, int16_t *gyro_data, int16_t *mag_data, float *orientation, float accel_scale, float gyro_scale, float mag_scale);
void north_position(float *orientation);
void calculate_position(float *orientation, float *latitude, float *longitude);

#endif /* INC_MADGWICKAHRS_H_ */
