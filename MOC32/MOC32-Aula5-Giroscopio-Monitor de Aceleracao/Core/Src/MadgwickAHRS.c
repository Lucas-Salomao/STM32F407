/*
 * MadgwickAHRS.c
 *
 *  Created on: 13 de fev de 2023
 *      Author: salom
 */

#include "MadgwickAHRS.h"

void calculate_orientation(int16_t *accel_data, int16_t *gyro_data, int16_t *mag_data, float *orientation, float accel_scale, float gyro_scale, float mag_scale)
{
	float ax = (float) accel_data[0] * accel_scale;
	float ay = (float) accel_data[1] * accel_scale;
	float az = (float) accel_data[2] * accel_scale;
	float gx = (float) gyro_data[0] * gyro_scale;
	float gy = (float) gyro_data[1] * gyro_scale;
	float gz = (float) gyro_data[2] * gyro_scale;
	float mx = (float) mag_data[0] * mag_scale;
	float my = (float) mag_data[1] * mag_scale;
	float mz = (float) mag_data[2] * mag_scale;

	float norm;
	float hx, hy, bx, bz;
	float vx, vy, vz, wx, wy, wz;
	float ex, ey, ez;

	// auxiliary variables to reduce number of repeated operations
	float q0q0 = q0 * q0;
	float q0q1 = q0 * q1;
	float q0q2 = q0 * q2;
	float q0q3 = q0 * q3;
	float q1q1 = q1 * q1;
	float q1q2 = q1 * q2;
	float q1q3 = q1 * q3;
	float q2q2 = q2 * q2;
	float q2q3 = q2 * q3;
	float q3q3 = q3 * q3;

	// normalize accelerometer measurement
	norm = sqrt(ax * ax + ay * ay + az * az);
	if (norm == 0.0f)
		return; // handle NaN
	norm = 1.0f / norm;
	ax *= norm;
	ay *= norm;
	az *= norm;

	// normalize magnetometer measurement
	norm = sqrt(mx * mx + my * my + mz * mz);
	if (norm == 0.0f)
		return; // handle NaN
	norm = 1.0f / norm;
	mx *= norm;
	my *= norm;
	mz *= norm;

	// compute reference direction of flux
	hx = 2.0f * mx * (0.5f - q2q2 - q3q3) + 2.0f * my * (q1q2 - q0q3)
			+ 2.0f * mz * (q1q3 + q0q2);
	hy = 2.0f * mx * (q1q2 + q0q3) + 2.0f * my * (0.5f - q1q1 - q3q3)
			+ 2.0f * mz * (q2q3 - q0q1);
	bx = sqrt((hx * hx) + (hy * hy));
	bz = 2.0f * mx * (q1q3 - q0q2) + 2.0f * my * (q2q3 + q0q1)
			+ 2.0f * mz * (0.5f - q1q1 - q2q2);

	// estimated direction of gravity and flux (v and w)
	vx = 2.0f * (q1q3 - q0q2);
	vy = 2.0f * (q0q1 + q2q3);
	vz = q0q0 - q1q1 - q2q2 + q3q3;
	wx = 2.0f * bx * (0.5f - q2q2 - q3q3) + 2.0f * bz * (q1q3 - q0q2);
	wy = 2.0f * bx * (q1q2 - q0q3) + 2.0f * bz * (q0q1 + q2q3);
	wz = 2.0f * bx * (q0q2 + q1q3) + 2.0f * bz * (0.5f - q1q1 - q2q2);

	// error is cross product between estimated direction and measured direction of gravity
	ex = (ay * vz - az * vy) + (my * wz - mz * wy);
	ey = (az * vx - ax * vz) + (mz * wx - mx * wz);
	ez = (ax * vy - ay * vx) + (mx * wy - my * wy);

	// proportional and integral error compensation
	gx = gx + beta * ex;
	gy = gy + beta * ey;
	gz = gz + beta * ez;

	// integrate quaternion rate and normalize
	q0 = q0 + (-q1 * gx - q2 * gy - q3 * gz) * (0.5f * (1.0f / sampleFreq));
	q1 = q1 + (q0 * gx + q2 * gz - q3 * gy) * (0.5f * (1.0f / sampleFreq));
	q2 = q2 + (q0 * gy - q1 * gz + q3 * gx) * (0.5f * (1.0f / sampleFreq));
	q3 = q3 + (q0 * gz + q1 * gy - q2 * gx);

	// normalize quaternion
	norm = sqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
	if (norm == 0.0f)
		return; // handle NaN
	norm = 1.0f / norm;
	q0 *= norm;
	q1 *= norm;
	q2 *= norm;
	q3 *= norm;

	// convert quaternion to rotation matrix
	float rot_mat[9];
	rot_mat[0] = q0q0 + q1q1 - q2q2 - q3q3;
	rot_mat[1] = 2.0f * (q1 * q2 + q0 * q3);
	rot_mat[2] = 2.0f * (q1 * q3 - q0 * q2);
	rot_mat[3] = 2.0f * (q1 * q2 - q0 * q3);
	rot_mat[4] = q0q0 - q1q1 + q2q2 - q3q3;
	rot_mat[5] = 2.0f * (q2 * q3 + q0 * q1);
	rot_mat[6] = 2.0f * (q1 * q3 + q0 * q2);
	rot_mat[7] = 2.0f * (q2 * q3 - q0 * q1);
	rot_mat[8] = q0q0 - q1q1 - q2q2 + q3q3;

	// convert rotation matrix to Euler angles
	orientation[0] = atan2(rot_mat[5], rot_mat[8]);
	orientation[1] = -asin(rot_mat[2]);
	orientation[2] = atan2(rot_mat[1], rot_mat[0]);
}

void north_position(float *orientation)
{
	float pitch = orientation[0];
	float roll = orientation[1];
	float yaw = orientation[2];

	float North_x = cos(yaw) * cos(pitch);
	float North_y = sin(yaw) * cos(pitch);
	float North_z = sin(pitch);

	float heading = atan2(North_y, North_x);
}

void calculate_position(float* orientation, float *latitude, float *longitude)
{
    float heading = orientation[0]; //localizacao do norte
    float pitch = orientation[1];
    float roll = orientation[2];

    // Cálculo da direção do Norte
    float North = heading;

    // Cálculo da inclinação do dispositivo
    float inclination = pitch;

    // Atualização da posição geográfica
    *latitude = *latitude + ((float)sin(inclination) * (float)cos(North));
    *longitude = *longitude + ((float)sin(inclination) * (float)sin(North));
}
