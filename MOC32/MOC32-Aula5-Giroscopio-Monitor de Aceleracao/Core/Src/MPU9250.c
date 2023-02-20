/*
 * MPU9250.c
 *
 *  Created on: Feb 13, 2023
 *      Author: salom
 */

#include "MPU9250.h"
#include "stm32f4xx_hal.h"
#include "math.h"

void I2C_ReadData(uint8_t device_address, uint8_t register_address, uint8_t *data, uint16_t data_size)
{
	HAL_I2C_Mem_Read(&hi2c, device_address, register_address,
	I2C_MEMADD_SIZE_8BIT, data, data_size, HAL_MAX_DELAY);
}

void I2C_WriteData(uint8_t device_address, uint8_t register_address, uint8_t *data, uint16_t data_size)
{
	HAL_I2C_Mem_Write(&hi2c, device_address, register_address,
	I2C_MEMADD_SIZE_8BIT, data, data_size, HAL_MAX_DELAY);
}

void MPU9250_init(I2C_HandleTypeDef i2c) {

	hi2c = i2c;
	// Verificação do MPU9250 conectado ao microcontrolador
	uint8_t who_am_i = 0;
	I2C_ReadData(MPU9250_ADDRESS, WHO_AM_I_REGISTER, &who_am_i, 1);
	if (who_am_i != WHO_AM_I_VAL) {
		// MPU9250 não foi encontrado
		// Trate o erro aqui
	}

	// Desativação do modo sleep
	uint8_t data[2];
	data[0] = PWR_MGMT_1;
	data[1] = 0x00;
	I2C_WriteData(MPU9250_ADDRESS, data[0], data[1], 2);

	// Configuração da taxa de amostragem do acelerômetro
	data[0] = ACCEL_CONFIG;
	data[1] = 0x08;  // Taxa de amostragem: 1 kHz, filtragem: 46 Hz
	I2C_WriteData(MPU9250_ADDRESS, data[0], data[1], 2);

	// Configuração da taxa de amostragem do giroscópio
	data[0] = GYRO_CONFIG;
	data[1] = 0x08;  // Taxa de amostragem: 1 kHz, filtragem: 46 Hz
	I2C_WriteData(MPU9250_ADDRESS, data[0], data[1], 2);

	// Ativação dos sensores
	data[0] = PWR_MGMT_2;
	data[1] = 0x00;
	I2C_WriteData(MPU9250_ADDRESS, data[0], data[1], 2);

	// Configuração da bússola
	// Verificação do AK8963 conectado ao MPU9250
	who_am_i = 0;
	I2C_ReadData(AK8963_ADDRESS, AK8963_WHO_AM_I, &who_am_i, 1);
	if (who_am_i != AK8963_WHO_AM_I_VAL) {
		// AK8963 não foi encontrado
		// Trate o erro aqui
	}

	data[0] = AK8963_CNTL1;
	data[1] = 0x16; // Modo de operação: continuo, taxa de amostragem: 100 Hz
	I2C_WriteData(AK8963_ADDRESS, data[0], data[1], 2);

	// Calibração da bússola
	uint8_t calibration[3];
	float magnetometer_scale[3];
	I2C_ReadData(AK8963_ADDRESS, AK8963_ASAX, calibration, 3);
	magnetometer_scale[0] = (float) (calibration[0] - 128) / 256.0 + 1.0;
	magnetometer_scale[1] = (float) (calibration[1] - 128) / 256.0 + 1.0;
	magnetometer_scale[2] = (float) (calibration[2] - 128) / 256.0 + 1.0;

}

void MPU9250_read_accel_data(int16_t *destination) {
	// Leitura dos dados do acelerômetro aqui
	uint8_t raw_data[6];
	I2C_ReadData(MPU9250_ADDRESS, ACCEL_XOUT_H, raw_data, 6);

	destination[0] = ((int16_t) raw_data[0] << 8) | raw_data[1];
	destination[1] = ((int16_t) raw_data[2] << 8) | raw_data[3];
	destination[2] = ((int16_t) raw_data[4] << 8) | raw_data[5];
}

void MPU9250_read_gyro_data(int16_t *destination) {
	// Leitura dos dados do giroscópio aqui
	uint8_t raw_data[6];
	I2C_ReadData(MPU9250_ADDRESS, GYRO_XOUT_H, raw_data, 6);

	destination[0] = ((int16_t) raw_data[0] << 8) | raw_data[1];
	destination[1] = ((int16_t) raw_data[2] << 8) | raw_data[3];
	destination[2] = ((int16_t) raw_data[4] << 8) | raw_data[5];

}

void MPU9250_read_mag_data(int16_t *destination) {
	// Leitura dos dados da bússola aqui
	uint8_t raw_data[6];
	I2C_ReadData(AK8963_ADDRESS, AK8963_XOUT_L, raw_data, 6);

	int16_t mag_x = ((int16_t) raw_data[1] << 8) | raw_data[0];
	int16_t mag_y = ((int16_t) raw_data[3] << 8) | raw_data[2];
	int16_t mag_z = ((int16_t) raw_data[5] << 8) | raw_data[4];

	destination[0] = mag_x * magnetometer_scale;
	destination[1] = mag_y * magnetometer_scale;
	destination[2] = mag_z * magnetometer_scale;
}

void MPU9250_read_all_data(int16_t *accel_data, int16_t *gyro_data,
		int16_t *mag_data) {
	// Leitura de todos os dados aqui
	MPU9250_read_accel_data(accel_data);
	MPU9250_read_gyro_data(gyro_data);
	MPU9250_read_mag_data(mag_data);
}

void updateIMU() {
	// Read the accelerometer data
	readAccelData(accelCount);

	// Convert the accelerometer values to g's
	ax = (float) accelCount[0] * 2 / 32768.0f;
	ay = (float) accelCount[1] * 2 / 32768.0f;
	az = (float) accelCount[2] * 2 / 32768.0f;

	// Read the gyroscope data
	readGyroData(gyroCount);

	// Convert the gyroscope values to degrees per second
	gx = (float) gyroCount[0] / 131.0f;
	gy = (float) gyroCount[1] / 131.0f;
	gz = (float) gyroCount[2] / 131.0f;

	// Read the magnetometer data
	readMagData(magCount);

	// Convert the magnetometer values to micro-teslas
	mx = (float) magCount[0] * 4912.0f / 32760.0f;
	my = (float) magCount[1] * 4912.0f / 32760.0f;
	mz = (float) magCount[2] * 4912.0f / 32760.0f;
}

void updateOrientation() {
	float norm;
	float hx, hy, hz, bx, bz;
	float vx, vy, vz, wx, wy, wz;

// Normalize the accelerometer measurement
	norm = sqrt(ax * ax + ay * ay + az * az);
	ax /= norm;
	ay /= norm;
	az /= norm;

// Normalize the magnetometer measurement
	norm = sqrt(mx * mx + my * my + mz * mz);
	mx /= norm;
	my /= norm;
	mz /= norm;

// Reference direction of Earth's magnetic field
	hx = 2 * mx * (0.5f - q[2] * q[2] - q[3] * q[3]) + 2 * my * (q[1] * q[2] - q[0] * q[3]) + 2 * mz * (q[1] * q[3] + q[0] * q[2]);
	hy = 2 * mx * (q[1] * q[2] + q[0] * q[3]) + 2 * my * (0.5f - q[1] * q[1] - q[3] * q[3]) + 2 * mz * (q[2] * q[3] - q[0] * q[1]);
	hz = 2 * mx * (q[1] * q[3] - q[0] * q[2]) + 2 * my * (q[2] * q[3] + q[0] * q[1]) + 2 * mz * (0.5f - q[1] * q[1] - q[2] * q[2]);
	bx = sqrt((hx * hx) + (hy * hy));
	bz = hz;

// Estimated direction of gravity and magnetic field
	vx = 2 * (q[1] * q[3] - q[0] * q[2]);
	vy = 2 * (q[0] * q[1] + q[2] * q[3]);
	vz = q[0] * q[0] - q[1] * q[1] - q[2] * q[2] + q[3] * q[3];

// Error is cross product between estimated direction and measured direction of gravity
	ex = (ay * vz - az * vy) + (my * wz - mz * wy);
	ey = (az * vx - ax * vz) + (mz * wx - mx * wz);
	ez = (ax * vy - ay * vx) + (mx * wy - my * wx);

// Apply feedback to rotate estimate vectors
	gx = gx + Kp * ex;
	gy = gy + Kp * ey;
	gz = gz + Kp * ez;

// Normalize the estimated direction of gravity
	norm = sqrt(gx * gx + gy * gy + gz * gz);
	gx /= norm;
	gy /= norm;
	gz /= norm;

// Apply feedback to rotate estimated quaternion
	q[0] = q[0] + (-q[1] * gx - q[2] * gy - q[3] * gz) * (0.5f * dt);
	q[1] = q[1] + (q[0] * gx + q[2] * gz - q[3] * gy) * (0.5f * dt);
	q[2] = q[2] + (q[0] * gy - q[1] * gz + q[3] * gx) * (0.5f * dt);
	q[3] = q[3] + (q[0] * gz + q[1] * gy - q[2] * gx) * (0.5f * dt);

// Normalize the quaternion
	norm = sqrt(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]);
	q[0] /= norm;
	q[1] /= norm;
	q[2] /= norm;
	q[3] /= norm;
}

void getEulerAngles_rad(float *roll, float *pitch, float *yaw)
{
	*roll = (float)atan2(2 * (q[0] * q[1] + q[2] * q[3]), 1 - 2 * (q[1] * q[1] + q[2] * q[2]));
	*pitch = (float)asin(2 * (q[0] * q[2] - q[1] * q[3]));
	*yaw = (float)atan2(2 * (q[0] * q[3] + q[1] * q[2]), 1 - 2 * (q[2] * q[2] + q[3] * q[3]));
}

void getEulerAngles_deg(float *roll, float *pitch, float *yaw)
{
*roll = (float)atan2(2 * (q[0] * q[1] + q[2] * q[3]), 1 - 2 * (q[1] * q[1] + q[2] * q[2])) * 180 / M_PI;
*pitch = (float)asin(2 * (q[0] * q[2] - q[1] * q[3])) * 180 / M_PI;
*yaw = (float)atan2(2 * (q[0] * q[3] + q[1] * q[2]), 1 - 2 * (q[2] * q[2] + q[3] * q[3])) * 180 / M_PI;
}
