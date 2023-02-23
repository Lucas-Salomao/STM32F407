/*
 * MPU9250.c
 *
 *  Created on: Feb 13, 2023
 *      Author: salom
 */
#include "stm32f4xx_hal.h"
#include "MPU9250.h"
#include <math.h>

extern I2C_HandleTypeDef hi2c2;

int16_t accelCount[3];
int16_t gyroCount[3];
int16_t magCount[3];
int16_t tempCount;

float ax, ay, az;
float gx, gy, gz;
float mx, my, mz;
float temperatura;
float ex, ey, ez;
float q[4] = { 1.0f, 0.0f, 0.0f, 0.0f };
float yaw, pitch, roll;
long preInterval;
float magnetometer_scale = MAGNETOMETER_SCALE;

void MPU9250_init()
{
	HAL_I2C_Mem_Write(&hi2c2, MPU9250_ADDRESS, USER_CTRL, 1, 0x0F,1,100);
	HAL_Delay(100);
	HAL_I2C_Mem_Write(&hi2c2, MPU9250_ADDRESS, PWR_MGMT_1, 1, 0x81, 1, 100);
	HAL_Delay(100);
	HAL_I2C_Mem_Write(&hi2c2, MPU9250_ADDRESS, PWR_MGMT_2, 1, 0x00,1,100);

	//HI2C=hi2c;
	// Verificação do MPU9250 conectado ao microcontrolador
	uint8_t who_am_i;
	HAL_I2C_Mem_Read(&hi2c2, MPU9250_ADDRESS, WHO_AM_I, 1, &who_am_i, 1, 100);
	if (who_am_i != WHO_AM_I) {
		// MPU9250 não foi encontrado
		// Trate o erro aqui
		//char message[] = "MPU9250 nao encontrado\r\n";
		//HAL_UART_Transmit(&huart2, (uint8_t*) message, strlen(message),HAL_MAX_DELAY);
	} else {
		//char message[] = "MPU9250 encontrado\r\n";
		//HAL_UART_Transmit(&huart2, (uint8_t*) message, strlen(message),HAL_MAX_DELAY);
	}

	//Habilita Bypass
	uint8_t writeData = 0x02;
	HAL_I2C_Mem_Write(&hi2c2, MPU9250_ADDRESS, INT_PIN_CFG, 1, &writeData, 1,100);

	// Configuração da bússola
	// Verificação do AK8963 conectado ao MPU9250

	HAL_I2C_Mem_Read(&hi2c2, AK8963_ADDRESS, AK8963_WHO_AM_I, 1, &who_am_i, 1,100);
	if (who_am_i != AK8963_WHO_AM_I) {
		// AK8963 não foi encontrado
		// Trate o erro aqui
		//char message[] = "AK8963 nao encontrado\r\n";
		//HAL_UART_Transmit(&huart2, (uint8_t*) message, strlen(message),HAL_MAX_DELAY);
	} else {
		//char message[] = "AK8963  encontrado\r\n";
		//HAL_UART_Transmit(&huart2, (uint8_t*) message, strlen(message),HAL_MAX_DELAY);
	}

	//Power down magnetometer
	writeData = POWER_DOWN;
	HAL_I2C_Mem_Write(&hi2c2, AK8963_ADDRESS, AK8963_CNTL, 1, &writeData, 1, 100);
	HAL_Delay(100);

	//Enter Fuse ROM access mode
	writeData = FUSE_ROM;
	HAL_I2C_Mem_Write(&hi2c2, AK8963_ADDRESS, AK8963_CNTL, 1, &writeData, 1, 100);
	HAL_Delay(100);

	//Read the x-, y-, and z-axis calibration values
	uint8_t rawMagCalData[3];
	HAL_I2C_Mem_Read(&hi2c2, AK8963_ADDRESS, AK8963_ASAX, 1, &rawMagCalData[0], 3, 100);
	float calMagX =  (float)(rawMagCalData[0] - 128)/256. + 1.;   // Return x-axis sensitivity adjustment values, etc.
	float calMagY =  (float)(rawMagCalData[1] - 128)/256. + 1.;
	float calMagZ =  (float)(rawMagCalData[2] - 128)/256. + 1.;

	//char message[50];
	//printf("Mag cal off X: %f\r\n", calMagX);
	//printf("Mag cal off Y: %f\r\n", calMagY);
	//printf("Mag cal off Z: %f\r\n", calMagZ);
	HAL_Delay(100);

	//Power down magnetometer
	writeData = POWER_DOWN;
	HAL_I2C_Mem_Write(&hi2c2, AK8963_ADDRESS, AK8963_CNTL, 1, &writeData, 1, 100);
	HAL_Delay(100);

	//Set magnetometer data resolution and sample ODR
	writeData = AK8963_MAG_SCALE_16BITS | CONTINOUS1;
	//printf("writeData: %d\r\n", writeData);
	HAL_I2C_Mem_Write(&hi2c2, AK8963_ADDRESS, AK8963_CNTL, 1, &writeData, 1, 100);
	HAL_Delay(100);

	// Desativação do modo sleep
	writeData = 0x00;
	HAL_I2C_Mem_Write(&hi2c2, MPU9250_ADDRESS, PWR_MGMT_1, 1, &writeData, 1, 100);

	// Configuração da taxa de amostragem do acelerômetro
	writeData=0x08; // Taxa de amostragem: 1 kHz, filtragem: 46 Hz
	HAL_I2C_Mem_Write(&hi2c2, MPU9250_ADDRESS, ACCEL_CONFIG, 1, &writeData, 1, 100);

	// Configuração da taxa de amostragem do giroscópio
	writeData=0x08; // Taxa de amostragem: 1 kHz, filtragem: 46 Hz
	HAL_I2C_Mem_Write(&hi2c2, MPU9250_ADDRESS, GYRO_CONFIG, 1, &writeData, 1, 100);

	// Ativação dos sensores
	writeData=0x00;
	HAL_I2C_Mem_Write(&hi2c2, MPU9250_ADDRESS, PWR_MGMT_2, 1, &writeData, 1, 100);

	preInterval = HAL_GetTick();
}

void MPU9250_read_accel_data(int16_t *destination) {
	// Leitura dos dados do acelerômetro aqui
	uint8_t raw_data[6];
	HAL_I2C_Mem_Read(&hi2c2, MPU9250_ADDRESS, ACCEL_XOUT_H, 1, raw_data, 6, 100);

	destination[0] = ((int16_t) raw_data[0] << 8) | raw_data[1];
	destination[1] = ((int16_t) raw_data[2] << 8) | raw_data[3];
	destination[2] = ((int16_t) raw_data[4] << 8) | raw_data[5];
}

void MPU9250_read_gyro_data(int16_t *destination) {
	// Leitura dos dados do giroscópio aqui
	uint8_t raw_data[6];
	HAL_I2C_Mem_Read(&hi2c2, MPU9250_ADDRESS, GYRO_XOUT_H, 1, raw_data, 6, 100);

	destination[0] = ((int16_t) raw_data[0] << 8) | raw_data[1];
	destination[1] = ((int16_t) raw_data[2] << 8) | raw_data[3];
	destination[2] = ((int16_t) raw_data[4] << 8) | raw_data[5];

}

void MPU9250_read_mag_data(int16_t *destination) {
	// Leitura dos dados da bússola aqui
	uint8_t raw_data[6];
	HAL_I2C_Mem_Read(&hi2c2, AK8963_ADDRESS, AK8963_XOUT_L, 1, raw_data, 6, 100);

	int16_t mag_x = ((int16_t) raw_data[1] << 8) | raw_data[0];
	int16_t mag_y = ((int16_t) raw_data[3] << 8) | raw_data[2];
	int16_t mag_z = ((int16_t) raw_data[5] << 8) | raw_data[4];

	destination[0] = mag_x * magnetometer_scale;
	destination[1] = mag_y * magnetometer_scale;
	destination[2] = mag_z * magnetometer_scale;
}

void MPU9250_read_temp(int16_t *destination)
{
	uint8_t buf[2];

	// Leitura dos 2 bytes do registrador de temperatura
	HAL_I2C_Mem_Read(&hi2c2, MPU9250_ADDRESS, TEMP_OUT_H, I2C_MEMADD_SIZE_8BIT, buf, 2, 100);

	// Conversão dos bytes em valor inteiro
	destination = (buf[0] << 8) | buf[1];

	// Conversão do valor inteiro em temperatura
	//temperatura = ((float)destination / 333.87f) + 21.0f;  // Veja a seção de calibração no datasheet para detalhes da conversão
}

void MPU9250_read_all_data()
{
	// Leitura de todos os dados aqui
	MPU9250_read_accel_data(accelCount);
	MPU9250_read_gyro_data(gyroCount);
	MPU9250_read_mag_data(magCount);
	MPU9250_read_temp(tempCount);
}

void updateIMU()
{
	// Read the accelerometer data
	MPU9250_read_accel_data(accelCount);

	// Convert the accelerometer values to g's
	ax = (float) accelCount[0] * 2 / 32768.0f;
	ay = (float) accelCount[1] * 2 / 32768.0f;
	az = (float) accelCount[2] * 2 / 32768.0f;

	// Read the gyroscope data
	MPU9250_read_gyro_data(gyroCount);

	// Convert the gyroscope values to degrees per second
	gx = (float) gyroCount[0] / 131.0f;
	gy = (float) gyroCount[1] / 131.0f;
	gz = (float) gyroCount[2] / 131.0f;

	// Read the magnetometer data
	MPU9250_read_mag_data(magCount);

	// Convert the magnetometer values to micro-teslas
	mx = (float) magCount[0] * 4912.0f / 32760.0f;
	my = (float) magCount[1] * 4912.0f / 32760.0f;
	mz = (float) magCount[2] * 4912.0f / 32760.0f;

	MPU9250_read_temp(tempCount);
	temperatura = ((float)tempCount / 333.87f) + 21.0f;  // Veja a seção de calibração no datasheet para detalhes da conversão
}

void updateOrientation() {
	float norm;
	float hx, hy, hz, bx, bz;
	float vx, vy, vz, wx, wy, wz;
	float Kp;

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
	hx = 2 * mx * (0.5f - q[2] * q[2] - q[3] * q[3])
			+ 2 * my * (q[1] * q[2] - q[0] * q[3])
			+ 2 * mz * (q[1] * q[3] + q[0] * q[2]);
	hy = 2 * mx * (q[1] * q[2] + q[0] * q[3])
			+ 2 * my * (0.5f - q[1] * q[1] - q[3] * q[3])
			+ 2 * mz * (q[2] * q[3] - q[0] * q[1]);
	hz = 2 * mx * (q[1] * q[3] - q[0] * q[2])
			+ 2 * my * (q[2] * q[3] + q[0] * q[1])
			+ 2 * mz * (0.5f - q[1] * q[1] - q[2] * q[2]);
	bx = sqrt((hx * hx) + (hy * hy));
	bz = hz;

// Estimated direction of gravity and magnetic field
	vx = 2 * (q[1] * q[3] - q[0] * q[2]);
	vy = 2 * (q[0] * q[1] + q[2] * q[3]);
	vz = q[0] * q[0] - q[1] * q[1] - q[2] * q[2] + q[3] * q[3];

// Error is cross product between estimated direction and measured direction of gravity
	wx = gx;
	wy = gy;
	wz = gz;
	ex = (ay * vz - az * vy) + (my * wz - mz * wy);
	ey = (az * vx - ax * vz) + (mz * wx - mx * wz);
	ez = (ax * vy - ay * vx) + (mx * wy - my * wx);

// Apply feedback to rotate estimate vectors
	Kp=0.5f;
	gx = gx + Kp * ex;
	gy = gy + Kp * ey;
	gz = gz + Kp * ez;

// Normalize the estimated direction of gravity
	norm = sqrt(gx * gx + gy * gy + gz * gz);
	gx /= norm;
	gy /= norm;
	gz /= norm;

// Apply feedback to rotate estimated quaternion
	unsigned long Tnew = HAL_GetTick();
	float dt = (Tnew - preInterval) * 1e-3;
	preInterval = Tnew;
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

void getEulerAngles_rad(float *roll, float *pitch, float *yaw) {
	*roll = (float) atan2(2 * (q[0] * q[1] + q[2] * q[3]),
			1 - 2 * (q[1] * q[1] + q[2] * q[2]));
	*pitch = (float) asin(2 * (q[0] * q[2] - q[1] * q[3]));
	*yaw = (float) atan2(2 * (q[0] * q[3] + q[1] * q[2]),
			1 - 2 * (q[2] * q[2] + q[3] * q[3]));
}

void getEulerAngles_deg(float *roll, float *pitch, float *yaw) {
	*roll = (float) atan2(2 * (q[0] * q[1] + q[2] * q[3]),
			1 - 2 * (q[1] * q[1] + q[2] * q[2])) * 180 / M_PI;
	*pitch = (float) asin(2 * (q[0] * q[2] - q[1] * q[3])) * 180 / M_PI;
	*yaw = (float) atan2(2 * (q[0] * q[3] + q[1] * q[2]),
			1 - 2 * (q[2] * q[2] + q[3] * q[3])) * 180 / M_PI;
}
