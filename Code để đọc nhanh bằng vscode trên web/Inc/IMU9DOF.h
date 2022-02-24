#ifndef _IMU9DOF_H
#define _IMU9DOF_H

#include "stdio.h"
#include "main.h"
#include "flash.h"
#include "math.h"

//#define USE_HAL_I2C_REGISTER_CALLBACKS  1
// enum trả về trạng thái của các hàm

typedef enum  {
	IMU9DOF_Result_Ok = 0x00,
	IMU9DOF_Result_Error,
	IMU9DOF_Result_DeviceNotConnected, // Thiết bị không được kết nối đến
	IMU9DOF_Result_DeviceInvalid,       // Không phải địa chỉ của mpu6050
	IMU9DFO_Result_MPU6050_NotConnected,
	IMU9DFO_Result_HMC5883L_NotConnected
} IMU9DOF_Result;


typedef enum  {
	MPU6050_Accelerometer_2G = 0x00,
	MPU6050_Accelerometer_4G = 0x01,
	MPU6050_Accelerometer_8G = 0x02,
	MPU6050_Accelerometer_16G = 0x03
} MPU6050_Accelerometer;



typedef enum {
	MPU6050_Gyroscope_250s = 0x00,
	MPU6050_Gyroscope_500s = 0x01,
	MPU6050_Gyroscope_1000s = 0x02,
	MPU6050_Gyroscope_2000s = 0x03
} MPU6050_Gyroscope;

typedef enum
{
    HMC5883L_RANGE_8_1GA     = 0b111,
    HMC5883L_RANGE_5_6GA     = 0b110,
    HMC5883L_RANGE_4_7GA     = 0b101,
    HMC5883L_RANGE_4GA       = 0b100,
    HMC5883L_RANGE_2_5GA     = 0b011,
    HMC5883L_RANGE_1_9GA     = 0b010,
    HMC5883L_RANGE_1_3GA     = 0b001,
    HMC5883L_RANGE_0_88GA    = 0b000
} HMC5883l_range;


typedef struct Acceleromoter_t{
	int16_t x;
	int16_t y;
	int16_t z;
	float accelMult;
}Accelerometer;

typedef struct Gyrometer_t {
	int16_t x;
	int16_t y;
	int16_t z;
	float rollInput;
	float pitchInput;
	float yawInput;
	float gyroMult;
} Gyrometer;

typedef struct Magnetometer_t {
	int16_t x;
	int16_t y;
	int16_t z;
	float mgausPerDigit;
} Magnetometer;

typedef enum
{
    MS5611_OSR_4096   = 0x08,
    MS5611_OSR_2048   = 0x06,
    MS5611_OSR_1024   = 0x04,
    MS5611_OSR_512    = 0x02,
    MS5611_OSR_256    = 0x00
} ms5611_osr;


class IMU9DOF{
public:
	bool headingLock;
	float courseLockHeading;
	float compassHorizontal_X;
	float compassHorizontal_Y;
	float actualCompassHeading;
	float declination;

	int16_t temp;
	Magnetometer magn;
	Accelerometer accel;
	Gyrometer gyro;

	int16_t accPitchCalValue;
	int16_t accRollCalValue;
	int32_t gyroRollCalValue;
	int32_t gyroPitchCalValue;
	int32_t gyroYawCalValue;

	float angleRoll;
	float anglePitch;
	float angleYaw;

	I2C_HandleTypeDef *hi2c;

	// index 0 - pitch
	// index 1 - roll
	int32_t accelCalValue[2];

	uint8_t indexShortAverageRotatingMem, indexLongAverageRotatingMem;
	int16_t shortAverageAccel_Z[25];
	int16_t longAverageAccel_Z[50];
	int32_t shortTotalAccel_Z;
	int32_t longTotalAccel_Z;
	int32_t accelAverageTotal;
	int32_t accelTotalVector;
	int32_t accelTotalVectorAtStart;
	int32_t accelAltIntegrated;
	float pitchLevelAdjust, rollLevelAdjust;

	float angleRollAccel, anglePitchAccel;

	IMU9DOF(I2C_HandleTypeDef * theI2c, uint8_t theSector, uint32_t theAddrs);
	void ReadGyroAccel();
	float CourseDeviation(float theValue1, float theValue2);

	void ReadAngleRPY();
	void CalculateGyroInput();
	void CalibGyro();
	void CalibCompass();
	void CalibLevel(uint8_t* theError);
	void VerticalAccelerationCalculations();
	IMU9DOF_Result ReadRawAllParameter();
	IMU9DOF_Result IsReadyToInterface();

	HAL_StatusTypeDef Init();
	HAL_StatusTypeDef SetupCompass();
	HAL_StatusTypeDef SetupGyro();
	HAL_StatusTypeDef SetupAutomaticReadCompass();

	HAL_StatusTypeDef SetGyroSensitivityMPU6050(MPU6050_Gyroscope theGyroSens);
	HAL_StatusTypeDef SetAccelSensitivityMPU6050(MPU6050_Accelerometer theAccelSens);
	HAL_StatusTypeDef SetMagnSensitivityHMC5883L(HMC5883l_range theMagnSens);
	IMU9DOF_Result ReadCompass();
	IMU9DOF_Result IsReadyToInterfaceHMC5883L();
	IMU9DOF_Result IsReadyToInterfaceMPU6050();

	void setLevelAdjust(uint8_t theRateRoll, uint8_t theRatePitch);
private:
	int16_t IsAlreadyCalibGyro;
	bool isAlreadyCalibCompass;
	bool isOnLevelCalib;

	int16_t compassOffset_X, compassOffset_Y, compassOffset_Z;
	float compassScaleY, compassScaleZ;

	uint8_t sectorFlash;
	uint32_t addrsFlash;
	int16_t dataFlash[6];
};
#endif
