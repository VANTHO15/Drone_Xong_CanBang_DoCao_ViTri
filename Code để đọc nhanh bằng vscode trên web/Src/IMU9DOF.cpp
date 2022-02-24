#include "IMU9DOF.h"
#include "dwt_stm32_delay.h"
#include <stdio.h>

#define MPU6050_I2C_ADDR			0xD0	// sau khi đã dịch bit
#define MPU6050_I_AM				0x68
#define MPU6050_AUX_VDDIO			0x01
#define MPU6050_SMPLRT_DIV			0x19
#define MPU6050_CONFIG				0x1A
#define MPU6050_GYRO_CONFIG			0x1B
#define MPU6050_ACCEL_CONFIG		0x1C
#define MPU6050_MOTION_THRESH		0x1F
#define MPU6050_I2C_SLV0_ADDR		0x25
#define	MPU6050_I2C_SLV0_REG		0x26
#define MPU6050_I2C_SLV0_CTRL		0x27
#define MPU6050_INT_PIN_CFG			0x37
#define MPU6050_INT_ENABLE			0x38
#define MPU6050_INT_STATUS			0x3A
#define MPU6050_ACCEL_XOUT_H		0x3B
#define MPU6050_ACCEL_XOUT_L		0x3C
#define MPU6050_ACCEL_YOUT_H		0x3D
#define MPU6050_ACCEL_YOUT_L		0x3E
#define MPU6050_ACCEL_ZOUT_H		0x3F
#define MPU6050_ACCEL_ZOUT_L		0x40
#define MPU6050_TEMP_OUT_H			0x41
#define MPU6050_TEMP_OUT_L			0x42
#define MPU6050_GYRO_XOUT_H			0x43
#define MPU6050_GYRO_XOUT_L			0x44
#define MPU6050_GYRO_YOUT_H			0x45
#define MPU6050_GYRO_YOUT_L			0x46
#define MPU6050_GYRO_ZOUT_H			0x47
#define MPU6050_GYRO_ZOUT_L			0x48
#define MPU6050_EXT_SENS_DATA_0		0x49
#define MPU6050_MOT_DETECT_STATUS	0x61
#define MPU6050_I2C_MST_DELAY_CTRL	0x67
#define MPU6050_SIGNAL_PATH_RESET	0x68
#define MPU6050_MOT_DETECT_CTRL		0x69
#define MPU6050_USER_CTRL			0x6A
#define MPU6050_PWR_MGMT_1			0x6B
#define MPU6050_PWR_MGMT_2			0x6C
#define MPU6050_FIFO_COUNTH			0x72
#define MPU6050_FIFO_COUNTL			0x73
#define MPU6050_FIFO_R_W			0x74
#define MPU6050_WHO_AM_I			0x75


/* Gyro sensitivities in degrees/s */
#define MPU6050_GYRO_SENS_250		((float) 131)
#define MPU6050_GYRO_SENS_500		((float) 65.5)
#define MPU6050_GYRO_SENS_1000		((float) 32.8)
#define MPU6050_GYRO_SENS_2000		((float) 16.4)

/* Acce sensitivities in g/s */
#define MPU6050_ACCE_SENS_2			((float) 16384)
#define MPU6050_ACCE_SENS_4			((float) 8192)
#define MPU6050_ACCE_SENS_8			((float) 4096)
#define MPU6050_ACCE_SENS_16		((float) 2048)

#define HMC5883L_ADDRESS_DATASHEET 	0b0011110
#define HMC5883L_ADDRESS 			0x3C
#define HMC5883L_CONFIG_A			0x00
#define HMC5883L_CONFIG_B			0x01
#define HMC5883L_MODE_REG			0x02
#define HMC5883L_MAGN_X_H			0x03
#define HMC5883L_MAGN_X_L			0x04
#define HMC5883L_MAGN_Z_H			0x05
#define HMC5883L_MAGN_Z_L			0x06
#define HMC5883L_MAGN_Y_H			0x07
#define HMC5883L_MAGN_Y_L			0x08
#define HMC5883L_STATUS_REG			0x09
#define HMC5883L_IDEN_A				0x10
#define HMC5883L_IDEN_B				0x11
#define HMC5883L_IDEN_C				0x12

void IMU9DOF::VerticalAccelerationCalculations()  //Tính toán gia tốc theo phương dọc
{
	indexShortAverageRotatingMem++;
	if (indexShortAverageRotatingMem == 25)indexShortAverageRotatingMem = 0;

	shortTotalAccel_Z -= shortAverageAccel_Z[indexShortAverageRotatingMem];
	shortAverageAccel_Z[indexShortAverageRotatingMem] = accelTotalVector;
	shortTotalAccel_Z += shortAverageAccel_Z[indexShortAverageRotatingMem];

	if (indexShortAverageRotatingMem == 0) {
		indexLongAverageRotatingMem++;

		if (indexLongAverageRotatingMem == 50)indexLongAverageRotatingMem = 0;

		longTotalAccel_Z -= longAverageAccel_Z[indexLongAverageRotatingMem];
		longAverageAccel_Z[indexLongAverageRotatingMem] = shortTotalAccel_Z / 25;
		longTotalAccel_Z += longAverageAccel_Z[indexLongAverageRotatingMem];
	}
	accelAverageTotal = longTotalAccel_Z / 50;


	accelAltIntegrated += accelTotalVector - accelAverageTotal;
	if (accelTotalVector - accelAverageTotal < 400 || accelTotalVector - accelAverageTotal > 400) {
		if (shortTotalAccel_Z / 25 - accelAverageTotal < 500 && shortTotalAccel_Z / 25 - accelAverageTotal > -500)
		{
			if (accelAltIntegrated > 200)accelAltIntegrated -= 200;
			else if (accelAltIntegrated < -200)accelAltIntegrated += 200;
		}
	}
}
IMU9DOF::IMU9DOF(I2C_HandleTypeDef * theI2c, uint8_t theSector, uint32_t theAddrs)
{
	this->hi2c = theI2c;
	this->sectorFlash = theSector;
	this->addrsFlash = theAddrs;
}

IMU9DOF_Result IMU9DOF::ReadCompass()
{
	magn.y *= -1;
	magn.x *= -1;

	// calib lại giá trị thô vừa đọc để sử dụng tính toán heading lúc khởi động
	if (isAlreadyCalibCompass == 0)
	{
		magn.y += compassOffset_Y;
		magn.y = (int16_t) ((float)(magn.y) *compassScaleY);
		magn.z += compassOffset_Z;
		magn.z = (int16_t) ((float)(magn.z) *compassScaleZ);
		magn.x += compassOffset_X;
	}

	/*
	 * Giá trị compass sẽ thay đổi khi góc roll và pitch thay đổi.
	 * Do đó các giá trị compass x&y cần được hiệu chỉnh lại theo
	 * các góc roll và pitch để đạt được vị trí phương ngang ảo.
	 */
	compassHorizontal_X = (float)magn.x * cos(anglePitch * -0.0174533) + (float)magn.y * sin(angleRoll * 0.0174533) * sin(anglePitch * -0.0174533) - (float)magn.z * cos(angleRoll * 0.0174533) * sin(anglePitch * -0.0174533);
	compassHorizontal_Y = (float)magn.y * cos(angleRoll * 0.0174533) + (float)magn.z * sin(angleRoll * 0.0174533);
	//Now that the horizontal values are known the heading can be calculated. With the following lines of code the heading is calculated in degrees.
	//Please note that the atan2 uses radians in stead of degrees. That is why the 180/3.14 is used.
	if (compassHorizontal_Y < 0)
	{
		actualCompassHeading = 180 + (180 + ((atan2(compassHorizontal_Y, compassHorizontal_X)) * (180 / 3.14)));
	}
	else
	{
		actualCompassHeading = (atan2(compassHorizontal_Y, compassHorizontal_X)) * (180 / 3.14);
	}

	actualCompassHeading += declination;                                 //Add the declination to the magnetic compass heading to get the geographic north.
	if (actualCompassHeading < 0) actualCompassHeading += 360;         //If the compass heading becomes smaller then 0, 360 is added to keep it in the 0 till 360 degrees range.
	else if (actualCompassHeading >= 360) actualCompassHeading -= 360; //If the compass heading becomes larger then 360, 360 is subtracted to keep it in the 0 till 360 degrees range.

	return IMU9DOF_Result_Ok;
}


IMU9DOF_Result IMU9DOF::IsReadyToInterfaceMPU6050()
{
	uint8_t data;	// value to wakeup MPU6050

	/**** Kiểm tra giao tiếp I2C của MPU6050 *****/
	if (HAL_I2C_IsDeviceReady(this->hi2c, (uint16_t)MPU6050_I2C_ADDR, 2, 5) != HAL_OK)
	{
		return IMU9DFO_Result_MPU6050_NotConnected;
	}

	/* Check who am I */
	if (HAL_I2C_Mem_Read(this->hi2c, (uint16_t)MPU6050_I2C_ADDR, (uint16_t)MPU6050_WHO_AM_I, I2C_MEMADD_SIZE_8BIT, &data, 1, 500) != HAL_OK)
	{
		return IMU9DFO_Result_MPU6050_NotConnected;
	}

	/* Checking */
	while(data != MPU6050_I_AM)
	{
		/* Return error */
		return IMU9DFO_Result_MPU6050_NotConnected;
	}

	data = 0x00;
	/* Wakeup MPU6050 */
	if (HAL_I2C_Mem_Write(this->hi2c, (uint16_t)MPU6050_I2C_ADDR, (uint16_t)MPU6050_PWR_MGMT_1, I2C_MEMADD_SIZE_8BIT, &data, 1, 500) != HAL_OK)
	{
		return IMU9DFO_Result_MPU6050_NotConnected;
	}

	return IMU9DOF_Result_Ok;
}

IMU9DOF_Result IMU9DOF::IsReadyToInterfaceHMC5883L()
{

	// Tắt chế độ I2C master mode
	uint8_t data = 0x00;
	HAL_I2C_Mem_Write(this->hi2c, (uint16_t)MPU6050_I2C_ADDR, (uint16_t)MPU6050_USER_CTRL, I2C_MEMADD_SIZE_8BIT, &data, 1, 50);

	// Bật chế độ I2C master bypass mode
	data = 0x02;
	HAL_I2C_Mem_Write(this->hi2c, (uint16_t)MPU6050_I2C_ADDR, (uint16_t)MPU6050_INT_PIN_CFG, I2C_MEMADD_SIZE_8BIT, &data, 1, 50);

	/**** Kiểm tra giao tiếp I2C của HMC5883L *****/
	if (HAL_I2C_IsDeviceReady(this->hi2c, (uint16_t)HMC5883L_ADDRESS, 2, 5) != HAL_OK)
	{
		return IMU9DFO_Result_HMC5883L_NotConnected;
	}

	// Tắt chế độ I2C master bypass mode
	data = 0x00;
	HAL_I2C_Mem_Write(this->hi2c, (uint16_t)MPU6050_I2C_ADDR, (uint16_t)MPU6050_INT_PIN_CFG, I2C_MEMADD_SIZE_8BIT, &data, 1, 50);

	// Bật chế độ I2C master mode
	data = 0x20;
	HAL_I2C_Mem_Write(this->hi2c, (uint16_t)MPU6050_I2C_ADDR, (uint16_t)MPU6050_USER_CTRL, I2C_MEMADD_SIZE_8BIT, &data, 1, 50);

	return IMU9DOF_Result_Ok;
}
IMU9DOF_Result IMU9DOF::IsReadyToInterface()
{
	uint8_t data;

	/**** Kiểm tra giao tiếp I2C của MPU6050 *****/
	if (HAL_I2C_IsDeviceReady(this->hi2c, (uint16_t)MPU6050_I2C_ADDR, 2, 5) != HAL_OK)
	{
		return IMU9DFO_Result_MPU6050_NotConnected;
	}

	/* Check who am I */
	if (HAL_I2C_Mem_Read(this->hi2c, (uint16_t)MPU6050_I2C_ADDR, (uint16_t)MPU6050_WHO_AM_I, I2C_MEMADD_SIZE_8BIT, &data, 1, 500) != HAL_OK)
	{
		return IMU9DFO_Result_MPU6050_NotConnected;
	}

	/* Checking */
	while(data != MPU6050_I_AM)
	{
		/* Return error */
		return IMU9DFO_Result_MPU6050_NotConnected;
	}

	/* Wakeup MPU6050 */
	if (HAL_I2C_Mem_Write(this->hi2c, (uint16_t)MPU6050_I2C_ADDR, (uint16_t)MPU6050_PWR_MGMT_1, I2C_MEMADD_SIZE_8BIT, &data, 1, 500) != HAL_OK)
	{
		return IMU9DFO_Result_MPU6050_NotConnected;
	}

	// Tắt chế độ I2C master mode
	data = 0x00;
	HAL_I2C_Mem_Write(this->hi2c, (uint16_t)MPU6050_I2C_ADDR, (uint16_t)MPU6050_USER_CTRL, I2C_MEMADD_SIZE_8BIT, &data, 1, 50);

	// Bật chế độ I2C master bypass mode
	data = 0x02;
	HAL_I2C_Mem_Write(this->hi2c, (uint16_t)MPU6050_I2C_ADDR, (uint16_t)MPU6050_INT_PIN_CFG, I2C_MEMADD_SIZE_8BIT, &data, 1, 50);

	/**** Kiểm tra giao tiếp I2C của HMC5883L *****/
	if (HAL_I2C_IsDeviceReady(this->hi2c, (uint16_t)HMC5883L_ADDRESS, 2, 5) != HAL_OK)
	{
		return IMU9DFO_Result_HMC5883L_NotConnected;
	}

	// Tắt chế độ I2C master bypass mode
	data = 0x00;
	HAL_I2C_Mem_Write(this->hi2c, (uint16_t)MPU6050_I2C_ADDR, (uint16_t)MPU6050_INT_PIN_CFG, I2C_MEMADD_SIZE_8BIT, &data, 1, 50);

	// Bật chế độ I2C master mode
	data = 0x20;
	HAL_I2C_Mem_Write(this->hi2c, (uint16_t)MPU6050_I2C_ADDR, (uint16_t)MPU6050_USER_CTRL, I2C_MEMADD_SIZE_8BIT, &data, 1, 50);

	return IMU9DOF_Result_Ok;
}

IMU9DOF_Result IMU9DOF::ReadRawAllParameter()   // đọc các giá trị thô
{
	uint8_t data[20];

	if (HAL_I2C_Mem_Read(this->hi2c, (uint16_t)MPU6050_I2C_ADDR, (uint16_t)MPU6050_ACCEL_XOUT_H, I2C_MEMADD_SIZE_8BIT, data, 20, 500) != HAL_OK)
	{
		return IMU9DOF_Result_Error;
	}

	/* Format accelerometer data */
	// -139, -640 la gia tri offset theo cac phuong x, y do viec dat cam bien MPU6050 khong nam thang bang
	this->accel.y = (int16_t)(data[0] << 8 | data[1]) - 139 ;
	this->accel.x = (int16_t)(data[2] << 8 | data[3]) - 640;
	this->accel.z = (int16_t)(data[4] << 8 | data[5]);

		/* Format temperature */
	this->temp = (data[6] << 8 | data[7]);

	/* Format gyroscope data */
	this->gyro.x = (int16_t)(data[8] << 8 | data[9]);
	this->gyro.y = (int16_t)(data[10] << 8 | data[11]);
	this->gyro.z = (int16_t)(data[12] << 8 | data[13]);

	this->magn.y   = (int16_t)(data[14] << 8 | data[15]);
	this->magn.z   = (int16_t)(data[16] << 8 | data[17]);
	this->magn.x   = (int16_t)(data[18] << 8 | data[19]);

	/* Return OK */
	return IMU9DOF_Result_Ok;
}

HAL_StatusTypeDef IMU9DOF::SetAccelSensitivityMPU6050(MPU6050_Accelerometer theAccelSens)  // SetupGyro
{
	uint8_t data;

	HAL_I2C_Mem_Read(this->hi2c, (uint16_t)MPU6050_I2C_ADDR, (uint16_t)MPU6050_ACCEL_CONFIG, I2C_MEMADD_SIZE_8BIT, &data, 1, 500);

	data = (data & 0xE7) | ((uint8_t)theAccelSens << 3);

	HAL_I2C_Mem_Write(this->hi2c, (uint16_t)MPU6050_I2C_ADDR, (uint16_t)MPU6050_ACCEL_CONFIG, I2C_MEMADD_SIZE_8BIT, &data, 1, 500);

	/* Set sensitivities for multiplying gyro and accelerometer data */
	switch (theAccelSens) {
	case MPU6050_Accelerometer_2G:
		this->accel.accelMult= (float)1 / MPU6050_ACCE_SENS_2;
		break;
	case MPU6050_Accelerometer_4G:
		this->accel.accelMult = (float)1 / MPU6050_ACCE_SENS_4;
		break;
	case MPU6050_Accelerometer_8G:
		this->accel.accelMult = (float)1 / MPU6050_ACCE_SENS_8;
		break;
	case MPU6050_Accelerometer_16G:
		this->accel.accelMult = (float)1 / MPU6050_ACCE_SENS_16;
		break;
	default:
		break;
	}
	/* Return OK */
	return HAL_OK;
}

HAL_StatusTypeDef IMU9DOF::SetGyroSensitivityMPU6050(MPU6050_Gyroscope theGyroSens) // SetupGyro
{
	uint8_t data;

	HAL_I2C_Mem_Read(this->hi2c, (uint16_t) MPU6050_I2C_ADDR, (uint16_t)MPU6050_GYRO_CONFIG, I2C_MEMADD_SIZE_8BIT, &data, 1, 500);


	data = (data & 0xE7) | ((uint8_t)theGyroSens << 3);

	HAL_I2C_Mem_Write(this->hi2c, (uint16_t) MPU6050_I2C_ADDR, (uint16_t)MPU6050_GYRO_CONFIG, I2C_MEMADD_SIZE_8BIT, &data, 1, 500);


	switch (theGyroSens) {
	case MPU6050_Gyroscope_250s:
		this->gyro.gyroMult = (float)1 / MPU6050_GYRO_SENS_250;
		break;
	case MPU6050_Gyroscope_500s:
		this->gyro.gyroMult = (float)1 / MPU6050_GYRO_SENS_500;
		break;
	case MPU6050_Gyroscope_1000s:
		this->gyro.gyroMult = (float)1 / MPU6050_GYRO_SENS_1000;
		break;
	case MPU6050_Gyroscope_2000s:
		this->gyro.gyroMult = (float)1 / MPU6050_GYRO_SENS_2000;
		break;
	default:
		break;
	}

	return HAL_OK;
}
HAL_StatusTypeDef IMU9DOF::SetupGyro(){ // init
	uint8_t data = 0x00;

	/* Config accelerometer */
	if ( SetAccelSensitivityMPU6050(MPU6050_Accelerometer_8G) != HAL_OK){
		return HAL_ERROR;
	}

	/* Config Gyroscope */
	if ( SetGyroSensitivityMPU6050(MPU6050_Gyroscope_500s) != HAL_OK){
		return HAL_ERROR;
	}

	// Thiết lập digital low pass filter với tần số là 43Hz
	data = 0x03;
	if ( HAL_I2C_Mem_Write(this->hi2c, (uint16_t)MPU6050_I2C_ADDR, (uint16_t)MPU6050_CONFIG, I2C_MEMADD_SIZE_8BIT, &data, 1, 50) != HAL_OK){
		return HAL_ERROR;
	}

	return HAL_OK;
}

HAL_StatusTypeDef IMU9DOF::SetupCompass(){ // init
	// Tắt chế độ I2C master mode
	uint8_t data = 0x00;
	if ( HAL_I2C_Mem_Write(this->hi2c, (uint16_t)MPU6050_I2C_ADDR, (uint16_t)MPU6050_USER_CTRL, I2C_MEMADD_SIZE_8BIT, &data, 1, 50) != HAL_OK){
		return HAL_ERROR;
	}

	// Bật chế độ I2C master bypass mode
	data = 0x02;
	if ( HAL_I2C_Mem_Write(this->hi2c, (uint16_t)MPU6050_I2C_ADDR, (uint16_t)MPU6050_INT_PIN_CFG, I2C_MEMADD_SIZE_8BIT, &data, 1, 50) != HAL_OK){
		return HAL_ERROR;
	}

	// Thiết lập sample rate = 75Hz và số lần lấy mẫu trung bình là 8
	data = 0x78;
	if ( HAL_I2C_Mem_Write(this->hi2c, (uint16_t)HMC5883L_ADDRESS, (uint16_t)HMC5883L_CONFIG_A, I2C_MEMADD_SIZE_8BIT, &data, 1, 50) != HAL_OK){
		return HAL_ERROR;
	}

	// Thiết lập full scale = +/- 1.3 Gauss
	data= 0x20;
	if ( HAL_I2C_Mem_Write(this->hi2c, (uint16_t)HMC5883L_ADDRESS, (uint16_t)HMC5883L_CONFIG_B, I2C_MEMADD_SIZE_8BIT, &data, 1, 50) != HAL_OK){
		return HAL_ERROR;
	}

	// Bật chế độ continuous measurement
	data = 0x00;
	if ( HAL_I2C_Mem_Write(this->hi2c, (uint16_t)HMC5883L_ADDRESS, (uint16_t)HMC5883L_MODE_REG, I2C_MEMADD_SIZE_8BIT, &data, 1, 50) != HAL_OK){
		return HAL_ERROR;
	}

	// Tắt chế độ I2C master bypass mode
	data = 0x00;
	if ( HAL_I2C_Mem_Write(this->hi2c, (uint16_t)MPU6050_I2C_ADDR, (uint16_t)MPU6050_INT_PIN_CFG, I2C_MEMADD_SIZE_8BIT, &data, 1, 50) != HAL_OK){
		return HAL_ERROR;
	}

	// Bật chế độ I2C master mode
	data = 0x20;
	if ( HAL_I2C_Mem_Write(this->hi2c, (uint16_t)MPU6050_I2C_ADDR, (uint16_t)MPU6050_USER_CTRL, I2C_MEMADD_SIZE_8BIT, &data, 1, 50) != HAL_OK){
		return HAL_ERROR;
	}

	return HAL_OK;
}

HAL_StatusTypeDef IMU9DOF::SetupAutomaticReadCompass(){  // init
	// Thiết lập địa chỉ của thanh ghi cần đọc là HMC5883L
	uint8_t data = HMC5883L_ADDRESS_DATASHEET | 0x80;
	if ( HAL_I2C_Mem_Write(this->hi2c, (uint16_t)MPU6050_I2C_ADDR, (uint16_t)MPU6050_I2C_SLV0_ADDR, I2C_MEMADD_SIZE_8BIT, &data, 1, 50) != HAL_OK){
		return HAL_ERROR;
	}

	// Bắt đầu đọc giá trị địa chỉ thanh ghi là 0x03 (x axis)
	data = 0x03;
	if (HAL_I2C_Mem_Write(this->hi2c, (uint16_t)MPU6050_I2C_ADDR, (uint16_t)MPU6050_I2C_SLV0_REG, I2C_MEMADD_SIZE_8BIT, &data, 1, 50) != HAL_OK){
		return HAL_ERROR;
	}

	// Đọc 6 thanh ghi liên tiếp trong HMC5883L
	data = 6 | 0x80;
	if ( HAL_I2C_Mem_Write(this->hi2c, (uint16_t)MPU6050_I2C_ADDR, (uint16_t)MPU6050_I2C_SLV0_CTRL, I2C_MEMADD_SIZE_8BIT, &data, 1, 50) != HAL_OK){
		return HAL_ERROR;
	}
	// cho phép slave 0
	data = 1;
	if ( HAL_I2C_Mem_Write(this->hi2c, (uint16_t)MPU6050_I2C_ADDR, (uint16_t)0x67, I2C_MEMADD_SIZE_8BIT, &data, 1, 50) != HAL_OK){
		return HAL_ERROR;
	}

	return HAL_OK;
}
HAL_StatusTypeDef IMU9DOF::Init()
{

	if (SetupGyro() != HAL_OK){
		return HAL_ERROR;
	}
	if (SetupCompass() != HAL_OK){
		return HAL_ERROR;
	}
	if (SetupAutomaticReadCompass() != HAL_OK){
		return HAL_ERROR;
	}


	MyFlash_SetSectorAddrs(this->sectorFlash, this->addrsFlash);
	MyFlash_ReadN(0, dataFlash, 6, DATA_TYPE_16);

	compassScaleY = ((float)dataFlash[1] - dataFlash[0]) / (dataFlash[3] - dataFlash[2]);
	compassScaleZ = ((float)dataFlash[1] - dataFlash[0]) / (dataFlash[5] - dataFlash[4]);

	compassOffset_X = (dataFlash[1] - dataFlash[0]) / 2 - dataFlash[1];
	compassOffset_Y = (((float)dataFlash[3] - dataFlash[2]) / 2 - dataFlash[3]) * compassScaleY;
	compassOffset_Z = (((float)dataFlash[5] - dataFlash[4]) / 2 - dataFlash[5]) * compassScaleZ;

	MyFlash_SetSectorAddrs(10, 0x080D0000);
	MyFlash_ReadN(0, accelCalValue, 2, DATA_TYPE_32);
	accPitchCalValue = accelCalValue[0];
	accRollCalValue = accelCalValue[1];

	return HAL_OK;
}

void IMU9DOF::ReadGyroAccel()
{
	ReadRawAllParameter();

	gyro.y *= -1;
	gyro.z *= -1;

	if (isOnLevelCalib == 0)
	{
		accel.y -= accPitchCalValue;
		accel.x -= accRollCalValue ;
	}

	if (IsAlreadyCalibGyro >= 2000)  // lấy 2000 mẫu để hiệu chuẩn
	{
		gyro.x -= gyroRollCalValue;
		gyro.y -= gyroPitchCalValue;
		gyro.z -= gyroYawCalValue;
	}
}

HAL_StatusTypeDef IMU9DOF::SetMagnSensitivityHMC5883L(HMC5883l_range theMagnSens)
{
	uint8_t data = 0x00;

	// Tắt chế độ I2C master mode
	HAL_I2C_Mem_Write(this->hi2c, (uint16_t)MPU6050_I2C_ADDR, (uint16_t)MPU6050_USER_CTRL, I2C_MEMADD_SIZE_8BIT, &data, 1, 50);

	// Bật chế độ I2C master bypass mode
	data = 0x02;
	HAL_I2C_Mem_Write(this->hi2c, (uint16_t)MPU6050_I2C_ADDR, (uint16_t)MPU6050_INT_PIN_CFG, I2C_MEMADD_SIZE_8BIT, &data, 1, 50);

	// Thiết lập full scale = +/- 1.3 Gauss
	data= theMagnSens << 5;
	HAL_I2C_Mem_Write(this->hi2c, (uint16_t)HMC5883L_ADDRESS, (uint16_t)HMC5883L_CONFIG_B, I2C_MEMADD_SIZE_8BIT, &data, 1, 50);

	// Tắt chế độ I2C master bypass mode
	data = 0x00;
	HAL_I2C_Mem_Write(this->hi2c, (uint16_t)MPU6050_I2C_ADDR, (uint16_t)MPU6050_INT_PIN_CFG, I2C_MEMADD_SIZE_8BIT, &data, 1, 50);

	// Bật chế độ I2C master mode
	data = 0x20;
	HAL_I2C_Mem_Write(this->hi2c, (uint16_t)MPU6050_I2C_ADDR, (uint16_t)MPU6050_USER_CTRL, I2C_MEMADD_SIZE_8BIT, &data, 1, 50);


	return HAL_OK;
}
void IMU9DOF::CalibCompass(){
	isAlreadyCalibCompass = 1;
	//TODO tao den bao trang thai

		// TODO gui du lieu
		DWT_Delay_us(3700);
		ReadRawAllParameter();
		ReadCompass();

		if ( magn.x < dataFlash[0]){
			dataFlash[0] = magn.x;
		}
		if ( magn.x > dataFlash[1]){
			dataFlash[1] = magn.x;
		}
		if ( magn.y < dataFlash[2]){
			dataFlash[2] = magn.y;
		}
		if ( magn.y > dataFlash[3]){
			dataFlash[3] = magn.y;
		}
		if ( magn.z < dataFlash[4]){
			dataFlash[4] = magn.z;
		}
		if ( magn.z > dataFlash[5]){
			dataFlash[5] = magn.z;
		}


	isAlreadyCalibCompass = 0;
	MyFlash_SetSectorAddrs(this->sectorFlash, this->addrsFlash);
	MyFlash_WriteN(0, dataFlash, 6, DATA_TYPE_16);

	SetupCompass();
	ReadRawAllParameter();
	ReadCompass();
	angleYaw = actualCompassHeading;

	// TODO bao de trang thai ket thuc qua trinh calib
}

void IMU9DOF::CalibLevel(uint8_t* theError){  // Qua trinh calib mat phang , lấy roll pitch khi khởi động
	isOnLevelCalib = 1;


	//TODO bao den trang thai

	accelCalValue[0] = 0;
	accelCalValue[1] = 0;

	for ( int i = 0; i < 64; i++){
		//TODO gui du lieu telemetry
		ReadGyroAccel();
		accelCalValue[0] += accel.y;
		accelCalValue[1] += accel.x;
		if ( accel.y > 500 || accel.y < -500){
			*theError = 80;
		}
		if ( accel.x > 500 || accel.x < -500){
			*theError = 80;
		}
		DWT_Delay_us(3700);
	}

	accelCalValue[0] /= 64;
	accelCalValue[1] /= 64;
	// TODO bao trang thai
	if (*theError < 80){
		MyFlash_SetSectorAddrs(10, 0x080D0000);
		MyFlash_WriteN(0, accelCalValue, 2, DATA_TYPE_32);
		//TODO nhay den bao trang thai
		*theError = 0;
	}
	else{
		*theError = 3;
	}

	isOnLevelCalib = 0;
	//todo moi them vao
	accPitchCalValue = (int16_t)accelCalValue[0];
	accRollCalValue = (int16_t)accelCalValue[1];
	ReadGyroAccel();

	accelTotalVector  = sqrt((accel.x * accel.x) + (accel.y * accel.y) + (accel.z * accel.z));    //Tính tổng vectơ gia tốc kế.

	if (abs(accel.y) < accelTotalVector) {                                             //Ngăn chặn chức năng asin tạo ra NaN.
	    anglePitchAccel = asin((float)accel.y / accelTotalVector) * 57.296;              //Calculate the pitch angle.
	  }
	  if (abs(accel.x) < accelTotalVector) {                                             //Ngăn chặn chức năng asin tạo ra NaN.
		  angleRollAccel = asin((float)accel.x / accelTotalVector) * 57.296;               //Calculate the roll angle.
	  }
	  anglePitch = anglePitchAccel;        //Đặt góc của con quay hồi chuyển bằng góc bước của gia tốc kế khi khởi động quadcopter.
	  angleRoll = angleRollAccel;

}

void IMU9DOF::CalibGyro()
{
	IsAlreadyCalibGyro = 0;
	if (IsAlreadyCalibGyro != 2000) {
		//Hãy lấy nhiều mẫu dữ liệu con quay hồi chuyển để chúng ta có thể xác định độ lệch con quay hồi chuyển trung bình (calibration).
		for (IsAlreadyCalibGyro = 0; IsAlreadyCalibGyro < 2000 ; IsAlreadyCalibGyro ++)
		{                                  //Take 2000 readings for calibration.
			if (IsAlreadyCalibGyro % 25 == 0)
			{
				// TODO : báo hiệu đèn đang trong trạng thái calib

			}
			ReadGyroAccel();                                                                //Read the gyro output.
			gyroRollCalValue += gyro.x;                                                     //Ad roll value to gyro_roll_cal.
			gyroPitchCalValue += gyro.y;                                                   //Ad pitch value to gyro_pitch_cal.
			gyroYawCalValue += gyro.z;                                                       //Ad yaw value to gyro_yaw_cal.
			HAL_Delay(4);                                                                       //Small delay to simulate a 250Hz loop during calibration.
		}
		// TODO báo hiệu
		//Now that we have 2000 measures, we need to devide by 2000 to get the average gyro offset.
		gyroRollCalValue /= 2000;                                                            //Divide the roll total by 2000.
		gyroPitchCalValue /= 2000;                                                           //Divide the pitch total by 2000.
		gyroYawCalValue /= 2000;                                                             //Divide the yaw total by 2000.
	}
}
void IMU9DOF::ReadAngleRPY()
{
	//Gyro angle calculations
	//0.0000611 = 1 / (250Hz / 65.5)
	anglePitch += (float)gyro.y * 0.0000611;        //tính góc pitch angle đã di chuyển and thêm nó vào biến anglepitch .
	angleRoll += (float)gyro.x * 0.0000611;
	angleYaw += (float)gyro.z * 0.0000611;
	if (angleYaw < 0) angleYaw += 360;                //Nếu la bàn nhỏ hơn thì 0, 360 được thêm vào để giữ nó trong phạm vi 0 đến 360 độ.
	else if (angleYaw >= 360) angleYaw -= 360;        //Nếu la bànlớn hơn thì 360, 360 được trừ đi để giữ nó trong phạm vi 0 đến 360 độ

	//0.000001066 = 0.0000611 * (3.142(PI) / 180degr) Hàm sin STM32 tính bằng radian chứ không phải độ
	anglePitch -= angleRoll * sin((float)gyro.z * 0.000001066);    //nếu IMU bị lệch thì chuyên roll sang pitch
	angleRoll += anglePitch * sin((float)gyro.z * 0.000001066);

	angleYaw -= CourseDeviation(angleYaw, actualCompassHeading) / 1200.0; //Tính toán sự khác biệt giữa actualCompassHeading và angleYaw và thực hiện một hiệu chỉnh nhỏ.
	if (angleYaw < 0) angleYaw += 360;
	else if (angleYaw >= 360) angleYaw -= 360;


	//Accelerometer angle calculations
	accelTotalVector = sqrt((accel.x * accel.x) + (accel.y * accel.y) + (accel.z * accel.z));    //Tính tổng vectơ gia tốc kế.

	if (abs(accel.y) < accelTotalVector) {                                             //tránh góc asin bị lỗi.
		anglePitchAccel = asin((float)accel.y / accelTotalVector) * 57.296;              //Calculate the pitch angle.
	}
	if (abs(accel.x) < accelTotalVector) {                                             //tránh góc asin bị lỗi.
		angleRollAccel = asin((float)accel.x / accelTotalVector) * 57.296;               //Calculate the roll angle.
	}

	anglePitch = anglePitch * 0.9996 + anglePitchAccel * 0.0004;                   //Hiệu chỉnh độ lệch góc của con quay hồi chuyển bằng góc của gia tốc kế..
	angleRoll = angleRoll * 0.9996 + angleRollAccel * 0.0004;                      //Điều chỉnh độ lệch của góc con quay hồi chuyển bằng góc của gia tốc kế

}
void IMU9DOF::CalculateGyroInput()   //65.5 = 1 deg/sec (check the datasheet of the MPU-6050 for more information).
{
	gyro.rollInput = (gyro.rollInput * 0.7) + (((float)gyro.x / 65.5) * 0.3);  //Gyro pid input is deg/sec.
	gyro.pitchInput = (gyro.pitchInput * 0.7) + (((float)gyro.y / 65.5) * 0.3);
	gyro.yawInput = (gyro.yawInput * 0.7) + (((float)gyro.z / 65.5) * 0.3);
}

void IMU9DOF::setLevelAdjust(uint8_t theRateRoll, uint8_t theRatePitch)// hiệu chỉnh cân bằng ban đầu khi khởi động
{
	pitchLevelAdjust = anglePitch * theRatePitch;
	rollLevelAdjust = angleRoll * theRateRoll;
}

float IMU9DOF::CourseDeviation(float theValue1, float theValue2)
{
	float actualCourseMirrored = 0;
	float baseCourseMirrored = 0;
	float result = theValue1 - theValue2;

	if (result < -180 || result > 180) {
		if (theValue2 > 180){
			baseCourseMirrored = theValue2 - 180;
		}
		else {
			baseCourseMirrored = theValue2 + 180;
		}
		if (theValue1 > 180){
			actualCourseMirrored = theValue1 - 180;
		}
		else {
			actualCourseMirrored = theValue1 + 180;
		}
		result = actualCourseMirrored - baseCourseMirrored;
	}
	return result;
}



