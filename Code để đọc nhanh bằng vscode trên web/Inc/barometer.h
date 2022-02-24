#ifndef _BAROMETER_H
#define _BAROMETER_H

#include "main.h"

typedef enum  {
	BAROMETER_Result_Ok = 0x00,
	BAROMETER_Result_Error,
	BAROMETER_Result_DeviceNotConnected, // Thiết bị không được kết nối đến
	BAROMETER_Result_DeviceInvalid       // Không phải địa chỉ của mpu6050
} BAROMETER_Result;


class Barometer {
private :

	I2C_HandleTypeDef* hi2c;
	// các hằng số đặc tính của cảm biến. Sử dụng cho việc calib
	uint16_t dataProm[6];
	int64_t OFF2;
	int64_t SENS2;



	// các biến lưu trữ giá trị thô chưa được calib
	uint32_t rawTemperature;
	uint32_t rawPressure;

	// biến lưu trứ giá trị áp suất sau khi đã calib
	int64_t compensatedPressure;   // 	áp xuất bù

	uint8_t stageOfBarometer;
	uint8_t stageOfTemperature;

	// biến sử dụng cho việc lấy trung bình từ luồng giá trị nhiệt độ
	uint8_t indexAverageTemperatureMem;
	uint32_t rawTemperatureRotatingMemory[6];
	uint32_t rawAverageTemperatureTotal;

	// biến sử dụng cho việc lấy trung ibnhf từ luồng giá trị áp suất
	int32_t pressureRotatingMemory[50];
	int32_t averagePressureTotal;
	uint8_t indexAveragePressureMem;

	// biến sử dụng cho bộ lọc complementary
	float actualPressure;
	float actualPressureSlow;
	float actualPressureFast;
	float actualPressureDiff;

	// biến sử dụng cho bộ điều khiển PID
	int16_t manualThrottle;
	uint8_t manualAltitudeChange;
	uint8_t parachuteRotatingMemLocation;
	int32_t parachuteBuffer[35], parachuteThrottle;
	float pressureParachutePrevious;
	float pidErrorGainAltitude;
	float pidErrorAltitudeTemp;
	float pid_Imem_Altitude;
	float pidAltitudeSetpoint;
	float pidAltitudeInput;
	float pidOutputAltitude;
	float pidLastAltitude_D_error;

	HAL_StatusTypeDef Reset();
	HAL_StatusTypeDef ReadProm();
	uint32_t GetDataFromPreviousRequest();
	uint32_t GetAverageTemperature();
	void RequestGetTemperatureData();
	void RequestGetPressureData();
	int64_t CompensatePressure();
	float GetAveragePressure();
	float UseComplementaryFilter();
	void CalculateLongtermChange();
	void CalculateAltitudePID(uint8_t TheStateThrottle);
	void ResetValuesOfPID();
public:
	// khai báo các hằng số khuếch đại của bộ PID hold Altitude
	float pid_Pgain_Altitude;
	float pid_Igain_Altitude;
	float pid_Dgain_Altitude;
	int16_t	pidMaxAltitude;

	void SetKgainPID(float thePgain,float theIgain, float theDgain, int16_t theMaxPID);
	void ReadAndCalculatePIDBarometer(uint8_t theFlightMode, uint8_t theTakeoffDetected,uint8_t TheStateThrottle);
	BAROMETER_Result IsReadyToInterface();
	HAL_StatusTypeDef Init();
	Barometer(I2C_HandleTypeDef * theI2c);
	void SetGroundPressure();
	int16_t GetManualThrottle();
	float GetActualPressure();
	void Set_PID_altitude_setpoint(float thePidAltitudeSetpoint);
	float GetPidOutputAltitude();
};

#endif
