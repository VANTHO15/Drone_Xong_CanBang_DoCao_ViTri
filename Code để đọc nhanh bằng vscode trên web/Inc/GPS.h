#ifndef __GPS_H
#define __GPS_H

#include "main.h"
#include "MySerial.h"



class GPS
{
private:
	uint8_t latitudeNorth, longtiudeEast ;
	uint16_t messageCounter;
	int32_t latGPS, lonGPS ;
	uint8_t newLineFound;
	int32_t previousLonGPS,previousLatGPS;
	uint8_t readSerialByte, incommingMessage[100];
	int32_t actualLatGPS, actualLonGPS;

	uint8_t waypointSet;
	int16_t gpsAddCounter;
	int32_t l_latWaypoint, l_lonWaypoint;
	float gpsPitchAdjustNorth, gpsRollAdjustNorth;
	float lat_gps_loop_add, lon_gps_loop_add, lat_gps_add, lon_gps_add;
	uint8_t new_gpsDataAvailable, new_gps_dataCounter;
	uint8_t gps_rotatingMemLocation;
	int32_t gps_latTotalAvarage, gps_lonTotalAvarage;
	int32_t gps_latRotatingMem[40], gps_lonRotatingMem[40];
	int32_t gps_latError, gps_lonError;
	int32_t gps_latErrorPrevious, gps_lonErrorPrevious;
	uint32_t gps_watchdogTimer;

	GPIO_TypeDef* port;
	uint16_t pin;

public:
	uint8_t numberUsedSats;
	uint8_t fixType;
	float Pgain, Dgain;
	float gpsPitchAdjust, gpsRollAdjust;

	GPS(GPIO_TypeDef* thePort, uint16_t thePin);
	void ReadGPS(uint8_t theStateMachine, uint8_t* theError, uint8_t* theFlightMode, float theAngleYaw);
	int GetLat(void);
	int GetLon(void);
	HAL_StatusTypeDef Init(void);
	void SetHardWare(UART_HandleTypeDef *huart,USART_TypeDef *UARTx);
	void MX_USART_UART_Init(unsigned int baud);
	void setKgainConstantRoll(float thePgainRoll, float theDgainRoll);

};


#endif

