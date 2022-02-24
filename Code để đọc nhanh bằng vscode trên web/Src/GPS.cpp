#include "GPS.h"
#include <stdlib.h>
#include <math.h>

extern UART_HandleTypeDef huart1;
MySerial Serial1(&huart1);

GPS::GPS(GPIO_TypeDef* thePort, uint16_t thePin){
	this->port = thePort;
	this->pin = thePin;
}

void GPS::setKgainConstantRoll(float thePgainRoll, float theDgainRoll){
	this->Pgain = thePgainRoll;
	this->Dgain = theDgainRoll;
}

HAL_StatusTypeDef GPS :: Init()
{
	Serial1.Init();
	uint8_t Disable_GPGSV[11] = {0xB5, 0x62, 0x06, 0x01, 0x03, 0x00, 0xF0, 0x03, 0x00, 0xFD, 0x15};
	uint8_t Set_to_5Hz[14] = {0xB5, 0x62, 0x06, 0x08, 0x06, 0x00,0xC8,
			0x00, 0x01, 0x00, 0x01, 0x00, 0xDE, 0x6A};
	uint8_t Set_to_57kbps[28] = {0xB5, 0x62, 0x06, 0x00, 0x14, 0x00, 0x01, 0x00, 0x00, 0x00,
			0xD0, 0x08, 0x00, 0x00, 0x00, 0xE1, 0x00, 0x00, 0x07, 0x00,
			0x07, 0x00, 0x00, 0x00, 0x00, 0x00, 0xE2, 0xE1};

	// Vo hieu hoa goi tin GPGSV bang giao thuc Ublox
	if ( HAL_UART_Transmit(Serial1.uart, Disable_GPGSV, 11, 500) != HAL_OK){
		return HAL_ERROR;
	}
	// Khoang thoi gian nho nhat de giao tiep voi module GPS o baundrat 9600bps
	HAL_Delay(350);

	// Thiet lap refresh rate la 5Hz bang giao thuc ublox
	if ( HAL_UART_Transmit(Serial1.uart, Set_to_5Hz, 14, 500) != HAL_OK){
		return HAL_ERROR;
	}
	// Khoang thoi gian nho nhat de giao tiep voi module GPS o baundrate 9600bps
	HAL_Delay(350);

	// Thiet lap toc do baud rate = 57.6kbps bang giao thuc ublox
	if ( HAL_UART_Transmit(Serial1.uart, Set_to_57kbps, 28, 500) != HAL_OK){
		return HAL_ERROR;
	}
	HAL_Delay(200);

	Serial1.uart->Init.BaudRate = 57600;
	if (HAL_UART_Init(Serial1.uart) != HAL_OK)
	{
		return HAL_ERROR;
	}
	HAL_Delay(200);

	return HAL_OK;
}

void GPS :: ReadGPS(uint8_t theStateMachine, uint8_t* theError, uint8_t* theFlightMode, float theAngleYaw)
{
	if (gpsAddCounter >= 0)
	{
		gpsAddCounter --;
	}
	while (Serial1.IsDataAvailable() && !newLineFound )
	{
		char readSerialByte = Serial1.Read();
		if (readSerialByte == '$') {
			for (messageCounter = 0; messageCounter <= 99; messageCounter ++)
			{
				incommingMessage[messageCounter] = '-';
			}
			messageCounter = 0;
		}
		else if (messageCounter <= 99)messageCounter ++;
		incommingMessage[messageCounter] = readSerialByte;
		if (readSerialByte == '*') newLineFound = 1;
	}
	if (newLineFound == 1)
	{
		newLineFound = 0;
		if (incommingMessage[4] == 'L' && incommingMessage[5] == 'L' && incommingMessage[7] == ',')
		{
// Đặt một số biến thành 0 nếu mô-đun GPS không tìm thấy thông tin hợp lệ. Điều này là cần thiết cho việc mất GPS khi bay.
			latGPS = 0;
			lonGPS = 0;
			previousLatGPS = 0;
			previousLonGPS = 0;
			numberUsedSats = 0;
		}
// Nếu đường bắt đầu bằng GA và nếu có bản sửa lỗi GPS, chúng ta có thể quét đường để tìm vĩ độ, kinh độ và số lượng vệ tinh.
		if (incommingMessage[4] == 'G' && incommingMessage[5] == 'A' && (incommingMessage[44] == '1' || incommingMessage[44] == '2'))
		{
			actualLatGPS = ((int)incommingMessage[19] - 48) *  (long)10000000;
			actualLatGPS += ((int)incommingMessage[20] - 48) * (long)1000000;
			actualLatGPS += ((int)incommingMessage[22] - 48) * (long)100000;
			actualLatGPS += ((int)incommingMessage[23] - 48) * (long)10000;
			actualLatGPS += ((int)incommingMessage[24] - 48) * (long)1000;
			actualLatGPS += ((int)incommingMessage[25] - 48) * (long)100;
			actualLatGPS += ((int)incommingMessage[26] - 48) * (long)10;
			actualLatGPS /= (long)6;
			actualLatGPS += ((int)incommingMessage[17] - 48) *  (long)100000000;
			actualLatGPS += ((int)incommingMessage[18] - 48) *  (long)10000000;
			actualLatGPS /= 10;

			actualLonGPS = ((int)incommingMessage[33] - 48) *  (long)10000000;
			actualLonGPS += ((int)incommingMessage[34] - 48) * (long)1000000;
			actualLonGPS += ((int)incommingMessage[36] - 48) * (long)100000;
			actualLonGPS += ((int)incommingMessage[37] - 48) * (long)10000;
			actualLonGPS += ((int)incommingMessage[38] - 48) * (long)1000;
			actualLonGPS += ((int)incommingMessage[39] - 48) * (long)100;
			actualLonGPS += ((int)incommingMessage[40] - 48) * (long)10;
			actualLonGPS /= (long)6;
			actualLonGPS += ((int)incommingMessage[30] - 48) * (long)1000000000;
			actualLonGPS += ((int)incommingMessage[31] - 48) * (long)100000000;
			actualLonGPS += ((int)incommingMessage[32] - 48) * (long)10000000;
			actualLonGPS /= 10;

			if (incommingMessage[28] == 'N')
				latitudeNorth = 1;
			else
				latitudeNorth = 0;

			if (incommingMessage[42] == 'E')
				longtiudeEast = 1;
			else
				longtiudeEast = 0;

			numberUsedSats = ((int)incommingMessage[46] - 48) * (long)10;
			numberUsedSats += (int)incommingMessage[47] - 48;

			if ( previousLatGPS == 0 && previousLonGPS == 0){
				previousLatGPS = actualLatGPS;
				previousLonGPS = actualLonGPS;
			}

			lat_gps_loop_add = (float)(actualLatGPS - previousLatGPS) / 10.0;
			lon_gps_loop_add = (float)(actualLonGPS - previousLonGPS) / 10.0;

			latGPS = previousLatGPS;
			lonGPS = previousLonGPS;

			previousLatGPS = actualLatGPS;
			previousLonGPS = actualLonGPS;

//GPS được đặt ở tốc độ làm mới 5Hz. Giữa mỗi 2 phép đo GPS, 9 giá trị GPS được mô phỏng.
			gpsAddCounter = 5;
			new_gps_dataCounter = 9;
			lat_gps_add = 0;
			lon_gps_add = 0;
			new_gpsDataAvailable = 1;
		}

//If the line starts with SA and if there is a GPS fix we can scan the line for the fix type (none, 2D or 3D).
		if (incommingMessage[4] == 'S' && incommingMessage[5] == 'A')
			fixType = (int)incommingMessage[9] - 48;

	}

//After 5 program loops 5 x 4ms = 20ms the gps_add_counter is 0.
	if (gpsAddCounter == 0 && new_gps_dataCounter > 0){
		new_gpsDataAvailable = 1;
		new_gps_dataCounter--;
		gpsAddCounter = 5;

		lat_gps_add += lat_gps_loop_add;
		if (abs(lat_gps_add) >= 1) {
			latGPS += (int)lat_gps_add;
			lat_gps_add -= (int)lat_gps_add;
		}

		lon_gps_add += lon_gps_loop_add;
		if (abs(lon_gps_add) >= 1) {
			lonGPS += (int)lon_gps_add;
			lon_gps_add -= (int)lon_gps_add;
		}
	}

	if (new_gpsDataAvailable) {
		if (numberUsedSats < 8){
			HAL_GPIO_TogglePin(this->port, this->pin);  // Thay đổi đèn LED trên STM32 để chỉ báo nhận GPS
		}
		else {
			HAL_GPIO_WritePin(this->port, this->pin, GPIO_PIN_RESET);
		}
		gps_watchdogTimer = HAL_GetTick();
		new_gpsDataAvailable = 0;

		if (*theFlightMode >= 3 && waypointSet == 0) { // bật FlightMode = 3 , thì lấy vị trí lưu lại
			waypointSet = 1;
			l_latWaypoint = latGPS;
			l_lonWaypoint = lonGPS;
		}

		if (*theFlightMode >= 3 && waypointSet == 1) {
			gps_lonError = l_lonWaypoint - lonGPS;
			gps_latError = latGPS - l_latWaypoint;

			gps_latTotalAvarage -=  gps_latRotatingMem[ gps_rotatingMemLocation];
			gps_latRotatingMem[ gps_rotatingMemLocation] = gps_latError - gps_latErrorPrevious;
			gps_latTotalAvarage +=  gps_latRotatingMem[ gps_rotatingMemLocation];
			gps_lonTotalAvarage -=  gps_lonRotatingMem[ gps_rotatingMemLocation];
			gps_lonRotatingMem[ gps_rotatingMemLocation] = gps_lonError - gps_lonErrorPrevious;
			gps_lonTotalAvarage +=  gps_lonRotatingMem[ gps_rotatingMemLocation];
			gps_rotatingMemLocation++;
			if ( gps_rotatingMemLocation == 35) gps_rotatingMemLocation = 0;

			gps_latErrorPrevious = gps_latError;
			gps_lonErrorPrevious = gps_lonError;

			gpsPitchAdjustNorth = (float)gps_latError * Pgain + (float)gps_latTotalAvarage * Dgain;
			gpsPitchAdjustNorth = (float)gps_lonError * Pgain + (float)gps_lonTotalAvarage * Dgain;

			if (!latitudeNorth)gpsPitchAdjustNorth *= -1;
			if (!longtiudeEast)gpsRollAdjustNorth *= -1;

			gpsRollAdjust = ((float)gpsRollAdjustNorth * cos(theAngleYaw * 0.017453)) + ((float)gpsPitchAdjustNorth * cos((theAngleYaw - 90) * 0.017453));
			gpsPitchAdjust = ((float)gpsPitchAdjustNorth * cos(theAngleYaw * 0.017453)) + ((float)gpsRollAdjustNorth * cos((theAngleYaw + 90) * 0.017453));

			if (gpsRollAdjust > 300) gpsRollAdjust = 300;
			if (gpsRollAdjust < -300) gpsRollAdjust = -300;
			if (gpsPitchAdjust > 300) gpsPitchAdjust = 300;
			if (gpsPitchAdjust < -300) gpsPitchAdjust = -300;
		}
	}

	if (gps_watchdogTimer + 1000 < HAL_GetTick()) {
		if (*theFlightMode >= 3 && theStateMachine > 0) {
			*theFlightMode = 2;
			*theError = 4;
		}
	}

	if (*theFlightMode < 3 && waypointSet > 0) {
		gpsRollAdjust = 0;
		gpsPitchAdjust = 0;
		if (waypointSet == 1) {
			gps_rotatingMemLocation = 0;
			waypointSet = 2;
		}
		gps_lonRotatingMem[ gps_rotatingMemLocation] = 0;
		gps_latRotatingMem[ gps_rotatingMemLocation] = 0;
		gps_rotatingMemLocation++;
		if (gps_rotatingMemLocation == 36) {
			waypointSet = 0;
			gps_latErrorPrevious = 0;
			gps_lonErrorPrevious = 0;
			gps_latTotalAvarage = 0;
			gps_lonTotalAvarage = 0;
			gps_rotatingMemLocation = 0;
		}
	}

}
int GPS :: GetLat()
{
	int lat;
	lat = actualLatGPS;
	return lat;

}
int GPS :: GetLon()
{
	int lon;
	lon = actualLatGPS;
	return lon;

}

extern "C"
{
void USART1_IRQHandler(void)
{
	/* USER CODE BEGIN USART1_IRQn 0 */
	Serial1.UartIsr();
	/* USER CODE END USART1_IRQn 0 */
	HAL_UART_IRQHandler(&huart1);
	/* USER CODE BEGIN USART1_IRQn 1 */

	/* USER CODE END USART1_IRQn 1 */
}
}


