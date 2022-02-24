/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2022 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "HuLaNRF24L01.h"
#include "BMP180.h"

#include "IMU9DOF.h"
#include "math.h"
#include "flash.h"
#include "stdio.h"

#include "barometer.h"
#include "GPS.h"
#include "PIDHorizontal.h"

#include "dwt_stm32_delay.h"
#include "MySerial.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define CHANGE_SPEED_MOTOR_FRONT_RIGHT_CCW(speedVal)   	htim4.Instance->CCR1=speedVal
#define CHANGE_SPEED_MOTOR_REAR_RIGHT_CW(speedVal)   	htim4.Instance->CCR2=speedVal
#define CHANGE_SPEED_MOTOR_REAR_LEFT_CCW(speedVal)  	htim4.Instance->CCR3=speedVal
#define CHANGE_SPEED_MOTOR_FRONT_LEFT_CW(speedVal)   	htim4.Instance->CCR4=speedVal
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

I2C_HandleTypeDef hi2c1;
I2C_HandleTypeDef hi2c2;

SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim4;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */

uint8_t RxAddress[] = {0x00,0xDD,0xCC,0xBB,0xAA};
uint8_t RxData[32], stateThrottle=255;
float x=0, DoCao,NhietDo;

int16_t escFrontRightCCW, escRearRightCW, escRearLeftCCW, escFrontLeftCW;
uint16_t roll, pitch, yaw , throttle=1200;


// Các biến liên quan đến trạng thái hoạt động của drone
uint8_t flightMode;
uint8_t waypointSet;
uint8_t takeoffDetected;
uint8_t stateMachine=0;
uint8_t error;

int Start=0, BTN6 =0, BTN5=0, BTN4=0,BTN3=0, BTN2=0, BTN1=0, FirstBTN4=0,FirstBTN3=0, BTNFlightMode=0,FirtsMachine0=0;

uint16_t notificationCount;
// Todo
int16_t Throttle;
float groundPressure;
int16_t takeoffThrottle;
int16_t idleSpeedMotor = 1200;  // tốc độ động cơ không bay
int16_t  manualTakeoffThrottle= 1500;  // Tốc độ cất cánh
uint16_t time =0, time1 =0;

IMU9DOF gy86(&hi2c1,11, 0x080E0000);
Barometer ms5611(&hi2c1);
GPS gpsM8N(LED1_GPIO_Port, LED1_Pin);  // led kiểm tra trạng thái kết nối của GPS
PID_Horizontal pidForHorizontal;


void ReceiveDataFromNRF24L01();
void ReadDataFromBMP180();
void StartStopTakeOff(void);
void ChangeSetting(void);


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM4_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_I2C1_Init(void);
static void MX_SPI1_Init(void);
static void MX_ADC1_Init(void);
static void MX_I2C2_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void ReceiveDataFromNRF24L01()
{
	// bấm BTN5 thì headingLock =1
	// bấm nút BTN1 , flightMode = 2, Bấm BTN6 thì FlightMode =3;
	if(isDataAvailable(2)==1)
	  {
		  NRF24_Receive(RxData);
		  HAL_GPIO_WritePin(LED3_GPIO_Port, LED3_Pin,GPIO_PIN_SET);

		  if(RxData[0]==1) //  Trai Len , Tang Do Cao
			{
			    stateThrottle =1;
				throttle = throttle + 2;
				if(throttle > 1800)  throttle = 1800;
			}
			else if(RxData[0]==0) // Trai Xuong , Giam Do Cao
			{
				stateThrottle=0;
				throttle =throttle - 2;
				if(throttle < 1000)  throttle = 1000;
			}
			else
			{
				stateThrottle = 255;
			}
		    yaw = RxData[1]*100 + RxData[2];
			roll = RxData[3]*100 + RxData[4];
			pitch = RxData[5]*100 + RxData[6];

			if(RxData[7] == 2) flightMode =2;
			else if(RxData[7] == 3) flightMode =3;
			else flightMode=1;

			BTN2 = RxData[8];
			BTN3 = RxData[9];
			BTN4 = RxData[10];
			BTN5 = RxData[11];
	  }
	else
	{
		if(takeoffDetected == 1)
		{
			 throttle =throttle - 0.1;
			 if(throttle < 1300) throttle= 1300;
			 yaw = 1473;
			 roll = 1490;
			 pitch = 1478;
			 stateThrottle = 0;
		}
		 HAL_GPIO_WritePin(LED3_GPIO_Port, LED3_Pin, GPIO_PIN_RESET);

	}
}
void ReadDataFromBMP180()
{
	DoCao = BMP180_GetAlt(0);
//	NhietDo = BMP180_GetTemp();
}

void ChangeSetting()
{
	//TODO nhay den bao hieu
	float adjustableRollPitchValuePgain = pidForHorizontal.roll.Pgain;
	float adjustableRollPitchValueIgain = pidForHorizontal.roll.Igain;
	float adjustableRollPitchValueDgain = pidForHorizontal.roll.Dgain;

	float adjustableYawValuePgain = pidForHorizontal.yaw.Pgain;
	float adjustableYawValueIgain = pidForHorizontal.yaw.Igain;
	float adjustableYawValueDgain = pidForHorizontal.yaw.Dgain;

	float adjustableAltitudeValuePgain = ms5611.pid_Pgain_Altitude;
	float adjustableAltitudeValueIgain = ms5611.pid_Igain_Altitude;
	float adjustableAltitudeValueDgain = ms5611.pid_Dgain_Altitude;


	for(int i=1;i<=5;i++)
	{
		DWT_Delay_us(4000);


			if (roll > 1550) adjustableRollPitchValuePgain += (float)(roll - 1550) * 0.000001;
			if (roll < 1450) adjustableRollPitchValuePgain -= (float)(1450 - roll) * 0.000001;
			if (adjustableRollPitchValuePgain < 0)adjustableRollPitchValuePgain = 0;
			pidForHorizontal.roll.Pgain = adjustableRollPitchValuePgain;
			pidForHorizontal.pitch.Pgain = adjustableRollPitchValuePgain;

			if (pitch > 1550)adjustableRollPitchValueIgain += (float)(pitch - 1550) * 0.000001;
			if (pitch < 1450)adjustableRollPitchValueIgain -= (float)(1450 - pitch) * 0.000001;
			if (adjustableRollPitchValueIgain < 0)adjustableRollPitchValueIgain = 0;
			pidForHorizontal.roll.Igain = adjustableRollPitchValueIgain;
			pidForHorizontal.pitch.Igain = adjustableRollPitchValueIgain;

			if (yaw > 1550)adjustableRollPitchValueDgain += (float)(yaw - 1550) * 0.000001;
			if (yaw < 1450)adjustableRollPitchValueDgain -= (float)(1450 - yaw) * 0.000001;
			if (adjustableRollPitchValueDgain < 0)adjustableRollPitchValueDgain = 0;
			pidForHorizontal.roll.Dgain = adjustableRollPitchValueDgain;
			pidForHorizontal.pitch.Dgain = adjustableRollPitchValueDgain;

			///////////////////////////////////////////////////

			if (roll > 1550) adjustableYawValuePgain += (float)(roll - 1550) * 0.000001;
			if (roll < 1450) adjustableYawValuePgain -= (float)(1450 - roll) * 0.000001;
			if (adjustableYawValuePgain < 0)adjustableYawValuePgain = 0;
			pidForHorizontal.yaw.Pgain = adjustableYawValuePgain;

			if (pitch > 1550)adjustableYawValueIgain += (float)(pitch - 1550) * 0.000001;
			if (pitch < 1450)adjustableYawValueIgain -= (float)(1450 - pitch) * 0.000001;
			if (adjustableYawValueIgain < 0)adjustableYawValueIgain = 0;
			pidForHorizontal.yaw.Igain = adjustableYawValueIgain;

			if (yaw > 1550)adjustableYawValueDgain += (float)(yaw - 1550) * 0.000001;
			if (yaw < 1450)adjustableYawValueDgain -= (float)(1450 - yaw) * 0.000001;
			if (adjustableYawValueDgain < 0)adjustableYawValueDgain = 0;
			pidForHorizontal.yaw.Dgain = adjustableYawValueDgain;

			////////////////////////////////////////////////////

			if (roll > 1550) adjustableAltitudeValuePgain += (float)(roll - 1550) * 0.000001;
			if (roll < 1450) adjustableAltitudeValuePgain -= (float)(1450 - roll) * 0.000001;
			if (adjustableAltitudeValuePgain < 0)adjustableAltitudeValuePgain = 0;
			ms5611.pid_Pgain_Altitude = adjustableAltitudeValuePgain;

			if (pitch > 1550)adjustableAltitudeValueIgain += (float)(pitch - 1550) * 0.000001;
			if (pitch < 1450)adjustableAltitudeValueIgain -= (float)(1450 - pitch) * 0.000001;
			if (adjustableAltitudeValueIgain < 0)adjustableAltitudeValueIgain = 0;
			ms5611.pid_Igain_Altitude = adjustableAltitudeValueIgain;

			if (yaw > 1550)adjustableAltitudeValueDgain += (float)(yaw - 1550) * 0.000001;
			if (yaw < 1450)adjustableAltitudeValueDgain -= (float)(1450 - yaw) * 0.000001;
			if (adjustableAltitudeValueDgain < 0)adjustableAltitudeValueDgain = 0;
			ms5611.pid_Dgain_Altitude = adjustableAltitudeValueDgain;

	}
}

void StartStopTakeOff(void)
{
	//TODO Bước 1 là nhấn nút BTN5 để bật cơ headingLock;
	// Bước 2 nhấn nút BTN4 để bật stateMachine = 1 ;
	// Bước 3 nhấn nút BTN3
	if ((BTN4 == 1) && (FirstBTN4==0))
	{
		FirstBTN4=1;
		stateMachine = 1;
	}

	// Bước 3 nhấn nút BTN3
	if ((stateMachine == 1) && (BTN3 == 1) && (FirstBTN3==0) )
	{
		FirstBTN3=1;
		// Thiết lập throttle value về giá trị idle motor speed
		Throttle = idleSpeedMotor;

		// Giá trị góc Roll và Pitch khởi điểm sẽ được tính toán chỉ dựa trên accel
		gy86.anglePitch = gy86.anglePitchAccel;
		gy86.angleRoll = gy86.angleRollAccel;

		// Lấy giá trị áp suất tại mặt đất, để tham chiếu cho sau này
		groundPressure = ms5611.GetActualPressure();

		gy86.courseLockHeading = gy86.angleYaw;

		gy86.accelTotalVectorAtStart = gy86.accelTotalVector;

		stateMachine = 2;
		gy86.accelAltIntegrated = 0;
		if ((manualTakeoffThrottle > 1400) && (manualTakeoffThrottle < 1600))  // Tốc độ cất cánh 1500
		{
			takeoffThrottle = 300;
			// Thiết lập cờ takeoffDetected để chỉ ra rằng quadcopter đang bay
			takeoffDetected = 1;
			//Reset PID để việc cất cánh được smooth hơn
			pidForHorizontal.roll.ImemValue = 0;
			pidForHorizontal.roll.previousError = 0;
			pidForHorizontal.roll.outputValue = 0;
			pidForHorizontal.pitch.ImemValue = 0;
			pidForHorizontal.pitch.previousError = 0;
			pidForHorizontal.pitch.outputValue = 0;
			pidForHorizontal.yaw.ImemValue = 0;
			pidForHorizontal.yaw.previousError = 0;
			pidForHorizontal.yaw.outputValue = 0;
		}
		else if (manualTakeoffThrottle)
		{                                            //Nếu giá trị không hợp lệ
			error = 5;                                //Error = 5.
			takeoffThrottle = 0;                      //No hover throttle compensation.
			stateMachine = 0;                          //Set the start variable to 0 to stop the motors.
		}
	}

	//Stopping the motors:Bấm nút BTN2.
	if ( BTN2==1)
	{
		stateMachine = 0;                   //Đặt biến khởi động thành 0 để tắt động cơ.
		takeoffDetected = 0;                  //Đặt lại phát hiện tự động cất cánh
	}


}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_TIM4_Init();
  MX_USART1_UART_Init();
  MX_I2C1_Init();
  MX_SPI1_Init();
  MX_ADC1_Init();
  MX_I2C2_Init();
  /* USER CODE BEGIN 2 */

  gy86.declination = -1.5;
  DWT_Delay_Init();
  BMP180_Start();

  // NRF24L01
   NRF24_Init();
   NRF24_RxMode(RxAddress, 10);
   HAL_Delay(50);  // �?ợi 50ms de on dinh con NRF

   ms5611.SetKgainPID(1.4, 0.2, 0.75, 400);
   gpsM8N.setKgainConstantRoll(2.7, 6.5);
//   pidForHorizontal.setKgainConstantRoll(1.4, 0.04 , 25, 400);
//   pidForHorizontal.setKgainConstantPitch(1.4, 0.04, 25, 400);
//   pidForHorizontal.setKgainConstantYaw(1.4, 0.02 , 0, 400);
   pidForHorizontal.setKgainConstantRoll(1.4, 0.052 , 8.08, 400);
   pidForHorizontal.setKgainConstantPitch(1.4, 0.052, 8.08, 400);
   pidForHorizontal.setKgainConstantYaw(1, 0.02 , 0, 400);

   // Khởi tạo PWM cho việc đi�?u khiển ESC
	HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_4);

	CHANGE_SPEED_MOTOR_FRONT_RIGHT_CCW(1000);  // 1580
	CHANGE_SPEED_MOTOR_REAR_RIGHT_CW(1000);		// 1580
	CHANGE_SPEED_MOTOR_REAR_LEFT_CCW(1000);		// 1580
	CHANGE_SPEED_MOTOR_FRONT_LEFT_CW(1000);		// 1580

	// Kiểm tra các cảm biến
	if (((ms5611.IsReadyToInterface() != BAROMETER_Result_Ok))||(gy86.IsReadyToInterfaceMPU6050() != IMU9DOF_Result_Ok) ||(gy86.IsReadyToInterfaceHMC5883L() != IMU9DOF_Result_Ok)) //(ms5611.IsReadyToInterface() != BAROMETER_Result_Ok)
	{
		HAL_GPIO_WritePin(BUZZ_GPIO_Port, BUZZ_Pin,GPIO_PIN_SET);
		for(;;)
		{
			// Reset Lại Drone ( Cảm biến bị lỗi )
		}
	}
//	 Init các cảm biến
	if ( (gy86.Init() != HAL_OK) || (ms5611.Init() != HAL_OK) || (gpsM8N.Init() != HAL_OK) )   //  || (gpsM8N.Init() != HAL_OK)
	  {
		HAL_GPIO_WritePin(BUZZ_GPIO_Port, BUZZ_Pin,GPIO_PIN_SET);
		for(;;)
		{
			// Reset lại Drone
		}
	  }

	// On dinh bo dem cho barometer, khong nhan gia tri sai ban dau (truoc khi vo chuong trinh chinh
	for (int i = 0; i < 100 ; i++)
	{
		ms5611.ReadAndCalculatePIDBarometer(flightMode, takeoffDetected,stateThrottle);
		HAL_Delay(4);
	}

	ms5611.SetGroundPressure();

	gy86.ReadRawAllParameter();
	gy86.ReadCompass();
	gy86.angleYaw = gy86.actualCompassHeading;

	gy86.CalibGyro();

//	// �?ưa gia tri accel vao bo dem de on dinh truoc khi vao chuong trinh chinh
	for (int i = 0; i <= 24; i++)
	{
		gy86.shortAverageAccel_Z[i] = gy86.accel.z;
	}
	for (int i = 0; i <= 49; i++)
	{
		gy86.longAverageAccel_Z[i] = gy86.accel.z;
	}

	gy86.shortTotalAccel_Z = gy86.accel.z * 25;
	gy86.longTotalAccel_Z = gy86.accel.z * 50;


//	todo while 1
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

	  time = HAL_GetTick();

	  // Nhận dữ liệu NRF24
	  ReceiveDataFromNRF24L01();
	  ReadDataFromBMP180();

	  if ((stateMachine == 0)&& (FirtsMachine0 == 0)  ) // Trạng thái khởi động
	  {
		  FirtsMachine0 = 1;

		  gy86.CalibCompass();  // hiệu chỉnh la bàn

		  gy86.CalibLevel(&error);  // Hiệu chỉnh cân bằng ban đầu

		  ChangeSetting();   // change setting

		  HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, GPIO_PIN_SET);

		  gy86.headingLock = 0; // Xóa cờ heading Lock


	  }

	  if(BTN5 == 1 )  // Bấm BTN5
	  {
//		  gy86.headingLock = 1;
	  }

	  gy86.ReadGyroAccel(); // tính toán Gyro và Accel
	  gy86.ReadCompass(); // Tính toán la bàn số

	  ms5611.ReadAndCalculatePIDBarometer(flightMode, takeoffDetected, stateThrottle);  // Đọc giá trị áp xuất và tính toán PID ổn định độ cao

	  gpsM8N.ReadGPS(stateMachine, &error, &flightMode, gy86.angleYaw);    // Đọc gia strij GPS và tính toán PID ổn định vị trí

	  gy86.CalculateGyroInput();  // tính toán gái trị đầu vào feedback cho bộ điều khiển PID, ổn định theo các phương
	  gy86.ReadAngleRPY();  // tính toán gọc đo Roll, Pitch, Yaw

	  gy86.setLevelAdjust(0, 0);  // hiệu chỉnh cân bằng ban đầu khi khởi động
	  gy86.VerticalAccelerationCalculations();  // Tính toán gia tốc

	  // Thiết lập lại setpoint cho đầu vào bộ PID ổn định roll pitch
	 pidForHorizontal.roll.setPointBase = roll;
	 pidForHorizontal.pitch.setPointBase = pitch;
	 pidForHorizontal.yaw.setPointBase = yaw;
	 pidForHorizontal.throttleSetpoint = throttle;

	 pidForHorizontal.roll.inputValue = gy86.gyro.rollInput;
	 pidForHorizontal.pitch.inputValue = gy86.gyro.pitchInput;
	 pidForHorizontal.yaw.inputValue = gy86.gyro.yawInput;

//	 if (gy86.headingLock == 1)  // Nếu bật headingLock thì tính tóan lại setpoint của roll pitch
//	  {
//		float headingLockCourseDeviation = gy86.CourseDeviation(gy86.angleYaw, gy86.courseLockHeading);
//		pidForHorizontal.roll.setPointBase = 1500 + ((float)(roll - 1500) * cos(headingLockCourseDeviation * 0.017453)) + ((float)(pitch - 1500) * cos((headingLockCourseDeviation - 90) * 0.017453));
//		pidForHorizontal.pitch.setPointBase = 1500 + ((float)(pitch - 1500) * cos(headingLockCourseDeviation * 0.017453)) + ((float)(roll - 1500) * cos((headingLockCourseDeviation + 90) * 0.017453));
//	  }

	 if (flightMode == 3 && waypointSet == 1)  // có thêm GPS
	 {
		pidForHorizontal.roll.setPointBase += gpsM8N.gpsRollAdjust;
		pidForHorizontal.pitch.setPointBase += gpsM8N.gpsPitchAdjust;
	 }

	 if (pidForHorizontal.roll.setPointBase > 2000)
	 {
		pidForHorizontal.roll.setPointBase = 2000;
	 }
	 if (pidForHorizontal.roll.setPointBase < 1000)
	 {
		pidForHorizontal.roll.setPointBase = 1000;
	 }
	 if (pidForHorizontal.pitch.setPointBase > 2000)
	 {
		pidForHorizontal.pitch.setPointBase = 2000;
	 }
	 if (pidForHorizontal.pitch.setPointBase < 1000)
	 {
		pidForHorizontal.pitch.setPointBase = 1000;
	 }

	 pidForHorizontal.setLevelAdjust(gy86.rollLevelAdjust, gy86.pitchLevelAdjust);
	 pidForHorizontal.calculatePID();

	 StartStopTakeOff();

	 // độ cao
	 if (takeoffDetected == 1 && stateMachine == 2)   // Nếu Drone được khởi động và đang bay
	 {
		Throttle = throttle + 0 ;  // takeoffThrottle=300   The base throttle is the receiver throttle tay cam + the detected take-off throttle.
		 if (flightMode == 2)
		{                                                          //nếu chế dộ độ cao đang hoạt động
			Throttle = throttle  + ms5611.GetPidOutputAltitude();
		}
	 }

	 if (stateMachine == 2) // motors đã đc khởi động
	 {
		if (Throttle > 1800)
		{
			Throttle = 1800;                                          //Giới hạn lại tốc độ
		}
		escFrontRightCCW = Throttle - pidForHorizontal.pitch.outputValue + pidForHorizontal.roll.outputValue - pidForHorizontal.yaw.outputValue;
		escRearRightCW = Throttle + pidForHorizontal.pitch.outputValue + pidForHorizontal.roll.outputValue + pidForHorizontal.yaw.outputValue;
		escRearLeftCCW = Throttle + pidForHorizontal.pitch.outputValue - pidForHorizontal.roll.outputValue - pidForHorizontal.yaw.outputValue;
		escFrontLeftCW = Throttle - pidForHorizontal.pitch.outputValue - pidForHorizontal.roll.outputValue + pidForHorizontal.yaw.outputValue;

		if (escFrontRightCCW < idleSpeedMotor)
		{
			escFrontRightCCW = idleSpeedMotor;                        //Keep the motors running.
		}
		if (escRearRightCW < idleSpeedMotor)
		{
			escRearRightCW = idleSpeedMotor;                        //Keep the motors running.
		}
		if (escRearLeftCCW < idleSpeedMotor)
		{
			escRearLeftCCW = idleSpeedMotor;                        //Keep the motors running.
		}
		if (escFrontLeftCW < idleSpeedMotor)
		{
			escFrontLeftCW = idleSpeedMotor;                        //Keep the motors running.
		}

		if (escFrontRightCCW > 1800)
		{
			escFrontRightCCW = 1800;                                                 //Limit the esc-1 pulse to 2000us.
		}
		if (escRearRightCW > 1800)
		{
			escRearRightCW = 1800;                                                 //Limit the esc-2 pulse to 2000us.
		}
		if (escRearLeftCCW > 1800)
		{
			escRearLeftCCW = 1800;                                                 //Limit the esc-3 pulse to 2000us.
		}
		if (escFrontLeftCW > 1800)
		{
			escFrontLeftCW = 1800;                                                 //Limit the esc-4 pulse to 2000us.
		}
	}
	else
	{
		escFrontRightCCW = 1000;                                                                  //If start is not 2 keep a 1000us pulse for ess-1.
		escRearRightCW = 1000;                                                                  //If start is not 2 keep a 1000us pulse for ess-2.
		escRearLeftCCW = 1000;                                                                  //If start is not 2 keep a 1000us pulse for ess-3.
		escFrontLeftCW = 1000;                                                                  //If start is not 2 keep a 1000us pulse for ess-4.
	}

	 if(stateMachine == 1)  // mới khởi động thì cho quay nhẹ
	 {
		 Throttle = 1200;
		 escFrontRightCCW = Throttle;
		 escRearRightCW = Throttle;
		 escRearLeftCCW = Throttle;
		 escFrontLeftCW = Throttle;
	 }

	 CHANGE_SPEED_MOTOR_FRONT_LEFT_CW(escFrontLeftCW + 375);
	 CHANGE_SPEED_MOTOR_FRONT_RIGHT_CCW(escFrontRightCCW + 375);
	 CHANGE_SPEED_MOTOR_REAR_LEFT_CCW(escRearLeftCCW + 375 );
	 CHANGE_SPEED_MOTOR_REAR_RIGHT_CW(escRearRightCW + 375);

	 time1 = (HAL_GetTick()-time);
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 72;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */
  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = DISABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_10;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_15CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 400000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C2_Init(void)
{

  /* USER CODE BEGIN I2C2_Init 0 */

  /* USER CODE END I2C2_Init 0 */

  /* USER CODE BEGIN I2C2_Init 1 */

  /* USER CODE END I2C2_Init 1 */
  hi2c2.Instance = I2C2;
  hi2c2.Init.ClockSpeed = 400000;
  hi2c2.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */

}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 71;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 19999;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */
  HAL_TIM_MspPostInit(&htim4);

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 9600;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, BUZZ_Pin|NRF_CSN_Pin|LED1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, NRF_CE_Pin|LED2_Pin|LED3_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(I2C1_SFYNC_GPIO_Port, I2C1_SFYNC_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : BUZZ_Pin NRF_CSN_Pin LED1_Pin */
  GPIO_InitStruct.Pin = BUZZ_Pin|NRF_CSN_Pin|LED1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : NRF_CE_Pin LED2_Pin LED3_Pin */
  GPIO_InitStruct.Pin = NRF_CE_Pin|LED2_Pin|LED3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : I2C1_DRDY_Pin I2C1_INTA_Pin */
  GPIO_InitStruct.Pin = I2C1_DRDY_Pin|I2C1_INTA_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pin : I2C1_SFYNC_Pin */
  GPIO_InitStruct.Pin = I2C1_SFYNC_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(I2C1_SFYNC_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);

  HAL_NVIC_SetPriority(EXTI1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI1_IRQn);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
