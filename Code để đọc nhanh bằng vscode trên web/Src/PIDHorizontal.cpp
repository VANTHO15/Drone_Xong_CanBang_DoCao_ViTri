
#include "PIDHorizontal.h"


PID_Horizontal::PID_Horizontal() {
	// TODO Auto-generated constructor stub

}

PID_Horizontal::~PID_Horizontal() {
	// TODO Auto-generated destructor stub
}

void PID_Horizontal::setKgainConstantRoll(float thePgainRoll, float theIgainRoll, float theDgainRoll, float theMaxOutputValue)
{
	this->roll.Pgain = thePgainRoll;
	this->roll.Igain = theIgainRoll;
	this->roll.Dgain = theDgainRoll;
	this->roll.maxOutputValue = theMaxOutputValue;
}
void PID_Horizontal::setKgainConstantPitch(float thePgainPitch, float theIgainPitch, float theDgainPitch, float theMaxOutputValue)
{
	this->pitch.Pgain = thePgainPitch;
	this->pitch.Igain = theIgainPitch;
	this->pitch.Dgain = theDgainPitch;
	this->pitch.maxOutputValue = theMaxOutputValue;
}
void PID_Horizontal::setKgainConstantYaw(float thePgainYaw, float theIgainYaw, float theDgainYaw, float theMaxOutputValue)
{
	this->yaw.Pgain = thePgainYaw;
	this->yaw.Igain = theIgainYaw;
	this->yaw.Dgain = theDgainYaw;
	this->yaw.maxOutputValue = theMaxOutputValue;
}

void PID_Horizontal::setLevelAdjust(float theLevelAdjustRoll, float theLevelAdjustPitch)
{
	roll.levelAdjust = theLevelAdjustRoll;
	pitch.levelAdjust = theLevelAdjustPitch;
}
void PID_Horizontal::calculatePID()
{
	// Chuyển đổi setPointRoll sang đơn vị dps để cùng kiểu dữ liệu với inputRoll
	// Chuyển đổi đơn vị bằng cách chia giá trị cho 3 ta được giá trị lớn nhất của setPointRoll ( (500-8)/3 = 164d/s ).
	roll.setPoint = 0;
	// Thiết lập khoảng deadBand là 16us để đạt được độ ổn định
	if (roll.setPointBase > 1526)
	{
		roll.setPoint = roll.setPointBase - 1526;
	}
	else if (roll.setPointBase < 1510)
	{
		roll.setPoint  =  roll.setPointBase - 1510;
	}

	roll.setPoint -= roll.levelAdjust;                                          //Subtract the angle correction from the standardized receiver roll input value.
	roll.setPoint /= 3.0;                                                        //Divide the setpoint for the PID roll controller by 3 to get angles in degrees.


	//The PID set point in degrees per second is determined by the pitch receiver input.
	//In the case of deviding by 3 the max pitch rate is aprox 164 degrees per second ( (500-8)/3 = 164d/s ).
	pitch.setPoint = 0;
	//We need a little dead band of 16us for better results.
	if (pitch.setPointBase > 1511)
	{
		pitch.setPoint = pitch.setPointBase - 1511;
	}
	else if (pitch.setPointBase < 1495)
	{
		pitch.setPoint = pitch.setPointBase - 1495;
	}

	pitch.setPoint -= pitch.levelAdjust;                                        //Subtract the angle correction from the standardized receiver pitch input value.
	pitch.setPoint /= 3.0;                                                       //Divide the setpoint for the PID pitch controller by 3 to get angles in degrees.

	//The PID set point in degrees per second is determined by the yaw receiver input.
	//In the case of deviding by 3 the max yaw rate is aprox 164 degrees per second ( (500-8)/3 = 164d/s ).
	yaw.setPoint = 0;
	//We need a little dead band of 16us for better results.
	if (throttleSetpoint > 1200) { //Do not yaw when turning off the motors.
		if (yaw.setPointBase > 1514)
		{
			yaw.setPoint = (yaw.setPointBase - 1514) / 3.0;
		}
		else if (yaw.setPointBase < 1498)
		{
			yaw.setPoint = (yaw.setPointBase - 1498) / 3.0;
		}
	}

	//Roll calculations
	roll.momentError = roll.inputValue - roll.setPoint;
	roll.ImemValue += roll.Igain * roll.momentError;
	if (roll.ImemValue > roll.maxOutputValue)
	{
		roll.ImemValue = roll.maxOutputValue;
	}
	else if (roll.ImemValue < roll.maxOutputValue * -1)
	{
		roll.ImemValue = roll.maxOutputValue * -1;
	}

	roll.outputValue = roll.Pgain * roll.momentError + roll.ImemValue + roll.Dgain * (roll.momentError - roll.previousError);
	if (roll.outputValue > roll.maxOutputValue)
	{
		roll.outputValue = roll.maxOutputValue;
	}
	else if (roll.outputValue  < roll.maxOutputValue * -1)
	{
		roll.outputValue = roll.maxOutputValue* -1;
	}

	roll.previousError = roll.momentError;

	//Pitch calculations
	pitch.momentError = pitch.inputValue - pitch.setPoint;
	pitch.ImemValue += pitch.Igain * pitch.momentError;
	if (pitch.ImemValue > pitch.maxOutputValue)
	{
		pitch.ImemValue = pitch.maxOutputValue;
	}
	else if (pitch.ImemValue < pitch.maxOutputValue * -1)
	{
		pitch.ImemValue = pitch.maxOutputValue * -1;
	}

	pitch.outputValue = pitch.Pgain * pitch.momentError + pitch.ImemValue + pitch.Dgain * (pitch.momentError - pitch.previousError);
	if (pitch.outputValue > pitch.maxOutputValue)
	{
		pitch.outputValue = pitch.maxOutputValue;
	}
	else if (pitch.outputValue  < pitch.maxOutputValue * -1)
	{
		pitch.outputValue = pitch.maxOutputValue* -1;
	}

	pitch.previousError = pitch.momentError;

	//Yaw calculations
	yaw.momentError = yaw.inputValue - yaw.setPoint;
	yaw.ImemValue += yaw.Igain * yaw.momentError;
	if (yaw.ImemValue > yaw.maxOutputValue)
	{
		yaw.ImemValue = yaw.maxOutputValue;
	}
	else if (yaw.ImemValue < yaw.maxOutputValue * -1)
	{
		yaw.ImemValue = yaw.maxOutputValue * -1;
	}

	yaw.outputValue = yaw.Pgain * yaw.momentError + yaw.ImemValue + yaw.Dgain * (yaw.momentError - yaw.previousError);
	if (yaw.outputValue > yaw.maxOutputValue)
	{
		yaw.outputValue = yaw.maxOutputValue;
	}
	else if (yaw.outputValue  < yaw.maxOutputValue * -1)
	{
		yaw.outputValue = yaw.maxOutputValue* -1;
	}

	yaw.previousError = yaw.momentError;
}
