/*
 * IRRobotDrive.cpp
 *
 *  Created on: 12 jan. 2016
 *      Author: Dylan Vos
 */

#include "IRCANRobotDrive.h"

#include <GenericHID.h>
#include <Utility.h>
#include <HAL/HAL.h>
#include <math.h>

#include <CanTalonSRX.h>

#include "../HumanInterfaceDevices/IRJoystick.h"

using namespace IR;

#define INVERT(x, y) (x-(2*y*x))

/**
 * Constructor for RobotDrive with 4 motors specified with channel numbers.
 * Set up parameters for a four wheel drive system where all four motor
 * pwm channels are specified in the call.
 * This call assumes Talons for controlling the motors.
 * Also to specify the DriveTrain type.
 * @param frontLeftMotor Front left motor channel number. 0-9 are on-board,
 * 10-19 are on the MXP port
 * @param rearLeftMotor Rear Left motor channel number. 0-9 are on-board, 10-19
 * are on the MXP port
 * @param frontRightMotor Front right motor channel number. 0-9 are on-board,
 * 10-19 are on the MXP port
 * @param rearRightMotor Rear Right motor channel number. 0-9 are on-board,
 * 10-19 are on the MXP port
 * @param driveTrain use this to set drivetrain type. Default = Tank
 */
IRCANRobotDrive::IRCANRobotDrive(uint32_t frontLeftMotorChannel, uint32_t rearLeftMotorChannel,
				uint32_t frontRightMotorChannel, uint32_t rearRightMotorChannel, DriveTrain driveTrain) :
				m_frontLeftMotor(frontLeftMotorChannel),
				m_rearLeftMotor(rearLeftMotorChannel),
				m_frontRightMotor(frontRightMotorChannel),
				m_rearRightMotor(rearRightMotorChannel),
				m_driveTrain(driveTrain)

{
	SetOutputMotors(0.0, 0.0);
}

void IRCANRobotDrive::SetMotorsInverted(bool inverted)
{
	SetMotorsInverted(inverted, inverted, inverted, inverted);
}

void IRCANRobotDrive::SetMotorsInverted(bool frontLeft, bool rearLeft, bool frontRight, bool rearRight)
{
	m_invertedFrontLeftMotor = frontLeft;
	m_invertedFrontRightMotor = frontRight;
	m_invertedRearLeftMotor = rearLeft;
	m_invertedRearRightMotor = rearRight;
}

/**
 * Drive the motors at "outputMagnitude" and "curve".
 * Both outputMagnitude and curve are -1.0 to +1.0 values, where 0.0 represents
 * stopped and not turning. curve < 0 will turn left and curve > 0 will turn
 * right.
 *
 * The algorithm for steering provides a constant turn radius for any normal
 * speed range, both forward and backward. Increasing m_sensitivity causes
 * sharper turns for fixed values of curve.
 *
 * This function will most likely be used in an autonomous routine.
 *
 * @param outputMagnitude The speed setting for the outside wheel in a turn,
 *        forward or backwards, +1 to -1.
 * @param curve The rate of turn, constant for different forward speeds. Set
 *        curve < 0 for left turn or curve > 0 for right turn.
 * Set curve = e^(-r/w) to get a turn radius r for wheelbase w of your robot.
 * Conversely, turn radius r = -ln(curve)*w for a given value of curve and
 * wheelbase w.
 */
void IRCANRobotDrive::Drive(double outputMagnitude, double curve) {
  float leftOutput, rightOutput;

  if (curve < 0) {
    float value = log(-curve);
    float ratio = (value - m_sensitivity) / (value + m_sensitivity);
    if (ratio == 0) ratio = .0000000001;
    leftOutput = outputMagnitude / ratio;
    rightOutput = outputMagnitude;
  } else if (curve > 0) {
    float value = log(curve);
    float ratio = (value - m_sensitivity) / (value + m_sensitivity);
    if (ratio == 0) ratio = .0000000001;
    leftOutput = outputMagnitude;
    rightOutput = outputMagnitude / ratio;
  } else {
    leftOutput = outputMagnitude;
    rightOutput = outputMagnitude;
  }
  SetOutputMotors(leftOutput, rightOutput);
}

void IRCANRobotDrive::Drive(double direction, double speed, double rotation, double gyro) {
  speed = Limit(speed) * std::sqrt(2.0);
  // The rollers are at 45 degree angles.
  double dirInRad = (direction + 45.0) * M_PI / 180.0;
  double cosD = std::cos(dirInRad);
  double sinD = std::sin(dirInRad);

  double wheelSpeeds[4];
  wheelSpeeds[0] = sinD * speed + rotation;
  wheelSpeeds[1] = cosD * speed + rotation;
  wheelSpeeds[2] = cosD * speed - rotation;
  wheelSpeeds[3] = sinD * speed - rotation;

  SetOutputMotors(wheelSpeeds[0], wheelSpeeds[1], wheelSpeeds[2], wheelSpeeds[3]);
}

/**
 * Arcade drive implements single stick driving.
 * Given a single Joystick, the class assumes the Y axis for the move value and
 * the X axis
 * for the rotate value.
 * (Should add more information here regarding the way that arcade drive works.)
 * @param stick The joystick to use for Arcade single-stick driving. The Y-axis
 * will be selected
 * for forwards/backwards and the X-axis will be selected for rotation rate.
 */
void IRCANRobotDrive::ArcadeDrive(frc::GenericHID *stick, double gyro)
{
	ArcadeDrive(stick->GetY(), stick->GetX(), 0.0, gyro);
}

/**
 * Arcade drive implements single stick driving.
 * Given a single Joystick, the class assumes the Y axis for the move value and
 * the X axis
 * for the rotate value.
 * (Should add more information here regarding the way that arcade drive works.)
 * @param stick The joystick to use for Arcade single-stick driving. The Y-axis
 * will be selected
 * for forwards/backwards and the X-axis will be selected for rotation rate.
 */
void IRCANRobotDrive::ArcadeDrive(frc::GenericHID &stick, double gyro)
{
	ArcadeDrive(stick.GetY(), stick.GetX(), 0.0, gyro);
}

/**
 * Arcade drive implements single stick driving.
 * Given a single IRJoystick, the class assumes the Y axis for the move value and
 * the X axis
 * for the rotate value.
 * (Should add more information here regarding the way that arcade drive works.)
 * @param stick The IRjoystick to use for Arcade single-stick driving. The Y-axis
 * will be selected
 * for forwards/backwards and the X-axis will be selected for rotation rate.
 */
void IRCANRobotDrive::ArcadeDrive(IR::IRJoystick *stick, double gyro)
{
	ArcadeDrive(stick->GetY(), stick->GetX(), stick->GetZ(), gyro, stick->GetLeveledThrottle());
}

/**
 * Arcade drive implements single stick driving.
 * Given a single IRJoystick, the class assumes the Y axis for the move value and
 * the X axis
 * for the rotate value.
 * (Should add more information here regarding the way that arcade drive works.)
 * @param stick The IRjoystick to use for Arcade single-stick driving. The Y-axis
 * will be selected
 * for forwards/backwards and the X-axis will be selected for rotation rate.
 */
void IRCANRobotDrive::ArcadeDrive(IR::IRJoystick &stick, double gyro)
{
	ArcadeDrive(stick.GetY(), stick.GetX(), stick.GetZ(), gyro, stick.GetLeveledThrottle());
}

/**
 * Arcade drive implements single stick driving.
 * Given a single IRJoystick, the class assumes the Y axis for the move value and
 * the X axis
 * for the rotate value.
 * (Should add more information here regarding the way that arcade drive works.)
 * @param stick The IRjoystick to use for Arcade single-stick driving. The Y-axis
 * will be selected
 * for forwards/backwards and the X-axis will be selected for rotation rate.
 */
void IRCANRobotDrive::ArcadeDrive(IR::IRJoystick *stick, bool deadZoned, double gyro)
{
	ArcadeDrive((deadZoned) ? stick->GetXDeadZoned() : stick->GetX(), (deadZoned) ? stick->GetYDeadZoned() : stick->GetY(), stick->GetZ(), gyro, stick->GetLeveledThrottle());
}

/**
 * Arcade drive implements single stick driving.
 * Given a single IRJoystick, the class assumes the Y axis for the move value and
 * the X axis
 * for the rotate value.
 * (Should add more information here regarding the way that arcade drive works.)
 * @param stick The IRjoystick to use for Arcade single-stick driving. The Y-axis
 * will be selected
 * for forwards/backwards and the X-axis will be selected for rotation rate.
 */
void IRCANRobotDrive::ArcadeDrive(IR::IRJoystick &stick, bool deadZoned, double gyro)
{
	ArcadeDrive((deadZoned) ? stick.GetXDeadZoned() : stick.GetX(), (deadZoned) ? stick.GetYDeadZoned() : stick.GetY(), stick.GetZ(), gyro, stick.GetLeveledThrottle());
}

/**
 * Arcade drive implements single stick driving.
 * This function lets you directly provide joystick values from any source.
 * @param y The value to use for fowards/backwards
 * @param x The value to use for the rotate right/left
 * @param t The value to use for the speedmodifier
 */
void IRCANRobotDrive::ArcadeDrive(double x, double y, double z, double t, double gyro)
{
	switch(m_driveTrain){
	case Tank:
		{
			float leftMotorsOutput = Limit(-(t * (y - x)));
			float rightMotorsOutput = Limit(t * (y + x));

			SetOutputMotors(leftMotorsOutput, rightMotorsOutput);
			break;
		}
	case Mecanum:
		{
			// Compenstate for gyro angle.
			RotateVector(x, y, gyro);

			double wheelSpeeds[4];
			wheelSpeeds[0] = (x + y + z) * t; //frontLeft
			wheelSpeeds[1] = (-x + y + z) * t; //rearLeft
			wheelSpeeds[2] = (-x + y - z) * t; //frontRight
			wheelSpeeds[3] = (x + y - z) * t; //rearRight

			Normalize(wheelSpeeds);

			SetOutputMotors(wheelSpeeds[0], wheelSpeeds[1], wheelSpeeds[2], wheelSpeeds[3]);

			break;
		}
	case Swerve:
		{
			break;
		}
	}
}

/** Set the speed of the right and left motors.
 * This is used once an appropriate drive setup function is called such as
 * TwoWheelDrive(). The motors are set to "leftOutput" and "rightOutput"
 * and includes flipping the direction of one side for opposing motors.
 * @param output The speed to send to the motors.
 */
void IRCANRobotDrive::SetOutputMotors(float output)
{
	m_rearLeftMotor.Set(INVERT(output, m_invertedRearLeftMotor));
	m_rearRightMotor.Set(INVERT(output, m_invertedRearRightMotor));
	m_frontLeftMotor.Set(INVERT(output, m_invertedFrontLeftMotor));
	m_frontRightMotor.Set(INVERT(output, m_invertedFrontRightMotor));
}

/** Set the speed of the right and left motors.
 * This is used once an appropriate drive setup function is called such as
 * TwoWheelDrive(). The motors are set to "leftOutput" and "rightOutput"
 * and includes flipping the direction of one side for opposing motors.
 * @param leftOutput The speed to send to the left side of the robot.
 * @param rightOutput The speed to send to the right side of the robot.
 */
void IRCANRobotDrive::SetOutputMotors(float leftOutput, float rightOutput)
{
	m_rearLeftMotor.Set(INVERT(leftOutput, m_invertedRearLeftMotor));
	m_rearRightMotor.Set(INVERT(rightOutput, m_invertedRearRightMotor));
	m_frontLeftMotor.Set(INVERT(leftOutput, m_invertedFrontLeftMotor));
	m_frontRightMotor.Set(INVERT(rightOutput, m_invertedFrontRightMotor));
}

/** Set the speed of the right and left motors.
 * The motors are set to their corresponding outputs.
 * and includes flipping the direction of one side for opposing motors.
 * @param frontLeftOutput The speed to send to the front left side of the robot.
 * @param rearLeftOutput The speed to send to the rear left side of the robot.
 * @param frontRightOutput The speed to send to the front right side of the robot.
 * @param rearRightOutput The speed to send to the rear right side of the robot.
 */
void IRCANRobotDrive::SetOutputMotors(float frontLeftOutput,float rearLeftOutput, float frontRightOutput, float rearRightOutput){
	m_rearLeftMotor.Set(INVERT(rearLeftOutput, m_invertedRearLeftMotor));
	m_rearRightMotor.Set(INVERT(rearRightOutput, m_invertedRearRightMotor));
	m_frontLeftMotor.Set(INVERT(frontLeftOutput, m_invertedFrontLeftMotor));
	m_frontRightMotor.Set(INVERT(frontRightOutput, m_invertedFrontRightMotor));
}

/**
 * Limit motor values to the -1.0 to +1.0 range.
 */
double IRCANRobotDrive::Limit(double num) {
  if (num > 1.0) {
    return 1.0;
  }
  if (num < -1.0) {
    return -1.0;
  }
  return num;
}

/**
 * Normalize all wheel speeds if the magnitude of any wheel is greater than 1.0.
 */
void IRCANRobotDrive::Normalize(double* wheelSpeeds) {
  double maxMagnitude = std::fabs(wheelSpeeds[0]);
  int i;
  for (i = 1; i < m_numMotors; i++) {
    double temp = std::fabs(wheelSpeeds[i]);
    if (maxMagnitude < temp) maxMagnitude = temp;
  }
  if (maxMagnitude > 1.0) {
    for (i = 0; i < m_numMotors; i++) {
      wheelSpeeds[i] = wheelSpeeds[i] / maxMagnitude;
    }
  }
}

/**
 * Rotate a vector in Cartesian space.
 */
void IRCANRobotDrive::RotateVector(double& x, double& y, double angle) {
  double cosA = std::cos(angle * (M_PI / 180.0));
  double sinA = std::sin(angle * (M_PI / 180.0));
  double xOut = x * cosA - y * sinA;
  double yOut = x * sinA + y * cosA;
  x = xOut;
  y = yOut;
}

