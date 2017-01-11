/*
 * IRRobotDrive.h
 *
 *  Created on: 12 jan. 2016
 *      Author: Dylan Vos
 */

#ifndef IRLIBRARY_IRROBOTDRIVE_H_
#define IRLIBRARY_IRROBOTDRIVE_H_

#include "WPILIB.h"

#include "../HumanInterfaceDevices/IRJoystick.h"

class IRRobotDrive{
public:
	IRRobotDrive(uint32_t frontLeftMotorChannel, uint32_t rearLeftMotorChannel,
	             uint32_t frontRightMotorChannel, uint32_t rearRightMotorChannel);

	void SetMotorsInverted(bool frontLeft, bool rearLeft, bool frontRight, bool rearRight);
	void SetMotorsInverted(bool inverted);

	void SetOutputMotors(float output);
	void SetOutputMotors(float leftOutput, float rightOutput);

	void ArcadeDrive(GenericHID *stick);
	void ArcadeDrive(IRJoystick *stick);
	void ArcadeDrive(GenericHID &stick);
	void ArcadeDrive(IRJoystick &stick);
	void ArcadeDrive(IRJoystick *stick, bool deadZoned);
	void ArcadeDrive(IRJoystick &stick, bool deadZoned);
	void ArcadeDrive(float moveValue, float rotateValue);
	void ArcadeDrive(float moveValue, float rotateValue, float modifierValue);

	void Drive(float outputeMagnitude, float curve);

protected:

	Talon m_frontLeftMotor;
	Talon m_frontRightMotor;
	Talon m_rearLeftMotor;
	Talon m_rearRightMotor;

	uint8_t m_syncGroup = 0;

	float m_sensitivity = 0.5;
};

#endif /* IRLIBRARY_IRROBOTDRIVE_H_ */
