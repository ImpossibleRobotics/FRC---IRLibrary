/*
 * IRRobotDrive.h
 *
 *  Created on: 12 jan. 2016
 *      Author: Dylan Vos
 */

#pragma once

#include <GenericHID.h>
#include <Talon.h>

#include <../IRLibrary/HumanInterfaceDevices/IRJoystick.h>

namespace IR {

	class IRRobotDrive{
	public:
		IRRobotDrive(uint32_t frontLeftMotorChannel, uint32_t rearLeftMotorChannel,
					 uint32_t frontRightMotorChannel, uint32_t rearRightMotorChannel);

		void SetMotorsInverted(bool frontLeft, bool rearLeft, bool frontRight, bool rearRight);
		void SetMotorsInverted(bool inverted);

		void SetOutputMotors(float output);
		void SetOutputMotors(float leftOutput, float rightOutput);

		void ArcadeDrive(frc::GenericHID *stick);
		void ArcadeDrive(IR::IRJoystick *stick);
		void ArcadeDrive(frc::GenericHID &stick);
		void ArcadeDrive(IR::IRJoystick &stick);
		void ArcadeDrive(IR::IRJoystick *stick, bool deadZoned);
		void ArcadeDrive(IR::IRJoystick &stick, bool deadZoned);
		void ArcadeDrive(float moveValue, float rotateValue);
		void ArcadeDrive(float moveValue, float rotateValue, float modifierValue);

		void Drive(float outputeMagnitude, float curve);

	protected:

		frc::Talon m_frontLeftMotor;
		frc::Talon m_frontRightMotor;
		frc::Talon m_rearLeftMotor;
		frc::Talon m_rearRightMotor;

		float m_sensitivity = 0.5;
	};
}
