/*
 * IRRobotDrive.h
 *
 *  Created on: 12 jan. 2016
 *      Author: Dylan Vos
 */

#pragma once

#include <GenericHID.h>
#include <ctre/Phoenix.h>

#include "../HumanInterfaceDevices/IRJoystick.h"

namespace IR {

enum DriveTrain {
	Tank,
	Mecanum,
	Swerve
};

	class IRCANRobotDrive{
	public:
		IRCANRobotDrive(uint32_t frontLeftMotorChannel, uint32_t rearLeftMotorChannel,
							 uint32_t frontRightMotorChannel, uint32_t rearRightMotorChannel, DriveTrain driveTrain = Tank);

		void SetMotorsInverted(bool frontLeft, bool rearLeft, bool frontRight, bool rearRight);
		void SetMotorsInverted(bool inverted);

		void SetOutputMotors(float output);
		void SetOutputMotors(float leftOutput, float rightOutput);
		void SetOutputMotors(float frontLeftOutput,float rearLeftOutput, float frontRightOutput, float rearRightOutput);

		void ArcadeDrive(frc::GenericHID *stick, double gyro = 0.0);
		void ArcadeDrive(IR::IRJoystick *stick, double gyro = 0.0);
		void ArcadeDrive(frc::GenericHID &stick, double gyro = 0.0);
		void ArcadeDrive(IR::IRJoystick &stick, double gyro = 0.0);
		void ArcadeDrive(IR::IRJoystick *stick, bool deadZoned, double gyro = 0.0);
		void ArcadeDrive(IR::IRJoystick &stick, bool deadZoned, double gyro = 0.0);
		void ArcadeDrive(double x, double y, double z, double gyro = 0.0, double t = 1.0);

		void Drive(double outputeMagnitude, double curve);

		void Drive(double direction, double speed, double rotation, double gyro = 0.0); //Mainly meant for driving with Mecanum wheels

		double Limit(double num);
		void Normalize(double* wheelSpeeds);
		void RotateVector(double& x, double& y, double angle);

	protected:

		WPI_TalonSRX m_frontLeftMotor;
		WPI_TalonSRX m_rearLeftMotor;
		WPI_TalonSRX m_frontRightMotor;
		WPI_TalonSRX m_rearRightMotor;

		DriveTrain m_driveTrain;

		bool m_invertedFrontLeftMotor = false;
		bool m_invertedFrontRightMotor = false;
		bool m_invertedRearLeftMotor = false;
		bool m_invertedRearRightMotor = false;

		float m_sensitivity = 0.5;

		float m_numMotors = GetNumMotors();

	private:
	 int GetNumMotors() {
	   int motors = 4;
//	   if (m_frontLeftMotor != NULL) motors++;
//	   if (m_frontRightMotor) motors++;
//	   if (m_rearLeftMotor) motors++;
//	   if (m_rearRightMotor) motors++;
	   return motors;
	 }
};
}
