/*
 * IRJoystick.h
 *
 *  Created on: 14 jan. 2016
 *      Author: Dylan Vos
 */

#pragma once

#include <Joystick.h>

namespace IR {

	class IRJoystick
	{
	public:
		static const uint32_t kDefaultThrottleAxis = 3;


		IRJoystick(uint32_t port);

		double GetY();
		double GetX();
		double GetZ();

		double GetY2();
		double GetX2();

		double GetYDeadZoned();
		double GetXDeadZoned();
		double GetZDeadZoned();

		double GetY2DeadZoned();
		double GetX2DeadZoned();

		double GetTwist();
		double GetRawAxis(uint32_t axis);

		double GetRawAxisDeadzoned(uint32_t axis);

		double GetThrottle();
		double GetLeveledThrottle();

		bool GetTrigger();
		bool GetTriggerRight();
		bool GetTriggerLeft();

		bool GetTop();

		bool GetRawButton(uint32_t button);

		int GetPOV();

		double LevelOut(double value);
		double abs(double value);

	protected:
		frc::Joystick joystick;

		static constexpr float kTriggerDeadZone = 0.6;
		static constexpr float kAxisDeadZone = 0.20;
		static constexpr uint32_t x2Axis = 4;
		static constexpr uint32_t y2Axis = 5;
	};
}
