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

		float GetY();
		float GetX();
		float GetZ();

		float GetYDeadZoned();
		float GetXDeadZoned();
		float GetZDeadZoned();

		float GetTwist();
		float GetRawAxis(uint32_t axis);

		float GetThrottle();
		float GetLeveledThrottle();

		bool GetTrigger();
		bool GetTriggerRight();
		bool GetTriggerLeft();

		bool GetTop();

		bool GetRawButton(uint32_t button);

		int GetPOV();

		float LevelOut(float value);

	protected:
		frc::Joystick joystick;

		static constexpr float kTriggerDeadZone = 0.6;
		static constexpr float kAxisDeadZone = 0.001;
	};
}
