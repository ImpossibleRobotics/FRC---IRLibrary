/*
 * IRJoystick.cpp
 *
 *  Created on: 14 jan. 2016
 *      Author: Dylan Vos
 */

#include "IRJoystick.h"

#include "WPILIB.h"


/**
 * Construct an instance of a joystick.
 * The joystick index is the usb port on the drivers station.
 *
 * @param port The port on the driver station that the joystick is plugged into
 * (0-5).
 */
IRJoystick::IRJoystick(uint32_t port) :
	joystick(port)
{}

/**
 * Get the X value of the current joystick.
 * This depends on the mapping of the joystick connected to the current port.
 */
float IRJoystick::GetX()
{
	return joystick.GetX();
}

/**
 * Get the Y value of the current joystick.
 * This depends on the mapping of the joystick connected to the current port.
 */
float IRJoystick::GetY()
{
	return joystick.GetY();
}

/**
 * Get the Z value of the current joystick.
 * This depends on the mapping of the joystick connected to the current port.
 */
float IRJoystick::GetZ()
{
	return joystick.GetZ();
}

/**
 * Get the X value of the current joystick.
 * This depends on the mapping of the joystick connected to the current port.
 */
float IRJoystick::GetXDeadZoned()
{
	return (LevelOut(joystick.GetX()) > kAxisDeadZone) ? joystick.GetX() : 0.0;
}

/**
 * Get the Y value of the current joystick.
 * This depends on the mapping of the joystick connected to the current port.
 */
float IRJoystick::GetYDeadZoned()
{
	return (LevelOut(joystick.GetY()) > kAxisDeadZone) ? joystick.GetY() : 0.0;
}

/**
 * Get the Z value of the current joystick.
 * This depends on the mapping of the joystick connected to the current port.
 */
float IRJoystick::GetZDeadZoned()
{
	return (LevelOut(joystick.GetZ()) > kAxisDeadZone) ? joystick.GetZ() : 0.0;
}

float IRJoystick::GetRawAxis(uint32_t axis)
{
	return joystick.GetRawAxis(axis);
}

/**
 * Get the twist value of the current joystick.
 * This depends on the mapping of the joystick connected to the current port.
 */
float IRJoystick::GetTwist()
{
	return joystick.GetTwist();
}

/**
 * Get the throttle value of the current joystick.
 * This depends on the mapping of the joystick connected to the current port.
 */
float IRJoystick::GetThrottle()
{
	return joystick.GetThrottle();
}

/**
 * Get the throttle value of the current joystick.
 * This value is level to get a value back between 0 and 1.
 * This depends on the mapping of the joystick connected to the current port.
 */
float IRJoystick::GetLeveledThrottle()
{
	return LevelOut(joystick.GetThrottle());
}

/**
 * Get the value of the axis.
 *
 * @param axis The axis to read, starting at 0.
 * @return The value of the axis.
 */
bool IRJoystick::GetRawButton(uint32_t button)
{
	return joystick.GetRawButton(button);
}

/**
 * Get the trigger value of the current joystick.
 * This depends on the mapping of the joystick connected to the current port.
 */
bool IRJoystick::GetTrigger()
{
	return joystick.GetTrigger();
}

bool IRJoystick::GetTriggerRight()
{
	if(GetRawAxis(3) > 0.6)
	return true;
	return false;
}

bool IRJoystick::GetTriggerLeft()
{
	if(GetRawAxis(2) > 0.6)
	return true;
	return false;
}

/**
 * Get the top value of the current joystick.
 * This depends on the mapping of the joystick connected to the current port.
 */
bool IRJoystick::GetTop()
{
	return joystick.GetTop();
}

/**
 * Get the bumper value of the current joystick.
 * This depends on the mapping of the joystick connected to the current port.
 */
bool IRJoystick::GetBumper()
{
	return joystick.GetBumper();
}

/**
 * Get the POV value of the current joystick.
 * This depends on the mapping of the joystick connected to the current port.
 */
int IRJoystick::GetPOV()
{
	return joystick.GetPOV();
}

float IRJoystick::LevelOut(float value)
{
	return (value - 1) / -2;
}
