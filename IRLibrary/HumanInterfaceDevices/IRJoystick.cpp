/*
 * IRJoystick.cpp
 *
 *  Created on: 14 jan. 2016
 *      Author: Dylan Vos
 */

#include "IRJoystick.h"

#include <Joystick.h>

using namespace IR;

#define ABS(x) ((x < 0) ? -x : x)

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
double IRJoystick::GetX()
{
	return joystick.GetX();
}

/**
 * Get the Y value of the current joystick.
 * This depends on the mapping of the joystick connected to the current port.
 */
double IRJoystick::GetY()
{
	return -joystick.GetY();
}

/**
 * Get the Z value of the current joystick.
 * This depends on the mapping of the joystick connected to the current port.
 */
double IRJoystick::GetZ()
{
	return joystick.GetZ();
}

/**
 * Get the X2 value of the current joystick.
 * This depends on the mapping of the joystick connected to the current port.
 */
double IRJoystick::GetX2()
{
	return joystick.GetRawAxis(x2Axis);
}

/**
 * Get the Y2 value of the current joystick.
 * This depends on the mapping of the joystick connected to the current port.
 */
double IRJoystick::GetY2()
{
	return -joystick.GetRawAxis(y2Axis);
}

/**
 * Get the X value of the current joystick.
 * This depends on the mapping of the joystick connected to the current port.
 */
double IRJoystick::GetXDeadZoned()
{
	return (abs(GetX()) > kAxisDeadZone) ? GetX() : 0.0;
}

/**
 * Get the Y value of the current joystick.
 * This depends on the mapping of the joystick connected to the current port.
 */
double IRJoystick::GetYDeadZoned()
{
	return -((abs(GetY()) > kAxisDeadZone) ? GetY() : 0.0);
}

/**
 * Get the Z value of the current joystick.
 * This depends on the mapping of the joystick connected to the current port.
 */
double IRJoystick::GetZDeadZoned()
{
	return (abs(GetZ()) > kAxisDeadZone) ? GetZ() : 0.0;
}

/**
 * Get the X2 value of the current joystick.
 * This depends on the mapping of the joystick connected to the current port.
 */
double IRJoystick::GetX2DeadZoned()
{
	return (abs(GetX2()) > kAxisDeadZone) ? GetX2() : 0.0;
}

/**
 * Get the Y2 value of the current joystick.
 * This depends on the mapping of the joystick connected to the current port.
 */
double IRJoystick::GetY2DeadZoned()
{
	return -((abs(GetY2()) > kAxisDeadZone) ? GetY2(): 0.0);
}

double IRJoystick::GetRawAxis(uint32_t axis)
{
	return joystick.GetRawAxis(axis);
}

double IRJoystick::GetRawAxisDeadzoned(uint32_t axis)
{
	return (abs(GetRawAxis(axis)) > kAxisDeadZone) ? GetRawAxis(axis) : 0.0;
}

/**
 * Get the twist value of the current joystick.
 * This depends on the mapping of the joystick connected to the current port.
 */
double IRJoystick::GetTwist()
{
	return joystick.GetTwist();
}

/**
 * Get the throttle value of the current joystick.
 * This depends on the mapping of the joystick connected to the current port.
 */
double IRJoystick::GetThrottle()
{
	return joystick.GetThrottle();
}

/**
 * Get the throttle value of the current joystick.
 * This value is level to get a value back between 0 and 1.
 * This depends on the mapping of the joystick connected to the current port.
 */
double IRJoystick::GetLeveledThrottle()
{
	return LevelOut(GetThrottle());
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
 * Get the POV value of the current joystick.
 * This depends on the mapping of the joystick connected to the current port.
 */
int IRJoystick::GetPOV()
{
	return joystick.GetPOV();
}

double IRJoystick::LevelOut(double value)
{
	return (value - 1) / -2;
}

double IRJoystick::abs(double value){
	return ABS(value);
}
