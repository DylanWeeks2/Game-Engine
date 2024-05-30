#define WIN32_LEAN_AND_MEAN
#include <Windows.h> // must #include Windows.h before #including Xinput.h
#include <Xinput.h> // include the Xinput API header file (interface)
#include "XboxController.hpp"
#pragma comment( lib, "xinput9_1_0" ) // Link in the xinput.lib static library // #Eiserloh: Xinput 1_4 doesn't work in Windows 7; use version 9_1_0 explicitly for broadest compatibility
#include "Engine/Core/ErrorWarningAssert.hpp"
#include "Engine/Math/MathUtils.hpp"

//-----------------------------------------------------------------------------------------------
XboxController::XboxController()
{
	//sets inner and outer thresholds for both sticks
	float innerThreshold = RangeMap(9800.f, 0.f, 32767.f, 0.f, 1.f);
	float outerThreshold = RangeMap(31000.f, 0.f, 32767.f, 0.f, 1.f);
	m_leftStick.SetDeadZoneThresholds(innerThreshold, outerThreshold);
	m_rightStick.SetDeadZoneThresholds(innerThreshold, outerThreshold);
}

//-----------------------------------------------------------------------------------------------
XboxController::~XboxController()
{
}

//-----------------------------------------------------------------------------------------------
bool XboxController::IsConnected() const
{	
	if (m_isConnected == true)
	{
		return true;
	}
	else
	{
		return false;
	}
}

//-----------------------------------------------------------------------------------------------
int XboxController::GetControllerID() const
{
	return m_id;
}

//-----------------------------------------------------------------------------------------------
AnalogJoystick const& XboxController::GetLeftStick() const
{
	return m_leftStick;
}

//-----------------------------------------------------------------------------------------------
AnalogJoystick const& XboxController::GetRightStick() const
{
	return m_rightStick;
}

//-----------------------------------------------------------------------------------------------
float XboxController::GetLeftTrigger() const
{
	return m_leftTrigger;
}

//-----------------------------------------------------------------------------------------------
float XboxController::GetRightTrigger() const
{
	return m_rightTrigger;
}

//-----------------------------------------------------------------------------------------------
float XboxController::GetLeftTriggerLastFrame() const
{
	return m_leftTriggerLastFrame;
}

//-----------------------------------------------------------------------------------------------
float XboxController::GetRightTriggerLastFrame() const
{
	return m_rightTriggerLastFrame;
}

//-----------------------------------------------------------------------------------------------
KeyButtonState const& XboxController::GetButton(XboxButtonID buttonID) const
{
	return m_buttons[buttonID];
}

//-----------------------------------------------------------------------------------------------
bool XboxController::IsButtonDown(XboxButtonID buttonID) const
{
	return m_buttons[buttonID].m_isPressed;
}

//-----------------------------------------------------------------------------------------------
bool XboxController::WasButtonJustPressed(XboxButtonID buttonID) const
{
	return m_buttons[buttonID].m_wasPressedLastFrame;
}

//-----------------------------------------------------------------------------------------------
bool XboxController::WasButtonJustReleased(XboxButtonID buttonID) const
{
	if (IsButtonDown(buttonID) == false && WasButtonJustPressed(buttonID) == true)
	{
		return true;
	}
	else
	{
		return false;
	}
}

//-----------------------------------------------------------------------------------------------
void XboxController::Update()
{
	XINPUT_STATE xboxControllerState = {};
	DWORD errorStatus = XInputGetState(m_id, &xboxControllerState);

	if (errorStatus == ERROR_SUCCESS)
	{
		//once connected updates all buttons, triggers, and sticks
		m_isConnected = true;

		UpdateJoyStick(m_leftStick, xboxControllerState.Gamepad.sThumbLX, xboxControllerState.Gamepad.sThumbLY);
		UpdateJoyStick(m_rightStick, xboxControllerState.Gamepad.sThumbRX, xboxControllerState.Gamepad.sThumbRY);

		UpdateTrigger(m_leftTrigger, xboxControllerState.Gamepad.bLeftTrigger);
		UpdateTrigger(m_rightTrigger, xboxControllerState.Gamepad.bRightTrigger);

		UpdateButton(A_BUTTON, xboxControllerState.Gamepad.wButtons, XINPUT_GAMEPAD_A);
		UpdateButton(B_BUTTON, xboxControllerState.Gamepad.wButtons, XINPUT_GAMEPAD_B);
		UpdateButton(X_BUTTON, xboxControllerState.Gamepad.wButtons, XINPUT_GAMEPAD_X);
		UpdateButton(Y_BUTTON, xboxControllerState.Gamepad.wButtons, XINPUT_GAMEPAD_Y);
		UpdateButton(RIGHT_THUMB, xboxControllerState.Gamepad.wButtons, XINPUT_GAMEPAD_RIGHT_THUMB);
		UpdateButton(LEFT_THUMB, xboxControllerState.Gamepad.wButtons, XINPUT_GAMEPAD_LEFT_THUMB);
		UpdateButton(RIGHT_BUMPER, xboxControllerState.Gamepad.wButtons, XINPUT_GAMEPAD_RIGHT_SHOULDER);
		UpdateButton(LEFT_BUMPER, xboxControllerState.Gamepad.wButtons, XINPUT_GAMEPAD_LEFT_SHOULDER);
		UpdateButton(DPAD_UP, xboxControllerState.Gamepad.wButtons, XINPUT_GAMEPAD_DPAD_UP);
		UpdateButton(DPAD_RIGHT, xboxControllerState.Gamepad.wButtons, XINPUT_GAMEPAD_DPAD_RIGHT);
		UpdateButton(DPAD_DOWN, xboxControllerState.Gamepad.wButtons, XINPUT_GAMEPAD_DPAD_DOWN);
		UpdateButton(DPAD_LEFT, xboxControllerState.Gamepad.wButtons, XINPUT_GAMEPAD_DPAD_LEFT);
		UpdateButton(START, xboxControllerState.Gamepad.wButtons, XINPUT_GAMEPAD_START);
		UpdateButton(BACK, xboxControllerState.Gamepad.wButtons, XINPUT_GAMEPAD_BACK);
	}
	else if (errorStatus == ERROR_DEVICE_NOT_CONNECTED)
	{
		m_isConnected = false;
		Reset();
	}
}

//-----------------------------------------------------------------------------------------------
void XboxController::Reset()
{
}

//-----------------------------------------------------------------------------------------------
void XboxController::UpdateJoyStick(AnalogJoystick& out_joystick, short rawX, short rawY)
{
	//normalizes raw values and sends to stick
	Vec2 rawNormal(static_cast<float>(rawX) / 32768.f, static_cast<float>(rawY) / 32768.f);
	out_joystick.UpdatePosition(rawNormal.x, rawNormal.y);
}

//-----------------------------------------------------------------------------------------------
void XboxController::UpdateTrigger(float& out_triggerValue, unsigned char rawValue)
{
	//rangemaps value to 0 to 1
	out_triggerValue = RangeMap(rawValue, 0.f, 255.f, 0.f, 1.f);
}

//-----------------------------------------------------------------------------------------------
void XboxController::UpdateButton(XboxButtonID buttonID, unsigned short buttonFlags, unsigned short buttonFlag)
{
	//checks if button is down and changes ispressed accordingly
	bool isButtonDown = (buttonFlags & buttonFlag) == buttonFlag;
	if (isButtonDown == true)
	{
		m_buttons[buttonID].m_isPressed = true;
	}
	else
	{
		m_buttons[buttonID].m_isPressed = false;
	}
}