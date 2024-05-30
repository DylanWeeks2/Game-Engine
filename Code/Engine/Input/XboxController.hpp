#pragma once
#include "AnalogJoystick.hpp"
#include "KeyButtonState.hpp"

//-----------------------------------------------------------------------------------------------
enum XboxButtonID
{
	A_BUTTON,
	B_BUTTON,
	X_BUTTON,
	Y_BUTTON,
	RIGHT_THUMB,
	LEFT_THUMB,
	RIGHT_BUMPER,
	LEFT_BUMPER,
	DPAD_UP,
	DPAD_RIGHT,
	DPAD_DOWN,
	DPAD_LEFT,
	START,
	BACK,
	NUM = 14
};

//-----------------------------------------------------------------------------------------------
class XboxController
{
	friend class			InputSystem;

public:
	XboxController();
	~XboxController();
	bool					IsConnected() const;
	int						GetControllerID() const;
	AnalogJoystick const&	GetLeftStick() const;
	AnalogJoystick const&	GetRightStick() const;
	float					GetLeftTrigger() const;
	float					GetRightTrigger() const;
	float					GetLeftTriggerLastFrame() const;
	float					GetRightTriggerLastFrame() const;
	KeyButtonState const&	GetButton(XboxButtonID buttonID) const;
	bool					IsButtonDown(XboxButtonID buttonID) const;
	bool					WasButtonJustPressed(XboxButtonID buttonID) const;
	bool					WasButtonJustReleased(XboxButtonID buttonID) const;

private:
	void					Update();
	void					Reset();
	void					UpdateJoyStick(AnalogJoystick& out_joystick, short rawX, short rawY);
	void					UpdateTrigger(float& out_triggerValue, unsigned char rawValue);
	void					UpdateButton(XboxButtonID buttonID, unsigned short buttonFlags, unsigned short buttonFlag);

private:
	int						m_id = -1;
	float					m_leftTrigger = 0.f;
	float					m_rightTrigger = 0.f;
	AnalogJoystick			m_leftStick;
	AnalogJoystick			m_rightStick;
	float					m_leftTriggerLastFrame = 0.f;
	float					m_rightTriggerLastFrame = 0.f;
	KeyButtonState			m_buttons[(int)XboxButtonID::NUM];
	bool					m_isConnected = false;
};