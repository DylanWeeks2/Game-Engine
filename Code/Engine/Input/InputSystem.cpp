#include "InputSystem.hpp"
#include "KeyButtonState.hpp"
#include "Engine/Core/ErrorWarningAssert.hpp"
#include "Engine/Core/EventSystem.hpp"
#include "Engine/Core/NamedStrings.hpp"
#include "Engine/Window/Window.hpp"
#include "Engine/Math/AABB2.hpp"
#include <windows.h>

const unsigned char KEYCODE_F1 = VK_F1;
const unsigned char KEYCODE_F2 = VK_F2; 
const unsigned char KEYCODE_F3 = VK_F3;
const unsigned char KEYCODE_F4 = VK_F4;
const unsigned char KEYCODE_F5 = VK_F5;
const unsigned char KEYCODE_F6 = VK_F6;
const unsigned char KEYCODE_F7 = VK_F7;
const unsigned char KEYCODE_F8 = VK_F8;
const unsigned char KEYCODE_F9 = VK_F9;
const unsigned char KEYCODE_F10 = VK_F10;
const unsigned char KEYCODE_F11 = VK_F11;
const unsigned char KEYCODE_F12 = VK_F12;
const unsigned char KEYCODE_ESC = VK_ESCAPE;
const unsigned char KEYCODE_SPACE = VK_SPACE;
const unsigned char KEYCODE_ENTER = VK_RETURN;
const unsigned char KEYCODE_UPARROW = VK_UP;
const unsigned char KEYCODE_DOWNARROW = VK_DOWN;
const unsigned char KEYCODE_LEFTARROW = VK_LEFT;
const unsigned char KEYCODE_RIGHTARROW = VK_RIGHT;
const unsigned char KEYCODE_NUMPAD_0 = VK_NUMPAD0;
const unsigned char KEYCODE_NUMPAD_1 = VK_NUMPAD1;
const unsigned char KEYCODE_NUMPAD_2 = VK_NUMPAD2;
const unsigned char KEYCODE_NUMPAD_3 = VK_NUMPAD3;
const unsigned char KEYCODE_NUMPAD_4 = VK_NUMPAD4;
const unsigned char KEYCODE_NUMPAD_5 = VK_NUMPAD5;
const unsigned char KEYCODE_NUMPAD_6 = VK_NUMPAD6;
const unsigned char KEYCODE_NUMPAD_7 = VK_NUMPAD7;
const unsigned char KEYCODE_NUMPAD_8 = VK_NUMPAD8;
const unsigned char KEYCODE_NUMPAD_9 = VK_NUMPAD9;
const unsigned char KEYCODE_LEFT_MOUSE = VK_LBUTTON;
const unsigned char KEYCODE_RIGHT_MOUSE = VK_RBUTTON;
const unsigned char KEYCODE_TILDE = 0xC0;
const unsigned char KEYCODE_LEFTBRACKET = 0xDB;
const unsigned char KEYCODE_RIGHTBRACKET = 0xDD;
const unsigned char KEYCODE_LEFTSQUAREBRACKET = VK_OEM_4;
const unsigned char KEYCODE_RIGHTSQUAREBRACKET = VK_OEM_6;
const unsigned char KEYCODE_BACKSPACE = VK_BACK;
const unsigned char KEYCODE_INSERT = VK_INSERT;
const unsigned char KEYCODE_DELETE = VK_DELETE;
const unsigned char KEYCODE_HOME = VK_HOME;
const unsigned char KEYCODE_END = VK_END;
const unsigned char KEYCODE_SHIFT = VK_SHIFT; 
const unsigned char KEYCODE_COMMA = VK_OEM_COMMA;
const unsigned char KEYCODE_PERIOD = VK_OEM_PERIOD;
const unsigned char KEYCODE_SEMICOLON = VK_OEM_1;
const unsigned char KEYCODE_SINGLEQUOTE = VK_OEM_7;
const unsigned char KEYCODE_PAGEUP = VK_PRIOR;
const unsigned char KEYCODE_PAGEDOWN = VK_NEXT;

//-----------------------------------------------------------------------------------------------
InputSystem::InputSystem(InputSystemConfig const& config)
	:m_config(config)
{
}

//-----------------------------------------------------------------------------------------------
InputSystem::~InputSystem()
{
}

//-----------------------------------------------------------------------------------------------
void InputSystem::Startup()
{
	g_theEventSystem->SubscribeEventCallbackFunciton("KeyPressed", InputSystem::Event_KeyPressed);
	g_theEventSystem->SubscribeEventCallbackFunciton("KeyReleased", InputSystem::Event_KeyReleased);

	//on startup set controller ids
	for (int xboxControllerCounter = 0; xboxControllerCounter < NUM_XBOX_CONTROLLERS; xboxControllerCounter++)
	{
		m_controllers[xboxControllerCounter].m_id = xboxControllerCounter;
	}
}

//-----------------------------------------------------------------------------------------------
void InputSystem::Shutdown()
{
}

//-----------------------------------------------------------------------------------------------
void InputSystem::BeginFrame()
{
	//update all controllers
	for (int xboxControllerCounter = 0; xboxControllerCounter < NUM_XBOX_CONTROLLERS; xboxControllerCounter++)
	{
		m_controllers[xboxControllerCounter].Update();
	}

	//Mouse Input Logic
	if (m_mouseState.m_cursorHidden != m_mouseState.m_desiredHidden)
	{
		m_mouseState.m_cursorHidden = m_mouseState.m_desiredHidden;
		ShowCursor(!m_mouseState.m_cursorHidden);
	}

	m_mouseState.m_cursorClientPosition = GetCursorClientPosition();

	if (m_mouseState.m_currentRelative != m_mouseState.m_desiredRelative)
	{
		m_mouseState.m_currentRelative = m_mouseState.m_desiredRelative;
		m_mouseState.m_cursorClientPosition = IntVec2(0, 0);
		m_mouseState.m_cursorClientDelta = IntVec2(0, 0);
	}

	if (m_mouseState.m_currentRelative == true)
	{
		m_mouseState.m_cursorClientDelta = GetCursorClientDelta();

		HWND hWnd = ::GetActiveWindow();
		RECT clientRect;
		GetClientRect(hWnd, &clientRect);
		POINT center;
		center.x = (clientRect.left + clientRect.right) / static_cast<LONG>(2);
		center.y = (clientRect.top + clientRect.bottom) / static_cast<LONG>(2);
		ClientToScreen(hWnd, &center);
		SetCursorPos(center.x, center.y);
	}

	m_mouseState.m_cursorClientPosition = GetCursorClientPosition();
}

//-----------------------------------------------------------------------------------------------
void InputSystem::EndFrame()
{
	//this loop sets the downlastFrame bool array to either true or false depending on the frame
	for (int boolKeyCounter = 0; boolKeyCounter < NUM_KEYCODES; boolKeyCounter++)
	{
		if (m_keyStates[boolKeyCounter].m_isPressed == true)
		{
			m_keyStates[boolKeyCounter].m_wasPressedLastFrame = true;
		}
		else if (m_keyStates[boolKeyCounter].m_isPressed == false && m_keyStates[boolKeyCounter].m_wasPressedLastFrame == true)
		{
			m_keyStates[boolKeyCounter].m_wasPressedLastFrame = false;
		}
	}

	//sets press bool values for each xbox button and triggers
	for (int xboxControllerCounter = 0; xboxControllerCounter < NUM_XBOX_CONTROLLERS; xboxControllerCounter++)
	{
		for (int buttonCounter = 0; buttonCounter < XboxButtonID::NUM; buttonCounter++)
		{
			if (m_controllers[xboxControllerCounter].m_buttons[buttonCounter].m_isPressed == true)
			{
				m_controllers[xboxControllerCounter].m_buttons[buttonCounter].m_wasPressedLastFrame = true;
			}
			else if (m_controllers[xboxControllerCounter].m_buttons[buttonCounter].m_isPressed == false && m_controllers[xboxControllerCounter].m_buttons[buttonCounter].m_wasPressedLastFrame == true)
			{
				m_controllers[xboxControllerCounter].m_buttons[buttonCounter].m_wasPressedLastFrame = false;
			}
		}
		
		m_controllers[xboxControllerCounter].m_rightTriggerLastFrame = m_controllers[xboxControllerCounter].m_rightTrigger;
		m_controllers[xboxControllerCounter].m_leftTriggerLastFrame = m_controllers[xboxControllerCounter].m_leftTrigger;
	}

	m_mouseState.m_cursorClientPosition = GetCursorClientPosition();
	m_scrollWheelDelta = 0;
}

//-----------------------------------------------------------------------------------------------
bool InputSystem::WasKeyJustPressed(unsigned char keyCode)
{
	return m_keyStates[keyCode].m_wasPressedLastFrame;
}

//-----------------------------------------------------------------------------------------------
bool InputSystem::WasKeyJustReleased(unsigned char keyCode)
{
	if (IsKeyDown(keyCode) == false && WasKeyJustPressed(keyCode) == true)
	{
		return true;
	}
	else
	{
		return false;
	}
}

//-----------------------------------------------------------------------------------------------
bool InputSystem::IsKeyDown(unsigned char keyCode)
{
	return m_keyStates[keyCode].m_isPressed;
}

//-----------------------------------------------------------------------------------------------
bool InputSystem::HandleKeyPressed(unsigned char keyCode)
{
	m_keyStates[keyCode].m_isPressed = true;
	return false;
}

//-----------------------------------------------------------------------------------------------
bool InputSystem::HandleKeyReleased(unsigned char keyCode)
{
	m_keyStates[keyCode].m_isPressed = false;

	return false;
}

//-----------------------------------------------------------------------------------------------
XboxController const& InputSystem::GetController(int controllerID)
{
	return m_controllers[controllerID];
}

//-----------------------------------------------------------------------------------------------
InputSystemConfig const& InputSystem::GetConfig() const
{
	return m_config;
}

//-----------------------------------------------------------------------------------------------
bool InputSystem::Event_KeyPressed(EventArgs& args)
{
	if (!g_theInput)
	{
		return false;
	}
	unsigned char keyCode = static_cast<unsigned char>(args.GetValue("KeyCode", -1));
	g_theInput->HandleKeyPressed(keyCode);
	return true;
}

//-----------------------------------------------------------------------------------------------
bool InputSystem::Event_KeyReleased(EventArgs& args)
{
	if (!g_theInput)
	{
		return false;
	}
	unsigned char keyCode = static_cast<unsigned char>(args.GetValue("KeyCode", -1));
	g_theInput->HandleKeyReleased(keyCode);
	return true;
}

//-----------------------------------------------------------------------------------------------
void InputSystem::SetCursorMode(bool hidden, bool relative)
{
	m_mouseState.m_desiredHidden = hidden;
	m_mouseState.m_desiredRelative = relative;

	if (relative == true)
	{
		HWND hWnd = ::GetActiveWindow();
		RECT clientRect;
		GetClientRect(hWnd, &clientRect);
		POINT center;
		center.x = (clientRect.left + clientRect.right) / static_cast<LONG>(2);
		center.y = (clientRect.top + clientRect.bottom) / static_cast<LONG>(2);
		ClientToScreen(hWnd, &center);
		SetCursorPos(center.x, center.y);
	}
}

//-----------------------------------------------------------------------------------------------
IntVec2 InputSystem::GetCursorClientDelta() const
{
	HWND hWnd = ::GetActiveWindow();
	RECT clientRect;
	GetClientRect(hWnd, &clientRect);
	POINT center;
	center.x = clientRect.left + clientRect.right / 2;
	center.y = clientRect.top + clientRect.bottom / 2;
	IntVec2 delta = IntVec2(GetCursorClientPosition().x - center.x, GetCursorClientPosition().y - center.y);
	return delta;
}

//-----------------------------------------------------------------------------------------------
IntVec2 InputSystem::GetCursorClientPosition() const
{
	HWND windowHandle = ::GetActiveWindow();
	POINT cursorPos;
	::GetCursorPos(&cursorPos); 
	::ScreenToClient(windowHandle, &cursorPos);

	return IntVec2(cursorPos.x, cursorPos.y);
}

//-----------------------------------------------------------------------------------------------
Vec2 InputSystem::GetCursorNormalizedPosition() const
{
	HWND windowHandle = ::GetActiveWindow(); //Gets open window
	POINT cursorPos;
	RECT clientRectangle;
	::GetCursorPos(&cursorPos); //gets cursor position on whole computer screen
	::ScreenToClient(windowHandle, &cursorPos); //cursor position for window
	::GetClientRect(windowHandle, &clientRectangle);

	float cursorX = static_cast<float>(cursorPos.x) / static_cast<float>(clientRectangle.right);
	float cursorY = static_cast<float>(cursorPos.y) / static_cast<float>(clientRectangle.bottom);

	return Vec2(cursorX, 1.0f - cursorY);
}

//-----------------------------------------------------------------------------------------------
AABB2 InputSystem::GetClientDimensions() const
{
	HWND hWnd = ::GetActiveWindow();
	RECT clientRect;
	GetClientRect(hWnd, &clientRect);
	AABB2 dimensions;
	dimensions.m_mins = Vec2(static_cast<float>(clientRect.left), static_cast<float>(clientRect.bottom));
	dimensions.m_mins = Vec2(static_cast<float>(clientRect.right), static_cast<float>(clientRect.top));
	return dimensions;
}

//-----------------------------------------------------------------------------------------------
bool InputSystem::isMouseBeingScrolled()
{
	if (m_scrollWheelDelta != 0)
	{
		return true;
	}

	return false;
}
