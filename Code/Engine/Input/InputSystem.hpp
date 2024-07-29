#pragma once
#include "XboxController.hpp"
#include "Engine/Math/IntVec2.hpp"
#include "Engine/Core/EventSystem.hpp"

//-----------------------------------------------------------------------------------------------
extern unsigned char const KEYCODE_F1;
extern unsigned char const KEYCODE_F2;
extern unsigned char const KEYCODE_F3;
extern unsigned char const KEYCODE_F4;
extern unsigned char const KEYCODE_F5;
extern unsigned char const KEYCODE_F6;
extern unsigned char const KEYCODE_F7;
extern unsigned char const KEYCODE_F8;
extern unsigned char const KEYCODE_F9;
extern unsigned char const KEYCODE_F10;
extern unsigned char const KEYCODE_F11;
extern unsigned char const KEYCODE_F12;
extern unsigned char const KEYCODE_ESC;
extern unsigned char const KEYCODE_UPARROW;
extern unsigned char const KEYCODE_DOWNARROW;
extern unsigned char const KEYCODE_LEFTARROW;
extern unsigned char const KEYCODE_RIGHTARROW;
extern unsigned char const KEYCODE_NUMPAD_0;
extern unsigned char const KEYCODE_NUMPAD_1;
extern unsigned char const KEYCODE_NUMPAD_2;
extern unsigned char const KEYCODE_NUMPAD_3;
extern unsigned char const KEYCODE_NUMPAD_4;
extern unsigned char const KEYCODE_NUMPAD_5;
extern unsigned char const KEYCODE_NUMPAD_6;
extern unsigned char const KEYCODE_NUMPAD_7;
extern unsigned char const KEYCODE_NUMPAD_8;
extern unsigned char const KEYCODE_NUMPAD_9;
extern unsigned char const KEYCODE_LEFT_MOUSE;
extern unsigned char const KEYCODE_RIGHT_MOUSE;
extern unsigned char const KEYCODE_TILDE;
extern unsigned char const KEYCODE_LEFTBRACKET;
extern unsigned char const KEYCODE_RIGHTBRACKET;
extern unsigned char const KEYCODE_LEFTSQUAREBRACKET;
extern unsigned char const KEYCODE_RIGHTSQUAREBRACKET;
extern unsigned char const KEYCODE_ENTER;
extern unsigned char const KEYCODE_BACKSPACE;
extern unsigned char const KEYCODE_INSERT;
extern unsigned char const KEYCODE_DELETE;
extern unsigned char const KEYCODE_HOME;
extern unsigned char const KEYCODE_END;
extern unsigned char const KEYCODE_SHIFT;
extern unsigned char const KEYCODE_COMMA;
extern unsigned char const KEYCODE_PERIOD;
extern unsigned char const KEYCODE_SEMICOLON;
extern unsigned char const KEYCODE_SINGLEQUOTE;
extern unsigned char const KEYCODE_PAGEUP;
extern unsigned char const KEYCODE_PAGEDOWN;

//-----------------------------------------------------------------------------------------------
constexpr int				NUM_KEYCODES = 256;
constexpr int				NUM_XBOX_CONTROLLERS = 4;

//-----------------------------------------------------------------------------------------------
struct AABB2;

//-----------------------------------------------------------------------------------------------
struct MouseState
{
	IntVec2 m_cursorClientPosition;
	IntVec2 m_cursorClientDelta;

	bool	m_cursorHidden = false;
	bool	m_desiredHidden = false;

	bool	m_currentRelative = false;
	bool	m_desiredRelative = false;
};

//-----------------------------------------------------------------------------------------------
struct InputSystemConfig
{
};

//-----------------------------------------------------------------------------------------------
class InputSystem
{
public:
	InputSystem(InputSystemConfig const& config);
	~InputSystem();
	void						Startup();
	void						Shutdown();
	void						BeginFrame();
	void						EndFrame();
	bool						WasKeyJustPressed(unsigned char keyCode);
	bool						WasKeyJustReleased(unsigned char keyCode);
	bool						IsKeyDown(unsigned char keyCode);
	bool						HandleKeyPressed(unsigned char keyCode);
	bool						HandleKeyReleased(unsigned char keyCode);
	XboxController const&		GetController(int controllerID);
	InputSystemConfig const&	GetConfig() const;
	static bool					Event_KeyPressed(EventArgs& args);
	static bool					Event_KeyReleased(EventArgs& args);
	void						SetCursorMode(bool hidden, bool relative);
	IntVec2						GetCursorClientDelta() const;
	IntVec2						GetCursorClientPosition() const;
	Vec2						GetCursorNormalizedPosition() const;
	AABB2						GetClientDimensions() const;
	bool						isMouseBeingScrolled();

public: 
	MouseState					m_mouseState;
	int							m_scrollWheelDelta = 0;

protected:
	XboxController				m_controllers[NUM_XBOX_CONTROLLERS];
	KeyButtonState				m_keyStates[NUM_KEYCODES];
	InputSystemConfig			m_config;
};

extern InputSystem*				g_theInput;