#pragma once
#include "Engine/Core/EngineCommon.hpp"
#include "Engine/Math/IntVec2.hpp"
#include <string>

//-----------------------------------------------------------------------------------------------
class	InputSystem;
struct	Vec2;
struct	FloatRange;

//-----------------------------------------------------------------------------------------------
struct WindowConfig
{
	InputSystem*	m_inputSystem = nullptr;
	std::string		m_windowTitle = "Untitled App";
	float			m_clientAspect = 2.0f;
	IntVec2			m_windowPosition = IntVec2(-1, -1);
	IntVec2			m_windowSize = IntVec2(-1, -1);
	bool			m_isFullscreen = false;
	bool			m_isUsingIMGUI = false;
};

//-----------------------------------------------------------------------------------------------
class Window
{
public:
	Window(WindowConfig const& config);
	~Window();

	void				Startup();
	void				BeginFrame();
	void				EndFrame();
	void				Shutdown();

	WindowConfig const& GetConfig() const;
	static Window*		GetWindowContext();
	void*				GetHwnd() const;
	IntVec2				GetClientDimensions() const;
	IntVec2				GetClientCenter() const;
	IntVec2				GetScreenDimensions() const;
	bool				DoesWindowHaveFocus();
	std::string			OpenFileBrowser(std::string directoryPath);

protected:
	void				CreateOSWindow();
	void				RunMessagePump();

protected:
	WindowConfig		m_config;
	static Window*		s_mainWindow;
	IntVec2				m_clientDimensions;
};

//-----------------------------------------------------------------------------------------------
extern Window*			g_theWindow;
