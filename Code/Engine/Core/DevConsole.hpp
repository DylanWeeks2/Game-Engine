#pragma once
#include "Clock.hpp"
#include "EngineCommon.hpp"
#include "Rgba8.hpp"
#include <string>
#include <vector>
#include <mutex>

//-----------------------------------------------------------------------------------------------
class	Renderer;
class	Camera;
class	BitmapFont;
class	Stopwatch;
struct	AABB2; 
class	DevConsole;

//-----------------------------------------------------------------------------------------------
extern	DevConsole* g_theDevConsole;

//-----------------------------------------------------------------------------------------------
struct DevConsoleLine
{
	HCIString	m_text;
	Rgba8		m_color;
};

//-----------------------------------------------------------------------------------------------
struct DevConsoleConfig
{
	Renderer*	m_renderer = nullptr;
	Camera*		m_camera = nullptr;
	std::string m_fontName = "SquirrelFixedFont";
	float		m_fontAspect = 0.7f;
	float		m_linesOnScreen = 40.0f;
	int			m_maxCommandHistory = 128;
};

//-----------------------------------------------------------------------------------------------
class DevConsole
{
public:
	DevConsole(DevConsoleConfig const& config);
	~DevConsole();

	void		Startup();
	void		Shutdown();
	void		BeginFrame();
	void		EndFrame();

	void		Execute(std::string const& consoleCommandText);
	void		AddLine(Rgba8 const& color, HCIString const& text);
	void		Render(AABB2 const& bounds);
	void		ToggleOpen();
	bool		IsOpen();

	static bool Event_KeyPressed(EventArgs& args);
	static bool Event_CharInput(EventArgs& args);
	static bool Event_Echo(EventArgs& args);
	static bool Command_Clear(EventArgs& args);
	static bool Command_Help(EventArgs& args);

	static const Rgba8 ERROR;
	static const Rgba8 WARNING;
	static const Rgba8 INFO_MAJOR;
	static const Rgba8 INFO_MINOR;
	static const Rgba8 COMMAND_ECHO;
	static const Rgba8 ECHO_COMMAND;
	static const Rgba8 INPUT_TEXT;
	static const Rgba8 INPUT_CARET;

protected:
	DevConsoleConfig			m_config;
	std::mutex					m_devConsoleMutex;
	std::vector<DevConsoleLine> m_lines;
	std::string					m_inputText;
	Stopwatch*					m_caretStopwatch = nullptr;
	std::vector<std::string>	m_commandHistory;
	int							m_caretPosition = 0;
	int							m_historyIndex = -1;
	bool						m_caretVisible = true;
	std::atomic<bool>			m_isOpen = false;
};
