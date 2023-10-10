#include "DevConsole.hpp"
#include "Clock.hpp"
#include "Stopwatch.hpp"
#include "Engine/Renderer/Camera.hpp"
#include "Engine/Renderer/Renderer.hpp"
#include "Engine/Math/MathUtils.hpp"
#include "Engine/Core/VertexUtils.hpp"
#include "Engine/Window/Window.hpp"
#include "Engine/Renderer/BitmapFont.hpp"

const Rgba8 DevConsole::ERROR = Rgba8(255, 0, 0, 255);
const Rgba8 DevConsole::WARNING = Rgba8(255, 255, 0, 255);
const Rgba8 DevConsole::INFO_MAJOR = Rgba8(0, 255, 255, 255);
const Rgba8 DevConsole::INFO_MINOR = Rgba8(0, 255, 255, 128);
const Rgba8 DevConsole::COMMAND_ECHO = Rgba8(255, 0, 255, 255);
const Rgba8 DevConsole::ECHO_COMMAND = Rgba8(255, 0, 255, 128);
const Rgba8 DevConsole::INPUT_TEXT = Rgba8(255, 255, 255, 255);
const Rgba8 DevConsole::INPUT_CARET = Rgba8(255, 255, 255, 255);

//-----------------------------------------------------------------------------------------------
DevConsole::DevConsole(DevConsoleConfig const& config)
{
	m_config = config;
}

//-----------------------------------------------------------------------------------------------
DevConsole::~DevConsole()
{
}

//-----------------------------------------------------------------------------------------------
void DevConsole::Startup()
{
	g_theEventSystem->SubscribeEventCallbackFunciton("KeyPressed", DevConsole::Event_KeyPressed);
	g_theEventSystem->SubscribeEventCallbackFunciton("CharInput", DevConsole::Event_CharInput);
	g_theEventSystem->SubscribeEventCallbackFunciton("clear", DevConsole::Command_Clear);
	g_theEventSystem->SubscribeEventCallbackFunciton("help", DevConsole::Command_Help);
	g_theEventSystem->SubscribeEventCallbackFunciton("Echo", DevConsole::Event_Echo);

	m_caretStopwatch = new Stopwatch(&Clock::GetSystemClock(), 0.5f);

	m_config.m_camera = new Camera();
	m_config.m_camera->SetOrthographicView(Vec2(0.f, 0.f), Vec2(1600.0f, 800.0f));
	m_config.m_camera->SetTransform(Vec3(0.0f, 0.0f, 0.0f), EulerAngles());

	DevConsoleLine line;
	line.m_color = DevConsole::INFO_MINOR;
	line.m_text = "Type help for a list of commands";
	m_lines.push_back(line);
	line.m_color = DevConsole::INFO_MAJOR;
	line.m_text = "Keys";
	m_lines.push_back(line);
}

//-----------------------------------------------------------------------------------------------
void DevConsole::Shutdown()
{
	delete m_config.m_camera;
	m_config.m_camera = nullptr;

	m_config = { 0 };

	delete m_caretStopwatch;
	m_caretStopwatch = nullptr;
}

//-----------------------------------------------------------------------------------------------
void DevConsole::BeginFrame()
{
}

//-----------------------------------------------------------------------------------------------
void DevConsole::EndFrame()
{
}

//-----------------------------------------------------------------------------------------------
void DevConsole::Execute(std::string const& consoleCommandText)
{
	if (consoleCommandText == "")
	{
		ToggleOpen();
		return;
	}

	m_commandHistory.push_back(consoleCommandText);

	Strings splitString = SplitStringWithQuotes(consoleCommandText, ' ');
	std::string eventName = "";
	std::string arguments = "";
	for (int stringIndex = 0; stringIndex < splitString.size(); stringIndex++)
	{
		if (stringIndex != 0)
		{
			arguments += splitString[stringIndex];
		}
		else
		{
			eventName += splitString[stringIndex];
		}
	}

	splitString = SplitStringWithQuotes(arguments, '=');

	EventArgs args;
	if (splitString.size() > 1)
	{
		args.SetValue("argument", splitString[1]);
	}

	g_theEventSystem->FireEvent(eventName, args);

	if (args.GetValue("returnValue", "") == "success")
	{
		if (eventName != "Echo")
		{
			g_theDevConsole->AddLine(DevConsole::COMMAND_ECHO, consoleCommandText);
		}
	}
	else if (args.GetValue("returnValue", "") == "failure")
	{
		std::string errorResponse = "Unknown Command: ";
		errorResponse += eventName;
		g_theDevConsole->AddLine(DevConsole::ERROR, errorResponse);
	}
}

//-----------------------------------------------------------------------------------------------
void DevConsole::AddLine(Rgba8 const& color, std::string const& text)
{
	m_devConsoleMutex.lock();
	DevConsoleLine line;
	line.m_color = color;
	line.m_text = text;
	m_lines.push_back(line);
	m_devConsoleMutex.unlock();
}

//-----------------------------------------------------------------------------------------------
void DevConsole::Render(AABB2 const& bounds)
{
	m_devConsoleMutex.lock();
	if (m_isOpen == false)
	{
		m_devConsoleMutex.unlock();
		return;
	}

	m_config.m_renderer->SetBlendMode(BlendMode::ALPHA);
	m_config.m_renderer->SetDepthMode(DepthMode::DISABLED);
	m_config.m_renderer->SetRasterizerMode(RasterizerMode::SOLID_CULL_NONE);
	m_config.m_renderer->BeginCamera(*m_config.m_camera);
	m_config.m_renderer->BindTexture(nullptr);
	std::vector<Vertex_PCU> backgroundVerts;
	AddVertsForAABB2D(backgroundVerts, bounds, Rgba8(0, 0, 0, 200));
	m_config.m_renderer->BindTexture(nullptr);
	m_config.m_renderer->SetModelConstants(Mat44());
	m_config.m_renderer->DrawVertexArray((int)backgroundVerts.size(), backgroundVerts.data());

	BitmapFont* theFont;
	std::string fileName = "Data/Fonts/" + m_config.m_fontName;
	theFont = m_config.m_renderer->CreateOrGetBitmapFont(fileName.c_str());

	//All Line rendering
	std::vector<Vertex_PCU> textVerts;
	int linePosition = 1;
	for (int lineIndex = static_cast<int>(m_lines.size()) - 1; lineIndex >= 0; lineIndex--)
	{
		//Only renders visible lines
		if (linePosition == m_config.m_linesOnScreen + 1)
		{
			break;
		}

		//Allows for rendering of large strings
		Strings splitString = SplitStringOnDelimiter(m_lines[lineIndex].m_text, '\n');
		if (splitString.size() > 1)
		{
			for (int splitStringIndex = static_cast<int>(splitString.size()) - 1; splitStringIndex >= 0; splitStringIndex--)
			{
				//Only renders visible lines
				if (linePosition == m_config.m_linesOnScreen + 1)
				{
					break;
				}

				theFont->AddVertsForTextInBox2D(textVerts, bounds, bounds.GetDimensions().y / m_config.m_linesOnScreen, splitString[splitStringIndex], m_lines[lineIndex].m_color, m_config.m_fontAspect, Vec2(0.0f, RangeMap(static_cast<float>(linePosition), 0.0f, 40.0f, 0.0f, 1.0f)), SHRINK_TO_FIT);
				linePosition++;
			}
		}
		else
		{
			theFont->AddVertsForTextInBox2D(textVerts, bounds, bounds.GetDimensions().y / m_config.m_linesOnScreen, m_lines[lineIndex].m_text, m_lines[lineIndex].m_color, m_config.m_fontAspect, Vec2(0.0f, RangeMap(static_cast<float>(linePosition), 0.0f, 40.0f, 0.0f, 1.0f)), SHRINK_TO_FIT);
			linePosition++;
		}

	}
	theFont->AddVertsForTextInBox2D(textVerts, bounds, bounds.GetDimensions().y / m_config.m_linesOnScreen, m_inputText, DevConsole::INPUT_TEXT, m_config.m_fontAspect, Vec2(0.0f, 0.0f), SHRINK_TO_FIT);
	m_config.m_renderer->BindTexture(&theFont->GetTexture());
	m_config.m_renderer->SetModelConstants(Mat44(), Rgba8::WHITE);
	m_config.m_renderer->DrawVertexArray(static_cast<int>(textVerts.size()), textVerts.data());
	m_config.m_renderer->BindTexture(nullptr);
	
	//Caret Logic
	if (m_caretStopwatch->HasDuraitonElapsed() == true)
	{
		m_caretStopwatch->Restart();
		m_caretVisible = !m_caretVisible;
	}
	
	if (m_caretVisible == false)
	{
		m_config.m_renderer->EndCamera(*m_config.m_camera);
		m_devConsoleMutex.unlock();
		return;
	}

	std::vector<Vertex_PCU> caretVerts;
	AABB2 caret;
	caret.m_mins = Vec2(0.0f, 0.0f);
	caret.m_mins = Vec2(2.0f, bounds.GetDimensions().y / m_config.m_linesOnScreen);
	AddVertsForAABB2D(caretVerts, caret, DevConsole::INPUT_CARET);
	Vec2 iBasis = Vec2(1.0f, 0.0f);
	Vec2 jBasis = iBasis.GetRotated90Degrees();

	//Keeps caret in Dev console render bounds
	float caretXPosiiton = bounds.GetDimensions().y / m_config.m_linesOnScreen * m_config.m_fontAspect * m_caretPosition;
	if (caretXPosiiton > bounds.GetDimensions().x)
	{
		caretXPosiiton = bounds.GetDimensions().x;
	}

	TransformVertexArrayXY3D((int)caretVerts.size(), caretVerts.data(), 1.0f, iBasis, jBasis, Vec2(caretXPosiiton, 0.0f));
	m_config.m_renderer->DrawVertexArray((int)caretVerts.size(), caretVerts.data());
	m_config.m_renderer->EndCamera(*m_config.m_camera);
	m_devConsoleMutex.unlock();
}

//-----------------------------------------------------------------------------------------------
void DevConsole::ToggleOpen()
{
	m_isOpen = !m_isOpen;

	if (m_isOpen == true)
	{
		m_caretStopwatch->Start();
	}
}

//-----------------------------------------------------------------------------------------------
bool DevConsole::IsOpen()
{
	return m_isOpen;
}

//-----------------------------------------------------------------------------------------------
bool DevConsole::Event_KeyPressed(EventArgs& args)
{
	unsigned char keyCode = static_cast<unsigned char>(args.GetValue("KeyCode", -1));
	if (keyCode == KEYCODE_TILDE)
	{
		g_theDevConsole->ToggleOpen();
	}
	else if (g_theDevConsole->m_isOpen == false)
	{
		return false;
	}

	//Reset Caret
	g_theDevConsole->m_caretVisible = true;
	g_theDevConsole->m_caretStopwatch->Restart();

	if (keyCode == KEYCODE_ESC && g_theDevConsole->m_inputText == "")
	{
		g_theDevConsole->m_isOpen = false;
		g_theDevConsole->m_historyIndex = -1;
	}
	else if (keyCode == KEYCODE_ESC)
	{
		g_theDevConsole->m_historyIndex = -1;
		g_theDevConsole->m_inputText = "";
		g_theDevConsole->m_caretPosition = 0;
	}
	else if (keyCode == KEYCODE_ENTER)
	{
		g_theDevConsole->Execute(g_theDevConsole->m_inputText);
		g_theDevConsole->m_historyIndex = -1;
		g_theDevConsole->m_inputText = "";
		g_theDevConsole->m_caretPosition = 0;
	}
	else if (keyCode == KEYCODE_BACKSPACE)
	{
		if (g_theDevConsole->m_caretPosition != 0)
		{
			g_theDevConsole->m_inputText.erase(g_theDevConsole->m_caretPosition - 1, 1);
			g_theDevConsole->m_caretPosition--;
		}
	}
	else if (keyCode == KEYCODE_DELETE)
	{
		if (g_theDevConsole->m_caretPosition != 0)
		{
			g_theDevConsole->m_inputText.erase(g_theDevConsole->m_caretPosition, 1);
		}
	}
	else if (keyCode == KEYCODE_UPARROW)
	{
		if (g_theDevConsole->m_commandHistory.size() == 0)
		{
			return true;
		}

		if (g_theDevConsole->m_historyIndex == -1)
		{
			g_theDevConsole->m_historyIndex = static_cast<int>(g_theDevConsole->m_commandHistory.size() - 1);
		}
		else if(g_theDevConsole->m_historyIndex != 0)
		{
			g_theDevConsole->m_historyIndex--;
		}

		g_theDevConsole->m_inputText = g_theDevConsole->m_commandHistory[g_theDevConsole->m_historyIndex];
		g_theDevConsole->m_caretPosition = static_cast<int>(g_theDevConsole->m_inputText.size());
	}
	else if (keyCode == KEYCODE_DOWNARROW)
	{
		if (g_theDevConsole->m_historyIndex == g_theDevConsole->m_commandHistory.size() - 1)
		{
			g_theDevConsole->m_inputText = "";
			g_theDevConsole->m_caretPosition = 0;
			g_theDevConsole->m_historyIndex = -1;
		}
		else if(g_theDevConsole->m_historyIndex != -1)
		{
			g_theDevConsole->m_historyIndex++;
			g_theDevConsole->m_inputText = g_theDevConsole->m_commandHistory[g_theDevConsole->m_historyIndex];
			g_theDevConsole->m_caretPosition = static_cast<int>(g_theDevConsole->m_inputText.size());
		} 
	}
	else if (keyCode == KEYCODE_RIGHTARROW)
	{
		if (g_theDevConsole->m_caretPosition != g_theDevConsole->m_inputText.size())
		{
			g_theDevConsole->m_caretPosition++;
		}
	}
	else if (keyCode == KEYCODE_LEFTARROW)
	{
		if (g_theDevConsole->m_caretPosition != 0)
		{
			g_theDevConsole->m_caretPosition--;
		}
	}
	else if (keyCode == KEYCODE_HOME)
	{
		g_theDevConsole->m_caretPosition = 0;
	}
	else if (keyCode == KEYCODE_END)
	{
		g_theDevConsole->m_caretPosition = static_cast<int>(g_theDevConsole->m_inputText.size());
	}

	return true;
}

//-----------------------------------------------------------------------------------------------
bool DevConsole::Event_CharInput(EventArgs& args)
{
	if (g_theDevConsole->m_isOpen == false)
	{
		return false;
	}

	//Reset Caret
	g_theDevConsole->m_caretVisible = true;
	g_theDevConsole->m_caretStopwatch->Restart();

	std::string stringChar = args.GetValue("Char", "test");
	char singleChar = stringChar[0];
	if (singleChar >= 32 && singleChar <= 126 && singleChar != '~' && singleChar != '`')
	{
		g_theDevConsole->m_inputText.insert(g_theDevConsole->m_caretPosition, 1, singleChar);
		g_theDevConsole->m_caretPosition++;

		return true;
	}

	return false;
}

//-----------------------------------------------------------------------------------------------
bool DevConsole::Event_Echo(EventArgs& args)
{
	if (args.GetValue("argument", "") == "")
	{
		std::string errorResponse = "Invalid Format:";
		g_theDevConsole->AddLine(DevConsole::ERROR, errorResponse);
		errorResponse = "Example format \"";
		errorResponse += "Echo Message = \"message\"\"";
		g_theDevConsole->AddLine(DevConsole::WARNING, errorResponse);
		return false;
	}

	std::string command = args.GetValue("argument", "");
	TrimString(command, '\"');
	g_theDevConsole->AddLine(ECHO_COMMAND, command);
	return true;
}

//-----------------------------------------------------------------------------------------------
bool DevConsole::Command_Clear(EventArgs& args)
{
	args;
	if (g_theDevConsole->m_isOpen == false)
	{
		return false;
	}

	g_theDevConsole->m_lines.clear();
	return true;
}

//-----------------------------------------------------------------------------------------------
bool DevConsole::Command_Help(EventArgs& args)
{
	args;
	if (g_theDevConsole->m_isOpen == false)
	{
		return false;
	}

	g_theDevConsole->AddLine(DevConsole::INFO_MAJOR, "Registered Commands");
	std::vector<std::string> commands = g_theEventSystem->GetRegisteredCommands();
	for (int commandIndex = 0; commandIndex < commands.size(); commandIndex++)
	{
		g_theDevConsole->AddLine(DevConsole::INFO_MINOR, commands[commandIndex]);
	}
	
	return true;
}
