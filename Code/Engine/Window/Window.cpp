#include "Window.hpp"
#include "Engine/Input/InputSystem.hpp"
#include "Engine/Core/ErrorWarningAssert.hpp"
#include "Engine/Core/EventSystem.hpp"
#include "Engine/Math/FloatRange.hpp"
#define WIN32_LEAN_AND_MEAN		// Always #define this before #including <windows.h>
#include <windows.h>			// #include this (massive, platform-specific) header in very few places
#include <commdlg.h>
#pragma comment(lib, "Comdlg32.lib")
#include <iostream>

//-----------------------------------------------------------------------------------------------
//IMGUI
#include "ThirdParty/ImGui/imgui.h"
#include "ThirdParty/ImGui/imgui_impl_win32.h"
#include "ThirdParty/ImGui/imgui_impl_dx11.h"

//-----------------------------------------------------------------------------------------------
Window* Window::s_mainWindow = nullptr;
HWND m_windowHandle = nullptr;								// ...becomes WindowContext::m_windowHandle
HDC m_displayContext = nullptr;				// ...becomes WindowContext::m_displayContext

//-----------------------------------------------------------------------------------------------
// Handles Windows (Win32) messages/events; i.e. the OS is trying to tell us something happened.
// This function is called by Windows whenever we ask it for notifications
//
// #SD1ToDo: We will move this function to a more appropriate place later on...
//
extern IMGUI_IMPL_API LRESULT ImGui_ImplWin32_WndProcHandler(HWND hWnd, UINT msg, WPARAM wParam, LPARAM lParam);

LRESULT CALLBACK WindowsMessageHandlingProcedure(HWND windowHandle, UINT wmMessageCode, WPARAM wParam, LPARAM lParam)
{
	Window* windowContext = Window::GetWindowContext();
	GUARANTEE_OR_DIE(windowContext != nullptr, "WindowContext was null!");
	InputSystem* input = windowContext->GetConfig().m_inputSystem;
	GUARANTEE_OR_DIE(input != nullptr, "No Input System!");

	if (ImGui_ImplWin32_WndProcHandler(windowHandle, wmMessageCode, wParam, lParam))
		return true;

	switch (wmMessageCode)
	{
		// App close requested via "X" button, or right-click "Close Window" on task bar, or "Close" from system menu, or Alt-F4
		case WM_CLOSE:
		{
			EventArgs args;
			g_theEventSystem->FireEvent("quit");
			return 0;
		}

		case WM_CHAR:
		{
			EventArgs args;
			args.SetValue("Char", Stringf("%c", (char)wParam));
			g_theEventSystem->FireEvent("CharInput", args);
			return 0;
		}

		// Raw physical keyboard "key-was-just-depressed" event (case-insensitive, not translated)
		case WM_KEYDOWN:
		{
			EventArgs args;
			args.SetValue("KeyCode", Stringf("%d", (unsigned char)wParam));
			g_theEventSystem->FireEvent("KeyPressed", args);
			return 0;
		}

		// Raw physical keyboard "key-was-just-released" event (case-insensitive, not translated)
		case WM_KEYUP:
		{
			EventArgs args;
			args.SetValue("KeyCode", Stringf("%d", (unsigned char)wParam));
			g_theEventSystem->FireEvent("KeyReleased", args);
			return 0;
		}

		case WM_LBUTTONDOWN:
		{
			unsigned char keyCode = KEYCODE_LEFT_MOUSE;
			bool wasConsumed = false;
			if (input)
			{
				wasConsumed = input->HandleKeyPressed(keyCode);
				if (wasConsumed)
				{
					return 0;
				}
			}
			break;
		}

		case WM_RBUTTONDOWN:
		{
			unsigned char keyCode = KEYCODE_RIGHT_MOUSE;
			bool wasConsumed = false;
			if (input)
			{
				wasConsumed = input->HandleKeyPressed(keyCode);
				if (wasConsumed)
				{
					return 0;
				}
			}
			break;
		}

		case WM_LBUTTONUP:
		{
			unsigned char keyCode = KEYCODE_LEFT_MOUSE;
			bool wasConsumed = false;
			if (input)
			{
				wasConsumed = input->HandleKeyReleased(keyCode);
				if (wasConsumed)
				{
					return 0;
				}
			}
			break;
		}

		case WM_RBUTTONUP:
		{
			unsigned char keyCode = KEYCODE_RIGHT_MOUSE;
			bool wasConsumed = false;
			if (input)
			{
				wasConsumed = input->HandleKeyReleased(keyCode);
				if (wasConsumed)
				{
					return 0;
				}
			}
			break;
		}

		case WM_MOUSEWHEEL:
		{
			int delta = GET_WHEEL_DELTA_WPARAM(wParam);
			if (delta > 0) 
			{
				// Scrolling Up
				input->m_scrollWheelDelta = delta;
			}
			else if (delta < 0) 
			{
				// Scrolling down
				input->m_scrollWheelDelta = delta;
			}
		}
	}

	// Send back to Windows any unhandled/unconsumed messages we want other apps to see (e.g. play/pause in music apps, etc.)
	return DefWindowProc(windowHandle, wmMessageCode, wParam, lParam);
}

//-----------------------------------------------------------------------------------------------
Window::Window(WindowConfig const& config)
	:m_config(config)
{
	s_mainWindow = this;
}

//-----------------------------------------------------------------------------------------------
Window::~Window()
{
}

//-----------------------------------------------------------------------------------------------
void Window::Startup()
{
	CreateOSWindow();

	if (m_config.m_isUsingIMGUI == true)
	{
		ImGui_ImplWin32_Init(GetHwnd());
	}
}

//-----------------------------------------------------------------------------------------------
void Window::BeginFrame()
{
	RunMessagePump();
}

//-----------------------------------------------------------------------------------------------
void Window::EndFrame()
{
}

//-----------------------------------------------------------------------------------------------
void Window::Shutdown()
{
}

//-----------------------------------------------------------------------------------------------
WindowConfig const& Window::GetConfig() const
{
	return m_config;
}

//-----------------------------------------------------------------------------------------------
Window* Window::GetWindowContext()
{
	return s_mainWindow;
}

//-----------------------------------------------------------------------------------------------
void* Window::GetHwnd() const
{
	return m_windowHandle;
}

//-----------------------------------------------------------------------------------------------
IntVec2 Window::GetClientDimensions() const
{
	return m_clientDimensions;
}

//-----------------------------------------------------------------------------------------------
IntVec2 Window::GetClientCenter() const
{
	HWND hWnd = ::GetActiveWindow();
	RECT clientRect;
	GetClientRect(hWnd, &clientRect);
	POINT center;
	center.x = clientRect.left + clientRect.right / 2;
	center.y = clientRect.top + clientRect.bottom / 2;

	return IntVec2(center.x, center.y);
}
//-----------------------------------------------------------------------------------------------
IntVec2 Window::GetScreenDimensions() const
{
	int screenWidth = GetSystemMetrics(SM_CXSCREEN);
	int screenHeight = GetSystemMetrics(SM_CYSCREEN);
	return IntVec2(screenWidth, screenHeight);
}

//-----------------------------------------------------------------------------------------------
bool Window::DoesWindowHaveFocus()
{
	HWND hWnd = ::GetForegroundWindow();

	if (hWnd != GetHwnd()) 
	{
		return false;
	}
	
	return true;
}

//-----------------------------------------------------------------------------------------------
std::string Window::OpenFileBrowser(std::string directoryPath)
{
	char originalFilePath[MAX_PATH] = "";
	char returnFilePath[MAX_PATH] = "";
	GetCurrentDirectoryA(MAX_PATH, originalFilePath);
	directoryPath = std::string(originalFilePath) + directoryPath;
	OPENFILENAMEA ofn = {};
	ZeroMemory(&ofn, sizeof(ofn));
	ofn.lStructSize = sizeof(ofn);
	ofn.lpstrFile = LPSTR(returnFilePath);
	ofn.nMaxFile = MAX_PATH;
	ofn.lpstrInitialDir = directoryPath.c_str();
	ofn.Flags = OFN_EXPLORER | OFN_FILEMUSTEXIST | OFN_HIDEREADONLY;
	ofn.lpstrFilter = "Description (XML Files)\0 *.xml\0";
	if (GetOpenFileNameA(&ofn) != 0)
	{
		SetCurrentDirectoryA(LPSTR(originalFilePath));
		return std::string(returnFilePath);
	}

	SetCurrentDirectoryA(LPSTR(originalFilePath));
	return "";
}

//-----------------------------------------------------------------------------------------------
void Window::CreateOSWindow()
{
	// Define a window style/class
	WNDCLASSEX windowClassDescription;
	memset(&windowClassDescription, 0, sizeof(windowClassDescription));
	windowClassDescription.cbSize = sizeof(windowClassDescription);
	windowClassDescription.style = CS_OWNDC; // Redraw on move, request own Display Context
	windowClassDescription.lpfnWndProc = static_cast<WNDPROC>(WindowsMessageHandlingProcedure); // Register our Windows message-handling function
	windowClassDescription.hInstance = GetModuleHandle(NULL);
	windowClassDescription.hIcon = NULL;
	windowClassDescription.hCursor = NULL;
	windowClassDescription.lpszClassName = TEXT("Simple Window Class");
	RegisterClassEx(&windowClassDescription);

	//Fullscreen Logic
	DWORD windowStyleFlags;
	if (m_config.m_isFullscreen == true)
	{
		windowStyleFlags = WS_POPUP | WS_SYSMENU;
	}
	else
	{
		windowStyleFlags = WS_CAPTION | WS_BORDER | WS_SYSMENU | WS_OVERLAPPED;
	}
	const DWORD windowStyleExFlags = WS_EX_APPWINDOW;

	// Get desktop rect, dimensions, aspect
	RECT desktopRect;
	HWND desktopWindowHandle = GetDesktopWindow();
	GetClientRect(desktopWindowHandle, &desktopRect);
	float desktopWidth = (float)(desktopRect.right - desktopRect.left);
	float desktopHeight = (float)(desktopRect.bottom - desktopRect.top);
	float desktopAspect = desktopWidth / desktopHeight;

	// Calculate maximum client size (as some % of desktop size)
	constexpr float maxClientFractionOfDesktop = 0.9f;
	float clientWidth;
	float clientHeight;

	//Check if the size of client was preset
	if (m_config.m_windowSize == IntVec2(-1, -1))
	{
		clientWidth = desktopWidth * maxClientFractionOfDesktop;
		clientHeight = desktopHeight * maxClientFractionOfDesktop;
		if (m_config.m_clientAspect > desktopAspect)
		{
			// Client window has a wider aspect than desktop; shrink client height to match its width
			clientHeight = clientWidth / m_config.m_clientAspect;
		}
		else
		{
			// Client window has a taller aspect than desktop; shrink client width to match its height
			clientWidth = clientHeight * m_config.m_clientAspect;
		}
	}
	else
	{
		clientWidth = float(m_config.m_windowSize.x);
		clientHeight = float(m_config.m_windowSize.y);
	}

	if (m_config.m_isFullscreen == true)
	{
		clientWidth = desktopWidth;
		clientHeight = desktopHeight;
	}

	m_clientDimensions.x = static_cast<int>(clientWidth);
	m_clientDimensions.y = static_cast<int>(clientHeight);

	// Calculate client rect bounds by centering the client area
	float clientMarginX;
	float clientMarginY;
	RECT clientRect;
	if (m_config.m_windowPosition == IntVec2(-1, -1))
	{
		clientMarginX = 0.5f * (desktopWidth - clientWidth);
		clientMarginY = 0.5f * (desktopHeight - clientHeight);
		clientRect.left = (int)clientMarginX;
		clientRect.right = clientRect.left + (int)clientWidth;
		clientRect.top = (int)clientMarginY;
		clientRect.bottom = clientRect.top + (int)clientHeight;
		m_config.m_windowPosition = IntVec2(clientRect.left, clientRect.top);
		m_config.m_windowSize = IntVec2(clientRect.right - clientRect.left, clientRect.bottom - clientRect.top);
	}
	else
	{
		clientRect.left = m_config.m_windowPosition.x;
		clientRect.right = clientRect.left + m_config.m_windowSize.x;
		clientRect.top = m_config.m_windowPosition.y;
		clientRect.bottom = clientRect.top + m_config.m_windowSize.y;
	}


	// Calculate the outer dimensions of the physical window, including frame et. al.
	RECT windowRect = clientRect;
	AdjustWindowRectEx(&windowRect, windowStyleFlags, FALSE, windowStyleExFlags);

	WCHAR windowTitle[1024];
	MultiByteToWideChar(GetACP(), 0, m_config.m_windowTitle.c_str(), -1, windowTitle, sizeof(windowTitle) / sizeof(windowTitle[0]));
	m_windowHandle = CreateWindowEx(
		windowStyleExFlags,
		windowClassDescription.lpszClassName,
		windowTitle,
		windowStyleFlags,
		windowRect.left,
		windowRect.top,
		windowRect.right - windowRect.left,
		windowRect.bottom - windowRect.top,
		NULL,
		NULL,
		(HINSTANCE)NULL,
		NULL);

	ShowWindow(m_windowHandle, SW_SHOW);
	SetForegroundWindow(m_windowHandle);
	SetFocus(m_windowHandle);

	m_displayContext = GetDC(m_windowHandle);

	HCURSOR cursor = LoadCursor(NULL, IDC_ARROW);
	SetCursor(cursor);
}

//-----------------------------------------------------------------------------------------------
// Processes all Windows messages (WM_xxx) for this app that have queued up since last frame.
// For each message in the queue, our WindowsMessageHandlingProcedure (or "WinProc") function
//	is called, telling us what happened (key up/down, minimized/restored, gained/lost focus, etc.)
//
// #SD1ToDo: We will move this function to a more appropriate place later on...
//
void Window::RunMessagePump()
{
	MSG queuedMessage;
	for (;; )
	{
		const BOOL wasMessagePresent = PeekMessage(&queuedMessage, NULL, 0, 0, PM_REMOVE);
		if (!wasMessagePresent)
		{
			break;
		}

		TranslateMessage(&queuedMessage);
		DispatchMessage(&queuedMessage); // This tells Windows to call our "WindowsMessageHandlingProcedure" (a.k.a. "WinProc") function
	}
}
