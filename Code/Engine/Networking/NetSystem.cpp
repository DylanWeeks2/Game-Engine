#include "NetSystem.hpp"
#include "Engine/Core/ErrorWarningAssert.hpp"
#include "Engine/Core/DevConsole.hpp"
#include "Engine/Core/StringUtils.hpp"

#include "Game/EngineBuildPreferences.hpp"
#if !defined( ENGINE_DISABLE_NETWORKING )

#define WIN32_LEAN_AND_MEAN
#include <windows.h>
#include <winsock2.h>
#include <ws2tcpip.h>
#pragma comment(lib, "Ws2_32.lib")

//-----------------------------------------------------------------------------------------------
NetSystem::NetSystem(const NetSystemConfig& config)
	:m_config(config)
{
	if (m_config.m_modeString == "Server")
	{
		m_mode = Mode::SERVER;
	}
	else if (m_config.m_modeString == "Client")
	{
		m_mode = Mode::CLIENT;
	}
	else
	{
		m_mode = Mode::NONE;
	}

	m_sendBuffer = (char*)malloc(m_config.m_sendBufferSize * sizeof(char));
	m_recvBuffer = (char*)malloc(m_config.m_recvBufferSize * sizeof(char));
}

//-----------------------------------------------------------------------------------------------
NetSystem::~NetSystem()
{
	free(m_sendBuffer);
	free(m_recvBuffer);
}

//-----------------------------------------------------------------------------------------------
void NetSystem::Startup()
{
	g_theEventSystem->SubscribeEventCallbackFunciton("RemoteCommand", NetSystem::Event_RemoteCommand);
	g_theEventSystem->SubscribeEventCallbackFunciton("BurstTest", NetSystem::Event_BurstTest);

	if (m_mode == Mode::SERVER)
	{
		WSADATA data;
		int startupResult = WSAStartup(MAKEWORD(2, 2), &data);
		if (startupResult != 0)
		{
			ERROR_AND_DIE(Stringf("WSAStartup failed with error: %d", startupResult));
		}

		// Create listen socket.
		m_listenSocket = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
		if (m_listenSocket != INVALID_SOCKET)
		{
			// Set blocking mode.
			unsigned long blockingMode = 1;
			int blockingResult = ioctlsocket(m_listenSocket, FIONBIO, &blockingMode);
			if (blockingResult != 0)
			{
				ERROR_AND_DIE(Stringf("Blocking mode failed to set: %d", blockingResult));
			}

			// Get host port from string.
			Strings splitHostAddress = SplitStringOnDelimiter(m_config.m_hostAddressString, ':');
			m_hostAddress = INADDR_ANY;
			m_hostPort = (unsigned short)(atoi(splitHostAddress[1].c_str()));

			// Bind the listen socket to a port.
			sockaddr_in addr;
			addr.sin_family = AF_INET;
			addr.sin_addr.S_un.S_addr = htonl(m_hostAddress);
			addr.sin_port = htons(m_hostPort);
			int bindResult = bind(m_listenSocket, (sockaddr*)&addr, (int)sizeof(addr));
			if (bindResult != 0)
			{
				ERROR_AND_DIE(Stringf("Failed to bind server socket: %d", bindResult));
			}

			// Listen for connections to accept.
			int listenResult = listen(m_listenSocket, SOMAXCONN);
			if (listenResult != SOCKET_ERROR)
			{
				m_serverState = ServerState::LISTENING;
			}
			else
			{
				m_serverState = ServerState::INVALID;
			}
		}
		else
		{
			m_serverState = ServerState::INVALID;
		}
		
	}
	else if (m_mode == Mode::CLIENT)
	{
		// Startup Windows sockets.
		WSADATA data;
		int startupResult = WSAStartup(MAKEWORD(2, 2), &data);
		if (startupResult != 0)
		{
			ERROR_AND_DIE(Stringf("WSAStartup failed with error : %d", startupResult));
		}

		// Create client socket.
		m_clientSocket = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
		if (m_clientSocket != INVALID_SOCKET)
		{
			m_clientState = ClientState::READY_TO_CONNECT;

			// Set blocking mode.
			unsigned long blockingMode = 1;
			int blockingResult = ioctlsocket(m_clientSocket, FIONBIO, &blockingMode);
			if (blockingResult != 0)
			{
				ERROR_AND_DIE(Stringf("Blocking mode failed to set: %d", blockingResult));
			}

			// Get host address from string. 
			IN_ADDR addr;
			Strings splitHostAddress = SplitStringOnDelimiter(m_config.m_hostAddressString, ':');
			int result = inet_pton(AF_INET, splitHostAddress[0].c_str(), &addr);
			if (result != 1)
			{
				DebuggerPrintf("Failed to get host address: %d", result);
			}
			m_hostAddress = ntohl(addr.S_un.S_addr);

			// Get host port from string. Do not hard - code the string in your code like the example below.
			m_hostPort = (unsigned short)(atoi(splitHostAddress[1].c_str()));
		}
		else
		{
			m_clientState = ClientState::INVALID;
		}

	}
}

//-----------------------------------------------------------------------------------------------
void NetSystem::Shutdown()
{
	if (m_mode == Mode::SERVER)
	{
		// Close all open sockets.
		closesocket(m_listenSocket);
		closesocket(m_clientSocket);

		// Shutdown Windows sockets.
		WSACleanup();
	}
	else if (m_mode == Mode::CLIENT)
	{
		// Close all open sockets.
		closesocket(m_clientSocket);

		// Shutdown Windows sockets.
		WSACleanup();
	}
}

//-----------------------------------------------------------------------------------------------
void NetSystem::BeginFrame()
{
	if (m_mode == Mode::SERVER)
	{
		//Invalid server check
		if (m_serverState == ServerState::INVALID)
		{
			int error = WSAGetLastError();
			ERROR_AND_DIE(Stringf("SERVER IN INVALID STATE, ERROR CODE = %d", error));
		}

		// If we do not have a connection, check for connections to accept.
		if(m_serverState == ServerState::LISTENING)
		{
			m_clientSocket = accept(m_listenSocket, NULL, NULL);

			// If a connection is accepted set blocking mode.
			if (m_clientSocket != INVALID_SOCKET)
			{
				m_serverState = ServerState::CONNECTED;
				DebuggerPrintf("Client Connected!\n");
				unsigned long blockingMode = 1;
				int blockingResult = ioctlsocket(m_clientSocket, FIONBIO, &blockingMode);
				if (blockingResult != 0)
				{
					ERROR_AND_DIE(Stringf("Blocking mode failed to set: %d", blockingResult));
				}
			}
			else
			{
				int error = WSAGetLastError();
				if (error == WSAEWOULDBLOCK)
				{
					m_serverState = ServerState::LISTENING;
				}
				else
				{
					m_serverState = ServerState::INVALID;
				}
			}
		}
		else if(m_serverState == ServerState::CONNECTED)
		{
			// Send and receive if we are connected.
			if (m_sendQueue.size() > 0)
			{
				for (int dequeIndex = 0; dequeIndex < m_sendQueue.size(); dequeIndex++)
				{
					if (m_sendQueue[dequeIndex].length() * sizeof(char) > m_config.m_sendBufferSize)
					{
						ERROR_AND_DIE("SEND BUFFER NOT BIG ENOUGH FOR MESSAGE BEING SENT!");
					}
					strncpy_s(m_sendBuffer, m_config.m_sendBufferSize, m_sendQueue[dequeIndex].c_str(), strlen(m_sendQueue[dequeIndex].c_str()) + 1);
					int sendResult = send(m_clientSocket, m_sendBuffer, (int)strlen(m_sendBuffer) + 1, 0);
					if (sendResult == SOCKET_ERROR)
					{
						int error = WSAGetLastError();
						if (error != WSAEWOULDBLOCK)
						{
							ERROR_AND_DIE(Stringf("SEND COMMAND RESULTED IN AN ERROR: %d", sendResult));
						}
					}
				}
				m_sendQueue.clear();
			}

			int receiveResult = recv(m_clientSocket, m_recvBuffer, m_config.m_recvBufferSize, 0);

			if (receiveResult > 0)
			{
				//RECEIVE LOGIC HERE
				std::string executeBuffer = m_recvRemaining;
				for (int index = 0; index < receiveResult; index++)
				{
					if (m_recvBuffer[index] == '\0')
					{
						g_theDevConsole->Execute(executeBuffer);
						std::string echo = "Echo message = \"Server executed remote command: ";
						//m_sendQueue.push_back(echo + executeBuffer + "\"");
						//DebuggerPrintf("Recv Worked: %s\n", executeBuffer.c_str());
						executeBuffer = "";
						m_recvRemaining = "";
						continue;
					}
					executeBuffer += m_recvBuffer[index];
				}
				
				if (executeBuffer != "")
				{
					m_recvRemaining = executeBuffer;
				}
			}
			else if (receiveResult == 0)
			{
				DebuggerPrintf("Client Disconnected: %d\n", receiveResult);
				m_serverState = ServerState::LISTENING;
				Shutdown();
				Startup();
			}
			else if(receiveResult < 0)
			{
				int error = WSAGetLastError();
				if (error == WSAECONNRESET)
				{
					DebuggerPrintf("Client Disconnected: %d\n", error);
					m_serverState = ServerState::LISTENING;
					Shutdown();
					Startup();
				}
				else if (error != WSAEWOULDBLOCK)
				{
					m_serverState = ServerState::INVALID;
				}
			}
		}
	}
	else if (m_mode == Mode::CLIENT)
	{		
		if (m_clientState == ClientState::INVALID)
		{
			int error = WSAGetLastError();
			ERROR_AND_DIE(Stringf("CLIENT IN INVALID STATE, ERROR CODE = %d", error));
		}

		// Attempt to connect if we haven't already.
		if (m_clientState == ClientState::READY_TO_CONNECT)
		{
			sockaddr_in addr;
			addr.sin_family = AF_INET;
			addr.sin_addr.S_un.S_addr = htonl(m_hostAddress);
			addr.sin_port = htons(m_hostPort);
			int connectResult = connect(m_clientSocket, (sockaddr*)(&addr), (int)sizeof(addr));

			// Check if our connection attempt completed.
			if (connectResult != SOCKET_ERROR)
			{
				m_clientState = ClientState::CONNECTED;
				DebuggerPrintf("Server Connected!\n");
			}
			else
			{
				int error = WSAGetLastError();
				if (error == WSAEWOULDBLOCK)
				{
					m_clientState = ClientState::CONNECTING;
				}
				else
				{
					m_clientState = ClientState::INVALID;
				}
			}
		}
		else if (m_clientState == ClientState::CONNECTING)
		{
			//Check if our connection attempt failed.
			fd_set sockets;
			FD_ZERO(&sockets);
			FD_SET(m_clientSocket, &sockets);
			timeval waitTime = { };
			int selectResult = select(0, NULL, NULL, &sockets, &waitTime);

			//The connection failed if the following is true, in which case we need to connect again.
			if (selectResult > 0 && FD_ISSET(m_clientSocket, &sockets))
			{
				m_clientState = ClientState::READY_TO_CONNECT;
				return;
			} 
			else if (selectResult == SOCKET_ERROR)
			{
				m_clientState = ClientState::INVALID;
				return;
			}

			//Check if our connection attempt completed.
			fd_set sockets2;
			FD_ZERO(&sockets2);
			FD_SET(m_clientSocket, &sockets2);
			timeval waitTime2 = { };
			int completeResult = select(0, NULL, &sockets2, NULL, &waitTime2);

			//We are connected if the following is true.
			if (completeResult > 0 && FD_ISSET(m_clientSocket, &sockets2))
			{
				m_clientState = ClientState::CONNECTED;
				DebuggerPrintf("Server Connected!\n");
			}
			else if(completeResult == SOCKET_ERROR)
			{
				m_clientState = ClientState::INVALID;
			}
		}
		else if (m_clientState == ClientState::CONNECTED)
		{
			// Send and receive if we are connected.
			if (m_sendQueue.size() > 0)
			{
				for (int dequeIndex = 0; dequeIndex < m_sendQueue.size(); dequeIndex++)
				{
					if (m_sendQueue[dequeIndex].length() * sizeof(char) > m_config.m_sendBufferSize)
					{
						ERROR_AND_DIE("SEND BUFFER NOT BIG ENOUGH FOR MESSAGE BEING SENT!");
					}
					strncpy_s(m_sendBuffer, m_config.m_sendBufferSize, m_sendQueue[dequeIndex].c_str(), strlen(m_sendQueue[dequeIndex].c_str()) + 1);
					int sendResult = send(m_clientSocket, m_sendBuffer, (int)strlen(m_sendBuffer) + 1, 0);
					
					if (sendResult == SOCKET_ERROR)
					{
						int error = WSAGetLastError();
						if (error != WSAEWOULDBLOCK)
						{
							ERROR_AND_DIE(Stringf("SEND COMMAND RESULTED IN AN ERROR: %d", sendResult));
						}
					}
				}
				m_sendQueue.clear();
			}

			int receiveResult = recv(m_clientSocket, m_recvBuffer, m_config.m_recvBufferSize, 0);
			if (receiveResult > 0)
			{
				std::string executeBuffer = m_recvRemaining;
				for (int index = 0; index < receiveResult; index++)
				{
					if (m_recvBuffer[index] == '\0')
					{
						g_theDevConsole->Execute(executeBuffer);
						//DebuggerPrintf("Recv Worked: %s\n", m_recvBuffer);
						executeBuffer = "";
						m_recvRemaining = "";
						continue;
					}
					executeBuffer += m_recvBuffer[index];
				}

				if (executeBuffer != "")
				{
					m_recvRemaining = executeBuffer;
				}
			}
			else if (receiveResult == 0)
			{
				DebuggerPrintf("Server Disconnected: %d\n", receiveResult);
				m_clientState = ClientState::READY_TO_CONNECT;
				Shutdown();
				Startup();
			}
			else if (receiveResult < 0)
			{
				int error = WSAGetLastError();
				if (error == WSAECONNRESET)
				{
					DebuggerPrintf("Server Disconnected: %d\n", error);
					m_clientState = ClientState::READY_TO_CONNECT;
					Shutdown();
					Startup();
				}
				else if (error != WSAEWOULDBLOCK)
				{
					m_clientState = ClientState::INVALID;
				}
			}
		}
	}
}

//-----------------------------------------------------------------------------------------------
void NetSystem::EndFrame()
{
}

//-----------------------------------------------------------------------------------------------
bool NetSystem::Event_RemoteCommand(EventArgs& args)
{
	if (args.GetValue("argument", "") == "")
	{
		std::string errorResponse = "Invalid Format:";
		g_theDevConsole->AddLine(Rgba8::RED, errorResponse);
		errorResponse = "Example format \"";
		errorResponse += "RemoteCommand Command = \"command\"\"";
		g_theDevConsole->AddLine(DevConsole::WARNING, errorResponse);
		return false;
	}

	std::string command = args.GetValue("argument", "");
	TrimString(command, '\"');
	g_theNetSystem->m_sendQueue.push_back(command);
	return true;
}

//-----------------------------------------------------------------------------------------------
bool NetSystem::Event_BurstTest(EventArgs& args)
{
	if (args.GetValue("argument", "") != "")
	{
		std::string errorResponse = "Invalid Format:";
		g_theDevConsole->AddLine(Rgba8::RED, errorResponse);
		errorResponse = "Example format \"";
		errorResponse += "BurstTest";
		g_theDevConsole->AddLine(DevConsole::WARNING, errorResponse);
		return false;
	}

	for (int index = 1; index <= 20; index++)
	{
		std::string command;
		command = "Echo Message=\"";
		command += std::to_string(index) + "\"";
		g_theNetSystem->m_sendQueue.push_back(command);
	}
	return true;
}

#endif