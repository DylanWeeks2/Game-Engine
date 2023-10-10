#pragma once
#include <vector>
#include <string>
#include <map>
#include <mutex>

class NamedStrings;

struct EventSystemConfig
{
};

typedef NamedStrings EventArgs;
typedef bool (*EventCallbackFuncPtr)(EventArgs& eventArgs);
typedef std::vector<EventCallbackFuncPtr> SubscriptionList;

class EventSystem
{
public:
	EventSystem(EventSystemConfig const& config);
	~EventSystem();
	void Startup();
	void Shutdown();
	void BeginFrame();
	void EndFrame();

	void SubscribeEventCallbackFunciton(std::string const& eventName, EventCallbackFuncPtr functionPtr);
	void UnsubscribeEventCallbackFunciton(std::string const& eventName, EventCallbackFuncPtr functionPtr);
	void UnsubscribeFromAllEvents(EventCallbackFuncPtr functionPtr);
	void FireEvent(std::string const& eventName, EventArgs& eventArgs);
	void FireEvent(std::string const& eventName);
	std::vector<std::string> GetRegisteredCommands();

protected:
	EventSystemConfig m_config;
	std::map<std::string, SubscriptionList> m_subscriptionListsByName;
	std::mutex m_eventSystemMutex;
};

extern EventSystem* g_theEventSystem;