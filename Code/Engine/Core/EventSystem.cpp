#include "EventSystem.hpp"
#include "NamedStrings.hpp"
#include "DevConsole.hpp"
#include <string>

//-----------------------------------------------------------------------------------------------
EventSystem::EventSystem(EventSystemConfig const& config)
	:m_config(config)
{
}

//-----------------------------------------------------------------------------------------------
EventSystem::~EventSystem()
{
}

//-----------------------------------------------------------------------------------------------
void EventSystem::Startup()
{
}

//-----------------------------------------------------------------------------------------------
void EventSystem::Shutdown()
{
}

//-----------------------------------------------------------------------------------------------
void EventSystem::BeginFrame()
{
}

//-----------------------------------------------------------------------------------------------
void EventSystem::EndFrame()
{
}

//-----------------------------------------------------------------------------------------------
void EventSystem::SubscribeEventCallbackFunciton(std::string const& eventName, EventCallbackFuncPtr functionPtr)
{
	m_eventSystemMutex.lock();
	SubscriptionList& subscribersForThisEvent = m_subscriptionListsByName[eventName];
	subscribersForThisEvent.push_back(functionPtr);
	m_eventSystemMutex.unlock();
}

//-----------------------------------------------------------------------------------------------
void EventSystem::UnsubscribeEventCallbackFunciton(std::string const& eventName, EventCallbackFuncPtr functionPtr)
{
	m_eventSystemMutex.lock();
	SubscriptionList& subscribersForThisEvent = m_subscriptionListsByName[eventName];
	for (int subscriberIndex = 0; subscriberIndex < subscribersForThisEvent.size(); subscriberIndex++)
	{
		if (subscribersForThisEvent[subscriberIndex] == functionPtr)
		{
			subscribersForThisEvent[subscriberIndex] = nullptr;
		}
	}
	m_eventSystemMutex.unlock();
}

//-----------------------------------------------------------------------------------------------
void EventSystem::UnsubscribeFromAllEvents(EventCallbackFuncPtr functionPtr)
{
	m_eventSystemMutex.lock();
	for (auto eventItr = m_subscriptionListsByName.begin(); eventItr != m_subscriptionListsByName.end(); eventItr++)
	{
		std::string const& eventName = eventItr->first;
		m_eventSystemMutex.unlock();
		UnsubscribeEventCallbackFunciton(eventName, functionPtr);
		m_eventSystemMutex.lock();
	}
	m_eventSystemMutex.unlock();
}

//-----------------------------------------------------------------------------------------------
void EventSystem::FireEvent(std::string const& eventName, EventArgs& eventArgs)
{
	//Bad Dev Console Command
	m_eventSystemMutex.lock();
	if (m_subscriptionListsByName.count(eventName) == 0)
	{
		eventArgs.SetValue("returnValue", "failure");
		m_eventSystemMutex.unlock();
		return;
	}

	SubscriptionList& subscribersForThisEvent = m_subscriptionListsByName.at(eventName);
	for (int subscriberIndex = 0; subscriberIndex < subscribersForThisEvent.size(); subscriberIndex++)
	{
		EventCallbackFuncPtr functionPtr = subscribersForThisEvent[subscriberIndex];
		if (functionPtr != nullptr)
		{
			m_eventSystemMutex.unlock();
			bool wasConsumed = functionPtr(eventArgs);
			m_eventSystemMutex.lock();
			if (wasConsumed == true)
			{
				eventArgs.SetValue("returnValue", "success");
				break; //event was consumed by this subscriber so no other subscribers will be called
			}
		}
	}
	m_eventSystemMutex.unlock();
}

//-----------------------------------------------------------------------------------------------
void EventSystem::FireEvent(std::string const& eventName)
{
	EventArgs emptyArgs;
	FireEvent(eventName, emptyArgs);
}

//-----------------------------------------------------------------------------------------------
std::vector<std::string> EventSystem::GetRegisteredCommands()
{
	std::vector<std::string> registerCommands;


	m_eventSystemMutex.lock();
	for (const auto& subscriptionLists : m_subscriptionListsByName)
	{
		registerCommands.push_back(subscriptionLists.first);
	}
	m_eventSystemMutex.unlock();
	
	return registerCommands;
}
