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
void EventSystem::SubscribeEventCallbackFunciton(HCIString const& eventName, EventCallbackFuncPtr functionPtr)
{
	m_eventSystemMutex.lock();
	SubscriptionList& subscribersForThisEvent = m_subscriptionListsByName[eventName];
	EventSubscription_Function* subscriber = new EventSubscription_Function(functionPtr);
	subscribersForThisEvent.push_back(subscriber);
	m_eventSystemMutex.unlock();
}

//-----------------------------------------------------------------------------------------------
void EventSystem::UnsubscribeEventCallbackFunciton(HCIString const& eventName, EventCallbackFuncPtr functionPtr)
{
	m_eventSystemMutex.lock();
	auto found = m_subscriptionListsByName.find(eventName);
	if (found == m_subscriptionListsByName.end())
	{
		m_eventSystemMutex.unlock();
		return;
	}

	SubscriptionList& subscribersForThisEvent = found->second;
	for (int index = 0; index < subscribersForThisEvent.size(); index++)
	{
		EventSubscriptionBase* subscriber = subscribersForThisEvent[index];
		EventSubscription_Function* funcPtrSubscriber = dynamic_cast<EventSubscription_Function*>(subscriber);
		if (funcPtrSubscriber && funcPtrSubscriber->m_funcPtr == functionPtr)
		{
			delete subscriber;
			subscribersForThisEvent[index] = nullptr;
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
		HCIString const& eventName = eventItr->first;
		m_eventSystemMutex.unlock();
		UnsubscribeEventCallbackFunciton(eventName, functionPtr);
		m_eventSystemMutex.lock();
	}
	m_eventSystemMutex.unlock();
}

//-----------------------------------------------------------------------------------------------
void EventSystem::FireEvent(HCIString const& eventName, EventArgs& eventArgs)
{
	m_eventSystemMutex.lock();
	auto found = m_subscriptionListsByName.find(eventName);
	if (found == m_subscriptionListsByName.end())
	{
		eventArgs.SetValue("returnValue", "failure");
		m_eventSystemMutex.unlock();
		return;
	}


	SubscriptionList& subscribersForThisEvent = found->second;
	for (int index = 0; index < subscribersForThisEvent.size(); index++)
	{
		EventSubscriptionBase* subscriber = subscribersForThisEvent[index];
		if (subscriber)
		{
			m_eventSystemMutex.unlock();
			bool wasConsumed = subscriber->Execute(eventArgs); 
			m_eventSystemMutex.lock();
			if (wasConsumed)
			{
				eventArgs.SetValue("returnValue", "success");
				break; //event was consumed by this subscriber so no other subscribers will be called
			}
		}
		
	}
	m_eventSystemMutex.unlock();
}

//-----------------------------------------------------------------------------------------------
void EventSystem::FireEvent(HCIString const& eventName)
{
	EventArgs emptyArgs;
	FireEvent(eventName, emptyArgs);
}

//-----------------------------------------------------------------------------------------------
std::vector<HCIString> EventSystem::GetRegisteredCommands()
{
	std::vector<HCIString> registerCommands;


	m_eventSystemMutex.lock();
	for (const auto& subscriptionLists : m_subscriptionListsByName)
	{
		registerCommands.push_back(subscriptionLists.first);
	}
	m_eventSystemMutex.unlock();
	
	return registerCommands;
}
