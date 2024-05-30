#pragma once
#include "NamedStrings.hpp"
#include <vector>
#include <string>
#include <map>
#include <mutex>

//-----------------------------------------------------------------------------------------------
typedef NamedStrings EventArgs;
typedef bool (*EventCallbackFuncPtr)(EventArgs& eventArgs);

//-----------------------------------------------------------------------------------------------
struct EventSystemConfig
{
};

//-----------------------------------------------------------------------------------------------
class EventSubscriptionBase
{
	friend class EventSystem;

protected:
	EventSubscriptionBase() = default;
	virtual ~EventSubscriptionBase() = default;
	virtual bool Execute(EventArgs args) = 0;

protected:
};

//-----------------------------------------------------------------------------------------------
class EventSubscription_Function: public EventSubscriptionBase
{
	friend class EventSystem;

protected:
	EventSubscription_Function(EventCallbackFuncPtr funcPtr) :m_funcPtr(funcPtr) {}
	virtual ~EventSubscription_Function() = default; 
	virtual bool Execute(EventArgs eventArgs) override { return m_funcPtr(eventArgs); }

protected:
	EventCallbackFuncPtr m_funcPtr = nullptr;
};

//-----------------------------------------------------------------------------------------------
template<typename T> 
class EventSubscription_ObjectMethod : public EventSubscriptionBase
{
	friend class EventSystem;
	typedef bool(T::* EventCallBackObjectMethod) (EventArgs& eventArgs);

protected:
	EventSubscription_ObjectMethod(T& object, EventCallBackObjectMethod objectMethod) :m_object(object), m_objectMethod(objectMethod) {}
	virtual ~EventSubscription_ObjectMethod() = default;
	virtual bool Execute(EventArgs eventArgs) override { return (m_object.*m_objectMethod)(eventArgs); }

protected:
	EventCallBackObjectMethod	m_objectMethod = nullptr;
	T&							m_object;
};

//-----------------------------------------------------------------------------------------------
typedef std::vector<EventSubscriptionBase*> SubscriptionList;

//-----------------------------------------------------------------------------------------------
class EventSystem
{
public:
	EventSystem(EventSystemConfig const& config);
	~EventSystem();
	void					Startup();
	void					Shutdown();
	void					BeginFrame();
	void					EndFrame();

	template<typename T>
	void					SubscribeEventCallbackObjectMethod(HCIString const& eventName, T& object, bool(T::*method)(EventArgs& eventArgs));
	template<typename T>
	void					UnsubscribeEventCallbackObjectMethod(HCIString const& eventName, T& object, bool(T::* method)(EventArgs& eventArgs));

	void					SubscribeEventCallbackFunciton(HCIString const& eventName, EventCallbackFuncPtr functionPtr);
	void					UnsubscribeEventCallbackFunciton(HCIString const& eventName, EventCallbackFuncPtr functionPtr);
	void					UnsubscribeFromAllEvents(EventCallbackFuncPtr functionPtr);
	void					FireEvent(HCIString const& eventName, EventArgs& eventArgs);
	void					FireEvent(HCIString const& eventName);
	std::vector<HCIString>	GetRegisteredCommands();

protected:
	std::map<HCIString, SubscriptionList>	m_subscriptionListsByName;
	std::mutex								m_eventSystemMutex;
	EventSystemConfig						m_config;
};

//-----------------------------------------------------------------------------------------------
extern EventSystem* g_theEventSystem;

//-----------------------------------------------------------------------------------------------
template<typename T>
inline void EventSystem::SubscribeEventCallbackObjectMethod(HCIString const& eventName, T& object, bool(T::* method)(EventArgs& eventArgs))
{
	m_eventSystemMutex.lock();
	SubscriptionList& subscribersForThisEvent = m_subscriptionListsByName[eventName];
	EventSubscription_ObjectMethod<T>* subscriber = new EventSubscription_ObjectMethod<T>(object, method);
	subscribersForThisEvent.push_back(subscriber);
	m_eventSystemMutex.unlock();
}

//-----------------------------------------------------------------------------------------------
template<typename T>
inline void EventSystem::UnsubscribeEventCallbackObjectMethod(HCIString const& eventName, T& object, bool(T::* method)(EventArgs& eventArgs))
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
		EventSubscription_ObjectMethod<T>* objectMethodSubscriber = dynamic_cast<EventSubscription_ObjectMethod<T>*>(subscriber);
		if (objectMethodSubscriber && objectMethodSubscriber->m_objectMethod == method)
		{
			delete subscriber;
			subscribersForThisEvent[index] = nullptr;
		}
	}
	m_eventSystemMutex.unlock();
}
