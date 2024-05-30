#pragma once
#include "Engine/Core/ErrorWarningAssert.hpp"
#include "HashedCaseInsensitiveString.hpp"
#include <map>
#include <string>

//-----------------------------------------------------------------------------------------------
class NamedPropertyBase
{
public:
	NamedPropertyBase(){}
	virtual ~NamedPropertyBase() {}
};

//-----------------------------------------------------------------------------------------------
template<typename T>
class NamedPropertyofType : public NamedPropertyBase
{
	friend class NamedProperties;

protected:
	NamedPropertyofType(T value) :m_value(value) {}
	T m_value;
};

//-----------------------------------------------------------------------------------------------
class NamedProperties
{
public:
	template<typename T>
	void SetValue(std::string const& keyName, T const& value);
	template<typename T>
	T GetValue(std::string const& keyName, T const& defaultValue);
	void SetValue(std::string const& keyName, const char* value);
	std::string GetValue(std::string const& keyName, const char* defaultValue);

private:
	std::map< HCIString, NamedPropertyBase*> m_keyValuePairs;
};

//c++-style
//-----------------------------------------------------------------------------------------------
template<typename T>
void NamedProperties::SetValue(std::string const& keyName, T const& value)
{
	HCIString key(keyName);
	m_keyValuePairs[key] = new NamedPropertyofType<T>(value);
}

//-----------------------------------------------------------------------------------------------
template<typename T>
inline T NamedProperties::GetValue(std::string const& keyName, T const& defaultValue)
{
	HCIString key(keyName);
	std::map<HCIString, NamedPropertyBase*>::iterator found = m_keyValuePairs.find(key);
	if (found == m_keyValuePairs.end())
		return defaultValue;

	NamedPropertyBase* property = found->second;
	NamedPropertyofType<T>* typedProperty = dynamic_cast<NamedPropertyofType<T>*>(property);
	if (typedProperty == nullptr)
	{
		ERROR_RECOVERABLE("Asked for value of the incorrect type!");
		return defaultValue;
	}

	return typedProperty->m_value;
}

//-----------------------------------------------------------------------------------------------
void NamedProperties::SetValue(std::string const& keyName, const char* value)
{
	HCIString key(keyName);
	m_keyValuePairs[key] = new NamedPropertyofType<std::string>(value);
}

//-----------------------------------------------------------------------------------------------
inline std::string NamedProperties::GetValue(std::string const& keyName, const char* defaultValue)
{
	HCIString key(keyName);
	std::map<HCIString, NamedPropertyBase*>::iterator found = m_keyValuePairs.find(key);
	if (found == m_keyValuePairs.end())
		return defaultValue;

	NamedPropertyBase* property = found->second;
	NamedPropertyofType<std::string>* typedProperty = dynamic_cast<NamedPropertyofType<std::string>*>(property);
	if (typedProperty == nullptr)
	{
		ERROR_RECOVERABLE("Asked for value of the incorrect type!");
		return defaultValue;
	}

	return typedProperty->m_value;
}
