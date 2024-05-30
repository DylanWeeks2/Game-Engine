#include "NamedStrings.hpp"
#include "Engine/Core/XmlUtils.hpp"
#include "Engine/Core/Rgba8.hpp"
#include "Engine/Math/Vec2.hpp"
#include "Engine/Math/IntVec2.hpp"
#include "Engine/Math/Vec3.hpp"

//-----------------------------------------------------------------------------------------------
void NamedStrings::PopulateFromXmlElementAttributes(XmlElement const& element)
{
	int loop = 0;
	XmlAttribute const* firstAttribute = element.FirstAttribute();

	auto iterator = m_keyValuePairs.find(firstAttribute->Name());
	if (iterator != m_keyValuePairs.end())
	{
		m_keyValuePairs[firstAttribute->Name()] = firstAttribute->Value();
	}
	else 
	{
		m_keyValuePairs.insert(std::pair<std::string, std::string>(firstAttribute->Name(), firstAttribute->Value()));
	}
	
	while (loop == 0)
	{
		XmlAttribute const* nextAttribute = firstAttribute->Next();
		
		if (nextAttribute == nullptr)
		{
			break;
		}

		auto iterator2 = m_keyValuePairs.find(nextAttribute->Name());
		if (iterator2 != m_keyValuePairs.end())
		{
			m_keyValuePairs[nextAttribute->Name()] = nextAttribute->Value();
		}
		else
		{
			m_keyValuePairs.insert(std::pair<std::string, std::string>(nextAttribute->Name(), nextAttribute->Value()));
		}
		firstAttribute = nextAttribute;
	}
}

//-----------------------------------------------------------------------------------------------
void NamedStrings::SetValue(HCIString const& keyName, std::string const& newValue)
{
	auto found = m_keyValuePairs.find(keyName);
	if (found != m_keyValuePairs.end())
	{
		found->second = newValue;
		return;
	}

	m_keyValuePairs.insert(std::pair<HCIString, std::string>(keyName, newValue));
}

//-----------------------------------------------------------------------------------------------
std::string NamedStrings::GetValue(HCIString const& keyName, std::string const& defaultValue) const
{
	std::string returnValue = defaultValue;
	auto found = m_keyValuePairs.find(keyName);
	if (found != m_keyValuePairs.end())
	{
		returnValue = found->second.c_str();
	}

	return returnValue;
}

//-----------------------------------------------------------------------------------------------
bool NamedStrings::GetValue(HCIString const& keyName, bool defaultValue) const
{
	bool returnValue = defaultValue;
	auto found = m_keyValuePairs.find(keyName);
	if (found != m_keyValuePairs.end())
	{
		std::string result = found->second.c_str();
		if (result == "true" || result == "True" || result == "TRUE")
		{
			returnValue = true;
		}
		else
		{
			returnValue = false;
		}
	}

	return returnValue;
}

//-----------------------------------------------------------------------------------------------
int NamedStrings::GetValue(HCIString const& keyName, int defaultValue) const
{
	int returnValue = defaultValue;
	auto found = m_keyValuePairs.find(keyName);
	if (found != m_keyValuePairs.end())
	{
		returnValue = atoi(found->second.c_str());
	}

	return returnValue;
}

//-----------------------------------------------------------------------------------------------
float NamedStrings::GetValue(HCIString const& keyName, float defaultValue) const
{
	float returnValue = defaultValue;
	auto found = m_keyValuePairs.find(keyName);
	if (found != m_keyValuePairs.end())
	{
		returnValue = static_cast<float>(atof(found->second.c_str()));
	}

	return returnValue;
}

//-----------------------------------------------------------------------------------------------
std::string NamedStrings::GetValue(HCIString const& keyName, char const* defaultValue) const
{
	std::string returnValue = defaultValue;
	auto found = m_keyValuePairs.find(keyName);
	if (found != m_keyValuePairs.end())
	{
		returnValue = found->second.c_str();
	}

	return returnValue;
}

//-----------------------------------------------------------------------------------------------
Rgba8 NamedStrings::GetValue(HCIString const& keyName, Rgba8 const& defaultValue) const
{
	Rgba8 returnValue = defaultValue;
	auto found = m_keyValuePairs.find(keyName);

	if (found != m_keyValuePairs.end())
	{
		returnValue.SetFromText(found->second.c_str());
	}

	return returnValue;
}

//-----------------------------------------------------------------------------------------------
Vec2 NamedStrings::GetValue(HCIString const& keyName, Vec2 const& defaultValue) const
{
	Vec2 returnValue = defaultValue;
	auto found = m_keyValuePairs.find(keyName);

	if (found != m_keyValuePairs.end())
	{
		returnValue.SetFromText(found->second.c_str());
	}

	return returnValue;
}

//-----------------------------------------------------------------------------------------------
IntVec2 NamedStrings::GetValue(HCIString const& keyName, IntVec2 const& defaultValue) const
{
	IntVec2 returnValue = defaultValue;
	auto found = m_keyValuePairs.find(keyName);

	if (found != m_keyValuePairs.end())
	{
		returnValue.SetFromText(found->second.c_str());
	}

	return returnValue;
}

//-----------------------------------------------------------------------------------------------
Vec3 NamedStrings::GetValue(HCIString const& keyName, Vec3 const& defaultValue) const
{
	Vec3 returnValue = defaultValue;
	auto found = m_keyValuePairs.find(keyName);

	if (found != m_keyValuePairs.end())
	{
		returnValue.SetFromText(found->second.c_str());
	}

	return returnValue;
}
