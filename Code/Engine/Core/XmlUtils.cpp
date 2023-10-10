#include "XmlUtils.hpp"
#include "Engine/Core/Rgba8.hpp"
#include "Engine/Math/Vec2.hpp"
#include "Engine/Math/Vec3.hpp"
#include "Engine/Math/IntVec2.hpp"
#include "Engine/Math/EulerAngles.hpp"
#include "Engine/Math/FloatRange.hpp"
#include "Engine/Renderer/SpriteAnimDefinition.hpp"

//-----------------------------------------------------------------------------------------------
int ParseXmlAttribute(XmlElement const& element, char const* attributeName, int defaultValue)
{
	char const* valueAsText = element.Attribute(attributeName);
	int returnValue = defaultValue;
	if (valueAsText)
	{
		returnValue = atoi(valueAsText);
	}

	return returnValue;
}

//-----------------------------------------------------------------------------------------------
char ParseXmlAttribute(XmlElement const& element, char const* attributeName, char defaultValue)
{
	char const* valueAsText = element.Attribute(attributeName);
	int returnValue = defaultValue;
	if (valueAsText)
	{
		returnValue = atoi(valueAsText);
	}

	return static_cast<char>(returnValue);
}

//-----------------------------------------------------------------------------------------------
bool ParseXmlAttribute(XmlElement const& element, char const* attributeName, bool defaultValue)
{
	char const* valueAsText = element.Attribute(attributeName);
	int returnValue = defaultValue;
	if (valueAsText)
	{
		std::string result = valueAsText;
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
float ParseXmlAttribute(XmlElement const& element, char const* attributeName, float defaultValue)
{
	char const* valueAsText = element.Attribute(attributeName);
	float returnValue = defaultValue;
	if (valueAsText)
	{
		returnValue = static_cast<float>(atof(valueAsText));
	}

	return returnValue;
}

//-----------------------------------------------------------------------------------------------
Vec2 ParseXmlAttribute(XmlElement const& element, char const* attributeName, Vec2 const& defaultValue)
{
	char const* valueAsText = element.Attribute(attributeName);
	Vec2 returnValue = defaultValue;
	if (valueAsText)
	{
		returnValue.SetFromText(valueAsText);
	}

	return returnValue;
}

//-----------------------------------------------------------------------------------------------
IntVec2 ParseXmlAttribute(XmlElement const& element, char const* attributeName, IntVec2 const& defaultValue)
{
	char const* valueAsText = element.Attribute(attributeName);
	IntVec2 returnValue = defaultValue;
	if (valueAsText)
	{
		returnValue.SetFromText(valueAsText);
	}

	return returnValue;
}

//-----------------------------------------------------------------------------------------------
std::string ParseXmlAttribute(XmlElement const& element, char const* attributeName, std::string const& defaultValue)
{
	char const* valueAsText = element.Attribute(attributeName);
	std::string returnValue = defaultValue;
	if (valueAsText)
	{
		returnValue = valueAsText;
	}

	return returnValue;
}

//-----------------------------------------------------------------------------------------------
Strings ParseXmlAttribute(XmlElement const& element, char const* attributeName, Strings const& defaultValues)
{
	char const* valueAsText = element.Attribute(attributeName);
	Strings returnValue = defaultValues;
	if (valueAsText)
	{
		returnValue = SplitStringOnDelimiter(valueAsText, ',');
	}

	return returnValue;
}

//-----------------------------------------------------------------------------------------------
std::string ParseXmlAttribute(XmlElement const& element, char const* attributeName, char const* defaultValue)
{
	char const* valueAsText = element.Attribute(attributeName);
	std::string returnValue = defaultValue;
	if (valueAsText)
	{
		returnValue = valueAsText;
	}

	return returnValue;
}

//-----------------------------------------------------------------------------------------------
SpriteAnimDefinition ParseXmlAttribute(XmlElement const& element, char const* attributeName, SpriteAnimDefinition defaultValue)
{
	char const* valueAsText = element.Attribute(attributeName);
	SpriteAnimDefinition returnValue = defaultValue;
	if (valueAsText)
	{
		returnValue.SetFromText(valueAsText);
	}

	return returnValue;
}

//-----------------------------------------------------------------------------------------------
Rgba8 ParseXmlAttribute(XmlElement const& element, char const* attributeName, Rgba8 const& defaultValue)
{
	char const* valueAsText = element.Attribute(attributeName);
	Rgba8 color = defaultValue;
	if (valueAsText)
	{
		color.SetFromText(valueAsText);
	}

	return color;
}

//-----------------------------------------------------------------------------------------------
Vec3 ParseXmlAttribute(XmlElement const& element, char const* attributeName, Vec3 const& defaultValue)
{
	char const* valueAsText = element.Attribute(attributeName);
	Vec3 returnValue = defaultValue;
	if (valueAsText)
	{
		returnValue.SetFromText(valueAsText);
	}

	return returnValue;
}

//-----------------------------------------------------------------------------------------------
EulerAngles ParseXmlAttribute(XmlElement const& element, char const* attributeName, EulerAngles const& defaultValue)
{
	char const* valueAsText = element.Attribute(attributeName);
	EulerAngles returnValue = defaultValue;
	if (valueAsText)
	{
		returnValue.SetFromText(valueAsText);
	}

	return returnValue;
}

//-----------------------------------------------------------------------------------------------
FloatRange ParseXmlAttribute(XmlElement const& element, char const* attributeName, FloatRange const& defaultValue)
{
	char const* valueAsText = element.Attribute(attributeName);
	FloatRange returnValue = defaultValue;
	if (valueAsText)
	{
		returnValue.SetFromText(valueAsText);
	}

	return returnValue;
}
