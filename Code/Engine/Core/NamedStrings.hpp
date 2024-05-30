#pragma once
#include <map>
#include <string>
#include "XmlUtils.hpp"
#include "HashedCaseInsensitiveString.hpp"

//-----------------------------------------------------------------------------------------------
struct Rgba8;
struct Vec2;
struct Vec3;
struct IntVec2;

//-----------------------------------------------------------------------------------------------
class NamedStrings 
{
public:
	void			PopulateFromXmlElementAttributes(XmlElement const& element);
	void			SetValue(HCIString const& keyName, std::string const& newValue);
	std::string		GetValue(HCIString const& keyName, std::string const& defaultValue) const;
	bool			GetValue(HCIString const& keyName, bool defaultValue) const;
	int				GetValue(HCIString const& keyName, int defaultValue) const;
	float			GetValue(HCIString const& keyName, float defaultValue) const;
	std::string		GetValue(HCIString const& keyName, char const* defaultValue) const;
	Rgba8			GetValue(HCIString const& keyName, Rgba8 const& defaultValue) const;
	Vec2			GetValue(HCIString const& keyName, Vec2 const& defaultValue) const;
	IntVec2			GetValue(HCIString const& keyName, IntVec2 const& defaultValue) const; 
	Vec3			GetValue(HCIString const& keyName, Vec3 const& defaultValue) const;


private:
	std::map< HCIString, std::string > m_keyValuePairs;
};