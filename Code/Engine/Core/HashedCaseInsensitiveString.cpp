#include "HashedCaseInsensitiveString.hpp"

//-----------------------------------------------------------------------------------------------
HashCaseInsensitiveString::HashCaseInsensitiveString(const char* originalText)
	:m_caseIntactText(originalText)
	,m_lowerCaseHash(CalculateHashForText(originalText))
{
}

//-----------------------------------------------------------------------------------------------
HashCaseInsensitiveString::HashCaseInsensitiveString(std::string const& originalText)
	:m_caseIntactText(originalText)
	,m_lowerCaseHash(CalculateHashForText(originalText))
{
}

//-----------------------------------------------------------------------------------------------
unsigned int HashCaseInsensitiveString::GetHash() const
{
	return m_lowerCaseHash;
}

//-----------------------------------------------------------------------------------------------
std::string const& HashCaseInsensitiveString::GetOriginalString() const
{
	return m_caseIntactText;
}

//-----------------------------------------------------------------------------------------------
char const* HashCaseInsensitiveString::c_str() const
{
	return m_caseIntactText.c_str();
}

//-----------------------------------------------------------------------------------------------
unsigned int HashCaseInsensitiveString::CalculateHashForText(const char* text)
{
	unsigned int hash = 0;
	const char* readPos = text;
	while (*readPos != '\0')
	{
		hash *= 31;
		hash += (unsigned int) tolower(*readPos);
		readPos++;
	}

	return hash;
}

//-----------------------------------------------------------------------------------------------
unsigned int HashCaseInsensitiveString::CalculateHashForText(std::string const& text)
{
	return CalculateHashForText(text.c_str());
}

//-----------------------------------------------------------------------------------------------
bool HashCaseInsensitiveString::operator<(HashCaseInsensitiveString const& compareHCIS) const
{
	if (m_lowerCaseHash < compareHCIS.m_lowerCaseHash)
	{
		return true;
	}
	else if (m_lowerCaseHash > compareHCIS.m_lowerCaseHash)
	{
		return false;
	}
	else
	{
		//Hash values are equal now need to do actual string comparison
		return _stricmp(m_caseIntactText.c_str(), compareHCIS.m_caseIntactText.c_str()) < 0;
	}
}

//-----------------------------------------------------------------------------------------------
bool HashCaseInsensitiveString::operator>(HashCaseInsensitiveString const& compareHCIS) const
{
	if (m_lowerCaseHash > compareHCIS.m_lowerCaseHash)
	{
		return true;
	}
	else if (m_lowerCaseHash < compareHCIS.m_lowerCaseHash)
	{
		return false;
	}
	else
	{
		//Hash values are equal now need to do actual string comparison
		return _stricmp(m_caseIntactText.c_str(), compareHCIS.m_caseIntactText.c_str()) > 0;
	}
}

//-----------------------------------------------------------------------------------------------
bool HashCaseInsensitiveString::operator==(HashCaseInsensitiveString const& compareHCIS) const
{
	if (m_lowerCaseHash != compareHCIS.m_lowerCaseHash)
	{
		return false;
	}
	else
	{
		//Hash values are equal now need to do actual string comparison
		return _stricmp(m_caseIntactText.c_str(), compareHCIS.m_caseIntactText.c_str()) == 0;
	}
}

//-----------------------------------------------------------------------------------------------
bool HashCaseInsensitiveString::operator!=(HashCaseInsensitiveString const& compareHCIS) const
{
	if (m_lowerCaseHash != compareHCIS.m_lowerCaseHash)
	{
		return true;
	}
	else
	{
		//Hash values are equal now need to do actual string comparison
		return _stricmp(m_caseIntactText.c_str(), compareHCIS.m_caseIntactText.c_str()) != 0;
	}
}

//-----------------------------------------------------------------------------------------------
bool HashCaseInsensitiveString::operator==(std::string const& compareText) const
{
	return _stricmp(m_caseIntactText.c_str(), compareText.c_str()) == 0;
}

//-----------------------------------------------------------------------------------------------
bool HashCaseInsensitiveString::operator!=(std::string const& compareText) const
{
	return _stricmp(m_caseIntactText.c_str(), compareText.c_str()) != 0;
}

//-----------------------------------------------------------------------------------------------
bool HashCaseInsensitiveString::operator==(const char* compareText) const
{
	return _stricmp(m_caseIntactText.c_str(), compareText) == 0;
}

//-----------------------------------------------------------------------------------------------
bool HashCaseInsensitiveString::operator!=(const char* compareText) const
{
	return _stricmp(m_caseIntactText.c_str(), compareText) != 0;
}

//-----------------------------------------------------------------------------------------------
void HashCaseInsensitiveString::operator=(HashCaseInsensitiveString const& assignFrom)
{
	m_caseIntactText = assignFrom.m_caseIntactText;
	m_lowerCaseHash = assignFrom.m_lowerCaseHash;
}

//-----------------------------------------------------------------------------------------------
void HashCaseInsensitiveString::operator=(std::string const& text)
{
	m_caseIntactText = text;
	m_lowerCaseHash = CalculateHashForText(m_caseIntactText);
}

//-----------------------------------------------------------------------------------------------
void HashCaseInsensitiveString::operator=(const char* text)
{
	m_caseIntactText = text;
	m_lowerCaseHash = CalculateHashForText(m_caseIntactText);
}
