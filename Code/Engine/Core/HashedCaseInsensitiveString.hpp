#pragma once
#include <string>

//-----------------------------------------------------------------------------------------------
class HashCaseInsensitiveString
{
public:
	HashCaseInsensitiveString() = default;
	HashCaseInsensitiveString( HashCaseInsensitiveString const& copyFrom ) = default;
	HashCaseInsensitiveString(const char* originalText);
	HashCaseInsensitiveString(std::string const& originalText);

	unsigned int		GetHash() const;
	std::string const&	GetOriginalString() const;
	char const*			c_str() const;

	static unsigned int CalculateHashForText(const char* text);
	static unsigned int CalculateHashForText(std::string const& text);

	bool				operator<(HashCaseInsensitiveString const& compareHCIS) const;
	bool				operator>(HashCaseInsensitiveString const& compareHCIS) const;
	bool				operator==(HashCaseInsensitiveString const& compareHCIS) const;
	bool				operator!=(HashCaseInsensitiveString const& compareHCIS) const;
	bool				operator==(std::string const& compareText) const;
	bool				operator!=(std::string const& compareText) const;
	bool				operator==(const char* compareText) const;
	bool				operator!=(const char* compareText) const;
	void				operator=(HashCaseInsensitiveString const& assignFrom);
	void				operator=(std::string const& text);
	void				operator=(const char* text);

private:
	std::string			m_caseIntactText = "";
	unsigned int		m_lowerCaseHash = 0;
};

//-----------------------------------------------------------------------------------------------
typedef HashCaseInsensitiveString HCIString;