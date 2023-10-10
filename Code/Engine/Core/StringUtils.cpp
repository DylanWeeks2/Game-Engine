#include "Engine/Core/StringUtils.hpp"
#include <stdarg.h>
#include "StringUtils.hpp"


//-----------------------------------------------------------------------------------------------
constexpr int STRINGF_STACK_LOCAL_TEMP_LENGTH = 2048;


//-----------------------------------------------------------------------------------------------
const std::string Stringf( char const* format, ... )
{
	char textLiteral[ STRINGF_STACK_LOCAL_TEMP_LENGTH ];
	va_list variableArgumentList;
	va_start( variableArgumentList, format );
	vsnprintf_s( textLiteral, STRINGF_STACK_LOCAL_TEMP_LENGTH, _TRUNCATE, format, variableArgumentList );	
	va_end( variableArgumentList );
	textLiteral[ STRINGF_STACK_LOCAL_TEMP_LENGTH - 1 ] = '\0'; // In case vsnprintf overran (doesn't auto-terminate)

	return std::string( textLiteral );
}

//-----------------------------------------------------------------------------------------------
const std::string Stringf( int maxLength, char const* format, ... )
{
	char textLiteralSmall[ STRINGF_STACK_LOCAL_TEMP_LENGTH ];
	char* textLiteral = textLiteralSmall;
	if( maxLength > STRINGF_STACK_LOCAL_TEMP_LENGTH )
		textLiteral = new char[ maxLength ];

	va_list variableArgumentList;
	va_start( variableArgumentList, format );
	vsnprintf_s( textLiteral, maxLength, _TRUNCATE, format, variableArgumentList );	
	va_end( variableArgumentList );
	textLiteral[ maxLength - 1 ] = '\0'; // In case vsnprintf overran (doesn't auto-terminate)

	std::string returnValue( textLiteral );
	if( maxLength > STRINGF_STACK_LOCAL_TEMP_LENGTH )
		delete[] textLiteral;

	return returnValue;
}

//-----------------------------------------------------------------------------------------------
Strings SplitStringOnDelimiter(std::string const& originalString, char delimiterToSplitOn)
{
	Strings returnStrings;
	int startStringPos = 0;
	int endStringPos = static_cast<int>(originalString.find(delimiterToSplitOn));

	while (endStringPos != -1)
	{
		std::string splitString= originalString.substr(startStringPos, endStringPos - startStringPos);
		returnStrings.push_back(splitString);
		startStringPos = endStringPos + 1;
		endStringPos = static_cast<int>(originalString.find(delimiterToSplitOn, startStringPos));
	}

	std::string splitString = originalString.substr(startStringPos, endStringPos - startStringPos);
	returnStrings.push_back(splitString);

	return returnStrings;
}

//-----------------------------------------------------------------------------------------------
Strings SplitStringWithQuotes(std::string const& originalString, char delimiterToSplitOn)
{
	Strings returnStrings;
	int startStringPos = 0;
	int endStringPos = int(originalString.length());

	int lastEndQuotePos = int(originalString.length());
	bool isQuotes = false;
	for (int stringIndex = 0; stringIndex < originalString.length(); stringIndex++)
	{
		if (originalString[stringIndex] == '\"')
		{
			isQuotes = !isQuotes;
		}
		if (isQuotes == false)
		{
			if (originalString[stringIndex] == delimiterToSplitOn)
			{
				std::string splitString = originalString.substr(startStringPos, stringIndex - startStringPos);
				returnStrings.push_back(splitString);
				startStringPos = stringIndex + 1;
			}
		}
		else
		{
			for (int quoteIndex = lastEndQuotePos; quoteIndex > 0; quoteIndex--)
			{
				if (originalString[quoteIndex] == '\"')
				{
					std::string splitString = originalString.substr(startStringPos, quoteIndex + 1 - (startStringPos));
					returnStrings.push_back(splitString);
					startStringPos = quoteIndex;
					lastEndQuotePos = quoteIndex;
					stringIndex = quoteIndex;
					break;
				}
			}
		}
	}

	std::string splitString = originalString.substr(startStringPos, endStringPos - startStringPos);
	if (splitString != "\"")
	{
		returnStrings.push_back(splitString);
	}
	return returnStrings;
}

//-----------------------------------------------------------------------------------------------
void TrimString(std::string& originalString, char delimiterToTrim)
{
	for (int stringIndex = 0; stringIndex < originalString.length(); stringIndex++)
	{
		if (originalString[stringIndex] == delimiterToTrim)
		{
			originalString.erase(stringIndex, 1);
			break;
		}
	}

	for (int stringIndex = int(originalString.length()) - 1; stringIndex > 0; stringIndex--)
	{
		if (originalString[stringIndex] == delimiterToTrim)
		{
			originalString.erase(stringIndex, 1);
			break;
		}
	}
}
