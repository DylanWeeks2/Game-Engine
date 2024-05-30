#include "FileUtils.hpp"
#include "ErrorWarningAssert.hpp"
#include "Engine/Math/IntVec3.hpp"
#include "Engine/Core/Rgba8.hpp"
#include <fstream>
#include <sstream>
#include <cctype>

//-----------------------------------------------------------------------------------------------
int FileReadToBuffer(std::vector<uint8_t>& outBuffer, const std::string& fileName)
{
	std::ifstream file; 
	file.open(fileName);

	if (file.is_open()) 
	{
		uint8_t theChar;
		while (file) 
		{
			theChar = static_cast<uint8_t>(file.get());
			outBuffer.push_back(theChar);
		}
	}
	else
	{
		ERROR_AND_DIE("Could not open the file.");
	}

	outBuffer.pop_back();
	file.close();
	return static_cast<int>(outBuffer.size());
}

//-----------------------------------------------------------------------------------------------
int FileReadToString(std::string& outString, const std::string& fileName)
{
	std::vector<uint8_t> buffer;
	FileReadToBuffer(buffer, fileName);
	std::string conversion(buffer.begin(), buffer.end());
	outString = conversion;
	return static_cast<int>(outString.length());
}

//-----------------------------------------------------------------------------------------------
bool FileReadToBufferBinary(std::vector<uint8_t>& outBuffer, const std::string& fileName)
{
	FILE* file;
	fopen_s(&file, fileName.c_str(), "rb");

	if (!file)
	{
		return false;
	}

	//Get file size and rewind
	fseek(file, 0, SEEK_END);
	long fileSize = ftell(file);
	rewind(file);

	//Write to buffer and close as fast as possible
	char* buffer = new char[fileSize];
	fread(buffer, 1, static_cast<size_t>(fileSize), file);
	fclose(file);

	//update out buffer
	for (int bufferIndex = 0; bufferIndex < fileSize; bufferIndex++)
	{
		outBuffer.push_back(buffer[bufferIndex]);
	}
	return true;
}

//-----------------------------------------------------------------------------------------------
bool FileWriteToFileBinary(std::vector<uint8_t>& inBuffer, const std::string& fileName)
{
	size_t fileSize = inBuffer.size();
	FILE* file;
	fopen_s(&file, fileName.c_str(), "wb");

	if (!file)
	{
		return false;
	}

	fwrite(inBuffer.data(), 1, static_cast<size_t>(fileSize), file);
	fclose(file);
	return true;
}

//-----------------------------------------------------------------------------------------------
std::map<IntVec3, Rgba8> Read3DSpriteToBuffer(const std::string& fileName)
{
	std::string buffer;
	FileReadToString(buffer, fileName);
	std::map<IntVec3, Rgba8> returnBuffer;
	//BlockTemplate* sprite = CreateNewBlockTemplate(templateName);
	std::string nameBuffer = "";
	std::string dataBuffer = "";
	bool startCheckingCategory = false;
	bool startCheckingData = false;
	bool startReadingData = false;
	for (int bufferIndex = 0; bufferIndex < buffer.length(); bufferIndex++)
	{
		if (startCheckingData == false)
		{
			if (buffer[bufferIndex] == '"' && startCheckingCategory == false)
			{
				startCheckingCategory = true;
			}
			else if (startCheckingCategory == true && buffer[bufferIndex] == '"')
			{
				if (nameBuffer == "data")
				{
					bufferIndex += 2;
					startCheckingData = true;
				}
				startCheckingCategory = false;
				nameBuffer = "";
			}
			else if (startCheckingCategory == true)
			{
				nameBuffer += buffer[bufferIndex];
			}
		}
		else
		{
			if (buffer[bufferIndex] == '[')
			{
				startReadingData = true;
			}
			else if (buffer[bufferIndex] == ']')
			{
				//Separate into substrings
				std::stringstream stringstream(dataBuffer);
				std::string currentString = "";
				std::vector<std::string> substrings;
				while (std::getline(stringstream, currentString, ','))
				{
					substrings.push_back(currentString);
				}

				//Get rid of extra chars in the substring
				for (int index = 0; index < substrings.size(); index++)
				{
					std::string tempString = "";
					if (index < 3)
					{
						for (int charIndex = 0; charIndex < substrings[index].length(); charIndex++)
						{
							if (substrings[index][charIndex] == '-' || std::isdigit(substrings[index][charIndex]))
							{
								tempString += substrings[index][charIndex];
							}
						}
					}
					else
					{
						for (int charIndex = 0; charIndex < substrings[index].length(); charIndex++)
						{
							if (std::isalpha(substrings[index][charIndex]) || std::isdigit(substrings[index][charIndex]))
							{
								tempString += substrings[index][charIndex];
							}
						}
					}

					substrings[index] = tempString;
				}

				//convert the substrings to actual values
				if (substrings.size() == 0)
				{
					break;
				}
				int x = std::stoi(substrings[0]);
				int y = std::stoi(substrings[2]);
				int z = std::stoi(substrings[1]);
				Rgba8 color;
				color.r = static_cast<unsigned char>(std::stoi(substrings[3].substr(0, 2), nullptr, 16));
				color.g = static_cast<unsigned char>(std::stoi(substrings[3].substr(2, 2), nullptr, 16));
				color.b = static_cast<unsigned char>(std::stoi(substrings[3].substr(4, 2), nullptr, 16));
				color.a = 255;
				
				returnBuffer.insert(std::make_pair(IntVec3(x, y, z), color));

				dataBuffer = "";
				startReadingData = false;
			}
			else if (startReadingData == true)
			{
				dataBuffer += buffer[bufferIndex];
			}
		}
	}

	return returnBuffer;
}
