#include "BufferUtilities.hpp"
#include "Engine/Math/Vec2.hpp"
#include "Engine/Math/Vec3.hpp"
#include "Engine/Math/Vec4.hpp"
#include "Engine/Math/IntVec2.hpp"
#include "Engine/Math/IntVec3.hpp"
#include "Engine/Core/Rgba8.hpp"
#include "Engine/Math/AABB2.hpp"
#include "Engine/Math/AABB3.hpp"
#include "Engine/Math/OBB2.hpp"
#include "Engine/Math/OBB3.hpp"
#include "Engine/Core/Vertex_PCU.hpp"
#include "Engine/Core/EngineCommon.hpp"
#include "Engine/Core/ErrorWarningAssert.hpp"

//-----------------------------------------------------------------------------------------------
BufferWriter::BufferWriter(std::vector<unsigned char>& buffer, EndianMode const& endianMode)
	:m_buffer(buffer),
	m_currentEndian(endianMode)
{
	if (m_currentEndian != EndianMode::NATIVE)
		SetEndian(endianMode);
}

//-----------------------------------------------------------------------------------------------
void BufferWriter::SetEndian(EndianMode const& endianMode)
{
	m_currentEndian = endianMode;
	EndianMode platformEndain = GetPlatformNativeEndianMode();
	if (platformEndain != m_currentEndian)
	{
		m_isEndianOppisiteOfPlatform = true;
	}
	else
	{
		m_isEndianOppisiteOfPlatform = false;
	}
}

//-----------------------------------------------------------------------------------------------
void BufferWriter::AppendByte(unsigned char const& byteToAppend)
{
	m_buffer.push_back(byteToAppend);
}

//-----------------------------------------------------------------------------------------------
void BufferWriter::AppendChar(char charToAppend)
{
	AppendByte(charToAppend);
}

//-----------------------------------------------------------------------------------------------
void BufferWriter::AppendUnsignedShort(unsigned short shortToAppend)
{
	unsigned short* addressOfTheShort = &shortToAppend;
	unsigned char* addressOfTheShortAsByteArray = reinterpret_cast<unsigned char*>(addressOfTheShort);

	if (m_isEndianOppisiteOfPlatform)
		Reverse2Bytes(addressOfTheShortAsByteArray);

	AppendByte(addressOfTheShortAsByteArray[0]);
	AppendByte(addressOfTheShortAsByteArray[1]);
}

//-----------------------------------------------------------------------------------------------
void BufferWriter::AppendShort(short shortToAppend)
{
	short* addressOfTheShort = &shortToAppend;
	unsigned char* addressOfTheShortAsByteArray = reinterpret_cast<unsigned char*>(addressOfTheShort);
	
	if (m_isEndianOppisiteOfPlatform)
		Reverse2Bytes(addressOfTheShortAsByteArray);
	
	AppendByte(addressOfTheShortAsByteArray[0]);
	AppendByte(addressOfTheShortAsByteArray[1]);
}

//-----------------------------------------------------------------------------------------------
void BufferWriter::AppendUnsignedInt(unsigned int intToAppend)
{
	unsigned int* addressOfTheInt = &intToAppend;
	unsigned char* addressOfTheIntAsByteArray = reinterpret_cast<unsigned char*>(addressOfTheInt);
	
	if (m_isEndianOppisiteOfPlatform)
		Reverse4Bytes(addressOfTheIntAsByteArray);
	
	AppendByte(addressOfTheIntAsByteArray[0]);
	AppendByte(addressOfTheIntAsByteArray[1]);
	AppendByte(addressOfTheIntAsByteArray[2]);
	AppendByte(addressOfTheIntAsByteArray[3]);
}

//-----------------------------------------------------------------------------------------------
void BufferWriter::AppendInt(int intToAppend)
{
	int* addressOfTheInt = &intToAppend;
	unsigned char* addressOfTheIntAsByteArray = reinterpret_cast<unsigned char*>(addressOfTheInt);
	
	if (m_isEndianOppisiteOfPlatform)
		Reverse4Bytes(addressOfTheIntAsByteArray);
	
	AppendByte(addressOfTheIntAsByteArray[0]);
	AppendByte(addressOfTheIntAsByteArray[1]);
	AppendByte(addressOfTheIntAsByteArray[2]);
	AppendByte(addressOfTheIntAsByteArray[3]);
}

//-----------------------------------------------------------------------------------------------
void BufferWriter::AppendUnsignedInt64(uint64_t int64ToAppend)
{
	uint64_t* addressOfTheInt = &int64ToAppend;
	unsigned char* addressOfTheIntAsByteArray = reinterpret_cast<unsigned char*>(addressOfTheInt);
	
	if (m_isEndianOppisiteOfPlatform)
		Reverse8Bytes(addressOfTheIntAsByteArray);
	
	AppendByte(addressOfTheIntAsByteArray[0]);
	AppendByte(addressOfTheIntAsByteArray[1]);
	AppendByte(addressOfTheIntAsByteArray[2]);
	AppendByte(addressOfTheIntAsByteArray[3]);
	AppendByte(addressOfTheIntAsByteArray[4]);
	AppendByte(addressOfTheIntAsByteArray[5]);
	AppendByte(addressOfTheIntAsByteArray[6]);
	AppendByte(addressOfTheIntAsByteArray[7]);
}

//-----------------------------------------------------------------------------------------------
void BufferWriter::AppendInt64(int64_t int64ToAppend)
{
	int64_t* addressOfTheInt = &int64ToAppend;
	unsigned char* addressOfTheIntAsByteArray = reinterpret_cast<unsigned char*>(addressOfTheInt);
	
	if (m_isEndianOppisiteOfPlatform)
		Reverse8Bytes(addressOfTheIntAsByteArray);
	
	AppendByte(addressOfTheIntAsByteArray[0]);
	AppendByte(addressOfTheIntAsByteArray[1]);
	AppendByte(addressOfTheIntAsByteArray[2]);
	AppendByte(addressOfTheIntAsByteArray[3]);
	AppendByte(addressOfTheIntAsByteArray[4]);
	AppendByte(addressOfTheIntAsByteArray[5]);
	AppendByte(addressOfTheIntAsByteArray[6]);
	AppendByte(addressOfTheIntAsByteArray[7]);
}

//-----------------------------------------------------------------------------------------------
void BufferWriter::AppendFloat(float floatToAppend)
{
	float* addressOfTheFloat = &floatToAppend;
	unsigned char* addressOfTheFloatAsByteArray = reinterpret_cast<unsigned char*>(addressOfTheFloat);
	
	if (m_isEndianOppisiteOfPlatform)
		Reverse4Bytes(addressOfTheFloatAsByteArray);
	
	AppendByte(addressOfTheFloatAsByteArray[0]);
	AppendByte(addressOfTheFloatAsByteArray[1]);
	AppendByte(addressOfTheFloatAsByteArray[2]);
	AppendByte(addressOfTheFloatAsByteArray[3]);
}

//-----------------------------------------------------------------------------------------------
void BufferWriter::AppendDouble(double doubleToAppend)
{
	double* addressOfTheDouble = &doubleToAppend;
	unsigned char* addressOfTheDoubleAsByteArray = reinterpret_cast<unsigned char*>(addressOfTheDouble);
	
	if (m_isEndianOppisiteOfPlatform)
		Reverse8Bytes(addressOfTheDoubleAsByteArray);
	
	AppendByte(addressOfTheDoubleAsByteArray[0]);
	AppendByte(addressOfTheDoubleAsByteArray[1]);
	AppendByte(addressOfTheDoubleAsByteArray[2]);
	AppendByte(addressOfTheDoubleAsByteArray[3]);
	AppendByte(addressOfTheDoubleAsByteArray[4]);
	AppendByte(addressOfTheDoubleAsByteArray[5]);
	AppendByte(addressOfTheDoubleAsByteArray[6]);
	AppendByte(addressOfTheDoubleAsByteArray[7]);
}

//-----------------------------------------------------------------------------------------------
void BufferWriter::AppendVec2(Vec2 vec2ToAppend)
{
	AppendFloat(vec2ToAppend.x);
	AppendFloat(vec2ToAppend.y);
}

//-----------------------------------------------------------------------------------------------
void BufferWriter::AppendVec3(Vec3 vec3ToAppend)
{
	AppendFloat(vec3ToAppend.x);
	AppendFloat(vec3ToAppend.y);
	AppendFloat(vec3ToAppend.z);
}

//-----------------------------------------------------------------------------------------------
void BufferWriter::AppendVec4(Vec4 vec4ToAppend)
{
	AppendFloat(vec4ToAppend.x);
	AppendFloat(vec4ToAppend.y);
	AppendFloat(vec4ToAppend.z);
	AppendFloat(vec4ToAppend.w);
}

//-----------------------------------------------------------------------------------------------
void BufferWriter::AppendIntVec2(IntVec2 intVec2ToAppend)
{
	AppendInt(intVec2ToAppend.x);
	AppendInt(intVec2ToAppend.y);
}

//-----------------------------------------------------------------------------------------------
void BufferWriter::AppendIntVec3(IntVec3 intVec3ToAppend)
{
	AppendInt(intVec3ToAppend.x);
	AppendInt(intVec3ToAppend.y);
	AppendInt(intVec3ToAppend.z);
}

//-----------------------------------------------------------------------------------------------
void BufferWriter::AppendRgba8(Rgba8 rgba8ToAppend)
{
	AppendByte(rgba8ToAppend.r);
	AppendByte(rgba8ToAppend.g);
	AppendByte(rgba8ToAppend.b);
	AppendByte(rgba8ToAppend.a);
}

//-----------------------------------------------------------------------------------------------
void BufferWriter::AppendAABB2(AABB2 aabbToAppend)
{
	AppendVec2(aabbToAppend.m_mins);
	AppendVec2(aabbToAppend.m_maxs);
}

//-----------------------------------------------------------------------------------------------
void BufferWriter::AppendAABB3(AABB3 aabbToAppend)
{
	AppendVec3(aabbToAppend.m_mins);
	AppendVec3(aabbToAppend.m_maxs);
}

//-----------------------------------------------------------------------------------------------
void BufferWriter::AppendOBB2(OBB2 obbToAppend)
{
	AppendVec2(obbToAppend.m_center);
	AppendVec2(obbToAppend.m_iBasisNormal);
	AppendVec2(obbToAppend.m_halfDimensions);
}

//-----------------------------------------------------------------------------------------------
void BufferWriter::AppendOBB3(OBB3 obbToAppend)
{
	AppendVec3(obbToAppend.m_center);
	AppendVec3(obbToAppend.m_iBasisNormal);
	AppendVec3(obbToAppend.m_halfDimensions);
}

//-----------------------------------------------------------------------------------------------
void BufferWriter::AppendVertexPCU(Vertex_PCU pcuToAppend)
{
	AppendVec3(pcuToAppend.m_position);
	AppendRgba8(pcuToAppend.m_color);
	AppendVec2(pcuToAppend.m_uvTexCoords);
}

//-----------------------------------------------------------------------------------------------
void BufferWriter::AppendStringZeroTerminated(std::string stringToAppend)
{
	if (m_isEndianOppisiteOfPlatform)
	{
		std::string reverseString = "";
		for (int index = int(stringToAppend.length()) - 1; index >= 0; index--)
		{
			reverseString += stringToAppend[index];
		}

		for (int stringIndex = 0; stringIndex < reverseString.length(); stringIndex++)
		{
			AppendChar(reverseString[stringIndex]);
		}
		AppendChar('\0');
		return;
	}

	for (int stringIndex = 0; stringIndex < stringToAppend.length(); stringIndex++)
	{
		AppendChar(stringToAppend[stringIndex]);
	}
	AppendChar('\0');
}

//-----------------------------------------------------------------------------------------------
void BufferWriter::AppendStringGivenSize32Bit(std::string stringToAppend, unsigned int size)
{
	if (m_isEndianOppisiteOfPlatform)
	{
		std::string reverseString = "";
		for (int stringIndex = size - 1; stringIndex >= 0; stringIndex--)
		{
			reverseString += stringToAppend[stringIndex];
		}

		for (unsigned int stringIndex = 0; stringIndex < size ; stringIndex++)
		{
			AppendChar(reverseString[stringIndex]);
		}
		return;
	}

	for (unsigned int stringIndex = 0; stringIndex < size; stringIndex++)
	{
		AppendChar(stringToAppend[stringIndex]);
	}
}

//-----------------------------------------------------------------------------------------------
void BufferWriter::OverwriteUInt32(uint32_t intToAppend, int position)
{
	if (position + sizeof(unsigned int) >= m_buffer.size())
		ERROR_AND_DIE("TRYING TO OVERWRITE OUTSIDE BUFFER BOUNDS");

	uint32_t* addressOfTheInt = &intToAppend;
	unsigned char* addressOfTheIntAsByteArray = reinterpret_cast<unsigned char*>(addressOfTheInt);

	if (m_isEndianOppisiteOfPlatform)
	{
		m_buffer[position] = addressOfTheIntAsByteArray[3];
		m_buffer[position + 1] = addressOfTheIntAsByteArray[2];
		m_buffer[position + 2] = addressOfTheIntAsByteArray[1];
		m_buffer[position + 3] = addressOfTheIntAsByteArray[0];
	}
	else
	{
		m_buffer[position] = addressOfTheIntAsByteArray[0];
		m_buffer[position + 1] = addressOfTheIntAsByteArray[1];
		m_buffer[position + 2] = addressOfTheIntAsByteArray[2];
		m_buffer[position + 3] = addressOfTheIntAsByteArray[3];
	}
}

//-----------------------------------------------------------------------------------------------
void BufferWriter::Reverse2Bytes(unsigned char*& bytesToReverse)
{
	unsigned char tempChar = bytesToReverse[0];
	bytesToReverse[0] = bytesToReverse[1];
	bytesToReverse[1] = tempChar;
}

//-----------------------------------------------------------------------------------------------
void BufferWriter::Reverse4Bytes(unsigned char*& bytesToReverse)
{
	unsigned char tempChar1 = bytesToReverse[0];
	unsigned char tempChar2 = bytesToReverse[1];
	bytesToReverse[0] = bytesToReverse[3];
	bytesToReverse[1] = bytesToReverse[2];
	bytesToReverse[3] = tempChar1;
	bytesToReverse[2] = tempChar2;
}

//-----------------------------------------------------------------------------------------------
void BufferWriter::Reverse8Bytes(unsigned char*& bytesToReverse)
{
	unsigned char tempChar1 = bytesToReverse[0];
	unsigned char tempChar2 = bytesToReverse[1];
	unsigned char tempChar3 = bytesToReverse[2];
	unsigned char tempChar4 = bytesToReverse[3];
	bytesToReverse[0] = bytesToReverse[7];
	bytesToReverse[1] = bytesToReverse[6];
	bytesToReverse[2] = bytesToReverse[5];
	bytesToReverse[3] = bytesToReverse[4];
	bytesToReverse[7] = tempChar1;
	bytesToReverse[6] = tempChar2;
	bytesToReverse[5] = tempChar3;
	bytesToReverse[4] = tempChar4;
}

//-----------------------------------------------------------------------------------------------
BufferParser::BufferParser(unsigned char const* bufferToParse, size_t bufferSizeInBytes, EndianMode const& endianMode)
	:m_bufferStart(bufferToParse)
	,m_bufferSizeInBytes(bufferSizeInBytes)
	,m_currentEndian(endianMode)
{
	if(m_currentEndian != EndianMode::NATIVE)
		SetEndian(endianMode);
}

//-----------------------------------------------------------------------------------------------
void BufferParser::SetEndian(EndianMode const& endianMode)
{
	m_currentEndian = endianMode;
	EndianMode platformEndain = GetPlatformNativeEndianMode();
	if (platformEndain != m_currentEndian)
	{
		m_isEndianOppisiteOfPlatform = true;
	}
	else
	{
		m_isEndianOppisiteOfPlatform = false;
	}
}

//-----------------------------------------------------------------------------------------------
unsigned char BufferParser::ParseByte()
{
	if (m_currentReadOffset + 1 > m_bufferSizeInBytes)
		ERROR_AND_DIE("TRYING TO READ OUTSIDE BUFFER BOUNDS");
	
	return m_bufferStart[m_currentReadOffset++];
}

//-----------------------------------------------------------------------------------------------
char BufferParser::ParseChar()
{
	if (m_currentReadOffset + sizeof(char) > m_bufferSizeInBytes)
		ERROR_AND_DIE("TRYING TO READ OUTSIDE BUFFER BOUNDS");

	unsigned char const* memoryAddressOfCharAsBytesInBuffer = &m_bufferStart[m_currentReadOffset];
	char const* memoryAddressOfCharInBuffer = reinterpret_cast<char const*>(memoryAddressOfCharAsBytesInBuffer);
	m_currentReadOffset += sizeof(char);
	return *memoryAddressOfCharInBuffer;
}

//-----------------------------------------------------------------------------------------------
unsigned short BufferParser::ParseUnsignedShort()
{
	if (m_currentReadOffset + sizeof(unsigned short) > m_bufferSizeInBytes)
		ERROR_AND_DIE("TRYING TO READ OUTSIDE BUFFER BOUNDS");

	unsigned char const* memoryAddressOfShortAsBytesInBuffer = &m_bufferStart[m_currentReadOffset];
	unsigned short memoryAddressOfShortInBuffer = *reinterpret_cast<unsigned short const*>(memoryAddressOfShortAsBytesInBuffer);

	if (m_isEndianOppisiteOfPlatform)
		Reverse2Bytes(&memoryAddressOfShortInBuffer);

	m_currentReadOffset += sizeof(unsigned short);
	return memoryAddressOfShortInBuffer;
}

//-----------------------------------------------------------------------------------------------
short BufferParser::ParseShort()
{
	if (m_currentReadOffset + sizeof(short) > m_bufferSizeInBytes)
		ERROR_AND_DIE("TRYING TO READ OUTSIDE BUFFER BOUNDS");

	unsigned char const* memoryAddressOfShortAsBytesInBuffer = &m_bufferStart[m_currentReadOffset];
	short memoryAddressOfShortInBuffer = *reinterpret_cast<short const*>(memoryAddressOfShortAsBytesInBuffer);
	
	if (m_isEndianOppisiteOfPlatform)
		Reverse2Bytes(&memoryAddressOfShortInBuffer);
	
	m_currentReadOffset += sizeof(short);
	return memoryAddressOfShortInBuffer;
}

//-----------------------------------------------------------------------------------------------
unsigned int BufferParser::ParseUnsignedInt()
{
	if (m_currentReadOffset + sizeof(unsigned int) > m_bufferSizeInBytes)
		ERROR_AND_DIE("TRYING TO READ OUTSIDE BUFFER BOUNDS");

	unsigned char const* memoryAddressOfIntAsBytesInBuffer = &m_bufferStart[m_currentReadOffset];
	unsigned int memoryAddressOfIntInBuffer = *reinterpret_cast<unsigned int const*>(memoryAddressOfIntAsBytesInBuffer);
	
	if (m_isEndianOppisiteOfPlatform)
		Reverse4Bytes(&memoryAddressOfIntInBuffer);
	
	m_currentReadOffset += sizeof(unsigned int);
	return memoryAddressOfIntInBuffer;
}

//-----------------------------------------------------------------------------------------------
int BufferParser::ParseInt()
{
	if (m_currentReadOffset + sizeof(int) > m_bufferSizeInBytes)
		ERROR_AND_DIE("TRYING TO READ OUTSIDE BUFFER BOUNDS");

	unsigned char const* memoryAddressOfIntAsBytesInBuffer = &m_bufferStart[m_currentReadOffset];
	int memoryAddressOfIntInBuffer = *reinterpret_cast<int const*>(memoryAddressOfIntAsBytesInBuffer);
	
	if (m_isEndianOppisiteOfPlatform)
		Reverse4Bytes(&memoryAddressOfIntInBuffer);
	
	m_currentReadOffset += sizeof(int);
	return memoryAddressOfIntInBuffer;
}

//-----------------------------------------------------------------------------------------------
uint64_t BufferParser::ParseUInt64()
{
	if (m_currentReadOffset + sizeof(uint64_t) > m_bufferSizeInBytes)
		ERROR_AND_DIE("TRYING TO READ OUTSIDE BUFFER BOUNDS");

	unsigned char const* memoryAddressOfIntAsBytesInBuffer = &m_bufferStart[m_currentReadOffset];
	uint64_t memoryAddressOfIntInBuffer = *reinterpret_cast<uint64_t const*>(memoryAddressOfIntAsBytesInBuffer);
	
	if (m_isEndianOppisiteOfPlatform)
		Reverse8Bytes(&memoryAddressOfIntInBuffer);

	m_currentReadOffset += sizeof(uint64_t);
	return memoryAddressOfIntInBuffer;
}

//-----------------------------------------------------------------------------------------------
int64_t BufferParser::ParseInt64()
{
	if (m_currentReadOffset + sizeof(int64_t) > m_bufferSizeInBytes)
		ERROR_AND_DIE("TRYING TO READ OUTSIDE BUFFER BOUNDS");

	unsigned char const* memoryAddressOfIntAsBytesInBuffer = &m_bufferStart[m_currentReadOffset];
	int64_t memoryAddressOfIntInBuffer = *reinterpret_cast<int64_t const*>(memoryAddressOfIntAsBytesInBuffer);
	
	if (m_isEndianOppisiteOfPlatform)
		Reverse8Bytes(&memoryAddressOfIntInBuffer);
	
	m_currentReadOffset += sizeof(int64_t);
	return memoryAddressOfIntInBuffer;
}

//-----------------------------------------------------------------------------------------------
float BufferParser::ParseFloat()
{
	if (m_currentReadOffset + sizeof(float) > m_bufferSizeInBytes)
		ERROR_AND_DIE("TRYING TO READ OUTSIDE BUFFER BOUNDS");

	unsigned char const* memoryAddressOfFloatAsBytesInBuffer = &m_bufferStart[m_currentReadOffset];
	float memoryAddressOfFloatInBuffer = *reinterpret_cast<float const*>(memoryAddressOfFloatAsBytesInBuffer);
	
	if (m_isEndianOppisiteOfPlatform)
		Reverse4Bytes(&memoryAddressOfFloatInBuffer);

	m_currentReadOffset += sizeof(float);
	return memoryAddressOfFloatInBuffer;
}

//-----------------------------------------------------------------------------------------------
double BufferParser::ParseDouble()
{
	if (m_currentReadOffset + sizeof(double) > m_bufferSizeInBytes)
		ERROR_AND_DIE("TRYING TO READ OUTSIDE BUFFER BOUNDS");

	unsigned char const* memoryAddressOfDoubleAsBytesInBuffer = &m_bufferStart[m_currentReadOffset];
	double memoryAddressOfDoubleInBuffer = *reinterpret_cast<double const*>(memoryAddressOfDoubleAsBytesInBuffer);
	
	if (m_isEndianOppisiteOfPlatform)
		Reverse8Bytes(&memoryAddressOfDoubleInBuffer);
	
	m_currentReadOffset += sizeof(double);
	return memoryAddressOfDoubleInBuffer;
}

//-----------------------------------------------------------------------------------------------
Vec2 BufferParser::ParseVec2()
{
	float x = ParseFloat();
	float y = ParseFloat();
	return Vec2(x, y);
}

//-----------------------------------------------------------------------------------------------
Vec3 BufferParser::ParseVec3()
{
	float x = ParseFloat();
	float y = ParseFloat();
	float z = ParseFloat();
	return Vec3(x, y, z);
}

//-----------------------------------------------------------------------------------------------
Vec4 BufferParser::ParseVec4()
{
	float x = ParseFloat();
	float y = ParseFloat();
	float z = ParseFloat();
	float w = ParseFloat();
	return Vec4(x, y, z, w);
}

//-----------------------------------------------------------------------------------------------
IntVec2 BufferParser::ParseIntVec2()
{
	int x = ParseInt();
	int y = ParseInt();
	return IntVec2(x, y);
}

//-----------------------------------------------------------------------------------------------
IntVec3 BufferParser::ParseIntVec3()
{
	int x = ParseInt();
	int y = ParseInt(); 
	int z = ParseInt();
	return IntVec3(x, y, z);
}

//-----------------------------------------------------------------------------------------------
Rgba8 BufferParser::ParseRgba8()
{
	unsigned char r = ParseByte();
	unsigned char g = ParseByte();
	unsigned char b = ParseByte();
	unsigned char a = ParseByte();
	return Rgba8(r, g, b, a);
}

//-----------------------------------------------------------------------------------------------
AABB2 BufferParser::ParseAABB2()
{
	Vec2 mins = ParseVec2();
	Vec2 maxs = ParseVec2();
	return AABB2(mins, maxs);
}

//-----------------------------------------------------------------------------------------------
AABB3 BufferParser::ParseAABB3()
{
	Vec3 mins = ParseVec3();
	Vec3 maxs = ParseVec3();
	return AABB3(mins, maxs);
}

//-----------------------------------------------------------------------------------------------
OBB2 BufferParser::ParseOBB2()
{
	Vec2 center = ParseVec2();
	Vec2 iBasis = ParseVec2();
	Vec2 halfDimensions = ParseVec2();
	return OBB2(center, iBasis, halfDimensions);
}

//-----------------------------------------------------------------------------------------------
OBB3 BufferParser::ParseOBB3()
{
	Vec3 center = ParseVec3();
	Vec3 iBasis = ParseVec3();
	Vec3 halfDimensions = ParseVec3();
	return OBB3(center, iBasis, halfDimensions);
}

//-----------------------------------------------------------------------------------------------
Vertex_PCU BufferParser::ParseVertexPCU()
{
	Vec3 position = ParseVec3();
	Rgba8 color = ParseRgba8();
	Vec2 uv = ParseVec2();
	return Vertex_PCU(position, color ,uv);
}

//-----------------------------------------------------------------------------------------------
std::string BufferParser::ParseStringZeroTerminated()
{
	std::string returnString = "";
	while(true)
	{
		char c = ParseChar();
		if (c == '\0')
			break;
		returnString += c;
	}

	if (m_isEndianOppisiteOfPlatform)
	{
		std::string reverseString = "";
		for(int index = int(returnString.length()) - 1; index >= 0; index--)
		{
			reverseString += returnString[index];
		}

		return reverseString;
	}

	return returnString;
}

//-----------------------------------------------------------------------------------------------
std::string BufferParser::ParseStringGivenSize32Bit(unsigned int size)
{
	std::string returnString = "";
	for (unsigned int index = 0; index < size; index++)
	{
		char c = ParseChar();
		returnString += c;
	}

	if (m_isEndianOppisiteOfPlatform)
	{
		std::string reverseString = "";
		for (int index = int(returnString.length()) - 1; index >= 0; index--)
		{
			reverseString += returnString[index];
		}

		return reverseString;
	}

	return returnString;
}

//-----------------------------------------------------------------------------------------------
void BufferParser::ShiftCurrentBufferReadPosition(int offset)
{
	m_currentReadOffset = offset;
}

//-----------------------------------------------------------------------------------------------
void BufferParser::Reverse2Bytes(void* bytesToReverse)
{
	unsigned char* bytes = reinterpret_cast<unsigned char*>(bytesToReverse);
	unsigned char tempChar = bytes[0];
	bytes[0] = bytes[1];
	bytes[1] = tempChar;
}

//-----------------------------------------------------------------------------------------------
void BufferParser::Reverse4Bytes(void* bytesToReverse)
{
	unsigned char* bytes = reinterpret_cast<unsigned char*>(bytesToReverse);
	unsigned char tempChar1 = bytes[0];
	unsigned char tempChar2 = bytes[1];
	bytes[0] = bytes[3];
	bytes[1] = bytes[2];
	bytes[3] = tempChar1;
	bytes[2] = tempChar2;
}

//-----------------------------------------------------------------------------------------------
void BufferParser::Reverse8Bytes(void* bytesToReverse)
{
	unsigned char* bytes = reinterpret_cast<unsigned char*>(bytesToReverse);
	unsigned char tempChar1 = bytes[0];
	unsigned char tempChar2 = bytes[1];
	unsigned char tempChar3 = bytes[2];
	unsigned char tempChar4 = bytes[3];
	bytes[0] = bytes[7];
	bytes[1] = bytes[6];
	bytes[2] = bytes[5];
	bytes[3] = bytes[4];
	bytes[7] = tempChar1;
	bytes[6] = tempChar2;
	bytes[5] = tempChar3;
	bytes[4] = tempChar4;
}
