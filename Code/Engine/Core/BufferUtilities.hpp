#pragma once
#include <vector>
#include <string>
#include <Engine/Core/FileUtils.hpp>

//-----------------------------------------------------------------------------------------------
struct Vec2;
struct Vec3;
struct Vec4;
struct IntVec2;
struct IntVec3;
struct Rgba8;
struct AABB2; 
struct AABB3; 
struct OBB2;
struct OBB3;
struct Vertex_PCU;

//-----------------------------------------------------------------------------------------------
class BufferWriter
{
public:
	BufferWriter(std::vector<unsigned char>& buffer, EndianMode const& endianMode);
	void SetEndian(EndianMode const& endianMode);
	void AppendByte(unsigned char const& byteToAppend);
	void AppendChar(char charToAppend);
	void AppendUnsignedShort(unsigned short shortToAppend);
	void AppendShort(short shortToAppend);
	void AppendUnsignedInt(unsigned int intToAppend);
	void AppendInt(int intToAppend); 
	void AppendUnsignedInt64(uint64_t int64ToAppend);
	void AppendInt64(int64_t int64ToAppend);
	void AppendFloat(float floatToAppend);
	void AppendDouble(double doubleToAppend);
	void AppendVec2(Vec2 vec2ToAppend);
	void AppendVec3(Vec3 vec3ToAppend);
	void AppendVec4(Vec4 vec4ToAppend);
	void AppendIntVec2(IntVec2 intVec2ToAppend);
	void AppendIntVec3(IntVec3 intVec3ToAppend);
	void AppendRgba8(Rgba8 rgba8ToAppend);
	void AppendAABB2(AABB2 aabbToAppend);
	void AppendAABB3(AABB3 aabbToAppend);
	void AppendOBB2(OBB2 obbToAppend);
	void AppendOBB3(OBB3 obbToAppend);
	void AppendVertexPCU(Vertex_PCU pcuToAppend);
	void AppendStringZeroTerminated(std::string stringToAppend);
	void AppendStringGivenSize32Bit(std::string stringToAppend, unsigned int size);
	void OverwriteUInt32(uint32_t intToAppend, int position);
	void Reverse2Bytes(unsigned char*& bytesToReverse);
	void Reverse4Bytes(unsigned char*& bytesToReverse);
	void Reverse8Bytes(unsigned char*& bytesToReverse);

public:
	EndianMode						m_currentEndian = EndianMode::NATIVE;
	std::vector<unsigned char>&		m_buffer;
	bool							m_isEndianOppisiteOfPlatform = false;
};

//-----------------------------------------------------------------------------------------------
class BufferParser
{
public:
	BufferParser(unsigned char const* bufferToParse, size_t bufferSizeInBytes, EndianMode const& endianMode);
	void					SetEndian(EndianMode const& endianMode);
	unsigned char			ParseByte();
	char					ParseChar();
	unsigned short			ParseUnsignedShort();
	short					ParseShort();
	unsigned int			ParseUnsignedInt();
	int						ParseInt();
	uint64_t				ParseUInt64();
	int64_t					ParseInt64();
	float					ParseFloat();
	double					ParseDouble();
	Vec2					ParseVec2();
	Vec3					ParseVec3();
	Vec4					ParseVec4();
	IntVec2					ParseIntVec2();
	IntVec3					ParseIntVec3();
	Rgba8					ParseRgba8();
	AABB2					ParseAABB2();
	AABB3					ParseAABB3();
	OBB2					ParseOBB2();
	OBB3					ParseOBB3();
	Vertex_PCU				ParseVertexPCU();
	std::string				ParseStringZeroTerminated();
	std::string				ParseStringGivenSize32Bit(unsigned int size);
	void					ShiftCurrentBufferReadPosition(int offset);
	void					Reverse2Bytes(void* addressToBytesToReverse);
	void					Reverse4Bytes(void* bytesToReverse);
	void					Reverse8Bytes(void* bytesToReverse);

public:
	unsigned char const*	m_bufferStart = nullptr;
	size_t					m_bufferSizeInBytes = 0;
	size_t					m_currentReadOffset = 0;
	EndianMode				m_currentEndian = EndianMode::NATIVE;
	bool					m_isEndianOppisiteOfPlatform = false;
};