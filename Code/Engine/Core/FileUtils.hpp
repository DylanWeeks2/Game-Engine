#pragma once
#include "Engine/Core/EngineCommon.hpp"
#include <string>
#include <vector>
#include <map>

struct IntVec3;
struct Rgba8;

int FileReadToBuffer(std::vector<uint8_t>& outBuffer, const std::string& fileName);
int FileReadToString(std::string& outString, const std::string& fileName);
bool FileReadToBufferBinary(std::vector<uint8_t>& outBuffer, const std::string& fileName);
bool FileWriteToFileBinary(std::vector<uint8_t>& inBuffer, const std::string& fileName);
std::map<IntVec3, Rgba8> Read3DSpriteToBuffer(const std::string& fileName);