#include "EngineCommon.hpp"

NamedStrings g_gameConfigBlackboard;

//-----------------------------------------------------------------------------------------------
EndianMode GetPlatformNativeEndianMode()
{
	int testEndian = 1;
	int* addressOfTheInt = &testEndian;
	unsigned char* addressOfTheIntAsByteArray = reinterpret_cast<unsigned char*>(addressOfTheInt);

	if (addressOfTheIntAsByteArray[0] == 1)
	{
		return EndianMode::LITTLE_ENDIAN;
	}
	else if (addressOfTheIntAsByteArray[0] == 0)
	{
		return EndianMode::BIG_ENDIAN;
	}

	return EndianMode::NATIVE;
}
