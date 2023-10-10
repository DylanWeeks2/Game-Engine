#include "RandomNumberGenerator.hpp"
#include "ThirdParty/Squirrel/RawNoise.hpp"
#include "ThirdParty/Squirrel/SmoothNoise.hpp"
#include <stdlib.h>

//-----------------------------------------------------------------------------------------------
RandomNumberGenerator::RandomNumberGenerator(unsigned int seed)
	:m_seed(seed)
{
}

//-----------------------------------------------------------------------------------------------
int RandomNumberGenerator::RollRandomIntLessThan(int maxNotInclusive)
{
	unsigned int randomUInt = Get1dNoiseUint(m_position++, m_seed);
	return randomUInt % maxNotInclusive;
}

//-----------------------------------------------------------------------------------------------
int RandomNumberGenerator::RollRandomIntInRange(int minInclusive, int maxInclusive)
{
	int range = maxInclusive - minInclusive;
	return (RollRandomIntLessThan(range + 1) + minInclusive);
}

//-----------------------------------------------------------------------------------------------
float RandomNumberGenerator::RollRandomFloatZeroToOne()
{
	/*unsigned int randomUInt = Get1dNoiseUint(m_position++, m_seed);
	constexpr unsigned int MAX_RANDOM_UINT = 0xFFFFFFFF;
	constexpr double ONE_OVER_RAND_MAX = 1.0 / double(MAX_RANDOM_UINT);
	return float(ONE_OVER_RAND_MAX * randomUInt);*/

	return Get1dNoiseZeroToOne(m_position++, m_seed);
}

//-----------------------------------------------------------------------------------------------
float RandomNumberGenerator::RollRandomFloatInRange(float minInclusive, float maxInclusive)
{
	float range = maxInclusive - minInclusive;
	return (RollRandomFloatZeroToOne() * range) + minInclusive;
}
