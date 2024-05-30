#pragma once

//-----------------------------------------------------------------------------------------------
class RandomNumberGenerator
{
public:
	RandomNumberGenerator(unsigned int seed = 0);

	void			SetSeed(unsigned int seed) { m_seed = seed; }
	int				RollRandomIntLessThan(int maxNotInclusive);
	int				RollRandomIntInRange(int minInclusive, int maxInclusive);
	float			RollRandomFloatZeroToOne();
	float			RollRandomFloatInRange(float minInclusive, float maxInclusive);
	double			RollRandomDoubleZeroToOne();
	double			RollRandomDoubleInRange(double minInclusive, double maxInclusive);

public:
	unsigned int	m_seed = 0;
	int				m_position =  0;
};