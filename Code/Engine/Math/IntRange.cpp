#include "IntRange.hpp"

//-----------------------------------------------------------------------------------------------
const IntRange IntRange::ZERO = IntRange(0, 0);
const IntRange IntRange::ONE = IntRange(1, 1);
const IntRange IntRange::ZERO_TO_ONE = IntRange(0, 1);

//-----------------------------------------------------------------------------------------------
IntRange::~IntRange()
{
}

//-----------------------------------------------------------------------------------------------
IntRange::IntRange(int min, int max)
{
	m_min = min;
	m_max = max;
}

//-----------------------------------------------------------------------------------------------
bool IntRange::operator==(IntRange const& compare) const
{
	if (compare.m_min == m_min && compare.m_max == m_max)
	{
		return true;
	}

	return false;
}

//-----------------------------------------------------------------------------------------------
bool IntRange::operator!=(IntRange const& compare) const
{
	if (compare.m_min != m_min || compare.m_max != m_max)
	{
		return true;
	}

	return false;
}

//-----------------------------------------------------------------------------------------------
void IntRange::operator=(IntRange const& copyFrom)
{
	m_min = copyFrom.m_min;
	m_max = copyFrom.m_max;
}

//-----------------------------------------------------------------------------------------------
bool IntRange::IsOnRange(int value)
{
	if (value >= m_min && value <= m_max)
	{
		return true;
	}

	return false;
}

//-----------------------------------------------------------------------------------------------
bool IntRange::IsOverlappingWith(IntRange const& compare)
{
	if (compare.m_min >= m_min && compare.m_min <= m_max)
	{
		return true;
	}
	else if (compare.m_max >= m_min && compare.m_max <= m_max)
	{
		return true;
	}

	return false;
}
