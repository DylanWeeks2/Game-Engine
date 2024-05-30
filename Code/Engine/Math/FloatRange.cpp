#include "FloatRange.hpp"
#include "Engine/Core/StringUtils.hpp"

//-----------------------------------------------------------------------------------------------
const FloatRange FloatRange::ZERO = FloatRange(0.0f, 0.0f);
const FloatRange FloatRange::ONE = FloatRange(1.0f, 1.0f);
const FloatRange FloatRange::ZERO_TO_ONE = FloatRange(0.0f, 1.0f);

//-----------------------------------------------------------------------------------------------
FloatRange::~FloatRange()
{
}

//-----------------------------------------------------------------------------------------------
FloatRange::FloatRange(float min, float max)
{
	m_min = min;
	m_max = max;
}

//-----------------------------------------------------------------------------------------------
void FloatRange::SetFromText(char const* text)
{
	Strings stringFloatRange;
	stringFloatRange = SplitStringOnDelimiter(text, '~');

	m_min = static_cast<float>(atof(stringFloatRange[0].c_str()));
	m_max = static_cast<float>(atof(stringFloatRange[1].c_str()));
}

//-----------------------------------------------------------------------------------------------
bool FloatRange::operator==(FloatRange const& compare) const
{
	if (compare.m_min == m_min && compare.m_max == m_max)
	{
		return true;
	}

	return false;
}

//-----------------------------------------------------------------------------------------------
bool FloatRange::operator!=(FloatRange const& compare) const
{
	if (compare.m_min != m_min || compare.m_max != m_max)
	{
		return true;
	}

	return false;
}

//-----------------------------------------------------------------------------------------------
void FloatRange::operator=(FloatRange const& copyFrom)
{
	m_min = copyFrom.m_min;
	m_max = copyFrom.m_max;
}

//-----------------------------------------------------------------------------------------------
bool FloatRange::IsOnRange(float value)
{
	if (value >= m_min && value <= m_max)
	{
		return true;
	}

	return false;
}

//-----------------------------------------------------------------------------------------------
bool FloatRange::IsOverlappingWith(FloatRange const& compare)
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
