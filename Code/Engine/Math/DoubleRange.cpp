#include "DoubleRange.hpp"
#include "Engine/Core/StringUtils.hpp"

//-----------------------------------------------------------------------------------------------
const DoubleRange DoubleRange::ZERO = DoubleRange(0.0f, 0.0f);
const DoubleRange DoubleRange::ONE = DoubleRange(1.0f, 1.0f);
const DoubleRange DoubleRange::ZERO_TO_ONE = DoubleRange(0.0f, 1.0f);

//-----------------------------------------------------------------------------------------------
DoubleRange::~DoubleRange()
{
}

//-----------------------------------------------------------------------------------------------
DoubleRange::DoubleRange(double min, double max)
{
	m_min = min;
	m_max = max;
}

//-----------------------------------------------------------------------------------------------
void DoubleRange::SetFromText(char const* text)
{
	Strings stringFloatRange;
	stringFloatRange = SplitStringOnDelimiter(text, '~');

	m_min = static_cast<double>(atof(stringFloatRange[0].c_str()));
	m_max = static_cast<double>(atof(stringFloatRange[1].c_str()));
}

//-----------------------------------------------------------------------------------------------
bool DoubleRange::operator==(DoubleRange const& compare) const
{
	if (compare.m_min == m_min && compare.m_max == m_max)
	{
		return true;
	}

	return false;
}

//-----------------------------------------------------------------------------------------------
bool DoubleRange::operator!=(DoubleRange const& compare) const
{
	if (compare.m_min != m_min || compare.m_max != m_max)
	{
		return true;
	}

	return false;
}

//-----------------------------------------------------------------------------------------------
void DoubleRange::operator=(DoubleRange const& copyFrom)
{
	m_min = copyFrom.m_min;
	m_max = copyFrom.m_max;
}

//-----------------------------------------------------------------------------------------------
bool DoubleRange::IsOnRange(double value)
{
	if (value >= m_min && value <= m_max)
	{
		return true;
	}

	return false;
}

//-----------------------------------------------------------------------------------------------
bool DoubleRange::IsOverlappingWith(DoubleRange const& compare)
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
