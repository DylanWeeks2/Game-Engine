#include "IntVec2.hpp"
#include "MathUtils.hpp"
#include "Engine/Core/StringUtils.hpp"
#include <math.h>

//-----------------------------------------------------------------------------------------------
IntVec2::IntVec2(const IntVec2& copyFrom)
	:x(copyFrom.x)
	,y(copyFrom.y)
{
}

//-----------------------------------------------------------------------------------------------
IntVec2::IntVec2(int initialX, int initialY)
	: x(initialX)
	, y(initialY)
{
}

//-----------------------------------------------------------------------------------------------
void IntVec2::SetFromText(char const* text)
{
	Strings stringXY;
	stringXY = SplitStringOnDelimiter(text, ',');

	x = atoi(stringXY[0].c_str());
	y = atoi(stringXY[1].c_str());
}

//-----------------------------------------------------------------------------------------------
float IntVec2::GetLength() const
{
	return sqrtf(static_cast<float>(x * x + y * y));
}

//-----------------------------------------------------------------------------------------------
int IntVec2::GetTaxicabLength() const
{
	return static_cast<int>(fabsf(static_cast<float>(x)) + fabsf(static_cast<float>(y)));
}

//-----------------------------------------------------------------------------------------------
int IntVec2::GetLengthSquared() const
{
	return static_cast<int>(GetLength() * GetLength());
}

//-----------------------------------------------------------------------------------------------
float IntVec2::GetOrientationRadians() const
{
	return atan2f(static_cast<float>(y), static_cast<float>(x));
}

//-----------------------------------------------------------------------------------------------
float IntVec2::GetOrientationDegrees() const
{
	return Atan2Degrees(static_cast<float>(y), static_cast<float>(x));
}

//-----------------------------------------------------------------------------------------------
IntVec2 const IntVec2::GetRotated90Degrees() const
{
	return IntVec2(-y, x);
}

//-----------------------------------------------------------------------------------------------
IntVec2 const IntVec2::GetRotatedMinus90Degrees() const
{
	return IntVec2(y, -x);
}

//-----------------------------------------------------------------------------------------------
void IntVec2::Rotate90Degrees()
{
	int tempX = x;

	x = -y;
	y = tempX;
}

//-----------------------------------------------------------------------------------------------
void IntVec2::RotateMinus90Degrees()
{
	int tempX = x;

	x = y;
	y = -tempX;
}

//-----------------------------------------------------------------------------------------------
void IntVec2::operator=(const IntVec2& copyFrom)
{
	x = copyFrom.x;
	y = copyFrom.y;
}

//-----------------------------------------------------------------------------------------------
bool IntVec2::operator<(const IntVec2& compareTo) const
{
	if (x < compareTo.x)
	{
		return true;
	}
	else if(x == compareTo.x)
	{
		if (y < compareTo.y)
		{
			return true;
		}
	}

	return false;
}

//-----------------------------------------------------------------------------------------------
bool IntVec2::operator==(const IntVec2& checkValue)
{
	if (x == checkValue.x && y == checkValue.y)
	{
		return true;
	}

	return false;
}

//-----------------------------------------------------------------------------------------------
bool IntVec2::operator!=(const IntVec2& checkValue)
{
	if (x != checkValue.x || y != checkValue.y)
	{
		return true;
	}
	return false;
}

//-----------------------------------------------------------------------------------------------
const IntVec2 IntVec2::operator+(const IntVec2& vecToAdd) const
{
	return IntVec2(x + vecToAdd.x, y + vecToAdd.y);
}

//-----------------------------------------------------------------------------------------------
const IntVec2 IntVec2::operator-(const IntVec2& vecToSubtract) const
{
	return IntVec2(x - vecToSubtract.x, y - vecToSubtract.y);
}
