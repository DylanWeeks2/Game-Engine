#include "IntVec3.hpp"
#include "MathUtils.hpp"
#include "Engine/Core/StringUtils.hpp"
#include <math.h>

//-----------------------------------------------------------------------------------------------
IntVec3::IntVec3(const IntVec3& copyFrom)
	:x(copyFrom.x)
	,y(copyFrom.y)
	,z(copyFrom.z)
{
}

//-----------------------------------------------------------------------------------------------
IntVec3::IntVec3(int initialX, int initialY, int initialZ)
	: x(initialX)
	, y(initialY)
	, z(initialZ)
{
}

//-----------------------------------------------------------------------------------------------
void IntVec3::SetFromText(char const* text)
{
	Strings stringXY;
	stringXY = SplitStringOnDelimiter(text, ',');

	x = atoi(stringXY[0].c_str());
	y = atoi(stringXY[1].c_str());
	z = atoi(stringXY[2].c_str());
}

//-----------------------------------------------------------------------------------------------
float IntVec3::GetLength() const
{
	return sqrtf(static_cast<float>(x * x + y * y + z * z));
}

//-----------------------------------------------------------------------------------------------
int IntVec3::GetTaxicabLength() const
{
	return static_cast<int>(fabsf(static_cast<float>(x)) + fabsf(static_cast<float>(y)) + fabsf(static_cast<float>(z)));
}

//-----------------------------------------------------------------------------------------------
int IntVec3::GetLengthSquared() const
{
	return static_cast<int>(GetLength() * GetLength());
}

//-----------------------------------------------------------------------------------------------
void IntVec3::operator=(const IntVec3& copyFrom)
{
	x = copyFrom.x;
	y = copyFrom.y;
	z = copyFrom.z;
}

//-----------------------------------------------------------------------------------------------
bool IntVec3::operator==(const IntVec3& compare) const
{
	if (compare.x == x && compare.y == y && compare.z == z)
	{
		return true;
	}

	return false;
}

//-----------------------------------------------------------------------------------------------
const IntVec3 IntVec3::operator+(const IntVec3& intVecToAdd) const
{
	return IntVec3(x + intVecToAdd.x, y + intVecToAdd.y, z + intVecToAdd.z);
}

//-----------------------------------------------------------------------------------------------
const IntVec3 IntVec3::operator-(const IntVec3& intVecToAdd) const
{
	return IntVec3(x - intVecToAdd.x, y - intVecToAdd.y, z - intVecToAdd.z);
}

//-----------------------------------------------------------------------------------------------
bool IntVec3::operator<(const IntVec3& compareTo) const
{
	if (x < compareTo.x)
	{
		return true;
	}
	else if (x == compareTo.x)
	{
		if (y < compareTo.y)
		{
			return true;
		}
		else if (y == compareTo.y)
		{
			if (z < compareTo.z)
			{
				return true;
			}
		}
	}

	return false;
}
