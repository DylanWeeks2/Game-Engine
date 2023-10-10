#include "Engine/Math/Vec2.hpp"
#include <math.h>
#include "Engine/Math/MathUtils.hpp"
#include "Engine/Core/StringUtils.hpp"
//#include "Engine/Core/EngineCommon.hpp"

//-----------------------------------------------------------------------------------------------
Vec2::Vec2(const Vec2& copy)
	: x(copy.x)
	, y(copy.y)
{
}


//-----------------------------------------------------------------------------------------------
Vec2::Vec2(float initialX, float initialY)
	: x(initialX)
	, y(initialY)
{
}

//-----------------------------------------------------------------------------------------------
void Vec2::SetFromText(char const* text)
{
	Strings stringXY;
	stringXY = SplitStringOnDelimiter(text, ',');
	
	x = static_cast<float>(atof(stringXY[0].c_str()));
	y = static_cast<float>(atof(stringXY[1].c_str()));
}

//-----------------------------------------------------------------------------------------------
Vec2 const Vec2::MakeFromPolarRadians(float orientationRadians, float length)
{
	return Vec2(length * cosf(orientationRadians), length * sinf(orientationRadians));
}

//-----------------------------------------------------------------------------------------------
Vec2 const Vec2::MakeFromPolarDegrees(float orientationDegrees, float length)
{
	return Vec2(length * CosDegrees(orientationDegrees), length * SinDegrees(orientationDegrees));
}

//-----------------------------------------------------------------------------------------------
float Vec2::GetLength() const
{
	float xSquared = x * x;
	float ySquared = y * y;
	float sqrtValue = xSquared + ySquared;
	return sqrtf(sqrtValue);
}

//-----------------------------------------------------------------------------------------------
float Vec2::GetLengthSquared() const
{
	float xSquared = x * x;
	float ySquared = y * y;
	float sqrtValue = xSquared + ySquared;
	float squareRoot = sqrtf(sqrtValue);
	return squareRoot * squareRoot;
}

//-----------------------------------------------------------------------------------------------
float Vec2::GetOrientationRadians() const
{
	return atan2f(y,x);
}

//-----------------------------------------------------------------------------------------------
float Vec2::GetOrientationDegrees() const
{
	return Atan2Degrees(y, x);
}

//-----------------------------------------------------------------------------------------------
Vec2 const Vec2::GetRotated90Degrees() const
{
	return Vec2(-y, x);
}

//-----------------------------------------------------------------------------------------------
Vec2 const Vec2::GetRotatedMinus90Degrees() const
{
	return Vec2(y, -x);
}

//-----------------------------------------------------------------------------------------------
Vec2 const Vec2::GetRotatedRadians(float deltaRadians) const
{
	return Vec2((x * cosf(deltaRadians)) - (y * sinf(deltaRadians)), (x * sinf(deltaRadians)) + (y * cosf(deltaRadians)));
}

//-----------------------------------------------------------------------------------------------
Vec2 const Vec2::GetRotatedDegrees(float deltaDegrees) const
{
	return Vec2((x * CosDegrees(deltaDegrees)) - (y * SinDegrees(deltaDegrees)), (x * SinDegrees(deltaDegrees)) + (y * CosDegrees(deltaDegrees)));
}

//-----------------------------------------------------------------------------------------------
Vec2 const Vec2::GetClamped(float maxLength) const
{
	float currentLength = GetLength();

	if (currentLength > maxLength)
	{
		float scale = 1 / (currentLength / maxLength);
		return Vec2(x * scale, y * scale);
	}

	return Vec2(x, y);
}

//-----------------------------------------------------------------------------------------------
Vec2 const Vec2::GetNormalized() const
{
	if (x == 0.0f && y == 0.0f)
	{
		return Vec2(0.0f, 0.0f);
	}
	float scale = 1.0f / GetLength();
	return Vec2(x * scale, y * scale);
}

//-----------------------------------------------------------------------------------------------
Vec2 const Vec2::GetReflected(Vec2 const& impactSurfaceNormal) const
{
	Vec2 translationDown = DotProduct2D(Vec2(x, y), impactSurfaceNormal) * impactSurfaceNormal;
	Vec2 tanslationOver = Vec2(x, y) - translationDown;
	return tanslationOver + translationDown * -1.0f;
}

//-----------------------------------------------------------------------------------------------
void Vec2::SetOrientationRadians(float newOrientationRadians)
{
	SetPolarRadians(newOrientationRadians, GetLength());
}

//-----------------------------------------------------------------------------------------------
void Vec2::SetOrientationDegrees(float newOrientationDegrees)
{
	SetPolarDegrees(newOrientationDegrees, GetLength());
}

//-----------------------------------------------------------------------------------------------
void Vec2::SetPolarRadians(float newOrientationRadians, float newLength)
{
	x = newLength * cosf(newOrientationRadians);
	y = newLength * sinf(newOrientationRadians);
}

//-----------------------------------------------------------------------------------------------
void Vec2::SetPolarDegrees(float newOrientaitonDegrees, float newLength)
{
	x = newLength * CosDegrees(newOrientaitonDegrees);
	y = newLength * SinDegrees(newOrientaitonDegrees);
}

//-----------------------------------------------------------------------------------------------
void Vec2::Rotate90Degrees()
{
	float tempX = x;
	float tempY = y;
	x = -tempY;
	y = tempX;
}

//-----------------------------------------------------------------------------------------------
void Vec2::RotateMinus90Degrees()
{
	float tempX = x;
	float tempY = y;
	x = tempY;
	y = -tempX;
}

//-----------------------------------------------------------------------------------------------
void Vec2::RotateRadians(float deltaRadians)
{
	float tempX = x;
	float tempY = y;
	x = (tempX * cosf(deltaRadians)) - (tempY * sinf(deltaRadians));
	y = (tempX * sinf(deltaRadians)) + (tempY * cosf(deltaRadians));
}

//-----------------------------------------------------------------------------------------------
void Vec2::RotateDegrees(float deltaDegrees)
{
	float tempX = x;
	float tempY = y;
	x = (tempX * CosDegrees(deltaDegrees)) - (tempY * SinDegrees(deltaDegrees));
	y = (tempX * SinDegrees(deltaDegrees)) + (tempY * CosDegrees(deltaDegrees));
}

//-----------------------------------------------------------------------------------------------
void Vec2::SetLength(float newLength)
{
	*this = GetNormalized() * newLength;
}

//-----------------------------------------------------------------------------------------------
void Vec2::ClampLength(float maxLength)
{
	float currentLength = GetLength();
	if (currentLength > maxLength)
	{
		float scale = 1 / (currentLength / maxLength);
		x *= scale;
		y *= scale;
	}
}

//-----------------------------------------------------------------------------------------------
void Vec2::Normalize()
{
	float scale = 1.0f / GetLength();
	x *= scale;
	y *= scale;
}

//-----------------------------------------------------------------------------------------------
float Vec2::NormalizeAndGetPreviousLength()
{ //INCORRECT
	float previousLength = GetLength();
	Normalize();
	return previousLength;
}

//-----------------------------------------------------------------------------------------------
void Vec2::Reflect(Vec2 const& impactSurfaceNormal)
{
	Vec2 translationDown = DotProduct2D(Vec2(x, y), impactSurfaceNormal) * impactSurfaceNormal;
	Vec2 tanslationOver = Vec2(x, y) - translationDown;
	Vec2 reflected =  tanslationOver + translationDown * -1.0f;
	x = reflected.x;
	y = reflected.y;
}

//-----------------------------------------------------------------------------------------------
const Vec2 Vec2::operator + (const Vec2& vecToAdd) const
{
	return Vec2(x + vecToAdd.x, y + vecToAdd.y);
}


//-----------------------------------------------------------------------------------------------
const Vec2 Vec2::operator-(const Vec2& vecToSubtract) const
{
	return Vec2(x - vecToSubtract.x, y - vecToSubtract.y);
}


//------------------------------------------------------------------------------------------------
const Vec2 Vec2::operator-() const
{
	return Vec2(123.f, 456.f);
}


//-----------------------------------------------------------------------------------------------
const Vec2 Vec2::operator*(float uniformScale) const
{
	return Vec2(x * uniformScale, y * uniformScale);
}


//------------------------------------------------------------------------------------------------
const Vec2 Vec2::operator*(const Vec2& vecToMultiply) const
{
	return Vec2(x * vecToMultiply.x, y * vecToMultiply.y);
}


//-----------------------------------------------------------------------------------------------
const Vec2 Vec2::operator/(float inverseScale) const
{
	float scale = (1.0f / inverseScale);
	return Vec2(x * scale, y * scale);
}


//-----------------------------------------------------------------------------------------------
void Vec2::operator+=(const Vec2& vecToAdd)
{
	x += vecToAdd.x;
	y += vecToAdd.y;
}


//-----------------------------------------------------------------------------------------------
void Vec2::operator-=(const Vec2& vecToSubtract)
{
	x -= vecToSubtract.x;
	y -= vecToSubtract.y;
}


//-----------------------------------------------------------------------------------------------
void Vec2::operator*=(const float uniformScale)
{
	x *= uniformScale;
	y *= uniformScale;
}


//-----------------------------------------------------------------------------------------------
void Vec2::operator/=(const float uniformDivisor)
{
	float scale = (1.0f / uniformDivisor);
	x *= scale;
	y *= scale;
}


//-----------------------------------------------------------------------------------------------
void Vec2::operator=(const Vec2& copyFrom)
{
	x = copyFrom.x;
	y = copyFrom.y;
}


//-----------------------------------------------------------------------------------------------
const Vec2 operator*(float uniformScale, const Vec2& vecToScale)
{
	return Vec2(uniformScale * vecToScale.x, uniformScale * vecToScale.y);
}


//-----------------------------------------------------------------------------------------------
bool Vec2::operator==(const Vec2& compare) const
{
	if (compare.x == x && compare.y == y)
	{
		return true;
	}

	return false;
}


//-----------------------------------------------------------------------------------------------
bool Vec2::operator!=(const Vec2& compare) const
{
	if ((compare.x != x) || (compare.y != y))
	{
		return true;
	}

	return false;
}

