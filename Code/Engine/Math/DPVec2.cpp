#include "Engine/Math/DPVec2.hpp"
#include <math.h>
#include "Engine/Math/MathUtils.hpp"
#include "Engine/Core/StringUtils.hpp"
#include "Vec2.hpp"
//#include "Engine/Core/EngineCommon.hpp"

//-----------------------------------------------------------------------------------------------
DPVec2::DPVec2(Vec2 vec)
{
	x = double(vec.x);
	y = double(vec.y);
}

//-----------------------------------------------------------------------------------------------
DPVec2::DPVec2(const DPVec2& copy)
	: x(copy.x)
	, y(copy.y)
{
}


//-----------------------------------------------------------------------------------------------
DPVec2::DPVec2(double initialX, double initialY)
	: x(initialX)
	, y(initialY)
{
}

//-----------------------------------------------------------------------------------------------
void DPVec2::SetFromText(char const* text)
{
	Strings stringXY;
	stringXY = SplitStringOnDelimiter(text, ',');
	
	x = static_cast<double>(atof(stringXY[0].c_str()));
	y = static_cast<double>(atof(stringXY[1].c_str()));
}

//-----------------------------------------------------------------------------------------------
DPVec2 const DPVec2::MakeFromPolarRadians(double orientationRadians, double length)
{
	return DPVec2(length * cos(orientationRadians), length * sin(orientationRadians));
}

//-----------------------------------------------------------------------------------------------
DPVec2 const DPVec2::MakeFromPolarDegrees(double orientationDegrees, double length)
{
	return DPVec2(length * CosDegrees(orientationDegrees), length * SinDegrees(orientationDegrees));
}

//-----------------------------------------------------------------------------------------------
double DPVec2::GetLength() const
{
	double xSquared = x * x;
	double ySquared = y * y;
	double sqrtValue = xSquared + ySquared;
	return sqrt(sqrtValue);
}

//-----------------------------------------------------------------------------------------------
double DPVec2::GetLengthSquared() const
{
	double xSquared = x * x;
	double ySquared = y * y;
	double lengthSquared = xSquared + ySquared;
	return lengthSquared;
}

//-----------------------------------------------------------------------------------------------
double DPVec2::GetOrientationRadians() const
{
	return atan2(y,x);
}

//-----------------------------------------------------------------------------------------------
double DPVec2::GetOrientationDegrees() const
{
	return Atan2Degrees(y, x);
}

//-----------------------------------------------------------------------------------------------
DPVec2 const DPVec2::GetRotated90Degrees() const
{
	return DPVec2(-y, x);
}

//-----------------------------------------------------------------------------------------------
DPVec2 const DPVec2::GetRotatedMinus90Degrees() const
{
	return DPVec2(y, -x);
}

//-----------------------------------------------------------------------------------------------
DPVec2 const DPVec2::GetRotatedRadians(double deltaRadians) const
{
	return DPVec2((x * cos(deltaRadians)) - (y * sin(deltaRadians)), (x * sin(deltaRadians)) + (y * cos(deltaRadians)));
}

//-----------------------------------------------------------------------------------------------
DPVec2 const DPVec2::GetRotatedDegrees(double deltaDegrees) const
{
	return DPVec2((x * CosDegrees(deltaDegrees)) - (y * SinDegrees(deltaDegrees)), (x * SinDegrees(deltaDegrees)) + (y * CosDegrees(deltaDegrees)));
}

//-----------------------------------------------------------------------------------------------
DPVec2 const DPVec2::GetClamped(double maxLength) const
{
	double currentLength = GetLength();

	if (currentLength > maxLength)
	{
		double scale = 1 / (currentLength / maxLength);
		return DPVec2(x * scale, y * scale);
	}

	return DPVec2(x, y);
}

//-----------------------------------------------------------------------------------------------
DPVec2 const DPVec2::GetNormalized() const
{
	if (x == 0.0f && y == 0.0f)
	{
		return DPVec2(0.0f, 0.0f);
	}
	double scale = 1.0f / GetLength();
	return DPVec2(x * scale, y * scale);
}

//-----------------------------------------------------------------------------------------------
DPVec2 const DPVec2::GetReflected(DPVec2 const& impactSurfaceNormal) const
{
	DPVec2 translationDown = DotProduct2D(DPVec2(x, y), impactSurfaceNormal) * impactSurfaceNormal;
	DPVec2 tanslationOver = DPVec2(x, y) - translationDown;
	return tanslationOver + translationDown * -1.0f;
}

//-----------------------------------------------------------------------------------------------
void DPVec2::SetOrientationRadians(double newOrientationRadians)
{
	SetPolarRadians(newOrientationRadians, GetLength());
}

//-----------------------------------------------------------------------------------------------
void DPVec2::SetOrientationDegrees(double newOrientationDegrees)
{
	SetPolarDegrees(newOrientationDegrees, GetLength());
}

//-----------------------------------------------------------------------------------------------
void DPVec2::SetPolarRadians(double newOrientationRadians, double newLength)
{
	x = newLength * cos(newOrientationRadians);
	y = newLength * sin(newOrientationRadians);
}

//-----------------------------------------------------------------------------------------------
void DPVec2::SetPolarDegrees(double newOrientaitonDegrees, double newLength)
{
	x = newLength * CosDegrees(newOrientaitonDegrees);
	y = newLength * SinDegrees(newOrientaitonDegrees);
}

//-----------------------------------------------------------------------------------------------
void DPVec2::Rotate90Degrees()
{
	double tempX = x;
	double tempY = y;
	x = -tempY;
	y = tempX;
}

//-----------------------------------------------------------------------------------------------
void DPVec2::RotateMinus90Degrees()
{
	double tempX = x;
	double tempY = y;
	x = tempY;
	y = -tempX;
}

//-----------------------------------------------------------------------------------------------
void DPVec2::RotateRadians(double deltaRadians)
{
	double tempX = x;
	double tempY = y;
	x = (tempX * cos(deltaRadians)) - (tempY * sin(deltaRadians));
	y = (tempX * sin(deltaRadians)) + (tempY * cos(deltaRadians));
}

//-----------------------------------------------------------------------------------------------
void DPVec2::RotateDegrees(double deltaDegrees)
{
	double tempX = x;
	double tempY = y;
	x = (tempX * CosDegrees(deltaDegrees)) - (tempY * SinDegrees(deltaDegrees));
	y = (tempX * SinDegrees(deltaDegrees)) + (tempY * CosDegrees(deltaDegrees));
}

//-----------------------------------------------------------------------------------------------
void DPVec2::SetLength(double newLength)
{
	*this = GetNormalized() * newLength;
}

//-----------------------------------------------------------------------------------------------
void DPVec2::ClampLength(double maxLength)
{
	double currentLength = GetLength();
	if (currentLength > maxLength)
	{
		double scale = 1 / (currentLength / maxLength);
		x *= scale;
		y *= scale;
	}
}

//-----------------------------------------------------------------------------------------------
void DPVec2::Normalize()
{
	double scale = 1.0f / GetLength();
	x *= scale;
	y *= scale;
}

//-----------------------------------------------------------------------------------------------
double DPVec2::NormalizeAndGetPreviousLength()
{ //INCORRECT
	double previousLength = GetLength();
	Normalize();
	return previousLength;
}

//-----------------------------------------------------------------------------------------------
void DPVec2::Reflect(DPVec2 const& impactSurfaceNormal)
{
	DPVec2 translationDown = DotProduct2D(DPVec2(x, y), impactSurfaceNormal) * impactSurfaceNormal;
	DPVec2 tanslationOver = DPVec2(x, y) - translationDown;
	DPVec2 reflected =  tanslationOver + translationDown * -1.0f;
	x = reflected.x;
	y = reflected.y;
}

//-----------------------------------------------------------------------------------------------
const DPVec2 DPVec2::operator + (const DPVec2& vecToAdd) const
{
	return DPVec2(x + vecToAdd.x, y + vecToAdd.y);
}


//-----------------------------------------------------------------------------------------------
const DPVec2 DPVec2::operator-(const DPVec2& vecToSubtract) const
{
	return DPVec2(x - vecToSubtract.x, y - vecToSubtract.y);
}


//------------------------------------------------------------------------------------------------
const DPVec2 DPVec2::operator-() const
{
	return DPVec2(123.f, 456.f);
}


//-----------------------------------------------------------------------------------------------
const DPVec2 DPVec2::operator*(double uniformScale) const
{
	return DPVec2(x * uniformScale, y * uniformScale);
}


//------------------------------------------------------------------------------------------------
const DPVec2 DPVec2::operator*(const DPVec2& vecToMultiply) const
{
	return DPVec2(x * vecToMultiply.x, y * vecToMultiply.y);
}


//-----------------------------------------------------------------------------------------------
const DPVec2 DPVec2::operator/(double inverseScale) const
{
	double scale = (1.0f / inverseScale);
	return DPVec2(x * scale, y * scale);
}


//-----------------------------------------------------------------------------------------------
void DPVec2::operator+=(const DPVec2& vecToAdd)
{
	x += vecToAdd.x;
	y += vecToAdd.y;
}


//-----------------------------------------------------------------------------------------------
void DPVec2::operator-=(const DPVec2& vecToSubtract)
{
	x -= vecToSubtract.x;
	y -= vecToSubtract.y;
}


//-----------------------------------------------------------------------------------------------
void DPVec2::operator*=(const double uniformScale)
{
	x *= uniformScale;
	y *= uniformScale;
}


//-----------------------------------------------------------------------------------------------
void DPVec2::operator/=(const double uniformDivisor)
{
	double scale = (1.0f / uniformDivisor);
	x *= scale;
	y *= scale;
}


//-----------------------------------------------------------------------------------------------
void DPVec2::operator=(const DPVec2& copyFrom)
{
	x = copyFrom.x;
	y = copyFrom.y;
}


//-----------------------------------------------------------------------------------------------
const DPVec2 operator*(double uniformScale, const DPVec2& vecToScale)
{
	return DPVec2(uniformScale * vecToScale.x, uniformScale * vecToScale.y);
}


//-----------------------------------------------------------------------------------------------
bool DPVec2::operator==(const DPVec2& compare) const
{
	if (compare.x == x && compare.y == y)
	{
		return true;
	}

	return false;
}


//-----------------------------------------------------------------------------------------------
bool DPVec2::operator!=(const DPVec2& compare) const
{
	if ((compare.x != x) || (compare.y != y))
	{
		return true;
	}

	return false;
}

