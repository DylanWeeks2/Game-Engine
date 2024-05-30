#include "DPVec3.hpp"
#include <math.h>
#include "MathUtils.hpp"
#include "Engine/Core/StringUtils.hpp"

//-----------------------------------------------------------------------------------------------
DPVec3::DPVec3(Vec3 vec)
{
	x = double(vec.x);  y = double(vec.y); z = double(vec.z);
}

//-----------------------------------------------------------------------------------------------
DPVec3::DPVec3(double initialX, double initialY, double initialZ)
	: x(initialX)
	, y(initialY)
	, z(initialZ) 
{
}

//-----------------------------------------------------------------------------------------------
void DPVec3::SetFromText(char const* text)
{
	Strings stringXYZ;
	stringXYZ = SplitStringOnDelimiter(text, ',');

	x = static_cast<double>(atof(stringXYZ[0].c_str()));
	y = static_cast<double>(atof(stringXYZ[1].c_str()));
	z = static_cast<double>(atof(stringXYZ[2].c_str()));
}

//-----------------------------------------------------------------------------------------------
double DPVec3::GetLength() const
{
	double xSquared = x * x;
	double ySquared = y * y;
	double zSquared = z * z;
	double sqrtValue = xSquared + ySquared + zSquared;
	return double(sqrt(sqrtValue));
}

//-----------------------------------------------------------------------------------------------
double DPVec3::GetLengthXY() const
{
	double xSquared = x * x;
	double ySquared = y * y;
	double sqrtValue = xSquared + ySquared;
	return double(sqrt(sqrtValue));
}

//-----------------------------------------------------------------------------------------------
double DPVec3::GetLengthSquared() const
{
	double xSquared = x * x;
	double ySquared = y * y;
	double zSquared = z * z;
	double sqrtValue = xSquared + ySquared + zSquared;
	return sqrtValue;
}

//-----------------------------------------------------------------------------------------------
double DPVec3::GetLengthXYSquared() const
{
	double squareRoot = GetLengthXY();
	return squareRoot * squareRoot;
}

//-----------------------------------------------------------------------------------------------
double DPVec3::GetAngleAboutZRadians() const
{
	return ConvertDegreesToRadians(Atan2Degrees(y, x));
}

//-----------------------------------------------------------------------------------------------
double DPVec3::GetAngleAboutZDegrees() const
{
	return Atan2Degrees(y, x);
}

//-----------------------------------------------------------------------------------------------
DPVec3 const DPVec3::GetRotatedAboutZRadians(double deltaRadians) const
{
	return DPVec3((x * CosDegrees(ConvertRadiansToDegrees(deltaRadians))) - (y * SinDegrees(ConvertRadiansToDegrees(deltaRadians))), (x * SinDegrees(ConvertRadiansToDegrees(deltaRadians))) + (y * CosDegrees(ConvertRadiansToDegrees(deltaRadians))), z);
}

//-----------------------------------------------------------------------------------------------
DPVec3 const DPVec3::GetRotatedAboutZDegrees(double deltaDegrees) const
{
	return DPVec3((x * CosDegrees(deltaDegrees)) - (y * SinDegrees(deltaDegrees)), (x * SinDegrees(deltaDegrees)) + (y * CosDegrees(deltaDegrees)), z);
}

//-----------------------------------------------------------------------------------------------
DPVec3 const DPVec3::GetClamped(double maxLength) const
{
	double currentLength = GetLength();

	if (currentLength > maxLength)
	{
		double scale = 1 / (currentLength / maxLength);
		return DPVec3(x * scale, y * scale, z * scale);
	}

	return DPVec3(x, y, z);
}

//-----------------------------------------------------------------------------------------------
DPVec3 const DPVec3::GetNormalized() const
{
	if (GetLength() == 0.0f)
	{
		return DPVec3();
	}

	double scale = 1.0f / GetLength();
	return DPVec3(x * scale, y * scale, z * scale);
}

//-----------------------------------------------------------------------------------------------
void DPVec3::SetLength(double maxLength)
{
	*this = GetNormalized() * maxLength;
}

//-----------------------------------------------------------------------------------------------
bool DPVec3::operator==(DPVec3 const& compare) const
{
	if (compare.x == x && compare.y == y && compare.z == z)
	{
		return true;
	}
	return false;
}

//-----------------------------------------------------------------------------------------------
bool DPVec3::operator!=(DPVec3 const& compare) const
{
	if (compare.x != x || compare.y != y || compare.z != z)
	{
		return true;
	}
	return false;
}

//-----------------------------------------------------------------------------------------------
DPVec3 const DPVec3::operator+(DPVec3 const& vecToAdd) const
{
	return DPVec3(vecToAdd.x + x, vecToAdd.y + y, vecToAdd.z + z);
}

//-----------------------------------------------------------------------------------------------
DPVec3 const DPVec3::operator-(DPVec3 const& vecToSubtract) const
{
	return DPVec3(x - vecToSubtract.x, y - vecToSubtract.y, z - vecToSubtract.z);
}

//-----------------------------------------------------------------------------------------------
DPVec3 const DPVec3::operator*(double uniformScale) const
{
	return DPVec3(x * uniformScale, y * uniformScale, z * uniformScale);
}

//-----------------------------------------------------------------------------------------------
DPVec3 const DPVec3::operator/(double inverseScale) const
{
	double scale = (1.0f / inverseScale);
	return DPVec3(x * scale, y * scale, z * scale);
}

//-----------------------------------------------------------------------------------------------
void DPVec3::operator+=(DPVec3 const& vecToAdd)
{
	x += vecToAdd.x;
	y += vecToAdd.y;
	z += vecToAdd.z;
}

//-----------------------------------------------------------------------------------------------
void DPVec3::operator-=(DPVec3 const& vecToSubtract)
{
	x -= vecToSubtract.x;
	y -= vecToSubtract.y;
	z -= vecToSubtract.z;
}

//-----------------------------------------------------------------------------------------------
void DPVec3::operator*=(double uniformScale)
{
	x *= uniformScale;
	y *= uniformScale;
	z *= uniformScale;
}

//-----------------------------------------------------------------------------------------------
void DPVec3::operator/=(double uniformDivisor)
{
	double scale = (1.0f / uniformDivisor);
	x *= scale;
	y *= scale;
	z *= scale;
}

//-----------------------------------------------------------------------------------------------
void DPVec3::operator=(DPVec3 const& copyFrom)
{
	x = copyFrom.x;
	y = copyFrom.y;
	z = copyFrom.z;
}

//-----------------------------------------------------------------------------------------------
DPVec3 DPVec3::operator*(const DPVec3& vecToMultiply) 
{
	return DPVec3(x * vecToMultiply.x, y * vecToMultiply.y, z * vecToMultiply.z);
}

//-----------------------------------------------------------------------------------------------
void DPVec3::Normalize()
{
	double scale = 1.0f / GetLength();
	x *= scale;
	y *= scale;
	z *= scale;
}

//-----------------------------------------------------------------------------------------------
DPVec3 const DPVec3::MakeFromPolarRadians(double longitudeRadians, double latitudeRadians, double length)
{
	//long = yaw
	//lat = pitch
	return DPVec3(length * cos(longitudeRadians) * cos(latitudeRadians), length * sin(longitudeRadians) * cos(latitudeRadians), length * -sin(latitudeRadians));
}

//-----------------------------------------------------------------------------------------------
DPVec3 const DPVec3::MakeFromPolarDegrees(double longitudeDegrees, double latitudeDegrees, double length)
{
	//long = yaw
	//lat = pitch
	return DPVec3(length * CosDegrees(longitudeDegrees) * CosDegrees(latitudeDegrees), length * SinDegrees(longitudeDegrees) * CosDegrees(latitudeDegrees), length * -SinDegrees(latitudeDegrees));
}

//-----------------------------------------------------------------------------------------------
DPVec3 const operator*(double uniformScale, DPVec3 const& vecToScale)
{
	return DPVec3(vecToScale.x * uniformScale, vecToScale.y * uniformScale, vecToScale.z * uniformScale);
}

//-----------------------------------------------------------------------------------------------
void DPVec3::ClampLength(double maxLength)
{
	double currentLength = GetLength();
	if (currentLength > maxLength)
	{
		double scale = 1 / (currentLength / maxLength);
		x *= scale;
		y *= scale;
		z *= scale;
	}
}