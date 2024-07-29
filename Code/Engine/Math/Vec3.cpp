#include "Vec3.hpp"
#include <math.h>
#include "MathUtils.hpp"
#include "Engine/Core/StringUtils.hpp"

//-----------------------------------------------------------------------------------------------
Vec3::Vec3(float initialX, float initialY, float initialZ)
	: x(initialX)
	, y(initialY)
	, z(initialZ) 
{
}

//-----------------------------------------------------------------------------------------------
Vec3::Vec3(DPVec3 dpVec)
{
	x = float(dpVec.x);  y = float(dpVec.y); z = float(dpVec.z);
}

//-----------------------------------------------------------------------------------------------
void Vec3::SetFromText(char const* text)
{
	Strings stringXYZ;
	stringXYZ = SplitStringOnDelimiter(text, ',');

	x = static_cast<float>(atof(stringXYZ[0].c_str()));
	y = static_cast<float>(atof(stringXYZ[1].c_str()));
	z = static_cast<float>(atof(stringXYZ[2].c_str()));
}

//-----------------------------------------------------------------------------------------------
float Vec3::GetLength() const
{
	float xSquared = x * x;
	float ySquared = y * y;
	float zSquared = z * z;
	float sqrtValue = xSquared + ySquared + zSquared;
	return float(sqrt(sqrtValue));
}

//-----------------------------------------------------------------------------------------------
float Vec3::GetLengthXY() const
{
	float xSquared = x * x;
	float ySquared = y * y;
	float sqrtValue = xSquared + ySquared;
	return float(sqrt(sqrtValue));
}

//-----------------------------------------------------------------------------------------------
float Vec3::GetLengthSquared() const
{
	float squareRoot = GetLength();
	return squareRoot * squareRoot;
}

//-----------------------------------------------------------------------------------------------
float Vec3::GetLengthXYSquared() const
{
	float squareRoot = GetLengthXY();
	return squareRoot * squareRoot;
}

//-----------------------------------------------------------------------------------------------
float Vec3::GetAngleAboutZRadians() const
{
	return ConvertDegreesToRadians(Atan2Degrees(y, x));
}

//-----------------------------------------------------------------------------------------------
float Vec3::GetAngleAboutZDegrees() const
{
	return Atan2Degrees(y, x);
}

//-----------------------------------------------------------------------------------------------
Vec3 const Vec3::GetRotatedAboutZRadians(float deltaRadians) const
{
	return Vec3((x * CosDegrees(ConvertRadiansToDegrees(deltaRadians))) - (y * SinDegrees(ConvertRadiansToDegrees(deltaRadians))), (x * SinDegrees(ConvertRadiansToDegrees(deltaRadians))) + (y * CosDegrees(ConvertRadiansToDegrees(deltaRadians))), z);
}

//-----------------------------------------------------------------------------------------------
Vec3 const Vec3::GetRotatedAboutZDegrees(float deltaDegrees) const
{
	return Vec3((x * CosDegrees(deltaDegrees)) - (y * SinDegrees(deltaDegrees)), (x * SinDegrees(deltaDegrees)) + (y * CosDegrees(deltaDegrees)), z);
}

//-----------------------------------------------------------------------------------------------
Vec3 const Vec3::GetClamped(float maxLength) const
{
	float currentLength = GetLength();

	if (currentLength > maxLength)
	{
		float scale = 1 / (currentLength / maxLength);
		return Vec3(x * scale, y * scale, z * scale);
	}

	return Vec3(x, y, z);
}

//-----------------------------------------------------------------------------------------------
Vec3 const Vec3::GetNormalized() const
{
	if (GetLength() == 0.0f)
	{
		return Vec3();
	}

	float scale = 1.0f / GetLength();
	return Vec3(x * scale, y * scale, z * scale);
}

//-----------------------------------------------------------------------------------------------
Vec3 const Vec3::GetReflected(Vec3 const& impactSurfaceNormal) const
{
	Vec3 translationDown = DotProduct3D(Vec3(x, y, z), impactSurfaceNormal) * impactSurfaceNormal;
	Vec3 tanslationOver = Vec3(x, y, z) - translationDown;
	return tanslationOver + translationDown * -1.0f;
}

//-----------------------------------------------------------------------------------------------
void Vec3::SetLength(float maxLength)
{
	*this = GetNormalized() * maxLength;
}

//-----------------------------------------------------------------------------------------------
bool Vec3::operator==(Vec3 const& compare) const
{
	if (compare.x == x && compare.y == y && compare.z == z)
	{
		return true;
	}
	return false;
}

//-----------------------------------------------------------------------------------------------
bool Vec3::operator!=(Vec3 const& compare) const
{
	if (compare.x != x || compare.y != y || compare.z != z)
	{
		return true;
	}
	return false;
}

//-----------------------------------------------------------------------------------------------
Vec3 const Vec3::operator+(Vec3 const& vecToAdd) const
{
	return Vec3(vecToAdd.x + x, vecToAdd.y + y, vecToAdd.z + z);
}

//-----------------------------------------------------------------------------------------------
Vec3 const Vec3::operator-(Vec3 const& vecToSubtract) const
{
	return Vec3(x - vecToSubtract.x, y - vecToSubtract.y, z - vecToSubtract.z);
}

//-----------------------------------------------------------------------------------------------
Vec3 const Vec3::operator*(float uniformScale) const
{
	return Vec3(x * uniformScale, y * uniformScale, z * uniformScale);
}

//-----------------------------------------------------------------------------------------------
Vec3 const Vec3::operator/(float inverseScale) const
{
	float scale = (1.0f / inverseScale);
	return Vec3(x * scale, y * scale, z * scale);
}

//-----------------------------------------------------------------------------------------------
void Vec3::operator+=(Vec3 const& vecToAdd)
{
	x += vecToAdd.x;
	y += vecToAdd.y;
	z += vecToAdd.z;
}

//-----------------------------------------------------------------------------------------------
void Vec3::operator-=(Vec3 const& vecToSubtract)
{
	x -= vecToSubtract.x;
	y -= vecToSubtract.y;
	z -= vecToSubtract.z;
}

//-----------------------------------------------------------------------------------------------
void Vec3::operator*=(float uniformScale)
{
	x *= uniformScale;
	y *= uniformScale;
	z *= uniformScale;
}

//-----------------------------------------------------------------------------------------------
void Vec3::operator/=(float uniformDivisor)
{
	float scale = (1.0f / uniformDivisor);
	x *= scale;
	y *= scale;
	z *= scale;
}

//-----------------------------------------------------------------------------------------------
void Vec3::operator=(Vec3 const& copyFrom)
{
	x = copyFrom.x;
	y = copyFrom.y;
	z = copyFrom.z;
}

//-----------------------------------------------------------------------------------------------
Vec3 Vec3::operator*(const Vec3& vecToMultiply) 
{
	return Vec3(x * vecToMultiply.x, y * vecToMultiply.y, z * vecToMultiply.z);
}

//-----------------------------------------------------------------------------------------------
void Vec3::Normalize()
{
	float scale = 1.0f / GetLength();
	x *= scale;
	y *= scale;
	z *= scale;
}

//-----------------------------------------------------------------------------------------------
Vec3 const Vec3::MakeFromPolarRadians(float longitudeRadians, float latitudeRadians, float length)
{
	//long = yaw
	//lat = pitch
	return Vec3(length * cosf(longitudeRadians) * cosf(latitudeRadians), length * sinf(longitudeRadians) * cosf(latitudeRadians), length * -sinf(latitudeRadians));
}

//-----------------------------------------------------------------------------------------------
Vec3 const Vec3::MakeFromPolarDegrees(float longitudeDegrees, float latitudeDegrees, float length)
{
	//long = yaw
	//lat = pitch
	return Vec3(length * CosDegrees(longitudeDegrees) * CosDegrees(latitudeDegrees), length * SinDegrees(longitudeDegrees) * CosDegrees(latitudeDegrees), length * -SinDegrees(latitudeDegrees));
}

//-----------------------------------------------------------------------------------------------
Vec3 const operator*(float uniformScale, Vec3 const& vecToScale)
{
	return Vec3(vecToScale.x * uniformScale, vecToScale.y * uniformScale, vecToScale.z * uniformScale);
}

//-----------------------------------------------------------------------------------------------
void Vec3::ClampLength(float maxLength)
{
	float currentLength = GetLength();
	if (currentLength > maxLength)
	{
		float scale = 1 / (currentLength / maxLength);
		x *= scale;
		y *= scale;
		z *= scale;
	}
}

//-----------------------------------------------------------------------------------------------
void Vec3::Reflect(Vec3 const& impactSurfaceNormal)
{
	Vec3 translationDown = DotProduct3D(Vec3(x, y, z), impactSurfaceNormal) * impactSurfaceNormal;
	Vec3 tanslationOver = Vec3(x, y, z) - translationDown;
	Vec3 reflected = tanslationOver + translationDown * -1.0f;
	x = reflected.x;
	y = reflected.y;
	z = reflected.z;
}
