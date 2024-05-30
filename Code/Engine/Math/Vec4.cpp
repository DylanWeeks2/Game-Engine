#include "Vec4.hpp"
#include <math.h>
#include "MathUtils.hpp"

//-----------------------------------------------------------------------------------------------
Vec4::Vec4(float initialX, float initialY, float initialZ, float initialW)
	: x(initialX)
	, y(initialY)
	, z(initialZ)
	, w(initialW)
{
}

//-----------------------------------------------------------------------------------------------
Vec4 const Vec4::operator-(Vec4 const& vecToSubtract) const
{
	return Vec4(x - vecToSubtract.x, y - vecToSubtract.y, z - vecToSubtract.z, w - vecToSubtract.w);
}

//-----------------------------------------------------------------------------------------------
void Vec4::operator*=(float uniformScale)
{
	x *= uniformScale;
	y *= uniformScale;
	z *= uniformScale;
	w *= uniformScale;
}
