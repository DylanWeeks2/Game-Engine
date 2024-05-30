#include "DPVec4.hpp"
#include <math.h>
#include "MathUtils.hpp"

//-----------------------------------------------------------------------------------------------
DPVec4::DPVec4(double initialX, double initialY, double initialZ, double initialW)
	: x(initialX)
	, y(initialY)
	, z(initialZ)
	, w(initialW)
{
}

//-----------------------------------------------------------------------------------------------
DPVec4 const DPVec4::operator-(DPVec4 const& vecToSubtract) const
{
	return DPVec4(x - vecToSubtract.x, y - vecToSubtract.y, z - vecToSubtract.z, w - vecToSubtract.w);
}

//-----------------------------------------------------------------------------------------------
void DPVec4::operator*=(double uniformScale)
{
	x *= uniformScale;
	y *= uniformScale;
	z *= uniformScale;
	w *= uniformScale;
}
