#include "OBB3.hpp"
#include "MathUtils.hpp"

//-----------------------------------------------------------------------------------------------
OBB3::OBB3(OBB3 const& copyFrom)
	:m_center(copyFrom.m_center)
	,m_iBasisNormal(copyFrom.m_iBasisNormal)
	,m_halfDimensions(copyFrom.m_halfDimensions)
{
}

//-----------------------------------------------------------------------------------------------
OBB3::OBB3(Vec3& center, Vec3& iBasisNormal, Vec3& halfDimensions)
	:m_center(center)
	,m_iBasisNormal(iBasisNormal)
	,m_halfDimensions(halfDimensions)
{
}
