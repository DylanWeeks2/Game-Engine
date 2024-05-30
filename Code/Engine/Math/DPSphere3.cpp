#include "DPSphere3.hpp"
#include "MathUtils.hpp"

//-----------------------------------------------------------------------------------------------
DPSphere3::DPSphere3(DPSphere3 const& copyFrom)
{
	m_center = copyFrom.m_center;
	m_radius = copyFrom.m_radius;
}

//-----------------------------------------------------------------------------------------------
DPSphere3::DPSphere3(double centerX, double centerY, double radius)
{
	m_center.x = centerX;
	m_center.y = centerY;
	m_radius = radius;
}

//-----------------------------------------------------------------------------------------------
DPSphere3::DPSphere3(DPVec3 const& center, double radius)
{
	m_center = center;
	m_radius = radius;
}
