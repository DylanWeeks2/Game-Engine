#include "Sphere3.hpp"
#include "MathUtils.hpp"

//-----------------------------------------------------------------------------------------------
Sphere3::Sphere3(Sphere3 const& copyFrom)
{
	m_center = copyFrom.m_center;
	m_radius = copyFrom.m_radius;
}

//-----------------------------------------------------------------------------------------------
Sphere3::Sphere3(float centerX, float centerY, float radius)
{
	m_center.x = centerX;
	m_center.y = centerY;
	m_radius = radius;
}

//-----------------------------------------------------------------------------------------------
Sphere3::Sphere3(Vec3 const& center, float radius)
{
	m_center = center;
	m_radius = radius;
}
