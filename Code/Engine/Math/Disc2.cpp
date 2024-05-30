#include "Disc2.hpp"
#include "Vec2.hpp"
#include "MathUtils.hpp"

//-----------------------------------------------------------------------------------------------
Disc2::Disc2(Disc2 const& copyFrom)
{
	m_center = copyFrom.m_center;
	m_radius = copyFrom.m_radius;
	m_inverseMass = copyFrom.m_inverseMass;
}

//-----------------------------------------------------------------------------------------------
Disc2::Disc2(float centerX, float centerY, float radius, float inverseMass)
{
	m_center.x = centerX;
	m_center.y = centerY;
	m_radius = radius;
	m_inverseMass = inverseMass;
}

//-----------------------------------------------------------------------------------------------
Disc2::Disc2(Vec2 const& center, float radius, float inverseMass)
{
	m_center = center;
	m_radius = radius;
	m_inverseMass = inverseMass;
}
