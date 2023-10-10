#include "Point3D.hpp"
#include "Point3D.hpp"

//-----------------------------------------------------------------------------------------------
Point3D::Point3D()
{
}

//-----------------------------------------------------------------------------------------------
Point3D::~Point3D()
{
}

//-----------------------------------------------------------------------------------------------
Point3D::Point3D(const Point3D& copyFrom)
	:m_mass(copyFrom.m_mass)
	,m_inverseMass(copyFrom.m_inverseMass)
	,m_position(copyFrom.m_position)
	,m_velocity(copyFrom.m_velocity)
	,m_isLocked(copyFrom.m_isLocked)
	,m_previousPosition(copyFrom.m_position)
	,m_proposedPosition(copyFrom.m_position)
{
}

//-----------------------------------------------------------------------------------------------
Point3D::Point3D(float mass, float inverseMass, Vec3 position, Vec3 velocity, bool isLocked)
	:m_mass(mass)
	,m_inverseMass(inverseMass)
	,m_position(position)
	,m_velocity(velocity)
	,m_isLocked(isLocked)
	, m_previousPosition(m_position)
	, m_proposedPosition(m_position)
{
}
