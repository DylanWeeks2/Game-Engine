#include "Particle2D.hpp"

//-----------------------------------------------------------------------------------------------
Particle2D::Particle2D()
{
}

//-----------------------------------------------------------------------------------------------
Particle2D::~Particle2D()
{
}

//-----------------------------------------------------------------------------------------------
Particle2D::Particle2D(const Particle2D& copyFrom)
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
Particle2D::Particle2D(float mass, float inverseMass, Vec2 position, Vec2 velocity, bool isLocked)
	:m_mass(mass)
	,m_inverseMass(inverseMass)
	,m_position(position)
	,m_velocity(velocity)
	,m_isLocked(isLocked)
	,m_previousPosition(m_position)
	,m_proposedPosition(m_position)
{
}
