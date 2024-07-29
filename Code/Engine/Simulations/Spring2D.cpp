#include "Spring2D.hpp"

//-----------------------------------------------------------------------------------------------
Spring2D::Spring2D()
{
}

//-----------------------------------------------------------------------------------------------
Spring2D::~Spring2D()
{
}

//-----------------------------------------------------------------------------------------------
Spring2D::Spring2D(const Spring2D& copyFrom)
{
	m_particleA = copyFrom.m_particleA;
	m_particleB = copyFrom.m_particleB;
	m_stiffness = copyFrom.m_stiffness;
	m_initialLength = copyFrom.m_initialLength;
}

//-----------------------------------------------------------------------------------------------
Spring2D::Spring2D(Particle2D* particleA, Particle2D* particleB, float stiffness, float initialLength)
	:m_particleA(particleA)
	,m_particleB(particleB)
	,m_stiffness(stiffness)
	,m_initialLength(initialLength)
{
}
