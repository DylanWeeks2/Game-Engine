#include "Particles3D.hpp"

//-----------------------------------------------------------------------------------------------
Particles3D::Particles3D()
{
}

//-----------------------------------------------------------------------------------------------
Particles3D::~Particles3D()
{
}

//-----------------------------------------------------------------------------------------------
Particles3D::Particles3D(const Particles3D& copyFrom)
	:m_masses(copyFrom.m_masses)
	,m_inverseMasses(copyFrom.m_inverseMasses)
	,m_positions(copyFrom.m_positions)
	,m_velocities(copyFrom.m_velocities)
	,m_isAttached(copyFrom.m_isAttached)
	,m_proposedPositions(copyFrom.m_proposedPositions)
	,m_macroBitRegions(copyFrom.m_macroBitRegions)
	,m_microBitRegions(copyFrom.m_microBitRegions)
	,m_collisionNormals(copyFrom.m_collisionNormals)
	,m_jacobiCorrections(copyFrom.m_jacobiCorrections)
	,m_isSelfCollision(copyFrom.m_isSelfCollision)
{
}
