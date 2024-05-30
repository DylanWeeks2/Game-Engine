#include "Capsule2.hpp"

//-----------------------------------------------------------------------------------------------
Capsule2::Capsule2(Capsule2 const& copyFrom)
	:m_bone(copyFrom.m_bone)
	,m_radius(copyFrom.m_radius)
	,m_inverseMass(copyFrom.m_inverseMass)
{
}

//-----------------------------------------------------------------------------------------------
Capsule2::Capsule2(LineSegment2 bone, float radius, float inverseMass)
	:m_bone(bone)
	,m_radius(radius)
	,m_inverseMass(inverseMass)
{
}
