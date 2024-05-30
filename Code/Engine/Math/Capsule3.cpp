#include "Capsule3.hpp"
#include "Cylinder3.hpp"

//-----------------------------------------------------------------------------------------------
Capsule3::Capsule3(Capsule3 const& copyFrom)
	:m_bone(copyFrom.m_bone)
	,m_radius(copyFrom.m_radius)
{
}

//-----------------------------------------------------------------------------------------------
Capsule3::Capsule3(Cylinder3 bone, float radius)
	: m_bone(bone)
	, m_radius(radius)
{
}

//-----------------------------------------------------------------------------------------------
Capsule3::Capsule3(Vec3 boneStart, Vec3 boneEnd, float radius)
{
	Cylinder3 bone(boneStart, boneEnd, radius);
	m_bone = bone;
	m_radius = radius;
}
