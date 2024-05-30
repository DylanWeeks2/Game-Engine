#include "DPCapsule3.hpp"
#include "DPCylinder3.hpp"

//-----------------------------------------------------------------------------------------------
DPCapsule3::DPCapsule3(DPCapsule3 const& copyFrom)
	:m_bone(copyFrom.m_bone)
	,m_radius(copyFrom.m_radius)
{
}

//-----------------------------------------------------------------------------------------------
DPCapsule3::DPCapsule3(DPCylinder3 bone, double radius)
	: m_bone(bone)
	, m_radius(radius)
{
}

//-----------------------------------------------------------------------------------------------
DPCapsule3::DPCapsule3(DPVec3 boneStart, DPVec3 boneEnd, double radius)
{
	DPCylinder3 bone(boneStart, boneEnd, radius);
	m_bone = bone;
	m_radius = radius;
}
