#include "Capsule3.hpp"

//-----------------------------------------------------------------------------------------------
Capsule3::Capsule3(Capsule3 const& copyFrom)
	:m_bone(copyFrom.m_bone)
	,m_radius(copyFrom.m_radius)
{
}

//-----------------------------------------------------------------------------------------------
Capsule3::Capsule3(LineSegment3 bone, float radius)
	: m_bone(bone)
	, m_radius(radius)
{
}
