#pragma once
#include "DPCylinder3.hpp"

//-----------------------------------------------------------------------------------------------
struct DPCapsule3
{
public:
	DPCylinder3 m_bone;
	double		m_radius = 0.0f;

public:
	//construction/destruction
	~DPCapsule3() {}
	DPCapsule3() {}
	DPCapsule3(DPCapsule3 const& copyFrom);
	explicit DPCapsule3(DPCylinder3 bone, double radius);
	explicit DPCapsule3(DPVec3 boneStart, DPVec3 boneEnd, double radius);
};