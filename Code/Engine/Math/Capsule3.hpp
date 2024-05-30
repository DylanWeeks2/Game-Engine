#pragma once
#include "Cylinder3.hpp"

//-----------------------------------------------------------------------------------------------
struct Capsule3
{
public:
	Cylinder3	m_bone;
	float		m_radius = 0.0f;

public:
	//construction/destruction
	~Capsule3() {}
	Capsule3() {}
	Capsule3(Capsule3 const& copyFrom);
	explicit Capsule3(Cylinder3 bone, float radius);
	explicit Capsule3(Vec3 boneStart, Vec3 boneEnd, float radius);
};