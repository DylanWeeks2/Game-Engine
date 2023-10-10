#pragma once
#include "LineSegment3.hpp"

struct Capsule3
{
public:
	LineSegment3 m_bone;
	float m_radius = 0.0f;

public:
	//construction/destruction
	~Capsule3() {}
	Capsule3() {}
	Capsule3(Capsule3 const& copyFrom);
	explicit Capsule3(LineSegment3 bone, float radius);
};