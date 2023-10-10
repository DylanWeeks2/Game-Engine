#pragma once
#include "LineSegment2.hpp"

struct Capsule2
{
public:
	LineSegment2 m_bone;
	float m_radius = 0.0f;
	float m_inverseMass = 0.0f;

public:
	//construction/destruction
	~Capsule2() {}
	Capsule2() {}
	Capsule2(Capsule2 const& copyFrom);
	explicit Capsule2(LineSegment2 bone, float radius, float inverseMass = 1.0f);
};