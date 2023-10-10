#pragma once
#include "Vec3.hpp"

struct Sphere3
{
public:
	Vec3 m_center;
	float m_radius = 0.0f;

public:
	//construction/destruction
	~Sphere3() {}
	Sphere3() {}
	Sphere3(Sphere3 const& copyFrom);
	explicit Sphere3(float centerX, float centerY, float radius);
	explicit Sphere3(Vec3 const& center, float radius);
};