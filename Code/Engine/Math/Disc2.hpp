#pragma once
#include "Vec2.hpp"

//-----------------------------------------------------------------------------------------------
struct Disc2
{
public:
	Vec2	m_center;
	float	m_radius = 0.0f;
	float	m_inverseMass = 0.0f;

public:
	//construction/destruction
	~Disc2() {}
	Disc2() {}
	Disc2(Disc2 const& copyFrom);
	explicit Disc2(float centerX, float centerY, float radius, float inverseMass = 1.0f);
	explicit Disc2(Vec2 const& center, float radius, float inverseMass = 1.0f);
};