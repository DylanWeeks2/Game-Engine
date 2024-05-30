#pragma once
#include "Vec2.hpp"

//-----------------------------------------------------------------------------------------------
struct Plane2D
{
public:
	Plane2D() {}
	Plane2D(Vec2 const& normal, float const& distanceFromOrigin);
	~Plane2D() {}

public:
	Vec2	m_normal;
	float	m_distanceFromOrigin;
};