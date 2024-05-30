#pragma once
#include "Vec3.hpp"

//-----------------------------------------------------------------------------------------------
struct Plane3D
{
public:
	Plane3D() {}
	Plane3D(Vec3 const& normal, float const& distanceFromOrigin);
	~Plane3D() {}

public:
	Vec3	m_normal;
	float	m_distanceFromOrigin;
};