#pragma once
#include "Vec3.hpp"

//-----------------------------------------------------------------------------------------------
struct ConvexPoly3D;

//-----------------------------------------------------------------------------------------------
struct Plane3D
{
public:
	Plane3D() {}
	Plane3D(Vec3 const& normal, float const& distanceFromOrigin);
	Plane3D(ConvexPoly3D const& poly);
	~Plane3D() {}
	
	//Operators
	bool	operator==(Plane3D& comparePlane);
	bool	operator!=(Plane3D& comparePlane);

	float GetAltitude(Vec3 const& point);

public:
	Vec3	m_normal;
	float	m_distanceFromOrigin = 0.0f;
};