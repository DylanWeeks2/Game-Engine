#pragma once
#include "Vec3.hpp"

//-----------------------------------------------------------------------------------------------
struct Cylinder3
{
public:
	Vec3	m_start;
	Vec3	m_end;
	Vec3	m_iBasis;
	Vec3	m_jBasis;
	Vec3	m_kBasis;
	float	m_radius = 0.0f;

public:
	//construction/destruction
	~Cylinder3() {}
	Cylinder3() {}
	Cylinder3(Cylinder3 const& copyFrom);
	explicit Cylinder3(Vec3 start, Vec3 end, float radius);
};