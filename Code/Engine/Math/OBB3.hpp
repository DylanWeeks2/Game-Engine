#pragma once
#include "Vec3.hpp"

struct OBB3
{
public:
	Vec3 m_center;
	Vec3 m_iBasisNormal;
	Vec3 m_halfDimensions;

public:
	//construction/destruction
	~OBB3() {}
	OBB3() {}
	OBB3(OBB3 const& copyFrom);
	explicit OBB3(Vec3& center, Vec3& iBasisNormal, Vec3& halfDimensions);
};