#pragma once
#include "Vec2.hpp"

//-----------------------------------------------------------------------------------------------
struct OBB2
{
public:
	Vec2 m_center;
	Vec2 m_iBasisNormal;
	Vec2 m_halfDimensions;

public:
	//construction/destruction
	~OBB2() {}
	OBB2() {}
	OBB2(OBB2 const& copyFrom);
	explicit OBB2(Vec2& center, Vec2& iBasisNormal, Vec2 halfDimensions);
};