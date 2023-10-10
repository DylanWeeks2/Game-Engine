#pragma once
#include "Vec3.hpp"

struct LineSegment3
{
public:
	Vec3 m_start;
	Vec3 m_end;

public:
	//construction/destruction
	~LineSegment3() {}
	LineSegment3() {}
	LineSegment3(LineSegment3 const& copyFrom);
	explicit LineSegment3(float startX, float startY, float startZ, float endX, float endY, float endZ);
	explicit LineSegment3(Vec3 const& start, Vec3 const& end);
};
