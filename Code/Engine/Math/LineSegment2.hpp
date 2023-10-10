#pragma once
#include "Vec2.hpp"

struct LineSegment2
{
public:
	Vec2 m_start;
	Vec2 m_end;

public:
	//construction/destruction
	~LineSegment2() {}
	LineSegment2() {}
	LineSegment2(LineSegment2 const& copyFrom);
	explicit LineSegment2(float startX, float startY, float endX, float endY);
	explicit LineSegment2(Vec2 const& start, Vec2 const& end);
};
