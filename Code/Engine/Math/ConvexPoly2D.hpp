#pragma once 
#include <vector>

//-----------------------------------------------------------------------------------------------
struct Vec2;

//-----------------------------------------------------------------------------------------------
struct ConvexPoly2D
{
public:
	ConvexPoly2D() {}
	ConvexPoly2D(std::vector<Vec2> const& ccwOrderedPoints);
	~ConvexPoly2D() {}

	void				SetAllPoints(std::vector<Vec2> const& ccwOrderedPoints);
	bool				IsPointInside(Vec2 const& point);

public:
	std::vector<Vec2>	m_ccwOrderedPoints;
};