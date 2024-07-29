#pragma once
#include "Plane2D.hpp"
#include <vector>

//-----------------------------------------------------------------------------------------------
struct ConvexPoly2D;

//-----------------------------------------------------------------------------------------------
struct ConvexHull2D
{
public:
	ConvexHull2D() {}
	ConvexHull2D(std::vector<Plane2D> const& boundingPlanes);
	ConvexHull2D(ConvexPoly2D convexPoly);
	~ConvexHull2D() {}

	bool IsPointInside(Vec2 const& point);

public:
	std::vector<Plane2D>	m_boundingPlanes;
};