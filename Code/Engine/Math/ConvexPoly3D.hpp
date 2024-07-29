#pragma once 
#include <vector>

//-----------------------------------------------------------------------------------------------
struct Vec3;

//-----------------------------------------------------------------------------------------------
struct ConvexPoly3D
{
public:
	ConvexPoly3D() {}
	ConvexPoly3D(std::vector<Vec3> const& ccwOrderedPoints);
	~ConvexPoly3D() {}

	void				SetAllPoints(std::vector<Vec3> const& ccwOrderedPoints);
	bool				IsPointInside(Vec3 const& point);
	void				AddPoint(Vec3 const& sentPoint, float epsilon);

public:
	std::vector<Vec3>	m_ccwOrderedPoints;
};