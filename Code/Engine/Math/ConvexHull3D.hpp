#pragma once
#include "Plane3D.hpp"
#include "ConvexPoly3D.hpp"
#include "Vec2.hpp"
#include "LineSegment3.hpp"
#include <vector>

//-----------------------------------------------------------------------------------------------
struct ConvexHull3D
{
public:
	ConvexHull3D() {}
	~ConvexHull3D() {}
	ConvexHull3D(std::vector<Vec3>& points, bool isDebug = false);

	bool IsPointInside(Vec3 const& point);
	void GenerateQuickhullInitialTetrahedron();
	void IterativeQuickullGeneration(bool isCoplanarAllowed = false, bool isDebug = false);
	void SetBoundingPointsAndEdges();
	void DebugDrawQuickull(bool drawWireOnly = false);
	void CalculateNewEpsilon(std::vector<Vec3> const& points);

public:
	std::vector<Plane3D>			m_boundingPlanes;
	std::vector<Vec3>				m_boundingPoints;
	std::vector<ConvexPoly3D>		m_boundingPolys;
	std::vector<LineSegment3>		m_boundingEdges;
	std::vector<Vec3>				m_pointsToPartition;
	std::vector<std::vector<Vec2>>	m_conflictLists;
	float							m_epsilon = 0.0f;
	int								m_debugCounter = 0;
};