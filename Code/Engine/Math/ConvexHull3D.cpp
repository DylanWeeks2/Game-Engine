#include "ConvexHull3D.hpp"
#include "Plane3D.hpp"
#include "ConvexPoly3D.hpp"
#include "MathUtils.hpp"
#include "AABB3.hpp"
#include "LineSegment3.hpp"
#include "Engine/Core/DebugRender.hpp"
#include "Engine/Core/ErrorWarningAssert.hpp"
#include "Engine/Core/Time.hpp"
#include <algorithm>


//-----------------------------------------------------------------------------------------------
int	g_maxDebugAmount = INT_MAX;

//-----------------------------------------------------------------------------------------------
ConvexHull3D::ConvexHull3D(std::vector<Vec3>& points, bool isDebug)
	:m_pointsToPartition(points)
{
	//float startTime = float(GetCurrentTimeSeconds());

	GenerateQuickhullInitialTetrahedron();
	if (isDebug)
	{
		return;
	}
	IterativeQuickullGeneration();
	SetBoundingPointsAndEdges();

	//float endTime = float(GetCurrentTimeSeconds());
	//DebuggerPrintf("Time = %f\n", endTime - startTime);
}

//-----------------------------------------------------------------------------------------------
bool ConvexHull3D::IsPointInside(Vec3 const& point)
{
	for (int planeIndex = 0; planeIndex < m_boundingPlanes.size(); planeIndex++)
	{
		Plane3D& plane = m_boundingPlanes[planeIndex];
		if (DotProduct3D(point, plane.m_normal) >= plane.m_distanceFromOrigin + m_epsilon)
		{
			return false;
		}
	}

	return true;
}

//-----------------------------------------------------------------------------------------------
void ConvexHull3D::GenerateQuickhullInitialTetrahedron()
{
	//QUICKHULL 3D ALGORITHM
	//Find Extreme Points
	float positiveZValue = -100000000000.0f;
	float positiveXValue = -100000000000.0f;
	float positiveYValue = -100000000000.0f;
	float negativeZValue = FLT_MAX;
	float negativeXValue = FLT_MAX;
	float negativeYValue = FLT_MAX;
	Vec3 positiveZVert;
	Vec3 positiveXVert;
	Vec3 positiveYVert;
	Vec3 negativeZVert;
	Vec3 negativeXVert;
	Vec3 negativeYVert;
	for (int pointIndex = 0; pointIndex < m_pointsToPartition.size(); pointIndex++)
	{
		if (m_pointsToPartition[pointIndex].z > positiveZValue)
		{
			positiveZValue = m_pointsToPartition[pointIndex].z;
			positiveZVert = m_pointsToPartition[pointIndex];
		}
		if (m_pointsToPartition[pointIndex].y > positiveYValue)
		{
			positiveYValue = m_pointsToPartition[pointIndex].y;
			positiveYVert = m_pointsToPartition[pointIndex];
		}
		if (m_pointsToPartition[pointIndex].x > positiveXValue)
		{
			positiveXValue = m_pointsToPartition[pointIndex].x;
			positiveXVert = m_pointsToPartition[pointIndex];
		}
		if (m_pointsToPartition[pointIndex].z < negativeZValue)
		{
			negativeZValue = m_pointsToPartition[pointIndex].z;
			negativeZVert = m_pointsToPartition[pointIndex];
		}
		if (m_pointsToPartition[pointIndex].y < negativeYValue)
		{
			negativeYValue = m_pointsToPartition[pointIndex].y;
			negativeYVert = m_pointsToPartition[pointIndex];
		}
		if (m_pointsToPartition[pointIndex].x < negativeXValue)
		{
			negativeXValue = m_pointsToPartition[pointIndex].x;
			negativeXVert = m_pointsToPartition[pointIndex];
		}
	}
	std::vector<Vec3> maxPoints;
	maxPoints.push_back(positiveZVert);
	maxPoints.push_back(positiveYVert);
	maxPoints.push_back(positiveXVert);
	maxPoints.push_back(negativeZVert);
	maxPoints.push_back(negativeYVert);
	maxPoints.push_back(negativeXVert);

	//Set epsilon
	float maxX = 0.0f;
	float maxY = 0.0f;
	float maxZ = 0.0f;
	if (fabsf(positiveXValue) > fabsf(negativeXValue))
		maxX = positiveXValue;
	else
		maxX = fabsf(negativeXValue);
	if (fabsf(positiveYValue) > fabsf(negativeYValue))
		maxY = positiveYValue;
	else
		maxY = fabsf(negativeYValue);
	if (fabsf(positiveZValue) > fabsf(negativeZValue))
		maxZ = positiveZValue;
	else
		maxZ = fabsf(negativeZValue);
	m_epsilon = 3.0f * (maxX + maxY + maxZ) * FLT_EPSILON;

	//Generate Initial Line Segment (From Furthest Points)
	float maxDist = -100000000000.0f;
	LineSegment3 maxDistLine;
	Vec3 p1;
	Vec3 p2;
	for (int pointIndexA = 0; pointIndexA < maxPoints.size(); pointIndexA++)
	{
		Vec3& pointA = maxPoints[pointIndexA];
		for (int pointIndexB = 0; pointIndexB < maxPoints.size(); pointIndexB++)
		{
			Vec3& pointB = maxPoints[pointIndexB];
			if (pointB != pointA)
			{
				float currentDist = GetDistanceSquared3D(pointA, pointB);
				if (currentDist > maxDist)
				{
					maxDist = currentDist;
					maxDistLine.m_start = pointA;
					maxDistLine.m_end = pointB;
					p1 = maxPoints[pointIndexA];
					p2 = maxPoints[pointIndexB];
				}
			}
		}
	}

	//Generate Initial Triangle
	maxDist = -100000000000.0f;
	Vec3 p3;
	for (int pointIndex = 0; pointIndex < m_pointsToPartition.size(); pointIndex++)
	{
		Vec3& point = m_pointsToPartition[pointIndex];
		float currentDist = GetDistanceToLineSegmentSquared3D(point, maxDistLine);
		if (currentDist > maxDist)
		{
			maxDist = currentDist;
			p3 = m_pointsToPartition[pointIndex];
		}
	}
	ConvexPoly3D initialPoly;
	initialPoly.m_ccwOrderedPoints.push_back(maxDistLine.m_start);
	initialPoly.m_ccwOrderedPoints.push_back(maxDistLine.m_end);
	initialPoly.m_ccwOrderedPoints.push_back(p3);
	m_boundingPolys.push_back(initialPoly);

	//Generate Initial Tetrahedron
	maxDist = -100000000000.0f;
	Vec3 p4;
	Vec3 normal = CrossProduct3D((p3 - maxDistLine.m_end), (maxDistLine.m_start - maxDistLine.m_end)).GetNormalized();
	Plane3D initialPlane(normal, DotProduct3D(maxDistLine.m_start, normal));
	for (int pointIndex = 0; pointIndex < m_pointsToPartition.size(); pointIndex++)
	{
		Vec3& point = m_pointsToPartition[pointIndex];
		float currentDist = fabsf(DotProduct3D(point, normal) - initialPlane.m_distanceFromOrigin);
		if (currentDist > maxDist)
		{
			maxDist = currentDist;
			p4 = m_pointsToPartition[pointIndex];
		}
	}

	//Reorder the ConvexPoly and Plane Based on Max Point
	if (DotProduct3D(p4, normal) - initialPlane.m_distanceFromOrigin > 0.0f)
	{
		Vec3 tempPoint = initialPoly.m_ccwOrderedPoints[1];
		initialPoly.m_ccwOrderedPoints[1] = initialPoly.m_ccwOrderedPoints[2];
		initialPoly.m_ccwOrderedPoints[2] = tempPoint;
		m_boundingPolys[0] = initialPoly;
		initialPlane.m_normal *= -1.0f;
		initialPlane.m_distanceFromOrigin *= -1.0f;
	}

	//Add Convex Polys and Planes
	ConvexPoly3D poly2;
	poly2.m_ccwOrderedPoints.push_back(initialPoly.m_ccwOrderedPoints[1]);
	poly2.m_ccwOrderedPoints.push_back(initialPoly.m_ccwOrderedPoints[0]);
	poly2.m_ccwOrderedPoints.push_back(p4);
	m_boundingPolys.push_back(poly2);
	ConvexPoly3D poly3;
	poly3.m_ccwOrderedPoints.push_back(p4);
	poly3.m_ccwOrderedPoints.push_back(initialPoly.m_ccwOrderedPoints[0]);
	poly3.m_ccwOrderedPoints.push_back(initialPoly.m_ccwOrderedPoints[2]);
	m_boundingPolys.push_back(poly3);
	ConvexPoly3D poly4;
	poly4.m_ccwOrderedPoints.push_back(initialPoly.m_ccwOrderedPoints[2]);
	poly4.m_ccwOrderedPoints.push_back(initialPoly.m_ccwOrderedPoints[1]);
	poly4.m_ccwOrderedPoints.push_back(p4);
	m_boundingPolys.push_back(poly4);
	m_boundingPlanes.push_back(initialPlane);
	normal = CrossProduct3D((poly2.m_ccwOrderedPoints[2] - poly2.m_ccwOrderedPoints[1]), (poly2.m_ccwOrderedPoints[0] - poly2.m_ccwOrderedPoints[1])).GetNormalized();
	m_boundingPlanes.push_back(Plane3D(normal, DotProduct3D(poly2.m_ccwOrderedPoints[2], normal)));
	normal = CrossProduct3D((poly3.m_ccwOrderedPoints[2] - poly3.m_ccwOrderedPoints[1]), (poly3.m_ccwOrderedPoints[0] - poly3.m_ccwOrderedPoints[1])).GetNormalized();
	m_boundingPlanes.push_back(Plane3D(normal, DotProduct3D(poly3.m_ccwOrderedPoints[2], normal)));
	normal = CrossProduct3D((poly4.m_ccwOrderedPoints[2] - poly4.m_ccwOrderedPoints[1]), (poly4.m_ccwOrderedPoints[0] - poly4.m_ccwOrderedPoints[1])).GetNormalized();
	m_boundingPlanes.push_back(Plane3D(normal, DotProduct3D(poly4.m_ccwOrderedPoints[2], normal)));
	m_conflictLists.resize(4);

	//Remove all inside m_pointsToPartition and maxPoints
	for (int pointIndex = 0; pointIndex < m_pointsToPartition.size(); pointIndex++)
	{
		Vec3& currentPoint = m_pointsToPartition[pointIndex];
		if (currentPoint == p1 || currentPoint == p2 || currentPoint == p3 || currentPoint == p4)
		{
			m_pointsToPartition.erase(m_pointsToPartition.begin() + pointIndex);
			pointIndex--;
			continue;
		}

		if (IsPointInside(currentPoint))
		{
			m_pointsToPartition.erase(m_pointsToPartition.begin() + pointIndex);
			pointIndex--;
			continue;
		}
	}

	//Initial Partitioning of Points and Faces
	for (int pointIndex = 0; pointIndex < m_pointsToPartition.size(); pointIndex++)
	{
		Vec3& point = m_pointsToPartition[pointIndex];
		float minDist = FLT_MAX;
		int closestPlaneIndex = -1;
		for (int planeIndex = 0; planeIndex < m_boundingPlanes.size(); planeIndex++)
		{
			Plane3D& plane = m_boundingPlanes[planeIndex];
			float altitude = DotProduct3D(point, plane.m_normal) - plane.m_distanceFromOrigin;
			if (altitude < minDist && altitude > 0.0f)
			{
				minDist = altitude;
				closestPlaneIndex = planeIndex;
			}
		}

		m_conflictLists[closestPlaneIndex].push_back(Vec2(float(pointIndex), minDist));
	}
}


//-----------------------------------------------------------------------------------------------
void ConvexHull3D::IterativeQuickullGeneration(bool isCoplanarAllowed, bool isDebug)
{
	//Iterative Loop
	Vec3 normal;
	while (true)
	{
		float maxDist = -100000000000.0f;
		int maxPointIndex = -1;
		int maxPlaneIndex = -1;
		for (int planeIndex = 0; planeIndex < m_conflictLists.size(); planeIndex++)
		{
			for (int pointIndex = 0; pointIndex < m_conflictLists[planeIndex].size(); pointIndex++)
			{
				float currentDist = m_conflictLists[planeIndex][pointIndex].y;
				if (currentDist > maxDist)
				{
					maxDist = currentDist;
					maxPointIndex = int(m_conflictLists[planeIndex][pointIndex].x);
					maxPlaneIndex = planeIndex;
				}
			}
		}

		//Early out check (tetrahedron case)
		if (maxPlaneIndex == -1)
		{
			break;
		}

		//Find all visible planes
		std::vector<int> visiblePlaneIndicies;
		std::vector<LineSegment3> horizonEdges;
		visiblePlaneIndicies.push_back(maxPlaneIndex);
		for (int pointIndex = 0; pointIndex < m_boundingPolys[maxPlaneIndex].m_ccwOrderedPoints.size(); pointIndex++)
		{
			if (pointIndex != m_boundingPolys[maxPlaneIndex].m_ccwOrderedPoints.size() - 1)
				horizonEdges.push_back(LineSegment3(m_boundingPolys[maxPlaneIndex].m_ccwOrderedPoints[pointIndex], m_boundingPolys[maxPlaneIndex].m_ccwOrderedPoints[pointIndex + 1]));
			else
				horizonEdges.push_back(LineSegment3(m_boundingPolys[maxPlaneIndex].m_ccwOrderedPoints[pointIndex], m_boundingPolys[maxPlaneIndex].m_ccwOrderedPoints[0]));
		}

		for (int planeIndex = 0; planeIndex < m_boundingPlanes.size(); planeIndex++)
		{
			if (planeIndex == maxPlaneIndex)
			{
				continue;
			}

			Plane3D& plane = m_boundingPlanes[planeIndex];
			float altitude = DotProduct3D(m_pointsToPartition[maxPointIndex], plane.m_normal) - plane.m_distanceFromOrigin;
			if (altitude > 0.0f)
			{
				visiblePlaneIndicies.push_back(planeIndex);

				std::vector<LineSegment3> lines;
				std::vector<bool> shouldAddLines;
				for (int pointIndex = 0; pointIndex < m_boundingPolys[planeIndex].m_ccwOrderedPoints.size(); pointIndex++)
				{
					if (pointIndex != m_boundingPolys[planeIndex].m_ccwOrderedPoints.size() - 1)
					{
						lines.push_back(LineSegment3(m_boundingPolys[planeIndex].m_ccwOrderedPoints[pointIndex], m_boundingPolys[planeIndex].m_ccwOrderedPoints[pointIndex + 1]));
					}
					else
					{
						lines.push_back(LineSegment3(m_boundingPolys[planeIndex].m_ccwOrderedPoints[pointIndex], m_boundingPolys[planeIndex].m_ccwOrderedPoints[0]));
					}
					
					shouldAddLines.push_back(true);
				}

				//Manage all horizon edges
				for (int edgeIndex = 0; edgeIndex < horizonEdges.size(); edgeIndex++)
				{
					for (int lineIndex = 0; lineIndex < lines.size(); lineIndex++)
					{
						if (lines[lineIndex] == horizonEdges[edgeIndex])
						{
							horizonEdges.erase(horizonEdges.begin() + edgeIndex);
							edgeIndex--;
							shouldAddLines[lineIndex] = false;
							break;
						}
					}
				}
				for (int lineIndex = 0; lineIndex < lines.size(); lineIndex++)
				{
					if (shouldAddLines[lineIndex])
					{
						horizonEdges.push_back(lines[lineIndex]);
					}
				}
			}
		}

		//Create New Faces From Horizon Edges
		for (int edgeIndex = 0; edgeIndex < horizonEdges.size(); edgeIndex++)
		{
			//Add Polygon
			ConvexPoly3D newPoly;
			newPoly.m_ccwOrderedPoints.push_back(horizonEdges[edgeIndex].m_start);
			newPoly.m_ccwOrderedPoints.push_back(horizonEdges[edgeIndex].m_end);
			newPoly.m_ccwOrderedPoints.push_back(m_pointsToPartition[maxPointIndex]);
			
			//Add Plane
			normal = CrossProduct3D((newPoly.m_ccwOrderedPoints[2] - newPoly.m_ccwOrderedPoints[1]), (newPoly.m_ccwOrderedPoints[0] - newPoly.m_ccwOrderedPoints[1]));
			if (normal == Vec3() || (fabsf(normal.x) < m_epsilon && fabsf(normal.y) < m_epsilon && fabsf(normal.z) < m_epsilon))
			{
				continue;
			}
			normal = normal.GetNormalized();
			float distanceToOrigin = DotProduct3D(newPoly.m_ccwOrderedPoints[2], normal);
			std::vector<Vec2> newList;

			//OLD CODE NO COPLANAR REMOVAL
			if (isCoplanarAllowed)
			{
				m_conflictLists.push_back(newList);
				m_boundingPlanes.push_back(Plane3D(normal, DotProduct3D(newPoly.m_ccwOrderedPoints[2], normal)));
				m_boundingPolys.push_back(newPoly);
				continue;
			}

			//Coplanar Removal
			//Check For Coplanar Planes
			bool addNewPlane = true;
			for (int planeIndex = 0; planeIndex < m_boundingPlanes.size(); planeIndex++)
			{
				bool isCoPlanar = true;
				for (int pointIndex = 0; pointIndex < m_boundingPolys[planeIndex].m_ccwOrderedPoints.size(); pointIndex++)
				{
					float altitude = DotProduct3D(m_boundingPolys[planeIndex].m_ccwOrderedPoints[pointIndex], normal) - distanceToOrigin;
					if (altitude > m_epsilon || altitude < -m_epsilon)
					{
						isCoPlanar = false;
						break;
					}
				}

				if (isCoPlanar)
				{
					if (m_debugCounter == g_maxDebugAmount)
					{
						DebugAddWorldConvexPoly3D(newPoly, 100.0f, Rgba8::MEDIUM_GREEN, Rgba8::MEDIUM_GREEN);
					}
					//Add Point to Existing Convex Polygon
					m_boundingPolys[planeIndex].AddPoint(m_pointsToPartition[maxPointIndex], m_epsilon);
					if (m_debugCounter == g_maxDebugAmount)
					{
						DebugAddWorldConvexPoly3D(m_boundingPolys[planeIndex], 100.0f, Rgba8::ORANGE, Rgba8::ORANGE);
					}
					//Get new normal
					normal = CrossProduct3D((m_boundingPolys[planeIndex].m_ccwOrderedPoints[2] - m_boundingPolys[planeIndex].m_ccwOrderedPoints[1]),
						(m_boundingPolys[planeIndex].m_ccwOrderedPoints[0] - m_boundingPolys[planeIndex].m_ccwOrderedPoints[1])).GetNormalized();
					if (normal == Vec3())
					{
						m_boundingPolys[planeIndex].m_ccwOrderedPoints.erase(m_boundingPolys[planeIndex].m_ccwOrderedPoints.begin() + 1);
						normal = CrossProduct3D((m_boundingPolys[planeIndex].m_ccwOrderedPoints[2] - m_boundingPolys[planeIndex].m_ccwOrderedPoints[1]),
							(m_boundingPolys[planeIndex].m_ccwOrderedPoints[0] - m_boundingPolys[planeIndex].m_ccwOrderedPoints[1])).GetNormalized();
					}
					m_boundingPlanes[planeIndex] = (Plane3D(normal, DotProduct3D(m_boundingPolys[planeIndex].m_ccwOrderedPoints[2], normal)));

					//Erase old visible plane from the list
					for (int visPlaneIndex = 0; visPlaneIndex < visiblePlaneIndicies.size(); visPlaneIndex++)
					{
						if (planeIndex == visiblePlaneIndicies[visPlaneIndex])
						{
							visiblePlaneIndicies.erase(visiblePlaneIndicies.begin() + visPlaneIndex);
						}
					}

					addNewPlane = false;
				}
			}

			if (addNewPlane)
			{
				m_conflictLists.push_back(newList);
				m_boundingPlanes.push_back(Plane3D(normal, DotProduct3D(newPoly.m_ccwOrderedPoints[2], normal)));
				m_boundingPolys.push_back(newPoly);
			}
		}
		
		//Remove old planes and polys
		std::sort(visiblePlaneIndicies.begin(), visiblePlaneIndicies.end(), std::greater<>());
		for (int planeIndex = 0; planeIndex < visiblePlaneIndicies.size(); planeIndex++)
		{
			m_boundingPlanes.erase(m_boundingPlanes.begin() + visiblePlaneIndicies[planeIndex]);
			m_boundingPolys.erase(m_boundingPolys.begin() + visiblePlaneIndicies[planeIndex]);
			m_conflictLists.erase(m_conflictLists.begin() + visiblePlaneIndicies[planeIndex]);
		}

		//Remove old Points
		m_pointsToPartition.erase(m_pointsToPartition.begin() + maxPointIndex);
		for (int pointIndex = 0; pointIndex < m_pointsToPartition.size(); pointIndex++)
		{
			Vec3& currentPoint = m_pointsToPartition[pointIndex];
			if (IsPointInside(currentPoint))
			{
				m_pointsToPartition.erase(m_pointsToPartition.begin() + pointIndex);
				pointIndex--;
				continue;
			}
		}

		//Repartition points to faces
		for (int planeIndex = 0; planeIndex < m_conflictLists.size(); planeIndex++)
		{
			m_conflictLists[planeIndex].clear();
		}
		for (int pointIndex = 0; pointIndex < m_pointsToPartition.size(); pointIndex++)
		{
			Vec3& point = m_pointsToPartition[pointIndex];
			float minDist = FLT_MAX;
			int closestPlaneIndex = -1;
			for (int planeIndex = 0; planeIndex < m_boundingPlanes.size(); planeIndex++)
			{
				Plane3D& plane = m_boundingPlanes[planeIndex];
				float altitude = DotProduct3D(point, plane.m_normal) - plane.m_distanceFromOrigin;
				if (altitude < minDist && altitude > 0.0f)
				{
					minDist = altitude;
					closestPlaneIndex = planeIndex;
				}
			}

			m_conflictLists[closestPlaneIndex].push_back(Vec2(float(pointIndex), minDist));
		}

		if (m_pointsToPartition.size() == 0)
		{
			break;
		}

		//NEED TO CLEAN THE DEBUG FACTOR UP TO BE MORE GAME SIDE INSTEAD OF ENGINE
		if (isDebug)
		{
			break;
		}
	}
}

//-----------------------------------------------------------------------------------------------
void ConvexHull3D::SetBoundingPointsAndEdges()
{
	//TODO: USE HALFEDGE STRUCTURE FOR OPTIMIZATION
	//Setting all polygon points
	for (int polyIndex = 0; polyIndex < m_boundingPolys.size(); polyIndex++)
	{
		for (int pointIndex = 0; pointIndex < m_boundingPolys[polyIndex].m_ccwOrderedPoints.size(); pointIndex++)
		{
			Vec3 p1 = m_boundingPolys[polyIndex].m_ccwOrderedPoints[pointIndex];
			Vec3 p2;
			if (pointIndex == m_boundingPolys[polyIndex].m_ccwOrderedPoints.size() - 1)
			{
				p2 = m_boundingPolys[polyIndex].m_ccwOrderedPoints[0];
			}
			else
			{
				p2 = m_boundingPolys[polyIndex].m_ccwOrderedPoints[pointIndex + 1];
			}
			
			m_boundingEdges.push_back(LineSegment3(p1, p2));
			m_boundingPoints.push_back(m_boundingPolys[polyIndex].m_ccwOrderedPoints[pointIndex]);
		}
	}

	//Erasing duplicates
	for (int i = 0; i < m_boundingPoints.size(); i++)
	{
		for (int j = i + 1; j < m_boundingPoints.size(); j++)
		{
			if (j == m_boundingPoints.size())
			{
				break;
			}

			if (m_boundingPoints[i] == m_boundingPoints[j])
			{
				m_boundingPoints.erase(m_boundingPoints.begin() + j);
				j--;
			}
		}
	}

	for (int i = 0; i < m_boundingEdges.size(); i++)
	{
		for (int j = i + 1; j < m_boundingEdges.size(); j++)
		{
			if (j == m_boundingEdges.size())
			{
				break;
			}

			if (m_boundingEdges[i] == m_boundingEdges[j])
			{
				m_boundingEdges.erase(m_boundingEdges.begin() + j);
				j--;
			}
		}
	}
}

//-----------------------------------------------------------------------------------------------
void ConvexHull3D::DebugDrawQuickull(bool drawWireOnly)
{
	//Existing points
	for (int pointIndex = 0; pointIndex < m_pointsToPartition.size(); pointIndex++)
	{
		Vec3& currentPoint = m_pointsToPartition[pointIndex];
		if (IsPointInside(currentPoint))
		{
			AABB3 point;
			point.SetCenter(currentPoint);
			point.SetDimensions(Vec3(0.01f, 0.01f, 0.01f));
			DebugAddWorldWireAABB3(point, 0.0001f, Rgba8::RED, Rgba8::RED);
			continue;
		}

		AABB3 point;
		point.SetCenter(currentPoint);
		point.SetDimensions(Vec3(0.01f, 0.01f, 0.01f));
		DebugAddWorldWireAABB3(point, 0.0001f, Rgba8::GREEN, Rgba8::GREEN);
	}

	Rgba8 color = Rgba8::RED;
	if (IsPointInside(Vec3(0.0f, 0.0f, 0.0f)))
		color = Rgba8::MEDIUM_GREEN;

	if (drawWireOnly == false)
	{
		for (int i = 0; i < m_boundingPolys.size(); i++)
		{
			DebugAddWorldConvexPoly3D(m_boundingPolys[i], 0.00001f, color, color);
		}
	}
	for (int i = 0; i < m_boundingPolys.size(); i++)
	{
		Vec3 center;
		for (int x = 0; x < m_boundingPolys[i].m_ccwOrderedPoints.size(); x++)
		{
			center += m_boundingPolys[i].m_ccwOrderedPoints[x];
		}
		center /= float(m_boundingPolys[i].m_ccwOrderedPoints.size());

		if (drawWireOnly == false)
		{
			DebugAddWorldWireConvexPoly3D(m_boundingPolys[i], 0.0001f, Rgba8::YELLOW, Rgba8::YELLOW);
		}
		else
		{
			DebugAddWorldWireConvexPoly3D(m_boundingPolys[i], 0.0001f, color, color);
		}
	}
}

//-----------------------------------------------------------------------------------------------
void ConvexHull3D::CalculateNewEpsilon(std::vector<Vec3> const& points)
{
	//Find Extreme Points
	float positiveZValue = -100000000000.0f;
	float positiveXValue = -100000000000.0f;
	float positiveYValue = -100000000000.0f;
	float negativeZValue = FLT_MAX;
	float negativeXValue = FLT_MAX;
	float negativeYValue = FLT_MAX;
	Vec3 positiveZVert;
	Vec3 positiveXVert;
	Vec3 positiveYVert;
	Vec3 negativeZVert;
	Vec3 negativeXVert;
	Vec3 negativeYVert;
	for (int pointIndex = 0; pointIndex < points.size(); pointIndex++)
	{
		if (points[pointIndex].z > positiveZValue)
		{
			positiveZValue = points[pointIndex].z;
			positiveZVert = points[pointIndex];
		}
		if (points[pointIndex].y > positiveYValue)
		{
			positiveYValue = points[pointIndex].y;
			positiveYVert = points[pointIndex];
		}
		if (points[pointIndex].x > positiveXValue)
		{
			positiveXValue = points[pointIndex].x;
			positiveXVert = points[pointIndex];
		}
		if (points[pointIndex].z < negativeZValue)
		{
			negativeZValue = points[pointIndex].z;
			negativeZVert = points[pointIndex];
		}
		if (points[pointIndex].y < negativeYValue)
		{
			negativeYValue = points[pointIndex].y;
			negativeYVert = points[pointIndex];
		}
		if (points[pointIndex].x < negativeXValue)
		{
			negativeXValue = points[pointIndex].x;
			negativeXVert = points[pointIndex];
		}
	}

	//Set epsilon
	float maxX = 0.0f;
	float maxY = 0.0f;
	float maxZ = 0.0f;
	if (fabsf(positiveXValue) > fabsf(negativeXValue))
		maxX = positiveXValue;
	else
		maxX = fabsf(negativeXValue);
	if (fabsf(positiveYValue) > fabsf(negativeYValue))
		maxY = positiveYValue;
	else
		maxY = fabsf(negativeYValue);
	if (fabsf(positiveZValue) > fabsf(negativeZValue))
		maxZ = positiveZValue;
	else
		maxZ = fabsf(negativeZValue);
	m_epsilon = 3.0f * (maxX + maxY + maxZ) * FLT_EPSILON;

}