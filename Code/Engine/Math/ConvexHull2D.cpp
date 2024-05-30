#include "ConvexHull2D.hpp"
#include "Plane2D.hpp"
#include "ConvexPoly2D.hpp"
#include "MathUtils.hpp"

//-----------------------------------------------------------------------------------------------
ConvexHull2D::ConvexHull2D(std::vector<Plane2D> const& boundingPlanes)
	:m_boundingPlanes(boundingPlanes)
{
}

//-----------------------------------------------------------------------------------------------
ConvexHull2D::ConvexHull2D(ConvexPoly2D convexPoly)
{
	for (int pointIndex = 0; pointIndex < convexPoly.m_ccwOrderedPoints.size(); pointIndex++)
	{
		Vec2 currentPoint;
		Vec2 nextPoint;
		if (pointIndex == convexPoly.m_ccwOrderedPoints.size() - 1)
		{
			currentPoint = convexPoly.m_ccwOrderedPoints[pointIndex];
			nextPoint = convexPoly.m_ccwOrderedPoints[0];
		}
		else
		{
			currentPoint = convexPoly.m_ccwOrderedPoints[pointIndex];
			nextPoint = convexPoly.m_ccwOrderedPoints[pointIndex + 1];
		}
		
		Vec2 normal = (nextPoint - currentPoint).GetRotatedMinus90Degrees().GetNormalized();
		float d = DotProduct2D(currentPoint, normal);
		Plane2D plane = Plane2D(normal, d);
		m_boundingPlanes.push_back(plane);
	}
}

//-----------------------------------------------------------------------------------------------
bool ConvexHull2D::IsPointInside(Vec2 const& point)
{
	for (int planeIndex = 0; planeIndex < m_boundingPlanes.size(); planeIndex++)
	{
		Plane2D& plane = m_boundingPlanes[planeIndex];
		if (DotProduct2D(point, plane.m_normal) >= plane.m_distanceFromOrigin)
		{
			return false;
		}
	}

	return true;
}
