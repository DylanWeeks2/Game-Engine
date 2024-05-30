#include "ConvexPoly2D.hpp"
#include "Vec2.hpp"
#include "MathUtils.hpp"
#include "Engine/Core/ErrorWarningAssert.hpp"

//-----------------------------------------------------------------------------------------------
ConvexPoly2D::ConvexPoly2D(std::vector<Vec2> const& ccwOrderedPoints)
	:m_ccwOrderedPoints(ccwOrderedPoints)
{
}

//-----------------------------------------------------------------------------------------------
void ConvexPoly2D::SetAllPoints(std::vector<Vec2> const& ccwOrderedPoints)
{
	m_ccwOrderedPoints = ccwOrderedPoints;
}

//-----------------------------------------------------------------------------------------------
bool ConvexPoly2D::IsPointInside(Vec2 const& point)
{
	for (int pointIndex = 0; pointIndex < m_ccwOrderedPoints.size(); pointIndex++)
	{
		Vec2 currentToNext;
		Vec2 currentToPoint = point - m_ccwOrderedPoints[pointIndex];
		if (pointIndex == m_ccwOrderedPoints.size() - 1)
		{
			currentToNext = m_ccwOrderedPoints[0] - m_ccwOrderedPoints[pointIndex];
		}
		else
		{
			currentToNext = m_ccwOrderedPoints[pointIndex + 1] - m_ccwOrderedPoints[pointIndex];
		}

		if (CrossProduct2D(currentToNext.GetNormalized(), currentToPoint.GetNormalized()) < 0.0f)
		{
			return false;
		}
	}

	return true;
}
