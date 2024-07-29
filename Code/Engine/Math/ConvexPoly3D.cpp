#include "ConvexPoly3D.hpp"
#include "Vec3.hpp"
#include "MathUtils.hpp"
#include "AABB3.hpp"
#include "LineSegment3.hpp"
#include "Engine/Core/DebugRender.hpp"
#include "Engine/Core/ErrorWarningAssert.hpp"

//-----------------------------------------------------------------------------------------------
ConvexPoly3D::ConvexPoly3D(std::vector<Vec3> const& ccwOrderedPoints)
	:m_ccwOrderedPoints(ccwOrderedPoints)
{
}

//-----------------------------------------------------------------------------------------------
void ConvexPoly3D::SetAllPoints(std::vector<Vec3> const& ccwOrderedPoints)
{
	m_ccwOrderedPoints = ccwOrderedPoints;
}

//-----------------------------------------------------------------------------------------------
bool ConvexPoly3D::IsPointInside(Vec3 const& point)
{
	Vec3 normal = CrossProduct3D(m_ccwOrderedPoints[2] - m_ccwOrderedPoints[1], m_ccwOrderedPoints[0] - m_ccwOrderedPoints[1]);
	for (int pointIndex = 0; pointIndex < m_ccwOrderedPoints.size(); pointIndex++)
	{
		Vec3 currentToNext;
		Vec3 currentToPoint = point - m_ccwOrderedPoints[pointIndex];
		if (pointIndex == m_ccwOrderedPoints.size() - 1)
		{
			currentToNext = m_ccwOrderedPoints[0] - m_ccwOrderedPoints[pointIndex];
		}
		else
		{
			currentToNext = m_ccwOrderedPoints[pointIndex + 1] - m_ccwOrderedPoints[pointIndex];
		}

		Vec3 crossResult = CrossProduct3D(currentToNext, currentToPoint);
		if (DotProduct3D(normal, crossResult) <= 0.0f)
		{
			return false;
		}
	}

	return true;
}

//-----------------------------------------------------------------------------------------------
void ConvexPoly3D::AddPoint(Vec3 const& sentPoint, float epsilon)
{
	//Dynamic Convex Hull Algorithm
	//Find the closest point on polygon
	Vec3 normal = CrossProduct3D((m_ccwOrderedPoints[2] - m_ccwOrderedPoints[1]), (m_ccwOrderedPoints[0] - m_ccwOrderedPoints[1]));
	float closestPointDist = FLT_MAX;
	int lowerTangnetIndex = -1;
	int upperTangnetIndex = -1;
	bool doesAlreadyExist = false;
	LineSegment3 closestLine;
 	for (int pointIndex = 0; pointIndex < m_ccwOrderedPoints.size(); pointIndex++)
	{
		if (sentPoint == m_ccwOrderedPoints[pointIndex])
		{
			doesAlreadyExist = true;
			break;
		}

		LineSegment3 line;
		int lowIndex = pointIndex;
		int highIndex = -1;
		if (pointIndex != m_ccwOrderedPoints.size() - 1)
		{
			line = LineSegment3(m_ccwOrderedPoints[pointIndex], m_ccwOrderedPoints[pointIndex + 1]);
			highIndex = pointIndex + 1;
		}
		else
		{
			highIndex = 0;
			line = LineSegment3(m_ccwOrderedPoints[pointIndex], m_ccwOrderedPoints[0]);
		}

		Vec3 tangentNormal = CrossProduct3D((line.m_end - line.m_start), normal).GetNormalized();
		Plane3D tangentPlane(tangentNormal, DotProduct3D(tangentNormal, line.m_end));
		float currentAlt = DotProduct3D(sentPoint, tangentNormal) - tangentPlane.m_distanceFromOrigin;
		if (currentAlt < closestPointDist && currentAlt > epsilon)
		{
			closestPointDist = currentAlt;
			lowerTangnetIndex = lowIndex;
			upperTangnetIndex = highIndex; 
			closestLine = line;
		}
	}

	float distToClosestLine = GetDistanceToLineSegmentSquared3D(sentPoint, closestLine);
	if (distToClosestLine < epsilon * epsilon || closestPointDist == FLT_MAX || doesAlreadyExist)
	{
		return;
	}

	//Find Upper Tangent
	while (true)
	{
		Vec3 tangentNormal = CrossProduct3D((m_ccwOrderedPoints[upperTangnetIndex] - sentPoint), normal).GetNormalized();
		Plane3D tangentPlane(tangentNormal, DotProduct3D(tangentNormal, m_ccwOrderedPoints[upperTangnetIndex]));
		int prevIndex = upperTangnetIndex - 1;
		int nextIndex = upperTangnetIndex + 1;
		if (prevIndex < 0)
		{
			prevIndex = int(m_ccwOrderedPoints.size()) - 1;
		}
		if (nextIndex > int(m_ccwOrderedPoints.size()) - 1)
		{
			nextIndex = 0;
		}

		float prevAlt = DotProduct3D(m_ccwOrderedPoints[prevIndex], tangentNormal) - tangentPlane.m_distanceFromOrigin;
		float nextAlt = DotProduct3D(m_ccwOrderedPoints[nextIndex], tangentNormal) - tangentPlane.m_distanceFromOrigin;
		if (prevAlt * nextAlt >= 0.0f)
		{
			break;
		}
		else if ((prevAlt <= epsilon && prevAlt >= -epsilon) || (nextAlt <= epsilon && nextAlt >= -epsilon))
		{
			break;
		}

		upperTangnetIndex++;
		if (upperTangnetIndex > m_ccwOrderedPoints.size() - 1)
		{
			upperTangnetIndex = 0;
		}
	}
	//Lower Tangent
	while (true)
	{
		Vec3 tangentNormal = CrossProduct3D((m_ccwOrderedPoints[lowerTangnetIndex] - sentPoint), normal).GetNormalized();
		Plane3D tangentPlane(tangentNormal, DotProduct3D(tangentNormal, m_ccwOrderedPoints[lowerTangnetIndex]));
		int prevIndex = lowerTangnetIndex - 1;
		int nextIndex = lowerTangnetIndex + 1;
		if (prevIndex < 0)
		{
			prevIndex = int(m_ccwOrderedPoints.size()) - 1;
		}
		if (nextIndex > int(m_ccwOrderedPoints.size()) - 1)
		{
			nextIndex = 0;
		}

		float prevAlt = DotProduct3D(m_ccwOrderedPoints[prevIndex], tangentNormal) - tangentPlane.m_distanceFromOrigin;
		float nextAlt = DotProduct3D(m_ccwOrderedPoints[nextIndex], tangentNormal) - tangentPlane.m_distanceFromOrigin;
		if (prevAlt * nextAlt >= 0.0f)
		{
			break;
		}
		else if ((prevAlt <= epsilon && prevAlt >= -epsilon) || (nextAlt <= epsilon && nextAlt >= -epsilon))
		{
			break;
		}

		lowerTangnetIndex--;
		if (lowerTangnetIndex < 0)
		{
			lowerTangnetIndex = int(m_ccwOrderedPoints.size()) - 1;
		}
	}

	//Add the New Point
	if (upperTangnetIndex > lowerTangnetIndex)
	{
		if (upperTangnetIndex - lowerTangnetIndex == 1)
		{
			m_ccwOrderedPoints.insert(m_ccwOrderedPoints.begin() + (lowerTangnetIndex + 1), sentPoint);
		}
		else
		{
			m_ccwOrderedPoints.erase(m_ccwOrderedPoints.begin() + (lowerTangnetIndex + 1), m_ccwOrderedPoints.begin() + upperTangnetIndex);
			m_ccwOrderedPoints.insert(m_ccwOrderedPoints.begin() + (lowerTangnetIndex + 1), sentPoint);
		}
	}
	else
	{
		if (lowerTangnetIndex - upperTangnetIndex == m_ccwOrderedPoints.size() - 1)
		{
			m_ccwOrderedPoints.push_back(sentPoint);
		}
		else
		{
			m_ccwOrderedPoints.erase(m_ccwOrderedPoints.begin() + (lowerTangnetIndex + 1), m_ccwOrderedPoints.end());
			m_ccwOrderedPoints.erase(m_ccwOrderedPoints.begin(), m_ccwOrderedPoints.begin() + upperTangnetIndex);

			if (lowerTangnetIndex > m_ccwOrderedPoints.size() - 1)
			{
				lowerTangnetIndex = int(m_ccwOrderedPoints.size()) - 1;
			}
			m_ccwOrderedPoints.insert(m_ccwOrderedPoints.begin() + (lowerTangnetIndex + 1), sentPoint);
		}
	}

	//Check For Inline Points
	for (int pointIndex = 0; pointIndex < m_ccwOrderedPoints.size(); pointIndex++)
	{
		int index1 = pointIndex;
		int index2 = -1;
		int index3 = -1;
		if (pointIndex + 1 > m_ccwOrderedPoints.size() - 1)
		{
			index2 = 0;
			index3 = 1;
		}
		else if (pointIndex + 2 > m_ccwOrderedPoints.size() - 1)
		{
			index2 = pointIndex + 1;
			index3 = 0;
		}
		else
		{
			index2 = pointIndex + 1;
			index3 = pointIndex + 2;
		}

		Vec3 disp1 = (m_ccwOrderedPoints[index2] - m_ccwOrderedPoints[index1]);
		Vec3 disp2 = (m_ccwOrderedPoints[index3] - m_ccwOrderedPoints[index2]);
		Vec3 cross = CrossProduct3D(disp1, disp2);
		if (cross == Vec3())
		{
			m_ccwOrderedPoints.erase(m_ccwOrderedPoints.begin() + index2);
		}
		else if (fabsf(cross.x) < epsilon && fabsf(cross.y) < epsilon && fabsf(cross.z) < epsilon)
		{
			m_ccwOrderedPoints.erase(m_ccwOrderedPoints.begin() + index2);
		}
	}
}
