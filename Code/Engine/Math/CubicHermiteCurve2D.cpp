#include "CubicHermiteCurve2D.hpp"
#include "MathUtils.hpp"
#include "LineSegment2.hpp"
#include "CubicBezierCurve2D.hpp"

//-----------------------------------------------------------------------------------------------
CubicHermiteCurve2D::CubicHermiteCurve2D(Vec2 starPos, Vec2 startVel, Vec2 endPos, Vec2 endVel)
	:m_startPos(starPos)
	,m_startVel(startVel)
	,m_endPos(endPos)
	,m_endVel(endVel)
{
}

//-----------------------------------------------------------------------------------------------
CubicHermiteCurve2D::CubicHermiteCurve2D(CubicBezierCurve2D const& fromBezier)
	:m_startPos(fromBezier.m_startPos)
	,m_startVel(3 * (fromBezier.m_guidePos1 - fromBezier.m_startPos))
	,m_endPos(fromBezier.m_endPos)
	,m_endVel(3 * (fromBezier.m_endPos - fromBezier.m_guidePos2))
{
}

//-----------------------------------------------------------------------------------------------
Vec2 CubicHermiteCurve2D::EvaluateAtParametric(float prametricZeroToOne) const
{
	Vec2 guidePos1 = m_startPos + (m_startVel / 3.0f);
	Vec2 guidePos2 = m_endPos - (m_endVel / 3.0f);
	float tX = ComputeCubicBezier1D(m_startPos.x, guidePos1.x, guidePos2.x, m_endPos.x, prametricZeroToOne);
	float tY = ComputeCubicBezier1D(m_startPos.y, guidePos1.y, guidePos2.y, m_endPos.y, prametricZeroToOne);
	return Vec2(tX, tY);
}

//-----------------------------------------------------------------------------------------------
float CubicHermiteCurve2D::GetApproximateLength(int numSubdivisions) const
{
	float totalLength = 0.0f;
	float t = 1.0f / numSubdivisions;
	for (int subDivisionIndex = 0; subDivisionIndex < numSubdivisions; subDivisionIndex++)
	{
		LineSegment2 newLine1;
		newLine1.m_start = EvaluateAtParametric(t * (subDivisionIndex));
		newLine1.m_end = EvaluateAtParametric(t * (subDivisionIndex + 1));

		totalLength += ((newLine1.m_start - newLine1.m_end).GetLength());
	}

	return totalLength;
}

//-----------------------------------------------------------------------------------------------
Vec2 CubicHermiteCurve2D::EvaluateAtApproximateDistance(float distanceAlongCurve, int numSubdivisions) const
{
	float currentLength = 0.0f;
	float t = 1.0f / numSubdivisions;
	for (int subDivisionIndex = 0; subDivisionIndex < numSubdivisions; subDivisionIndex++)
	{
		LineSegment2 newLine1;
		newLine1.m_start = EvaluateAtParametric(t * (subDivisionIndex));
		newLine1.m_end = EvaluateAtParametric(t * (subDivisionIndex + 1));

		float lineLength = ((newLine1.m_start - newLine1.m_end).GetLength());
		currentLength += lineLength;

		if (currentLength > distanceAlongCurve)
		{
			float currentT = RangeMap(lineLength - (currentLength - distanceAlongCurve), 0.0f, lineLength, 0.0f, 1.0f);
			float tX = Interpolate(newLine1.m_start.x, newLine1.m_end.x, currentT);
			float tY = Interpolate(newLine1.m_start.y, newLine1.m_end.y, currentT);
			return Vec2(tX, tY);
		}
	}
	return Vec2();
}
