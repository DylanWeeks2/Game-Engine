#include "CubicBezierCurve2D.hpp"
#include "MathUtils.hpp"
#include "LineSegment2.hpp"
#include "CubicHermiteCurve2D.hpp"

//-----------------------------------------------------------------------------------------------
CubicBezierCurve2D::CubicBezierCurve2D(Vec2 starPos, Vec2 guidePos1, Vec2 guidePos2, Vec2 endPos)
	:m_startPos(starPos)
	,m_guidePos1(guidePos1)
	,m_guidePos2(guidePos2)
	,m_endPos(endPos)
{
}

//-----------------------------------------------------------------------------------------------
CubicBezierCurve2D::CubicBezierCurve2D(CubicHermiteCurve2D const& fromHermite)
	:m_startPos(fromHermite.m_startPos)
	,m_guidePos1(fromHermite.m_startPos + (fromHermite.m_startVel / 3.0f))
	,m_guidePos2(fromHermite.m_endPos + (fromHermite.m_endVel / 3.0f))
	,m_endPos(fromHermite.m_endPos)
{
}

//-----------------------------------------------------------------------------------------------
Vec2 CubicBezierCurve2D::EvaluateAtParametric(float prametricZeroToOne) const
{
	float tX = ComputeCubicBezier1D(m_startPos.x, m_guidePos1.x, m_guidePos2.x, m_endPos.x, prametricZeroToOne);
	float tY = ComputeCubicBezier1D(m_startPos.y, m_guidePos1.y, m_guidePos2.y, m_endPos.y, prametricZeroToOne);
	return Vec2(tX, tY);
}

//-----------------------------------------------------------------------------------------------
float CubicBezierCurve2D::GetApproximateLength(int numSubdivisions) const
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
Vec2 CubicBezierCurve2D::EvaluateAtApproximateDistance(float distanceAlongCurve, int numSubdivisions) const
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
