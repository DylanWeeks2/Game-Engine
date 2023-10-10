#pragma once
#include "Vec2.hpp"

class CubicBezierCurve2D;

class CubicHermiteCurve2D
{
public:
	CubicHermiteCurve2D(Vec2 starPos, Vec2 startVel, Vec2 endPos, Vec2 endVel);
	explicit CubicHermiteCurve2D(CubicBezierCurve2D const& fromBezier);
	Vec2 EvaluateAtParametric(float prametricZeroToOne) const;
	float GetApproximateLength(int numSubdivisions = 64) const;
	Vec2 EvaluateAtApproximateDistance(float distanceAlongCurve, int numSubdivisions = 64) const;

public:
	Vec2 m_startPos;
	Vec2 m_startVel;
	Vec2 m_endPos;
	Vec2 m_endVel;
};