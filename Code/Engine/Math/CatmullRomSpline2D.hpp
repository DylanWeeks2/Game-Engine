#pragma once
#include <vector>

class CubicHermiteCurve2D;
struct Vec2;

class CatmullRomSpline2D
{
public:
	CatmullRomSpline2D(std::vector<Vec2> positions);
	~CatmullRomSpline2D();

public:
	std::vector<CubicHermiteCurve2D*> m_hermiteCurves;
};