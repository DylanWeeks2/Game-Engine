#include "CatmullRomSpline2D.hpp"
#include "MathUtils.hpp"
#include "LineSegment2.hpp"
#include "CubicHermiteCurve2D.hpp"
#include "Vec2.hpp"

//-----------------------------------------------------------------------------------------------
CatmullRomSpline2D::CatmullRomSpline2D(std::vector<Vec2> positions)
{
	for (int positionIndex = 0; positionIndex < positions.size(); positionIndex++)
	{
		if (positionIndex != 0)
		{
			
			Vec2 startPos = positions[positionIndex - 1];
			Vec2 startVel = Vec2();
			if(positionIndex != 1)
			{
				startVel = (positions[positionIndex] - positions[positionIndex - 2]) / 2;
			}
			Vec2 endVel = Vec2();
			if (positionIndex != positions.size() - 1)
			{
				endVel = (positions[positionIndex + 1] - positions[positionIndex - 1]) / 2;
			}
			Vec2 endPos = positions[positionIndex];
			CubicHermiteCurve2D* newCurve = new CubicHermiteCurve2D(startPos, startVel, endPos, endVel);
			m_hermiteCurves.push_back(newCurve);
		}
	}
}


//-----------------------------------------------------------------------------------------------
CatmullRomSpline2D::~CatmullRomSpline2D()
{
	for (int curveIndex = 0; curveIndex < m_hermiteCurves.size(); curveIndex++)
	{
		if (m_hermiteCurves[curveIndex] != nullptr)
		{
			delete m_hermiteCurves[curveIndex];
			m_hermiteCurves[curveIndex] = nullptr;
		}
	}
}
