#include "LineSegment2.hpp"

//-----------------------------------------------------------------------------------------------
LineSegment2::LineSegment2(LineSegment2 const& copyFrom)
	:m_start(copyFrom.m_start)
	,m_end(copyFrom.m_end)
{
}

//-----------------------------------------------------------------------------------------------
LineSegment2::LineSegment2(float startX, float startY, float endX, float endY)
	: m_start(Vec2(startX, startY))
	, m_end(Vec2(endX, endY))
{
}

//-----------------------------------------------------------------------------------------------
LineSegment2::LineSegment2(Vec2 const& start, Vec2 const& end)
	: m_start(start)
	, m_end(end)
{
}
