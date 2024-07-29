#include "LineSegment3.hpp"

//-----------------------------------------------------------------------------------------------
LineSegment3::LineSegment3(LineSegment3 const& copyFrom)
	:m_start(copyFrom.m_start)
	,m_end(copyFrom.m_end)
{
}

//-----------------------------------------------------------------------------------------------
LineSegment3::LineSegment3(float startX, float startY, float startZ, float endX, float endY, float endZ)
	: m_start(Vec3(startX, startY, startZ))
	, m_end(Vec3(endX, endY, endZ))
{
}

//-----------------------------------------------------------------------------------------------
LineSegment3::LineSegment3(Vec3 const& start, Vec3 const& end)
	: m_start(start)
	, m_end(end)
{
}

//-----------------------------------------------------------------------------------------------
bool LineSegment3::operator!=(LineSegment3& compareLine)
{
	if ((m_start != compareLine.m_start && m_start != compareLine.m_end)
		|| (m_end != compareLine.m_end && m_end != compareLine.m_start))
	{
		return true;
	}
	
	return false;
}

//-----------------------------------------------------------------------------------------------
bool LineSegment3::operator==(LineSegment3& compareLine)
{
	if ((m_start == compareLine.m_start || m_start == compareLine.m_end)
		&& (m_end == compareLine.m_end || m_end == compareLine.m_start))
	{
		return true;
	}

	return false;
}
