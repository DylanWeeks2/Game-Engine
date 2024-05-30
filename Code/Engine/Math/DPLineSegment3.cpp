#include "DPLineSegment3.hpp"

//-----------------------------------------------------------------------------------------------
DPLineSegment3::DPLineSegment3(DPLineSegment3 const& copyFrom)
	:m_start(copyFrom.m_start)
	,m_end(copyFrom.m_end)
{
}

//-----------------------------------------------------------------------------------------------
DPLineSegment3::DPLineSegment3(double startX, double startY, double startZ, double endX, double endY, double endZ)
	: m_start(DPVec3(startX, startY, startZ))
	, m_end(DPVec3(endX, endY, endZ))
{
}

//-----------------------------------------------------------------------------------------------
DPLineSegment3::DPLineSegment3(DPVec3 const& start, DPVec3 const& end)
	: m_start(start)
	, m_end(end)
{
}
