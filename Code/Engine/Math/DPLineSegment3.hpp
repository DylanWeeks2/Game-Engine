#pragma once
#include "DPVec3.hpp"

//-----------------------------------------------------------------------------------------------
struct DPLineSegment3
{
public:
	DPVec3 m_start;
	DPVec3 m_end;

public:
	//construction/destruction
	~DPLineSegment3() {}
	DPLineSegment3() {}
	DPLineSegment3(DPLineSegment3 const& copyFrom);
	explicit DPLineSegment3(double startX, double startY, double startZ, double endX, double endY, double endZ);
	explicit DPLineSegment3(DPVec3 const& start, DPVec3 const& end);
};
