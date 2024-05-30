#pragma once
#include "DPVec3.hpp"

//-----------------------------------------------------------------------------------------------
struct DPCylinder3
{
public:
	DPVec3 m_start;
	DPVec3 m_end;
	DPVec3 m_iBasis;
	DPVec3 m_jBasis;
	DPVec3 m_kBasis;
	double m_radius = 0.0f;

public:
	//construction/destruction
	~DPCylinder3() {}
	DPCylinder3() {}
	DPCylinder3(DPCylinder3 const& copyFrom);
	explicit DPCylinder3(DPVec3 start, DPVec3 end, double radius);
};