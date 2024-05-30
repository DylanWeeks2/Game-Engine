#pragma once
#include "DPVec3.hpp"

//-----------------------------------------------------------------------------------------------
struct DPSphere3
{
public:
	DPVec3 m_center;
	double m_radius = 0.0f;

public:
	//construction/destruction
	~DPSphere3() {}
	DPSphere3() {}
	DPSphere3(DPSphere3 const& copyFrom);
	explicit DPSphere3(double centerX, double centerY, double radius);
	explicit DPSphere3(DPVec3 const& center, double radius);
};