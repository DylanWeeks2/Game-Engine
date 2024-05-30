#pragma once
#include "DPVec3.hpp"

//-----------------------------------------------------------------------------------------------
struct DPOBB3
{
public:
	DPVec3 m_center;
	DPVec3 m_iBasisNormal;
	DPVec3 m_jBasisNormal;
	DPVec3 m_kBasisNormal;
	DPVec3 m_halfDimensions;

public:
	//construction/destruction
	~DPOBB3() {}
	DPOBB3() {}
	DPOBB3(DPOBB3 const& copyFrom);
	explicit DPOBB3(DPVec3& center, DPVec3& iBasisNormal, DPVec3& halfDimensions);
};