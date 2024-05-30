#include "DPOBB3.hpp"
#include "MathUtils.hpp"

//-----------------------------------------------------------------------------------------------
DPOBB3::DPOBB3(DPOBB3 const& copyFrom)
	:m_center(copyFrom.m_center)
	,m_iBasisNormal(copyFrom.m_iBasisNormal)
	,m_jBasisNormal(copyFrom.m_jBasisNormal)
	,m_kBasisNormal(copyFrom.m_kBasisNormal)
	,m_halfDimensions(copyFrom.m_halfDimensions)
{
}

//-----------------------------------------------------------------------------------------------
DPOBB3::DPOBB3(DPVec3& center, DPVec3& iBasisNormal, DPVec3& halfDimensions)
	:m_center(center)
	,m_iBasisNormal(iBasisNormal)
	,m_halfDimensions(halfDimensions)
{
	DPVec3 jBasis = CrossProduct3D(DPVec3(0.0f, 0.0f, 1.0f), m_iBasisNormal);
	if (jBasis == DPVec3(0.0f, 0.0f, 0.0f))
	{
		jBasis = DPVec3(0.0f, 1.0f, 0.0f);
	}
	else
	{
		jBasis.Normalize();
	}
	DPVec3 kBasis = CrossProduct3D(m_iBasisNormal, jBasis).GetNormalized();

	m_jBasisNormal = jBasis;
	m_kBasisNormal = kBasis;
}
