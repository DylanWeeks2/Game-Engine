#include "DPCylinder3.hpp"
#include "Engine/Math/MathUtils.hpp"

//-----------------------------------------------------------------------------------------------
DPCylinder3::DPCylinder3(DPCylinder3 const& copyFrom)
	: m_start(copyFrom.m_start)
	, m_end(copyFrom.m_end)
	, m_iBasis(copyFrom.m_iBasis)
	, m_jBasis(copyFrom.m_jBasis)
	, m_kBasis(copyFrom.m_kBasis)
	, m_radius(copyFrom.m_radius)
{
}

//-----------------------------------------------------------------------------------------------
DPCylinder3::DPCylinder3(DPVec3 start, DPVec3 end, double radius)
	: m_start(start)
	, m_end(end)
	, m_radius(radius)
{
	DPVec3 kBasis = (m_end - m_start).GetNormalized();
	DPVec3 iBasis = CrossProduct3D(DPVec3(0.0f, 1.0f, 0.0f), kBasis);
	if (iBasis == DPVec3(0.0f, 0.0f, 0.0f))
	{
		iBasis = DPVec3(1.0f, 0.0f, 0.0f);
	}
	else
	{
		iBasis.Normalize();
	}
	DPVec3 jBasis = CrossProduct3D(kBasis, iBasis).GetNormalized();

	m_iBasis = iBasis;
	m_jBasis = jBasis;
	m_kBasis = kBasis;
}
