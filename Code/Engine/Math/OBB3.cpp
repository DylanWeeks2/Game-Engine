#include "OBB3.hpp"
#include "MathUtils.hpp"

//-----------------------------------------------------------------------------------------------
OBB3::OBB3(OBB3 const& copyFrom)
	:m_center(copyFrom.m_center)
	,m_iBasisNormal(copyFrom.m_iBasisNormal)
	,m_jBasisNormal(copyFrom.m_jBasisNormal)
	,m_kBasisNormal(copyFrom.m_kBasisNormal)
	,m_halfDimensions(copyFrom.m_halfDimensions)
{
}

//-----------------------------------------------------------------------------------------------
OBB3::OBB3(Vec3& center, Vec3& iBasisNormal, Vec3& halfDimensions)
	:m_center(center)
	,m_iBasisNormal(iBasisNormal)
	,m_halfDimensions(halfDimensions)
{
	Vec3 jBasis = CrossProduct3D(Vec3(0.0f, 0.0f, 1.0f), m_iBasisNormal);
	if (jBasis == Vec3(0.0f, 0.0f, 0.0f))
	{
		jBasis = Vec3(0.0f, 1.0f, 0.0f);
	}
	else
	{
		jBasis.Normalize();
	}
	Vec3 kBasis = CrossProduct3D(m_iBasisNormal, jBasis).GetNormalized();

	m_jBasisNormal = jBasis;
	m_kBasisNormal = kBasis;
}
