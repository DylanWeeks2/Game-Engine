#include "Cylinder3.hpp"
#include "Engine/Math/MathUtils.hpp"

//-----------------------------------------------------------------------------------------------
Cylinder3::Cylinder3(Cylinder3 const& copyFrom)
	: m_start(copyFrom.m_start)
	, m_end(copyFrom.m_end)
	, m_iBasis(copyFrom.m_iBasis)
	, m_jBasis(copyFrom.m_jBasis)
	, m_kBasis(copyFrom.m_kBasis)
	, m_radius(copyFrom.m_radius)
{
}

//-----------------------------------------------------------------------------------------------
Cylinder3::Cylinder3(Vec3 start, Vec3 end, float radius)
	: m_start(start)
	, m_end(end)
	, m_radius(radius)
{
	Vec3 kBasis = (m_end - m_start).GetNormalized();
	Vec3 iBasis = CrossProduct3D(Vec3(0.0f, 1.0f, 0.0f), kBasis);
	if (iBasis == Vec3(0.0f, 0.0f, 0.0f))
	{
		iBasis = Vec3(1.0f, 0.0f, 0.0f);
	}
	else
	{
		iBasis.Normalize();
	}
	Vec3 jBasis = CrossProduct3D(kBasis, iBasis).GetNormalized();

	m_iBasis = iBasis;
	m_jBasis = jBasis;
	m_kBasis = kBasis;
}
