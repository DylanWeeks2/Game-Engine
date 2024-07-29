#include "Plane3D.hpp"
#include "ConvexPoly3D.hpp"
#include "MathUtils.hpp"

//-----------------------------------------------------------------------------------------------
Plane3D::Plane3D(Vec3 const& normal, float const& distanceFromOrigin)
	:m_normal(normal)
	, m_distanceFromOrigin(distanceFromOrigin) {
}

//-----------------------------------------------------------------------------------------------
Plane3D::Plane3D(ConvexPoly3D const& poly)
{
	m_normal = CrossProduct3D((poly.m_ccwOrderedPoints[2] - poly.m_ccwOrderedPoints[1]), (poly.m_ccwOrderedPoints[0] - poly.m_ccwOrderedPoints[1])).GetNormalized();
	m_distanceFromOrigin = DotProduct3D(m_normal, poly.m_ccwOrderedPoints[2]);
}

//-----------------------------------------------------------------------------------------------
bool Plane3D::operator==(Plane3D& comparePlane)
{
	if (m_normal == comparePlane.m_normal && m_distanceFromOrigin == comparePlane.m_distanceFromOrigin)
	{
		return true;
	}

	return false;
}

//-----------------------------------------------------------------------------------------------
bool Plane3D::operator!=(Plane3D& comparePlane)
{
	if (m_normal != comparePlane.m_normal || m_distanceFromOrigin != comparePlane.m_distanceFromOrigin)
	{
		return true;
	}

	return false;
}

//-----------------------------------------------------------------------------------------------
float Plane3D::GetAltitude(Vec3 const& point)
{
	return DotProduct3D(m_normal, point) - m_distanceFromOrigin;
}
