#include "Plane3D.hpp"

//-----------------------------------------------------------------------------------------------
Plane3D::Plane3D(Vec3 const& normal, float const& distanceFromOrigin)
	:m_normal(normal)
	, m_distanceFromOrigin(distanceFromOrigin) {
}
