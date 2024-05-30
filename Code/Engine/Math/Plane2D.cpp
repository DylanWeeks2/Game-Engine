#include "Plane2D.hpp"

//-----------------------------------------------------------------------------------------------
Plane2D::Plane2D(Vec2 const& normal, float const& distanceFromOrigin)
	:m_normal(normal)
	,m_distanceFromOrigin(distanceFromOrigin)
{
}
