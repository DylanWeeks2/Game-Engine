#include "PointLight.hpp"

//-----------------------------------------------------------------------------------------------
PointLight::PointLight()
{
}

//-----------------------------------------------------------------------------------------------
PointLight::PointLight(Vec3 position, float range, float intensity, Rgba8 color)
	:m_position(position)
	,m_range(range)
	,m_intensity(intensity)
	,m_color(color)
{
}

//-----------------------------------------------------------------------------------------------
PointLight::~PointLight()
{
}
