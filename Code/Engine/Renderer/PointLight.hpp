#pragma once
#include "Engine/Math/Vec3.hpp"
#include "Engine/Core/Rgba8.hpp"

//-----------------------------------------------------------------------------------------------
struct PointLight
{
public:
	PointLight();
	PointLight(Vec3 position, float range, float intensity, Rgba8 color);
	~PointLight();

public:
	Vec3  m_position;
	float m_range;
	float m_intensity;
	Rgba8 m_color;
};