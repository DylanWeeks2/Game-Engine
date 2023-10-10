#pragma once
#include "Rgba8.hpp"
#include "Engine/Math/Vec2.hpp"
#include "Engine/Math/Vec3.hpp"

struct Vertex_PNCU
{
public:
	Vec3 m_position;
	Vec3 m_normal;
	Rgba8 m_color;
	Vec2 m_uvTexCoords;

	Vertex_PNCU();
	explicit Vertex_PNCU(Vec3 const& position, Vec3 m_normal, Rgba8 const& tint, Vec2 const& uvTexCoords);
};