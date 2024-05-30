#pragma once
#include "Rgba8.hpp"
#include "Engine/Math/Vec2.hpp"
#include "Engine/Math/Vec3.hpp"

//-----------------------------------------------------------------------------------------------
struct Vertex_PCUTBN
{
public:
	Vertex_PCUTBN();
	explicit Vertex_PCUTBN(Vec3 const& position, Rgba8 const& tint, Vec2 const& uvTexCoords, 
		Vec3 tangent, Vec3 binormal, Vec3 normal);

public:
	Vec3	m_position;
	Rgba8	m_color;
	Vec2	m_uvTexCoords;
	Vec3	m_tangent;
	Vec3	m_binormal;
	Vec3	m_normal;
};
