#include "Vertex_PNCU.hpp"

//-----------------------------------------------------------------------------------------------
Vertex_PNCU::Vertex_PNCU()
{
}

//-----------------------------------------------------------------------------------------------
Vertex_PNCU::Vertex_PNCU(Vec3 const& position, Vec3 normal, Rgba8 const& tint, Vec2 const& uvTexCoords)
	: m_position(position)
	, m_normal(normal)
	, m_color(tint)
	, m_uvTexCoords(uvTexCoords)
{
}