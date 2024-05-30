#include "Vertex_PCUTBN.hpp"

//-----------------------------------------------------------------------------------------------
Vertex_PCUTBN::Vertex_PCUTBN()
{
}

//-----------------------------------------------------------------------------------------------
Vertex_PCUTBN::Vertex_PCUTBN(Vec3 const& position, Rgba8 const& tint, Vec2 const& uvTexCoords, Vec3 tangent, Vec3 binormal, Vec3 normal)
	: m_position(position)
	, m_uvTexCoords(uvTexCoords)
	, m_tangent(tangent)
	, m_binormal(binormal)
	, m_normal(normal)
	, m_color(tint)
{
}
