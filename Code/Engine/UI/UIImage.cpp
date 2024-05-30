#include "UIImage.hpp"
#include "Widget.hpp"
#include "Engine/Renderer/Texture.hpp"
#include "Engine/Core/Vertex_PCU.hpp"
#include "Engine/Core/VertexUtils.hpp"
#include "Engine/Core/EngineCommon.hpp"
#include "Engine/Renderer/Renderer.hpp"
#include "Engine/Math/MathUtils.hpp"

//-----------------------------------------------------------------------------------------------
UIImage::UIImage(Texture* texture, Widget* parentWidget, AABB2 const& bounds, Vec2 pivot, Vec2 widgetAlignment, Vec2 localAlignment, Vec2 scale, 
	float orientation, bool isFocusable, IntVec2 widgetOrder, Rgba8 color, Rgba8 backgroundColor, Rgba8 borderColor, float borderTickness)
	:UIEntity(parentWidget, bounds, pivot, widgetAlignment, localAlignment, scale, orientation, isFocusable, widgetOrder, color, backgroundColor,
		borderColor, borderTickness)
	,m_texture(texture)
{
}

//-----------------------------------------------------------------------------------------------
UIImage::~UIImage()
{
}

//-----------------------------------------------------------------------------------------------
void UIImage::Update()
{
}

//-----------------------------------------------------------------------------------------------
void UIImage::Render() const
{
	std::vector<Vertex_PCU> verts;
	std::vector<Vertex_PCU> backgroundVerts;
	Vec2 dimensions = m_bounds.GetDimensions();
	Vec2 center = m_bounds.GetCenter();
	AABB2 tempBounds;
	tempBounds.m_maxs = dimensions;
	tempBounds.SetCenter(Vec2());
	AddVertsForAABB2D(verts, tempBounds, m_color);
	if (m_backgroundColor.a != 0)
	{
		AddVertsForAABB2D(backgroundVerts, tempBounds, m_backgroundColor);
	}

	//Transforming and Drawing
	Vec2 iBasis = Vec2(CosDegrees(m_orientation), SinDegrees(m_orientation)).GetNormalized();
	Vec2 jBasis = iBasis.GetRotated90Degrees();
	TransformVertexArrayXY3D(int(verts.size()), verts.data(), 1.0f, iBasis, jBasis, center);
	TransformVertexArrayXY3D(int(backgroundVerts.size()), backgroundVerts.data(), 1.0f, iBasis, jBasis, center);
	m_parentWidget->GetRenderer()->BindTexture(nullptr);
	m_parentWidget->GetRenderer()->DrawVertexArray(int(backgroundVerts.size()), backgroundVerts.data());
	m_parentWidget->GetRenderer()->BindTexture(m_texture);
	m_parentWidget->GetRenderer()->DrawVertexArray(int(verts.size()), verts.data());
}

//-----------------------------------------------------------------------------------------------
Texture* UIImage::GetTexture() const
{
	return m_texture;
}

//-----------------------------------------------------------------------------------------------
void UIImage::SetTexture(Texture* texture)
{
	m_texture = texture;
}
