#include "UIText.hpp"
#include "Widget.hpp"
#include "Engine/Renderer/Texture.hpp"
#include "Engine/Core/Vertex_PCU.hpp"
#include "Engine/Core/VertexUtils.hpp"
#include "Engine/Core/EngineCommon.hpp"
#include "Engine/Renderer/Renderer.hpp"
#include "Engine/Renderer/BitmapFont.hpp"
#include "Engine/Math/MathUtils.hpp"

//-----------------------------------------------------------------------------------------------
UIText::UIText(BitmapFont* font, std::string text, float textHeight, Widget* parentWidget, AABB2 const& bounds, Vec2 pivot, Vec2 widgetAlignment,
	Vec2 localAlignment, Vec2 scale, float orientation, bool isFocusable, IntVec2 widgetOrder, Rgba8 color, Rgba8 backgroundColor,
	Rgba8 borderColor, float borderTickness)
	:UIEntity(parentWidget, bounds, pivot, widgetAlignment, localAlignment, scale, orientation, isFocusable, widgetOrder, color, backgroundColor,
		borderColor, borderTickness)
	,m_font(font)
	,m_text(text)
	,m_textHeight(textHeight)
{
}

//-----------------------------------------------------------------------------------------------
UIText::~UIText()
{
}

//-----------------------------------------------------------------------------------------------
void UIText::Update()
{
}

//-----------------------------------------------------------------------------------------------
void UIText::Render() const
{
	std::vector<Vertex_PCU> verts;
	std::vector<Vertex_PCU> backgroundVerts;
	Vec2 dimensions = m_bounds.GetDimensions();
	Vec2 center = m_bounds.GetCenter();
	AABB2 borderBoundsNoFocus;
	borderBoundsNoFocus.m_maxs = dimensions;
	borderBoundsNoFocus.SetCenter(Vec2());
	AABB2 noBorderBoundsNoFocus;
	noBorderBoundsNoFocus.m_maxs = Vec2(m_borderThickness, m_borderThickness);
	noBorderBoundsNoFocus.m_maxs = Vec2(dimensions.x - m_borderThickness, dimensions.y - m_borderThickness);
	noBorderBoundsNoFocus.SetCenter(Vec2());

	m_font->AddVertsForTextInBox2D(verts, noBorderBoundsNoFocus, m_textHeight, m_text, m_color, 1.0f, m_localAlignment);
	AddVertsForAABB2D(backgroundVerts, borderBoundsNoFocus, m_borderColor);
	AddVertsForAABB2D(backgroundVerts, noBorderBoundsNoFocus, m_backgroundColor);

	//Transforming and Drawing
	Vec2 iBasis = Vec2(CosDegrees(m_orientation), SinDegrees(m_orientation)).GetNormalized();
	Vec2 jBasis = iBasis.GetRotated90Degrees();
	TransformVertexArrayXY3D(int(verts.size()), verts.data(), 1.0f, iBasis, jBasis, center);
	TransformVertexArrayXY3D(int(backgroundVerts.size()), backgroundVerts.data(), 1.0f, iBasis, jBasis, center);
	m_parentWidget->GetRenderer()->BindTexture(nullptr);
	m_parentWidget->GetRenderer()->DrawVertexArray(int(backgroundVerts.size()), backgroundVerts.data());
	m_parentWidget->GetRenderer()->BindTexture(&m_font->GetTexture());
	m_parentWidget->GetRenderer()->DrawVertexArray(int(verts.size()), verts.data());
}

//-----------------------------------------------------------------------------------------------
std::string UIText::GetText() const
{
	return m_text;
}

//-----------------------------------------------------------------------------------------------
float UIText::GetTextHeight() const
{
	return m_textHeight;
}

//-----------------------------------------------------------------------------------------------
BitmapFont* UIText::GetFont() const
{
	return m_font;
}

//-----------------------------------------------------------------------------------------------
void UIText::SetText(std::string const& text)
{
	m_text = text;
}

//-----------------------------------------------------------------------------------------------
void UIText::SetTextHeight(float textHeight)
{
	m_textHeight = textHeight;
}

//-----------------------------------------------------------------------------------------------
void UIText::SetFont(BitmapFont* font)
{
	m_font = font;
}
