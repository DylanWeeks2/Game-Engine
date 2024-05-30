#include "UIEntity.hpp"
#include "Widget.hpp"

//-----------------------------------------------------------------------------------------------
UIEntity::UIEntity(Widget* parentWidget, AABB2 const& bounds, Vec2 pivot, Vec2 widgetAlignment, Vec2 localAlignment, Vec2 scale,
	float orientation, bool isFocusable, IntVec2 widgetOrder, Rgba8 color, Rgba8 backgroundColor, Rgba8 borderColor, float borderTickness)
	:m_parentWidget(parentWidget)
	,m_bounds(bounds)
	,m_pivot(pivot)
	,m_widgetAlignment(widgetAlignment)
	,m_localAlignment(localAlignment)
	,m_scale(scale)
	,m_orientation(orientation)
	,m_isFocusable(isFocusable)
	,m_widgetOrder(widgetOrder)
	,m_color(color)
	,m_backgroundColor(backgroundColor)
	,m_borderColor(borderColor)
	,m_borderThickness(borderTickness)
{
	Vec2 newCenter = m_parentWidget->GetBounds().GetPointAtUV(m_widgetAlignment);
	m_bounds.SetCenter(newCenter + m_bounds.GetDimensions() * 0.5f - m_bounds.GetDimensions() * m_pivot);
}

//-----------------------------------------------------------------------------------------------
UIEntity::~UIEntity()
{
}

//-----------------------------------------------------------------------------------------------
Widget* UIEntity::GetParentWidget() const
{
	return m_parentWidget;
}

//-----------------------------------------------------------------------------------------------
AABB2 UIEntity::GetBounds() const
{
	return m_bounds;
}

//-----------------------------------------------------------------------------------------------
Vec2 UIEntity::GetWidgetAlignment() const
{
	return m_widgetAlignment;
}

//-----------------------------------------------------------------------------------------------
Vec2 UIEntity::GetLocalAlignment() const
{
	return m_localAlignment;
}

//-----------------------------------------------------------------------------------------------
Vec2 UIEntity::GetScale() const
{
	return m_scale;
}

//-----------------------------------------------------------------------------------------------
float UIEntity::GetOrientation() const
{
	return m_orientation;
}

//-----------------------------------------------------------------------------------------------
Rgba8 UIEntity::GetColor() const
{
	return m_color;
}

//-----------------------------------------------------------------------------------------------
Rgba8 UIEntity::GetBackgroundColor() const
{
	return m_backgroundColor;
}

//-----------------------------------------------------------------------------------------------
bool UIEntity::GetIsHidden() const
{
	return m_isHidden;
}

//-----------------------------------------------------------------------------------------------
IntVec2 UIEntity::GetWidgetOrder() const
{
	return m_widgetOrder;
}

//-----------------------------------------------------------------------------------------------
bool UIEntity::GetIsFocusable() const
{
	return m_isFocusable;
}

//-----------------------------------------------------------------------------------------------
Vec2 UIEntity::GetPivot() const
{
	return m_pivot;
}

//-----------------------------------------------------------------------------------------------
void UIEntity::SetParentWidget(Widget* parentWidget)
{
	m_parentWidget = parentWidget;
}

//-----------------------------------------------------------------------------------------------
void UIEntity::SetBounds(AABB2 const& bounds)
{
	m_bounds = bounds;
}

//-----------------------------------------------------------------------------------------------
void UIEntity::SetPivot(Vec2 const& pivot)
{
	m_pivot = pivot;
}

//-----------------------------------------------------------------------------------------------
void UIEntity::SetWidgetAlignment(Vec2 const& widgetAlignment)
{
	m_widgetAlignment = widgetAlignment;
}

//-----------------------------------------------------------------------------------------------
void UIEntity::SetLocalAlignment(Vec2 const& localAlignment)
{
	m_localAlignment = localAlignment;
}

//-----------------------------------------------------------------------------------------------
void UIEntity::SetScale(Vec2 const& scale)
{
	m_scale = scale;
}

//-----------------------------------------------------------------------------------------------
void UIEntity::SetOrientation(float const& orientation)
{
	m_orientation = orientation;
}

//-----------------------------------------------------------------------------------------------
void UIEntity::SetColor(Rgba8 const& color)
{
	m_color = color;
}

//-----------------------------------------------------------------------------------------------
void UIEntity::SetBackgroundColor(Rgba8 const& color)
{
	m_backgroundColor = color;
}

//-----------------------------------------------------------------------------------------------
void UIEntity::SetIsHidden(bool const& isHidden)
{
	m_isHidden = isHidden;
}

//-----------------------------------------------------------------------------------------------
void UIEntity::SetWidgetOrder(IntVec2 const& order)
{
	m_widgetOrder = order;
}

//-----------------------------------------------------------------------------------------------
void UIEntity::SetIsFocusable(bool const& isFocusable)
{
	m_isFocusable = isFocusable;
}
