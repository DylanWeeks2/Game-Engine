#pragma once
#include "Engine/Math/AABB2.hpp"
#include "Engine/Core/Rgba8.hpp"
#include "Engine/Math/Vec2.hpp"
#include "Engine/Math/IntVec2.hpp"

//-----------------------------------------------------------------------------------------------
class Widget;

//-----------------------------------------------------------------------------------------------
class UIEntity
{
public:
	UIEntity(Widget* parentWidget, AABB2 const& bounds, Vec2 pivot = Vec2(), Vec2 widgetAlignment = Vec2(), Vec2 localAlignment = Vec2(0.5f, 0.5f),
		Vec2 scale = Vec2(1.0f, 1.0f), float orientation = 0.0f, bool isFocusable = false, IntVec2 widgetOrder = IntVec2(), Rgba8 color = Rgba8::WHITE, 
		Rgba8 backgroundColor = Rgba8(255, 255, 255, 0), Rgba8 borderColor = Rgba8::BLACK, float borderTicknessPercent = 0.0f);
	virtual ~UIEntity();

	virtual void Update() = 0;
	virtual void Render() const = 0;

	//Getters
	Widget* GetParentWidget() const;
	AABB2	GetBounds() const;
	Vec2	GetWidgetAlignment() const;
	Vec2	GetLocalAlignment() const;
	Vec2	GetScale() const;
	float	GetOrientation() const;
	Rgba8	GetColor() const;
	Rgba8	GetBackgroundColor() const;
	bool	GetIsHidden() const;
	IntVec2	GetWidgetOrder() const;
	bool	GetIsFocusable() const;
	Vec2	GetPivot() const;

	//Setters
	void	SetParentWidget(Widget* parentWidget);
	void	SetBounds(AABB2 const& bounds);
	void	SetPivot(Vec2 const& pivot);
	void	SetWidgetAlignment(Vec2 const& widgetAlignment);
	void	SetLocalAlignment(Vec2 const& localAlignment);
	void	SetScale(Vec2 const& scale);
	void	SetOrientation(float const& orientation);
	void	SetColor(Rgba8 const& color);
	void	SetBackgroundColor(Rgba8 const& color);
	void	SetIsHidden(bool const& isHidden);
	void	SetWidgetOrder(IntVec2 const& order);
	void	SetIsFocusable(bool const& isFocusable);

protected:
	Widget* m_parentWidget;
	AABB2	m_bounds;
	Vec2	m_pivot;
	Vec2	m_widgetAlignment;
	Vec2	m_localAlignment;
	Vec2	m_scale;
	float	m_orientation;
	IntVec2 m_widgetOrder;
	float	m_borderThickness = 0.0f;
	Rgba8	m_color;
	Rgba8	m_backgroundColor;
	Rgba8	m_borderColor;
	bool	m_isHidden = false;
	bool	m_isFocusable = false;
};