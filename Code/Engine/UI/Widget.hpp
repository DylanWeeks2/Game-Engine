#pragma once
#include "Engine/Math/AABB2.hpp"

//-----------------------------------------------------------------------------------------------
class	UIEntity;
class	Renderer;
struct	IntVec2;

//-----------------------------------------------------------------------------------------------
class Widget
{
public:
	Widget(Renderer* renderer, AABB2 bounds);
	~Widget();

	void Update();
	void Render() const;

	//Getters
	bool		GetIsHidden() const;
	AABB2		GetBounds() const;
	Renderer*	GetRenderer() const;
	UIEntity*	GetFocusedEntity() const;
	bool		GetHasKeyboardFocus() const;
	bool		GetHasMouseFocus() const;
	bool		GetHasControllerFocus() const;
	Vec2		GetCurrentMousePosition() const;
	UIEntity*	GetNextFocusableEntity(IntVec2 const& targetWidgetOrder);
	int			GetTotalNumberOfUIEntities();

	//Setters
	void		SetIsHidden(bool const& isHidden);
	void		SetBounds(AABB2 const& bounds);
	void		SetFocusedEntity(UIEntity* entity);
	void		SetHasKeyboardFocus(bool const& focus);
	void		SetHasMouseFocus(bool const& focus);
	void		SetHasControllerFocus(bool const& focus);
	void		SetUIEntityVisible(int index, bool visible);

	//Adding UI Entities
	void		AddUIEntity(UIEntity* entityToAdd);

protected:
	Renderer*				m_renderer;
	std::vector<UIEntity*>	m_UIEntities;
	UIEntity*				m_focusedEntity = nullptr;
	AABB2					m_bounds;
	Vec2					m_previousMousePosition = Vec2();
	Vec2					m_currentMousePosition = Vec2();
	bool					m_isHidden = false;
	bool					m_hasKeyboardFocus = false;
	bool					m_hasMouseFocus = false;
	bool					m_hasControllerFocus = false;
};