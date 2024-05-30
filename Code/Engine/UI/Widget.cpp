#include "Widget.hpp"
#include "UIEntity.hpp"
#include "Engine/Input/InputSystem.hpp"
#include "Engine/Math/IntVec2.hpp"
#include "Engine/Renderer/Renderer.hpp"

//-----------------------------------------------------------------------------------------------
Widget::Widget(Renderer* renderer, AABB2 bounds)
	:m_renderer(renderer)
	,m_bounds(bounds)
{
}

//-----------------------------------------------------------------------------------------------
Widget::~Widget()
{
	for (int entityIndex = 0; entityIndex < m_UIEntities.size(); entityIndex++)
	{
		UIEntity* entity = m_UIEntities[entityIndex];
		if (entity)
		{
			delete entity;
			entity = nullptr;
		}
	}
}

//-----------------------------------------------------------------------------------------------
void Widget::Update()
{
	//Update Mouse
	m_previousMousePosition = m_currentMousePosition;
	m_currentMousePosition = Vec2(m_bounds.GetPointAtUV(g_theInput->GetCursorNormalizedPosition()));
	if (m_currentMousePosition != m_previousMousePosition)
	{
		m_hasMouseFocus = true;
	}
	
	//Update Entities
	for (int entityIndex = 0; entityIndex < m_UIEntities.size(); entityIndex++)
	{
		UIEntity* entity = m_UIEntities[entityIndex];
		if (entity)
		{
			entity->Update();
		}
	}
}

//-----------------------------------------------------------------------------------------------
void Widget::Render() const
{
	m_renderer->SetModelConstants();
	for (int entityIndex = 0; entityIndex < m_UIEntities.size(); entityIndex++)
	{
		UIEntity const* entity = m_UIEntities[entityIndex];
		if (entity && entity->GetIsHidden() == false)
		{
			entity->Render();
		}
	}
}

//-----------------------------------------------------------------------------------------------
bool Widget::GetIsHidden() const
{
	return m_isHidden;
}

//-----------------------------------------------------------------------------------------------
AABB2 Widget::GetBounds() const
{
	return m_bounds;
}

//-----------------------------------------------------------------------------------------------
Renderer* Widget::GetRenderer() const
{
	return m_renderer;
}

//-----------------------------------------------------------------------------------------------
UIEntity* Widget::GetFocusedEntity() const
{
	return m_focusedEntity;
}

//-----------------------------------------------------------------------------------------------
bool Widget::GetHasKeyboardFocus() const
{
	return m_hasKeyboardFocus;
}

//-----------------------------------------------------------------------------------------------
bool Widget::GetHasMouseFocus() const
{
	return m_hasMouseFocus;
}

//-----------------------------------------------------------------------------------------------
bool Widget::GetHasControllerFocus() const
{
	return m_hasControllerFocus;
}

//-----------------------------------------------------------------------------------------------
Vec2 Widget::GetCurrentMousePosition() const
{
	return m_currentMousePosition;
}

//-----------------------------------------------------------------------------------------------
UIEntity* Widget::GetNextFocusableEntity(IntVec2 const& targetWidgetOrder)
{
	for (int entityIndex = 0; entityIndex < m_UIEntities.size(); entityIndex++)
	{
		UIEntity* entity = m_UIEntities[entityIndex];
		if (entity)
		{
			if (entity->GetIsFocusable())
			{
				if (entity->GetWidgetOrder() == targetWidgetOrder)
				{
					return entity;
				}
			}
		}
	}

	return nullptr;
}

//-----------------------------------------------------------------------------------------------
int Widget::GetTotalNumberOfUIEntities()
{
	return int(m_UIEntities.size());
}

//-----------------------------------------------------------------------------------------------
void Widget::SetIsHidden(bool const& isHidden)
{
	m_isHidden = isHidden;
}

//-----------------------------------------------------------------------------------------------
void Widget::SetBounds(AABB2 const& bounds)
{
	m_bounds = bounds;
}

//-----------------------------------------------------------------------------------------------
void Widget::SetFocusedEntity(UIEntity* entity)
{
	m_focusedEntity = entity;
}

//-----------------------------------------------------------------------------------------------
void Widget::SetHasKeyboardFocus(bool const& focus)
{
	m_hasKeyboardFocus = focus;
}

//-----------------------------------------------------------------------------------------------
void Widget::SetHasMouseFocus(bool const& focus)
{
	m_hasMouseFocus = focus;
}

//-----------------------------------------------------------------------------------------------
void Widget::SetHasControllerFocus(bool const& focus)
{
	m_hasControllerFocus = focus;
}

//-----------------------------------------------------------------------------------------------
void Widget::SetUIEntityVisible(int index, bool visible)
{
	m_UIEntities[index]->SetIsHidden(visible);
}

//-----------------------------------------------------------------------------------------------
void Widget::AddUIEntity(UIEntity* entityToAdd)
{
	m_UIEntities.push_back(entityToAdd);
}
