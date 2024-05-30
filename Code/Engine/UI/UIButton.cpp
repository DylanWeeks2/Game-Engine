#include "UIButton.hpp"
#include "Widget.hpp"
#include "Engine/Renderer/Texture.hpp"
#include "Engine/Core/Vertex_PCU.hpp"
#include "Engine/Core/VertexUtils.hpp"
#include "Engine/Core/EngineCommon.hpp"
#include "Engine/Renderer/Renderer.hpp"
#include "Engine/Renderer/BitmapFont.hpp"
#include "Engine/Math/MathUtils.hpp"

//-----------------------------------------------------------------------------------------------
UIButton::UIButton(BitmapFont* font, std::string onClickEventName, std::string text, float textHeight, Widget* parentWidget, AABB2 const& bounds,
	Vec2 pivot, Vec2 widgetAlignment, Vec2 localAlignment, Vec2 scale, float orientation, bool isFocusable, IntVec2 widgetOrder, Rgba8 color, Rgba8 backgroundColor, 
	Rgba8 borderColor, float borderTickness, Rgba8 focusTextColor, Rgba8 focusBackgroundColor, Rgba8 focusBorderColor, float focusBorderTickness,
	bool isKeyboardControlled)
	:UIEntity(parentWidget, bounds, pivot, widgetAlignment, localAlignment, scale, orientation, isFocusable, widgetOrder, color, backgroundColor, 
		borderColor, borderTickness)
	,m_font(font)
	,m_onClickEventName(onClickEventName)
	,m_text(text)
	,m_textHeight(textHeight)
	,m_focusTextColor(focusTextColor)
	,m_focusBackgroundColor(focusBackgroundColor)
	,m_focusBorderColor(focusBorderColor)
	,m_focusBorderThickness(focusBorderTickness)
	,m_isKeyboardControlled(isKeyboardControlled)
{
}

//-----------------------------------------------------------------------------------------------
UIButton::~UIButton()
{
}

//-----------------------------------------------------------------------------------------------
void UIButton::Update()
{
	if (m_isFocusable == false)
	{
		return;
	}

	if (m_parentWidget->GetHasMouseFocus())
	{
		if (IsPointInsideAABB2D(m_parentWidget->GetCurrentMousePosition(), m_bounds))
		{
			m_parentWidget->SetFocusedEntity(this);
			if (g_theInput->IsKeyDown(KEYCODE_LEFT_MOUSE) == true && g_theInput->WasKeyJustPressed(KEYCODE_LEFT_MOUSE) == false)
			{
				OnClick();
			}
		}
	}

	//Keyboard
	if (m_isKeyboardControlled)
	{
		if (g_theInput->IsKeyDown(KEYCODE_UPARROW) == true && g_theInput->WasKeyJustPressed(KEYCODE_UPARROW) == false)
		{
			m_parentWidget->SetHasMouseFocus(false);
			m_parentWidget->SetHasControllerFocus(false);
			m_parentWidget->SetHasKeyboardFocus(true);

			UIEntity* focusedEntity = m_parentWidget->GetNextFocusableEntity(IntVec2(m_widgetOrder.x + 1, m_widgetOrder.y));
			if (focusedEntity)
			{
				m_parentWidget->SetFocusedEntity(focusedEntity);
			}
		}
		else if (g_theInput->IsKeyDown(KEYCODE_DOWNARROW) == true && g_theInput->WasKeyJustPressed(KEYCODE_DOWNARROW) == false)
		{
			m_parentWidget->SetHasMouseFocus(false);
			m_parentWidget->SetHasControllerFocus(false);
			m_parentWidget->SetHasKeyboardFocus(true);

			UIEntity* focusedEntity = m_parentWidget->GetNextFocusableEntity(IntVec2(m_widgetOrder.x - 1, m_widgetOrder.y));
			if (focusedEntity)
			{
				m_parentWidget->SetFocusedEntity(focusedEntity);
			}
		}
		if (g_theInput->IsKeyDown(KEYCODE_RIGHTARROW) == true && g_theInput->WasKeyJustPressed(KEYCODE_RIGHTARROW) == false)
		{
			m_parentWidget->SetHasMouseFocus(false);
			m_parentWidget->SetHasControllerFocus(false);
			m_parentWidget->SetHasKeyboardFocus(true);

			UIEntity* focusedEntity = m_parentWidget->GetNextFocusableEntity(IntVec2(m_widgetOrder.x, m_widgetOrder.y + 1));
			if (focusedEntity)
			{
				m_parentWidget->SetFocusedEntity(focusedEntity);
			}
		}
		else if (g_theInput->IsKeyDown(KEYCODE_LEFTARROW) == true && g_theInput->WasKeyJustPressed(KEYCODE_LEFTARROW) == false)
		{
			m_parentWidget->SetHasMouseFocus(false);
			m_parentWidget->SetHasControllerFocus(false);
			m_parentWidget->SetHasKeyboardFocus(true);

			UIEntity* focusedEntity = m_parentWidget->GetNextFocusableEntity(IntVec2(m_widgetOrder.x, m_widgetOrder.y - 1));
			if (focusedEntity)
			{
				m_parentWidget->SetFocusedEntity(focusedEntity);
			}
		}

		//Click for Keyboard
		if (g_theInput->IsKeyDown(KEYCODE_ENTER) == true && g_theInput->WasKeyJustPressed(KEYCODE_ENTER) == false)
		{
			m_parentWidget->SetHasMouseFocus(false);
			m_parentWidget->SetHasControllerFocus(false);
			m_parentWidget->SetHasKeyboardFocus(true);
			if (m_parentWidget->GetFocusedEntity() == this)
			{
				OnClick();
			}
		}
	}
}

//-----------------------------------------------------------------------------------------------
void UIButton::Render() const
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
	AABB2 borderBoundsFocus;
	borderBoundsFocus.m_maxs = dimensions;
	borderBoundsFocus.SetCenter(Vec2());
	AABB2 noBorderBoundsFocus;
	noBorderBoundsFocus.m_maxs = Vec2(m_focusBorderThickness, m_focusBorderThickness);
	noBorderBoundsFocus.m_maxs = Vec2(dimensions.x - m_focusBorderThickness, dimensions.y - m_focusBorderThickness);
	noBorderBoundsFocus.SetCenter(Vec2());
	
	if (m_parentWidget->GetFocusedEntity() == this)
	{
		m_font->AddVertsForTextInBox2D(verts, noBorderBoundsFocus, m_textHeight, m_text, m_focusTextColor, 1.0f, m_localAlignment);
		AddVertsForAABB2D(backgroundVerts, borderBoundsFocus, m_focusBorderColor);
		AddVertsForAABB2D(backgroundVerts, noBorderBoundsFocus, m_focusBackgroundColor);
	}
	else
	{
		m_font->AddVertsForTextInBox2D(verts, noBorderBoundsNoFocus, m_textHeight, m_text, m_color, 1.0f, m_localAlignment);
		AddVertsForAABB2D(backgroundVerts, borderBoundsNoFocus, m_borderColor);
		AddVertsForAABB2D(backgroundVerts, noBorderBoundsNoFocus, m_backgroundColor);
	}

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
std::string UIButton::GetText() const
{
	return m_text;
}

//-----------------------------------------------------------------------------------------------
float UIButton::GetTextHeight() const
{
	return m_textHeight;
}

//-----------------------------------------------------------------------------------------------
BitmapFont* UIButton::GetFont() const
{
	return m_font;
}

//-----------------------------------------------------------------------------------------------
bool UIButton::GetFocus() const
{
	if (m_parentWidget->GetFocusedEntity() == this)
	{
		return true;
	}

	return false;
}

//-----------------------------------------------------------------------------------------------
Rgba8 UIButton::GetFocusTextColor() const
{
	return m_focusTextColor;
}

//-----------------------------------------------------------------------------------------------
Rgba8 UIButton::GetFocusBackgroundColor() const
{
	return m_focusBackgroundColor;
}

//-----------------------------------------------------------------------------------------------
std::string UIButton::GetEventOnClickName() const
{
	return m_onClickEventName;
}

//-----------------------------------------------------------------------------------------------
bool UIButton::GetIsKeyboardControlled() const
{
	return m_isKeyboardControlled;
}

//-----------------------------------------------------------------------------------------------
void UIButton::SetText(std::string const& text)
{
	m_text = text;
}

//-----------------------------------------------------------------------------------------------
void UIButton::SetTextHeight(float textHeight)
{
	m_textHeight = textHeight;
}

//-----------------------------------------------------------------------------------------------
void UIButton::SetFont(BitmapFont* font)
{
	m_font = font;
}

//-----------------------------------------------------------------------------------------------
void UIButton::SetFocusTextColor(Rgba8 const& textColor)
{
	m_focusTextColor = textColor;
}

//-----------------------------------------------------------------------------------------------
void UIButton::SetFocusBackgroundColor(Rgba8 const& backgroundColor)
{
	m_focusBackgroundColor = backgroundColor;
}

//-----------------------------------------------------------------------------------------------
void UIButton::SetEventOnClickName(std::string const& name)
{
	m_onClickEventName = name;
}

//-----------------------------------------------------------------------------------------------
void UIButton::SetIsKeyboardControlled(bool const& isKeyboardControlled)
{
	m_isKeyboardControlled = isKeyboardControlled;
}

//-----------------------------------------------------------------------------------------------
void UIButton::OnClick()
{
	g_theEventSystem->FireEvent(m_onClickEventName);
}
