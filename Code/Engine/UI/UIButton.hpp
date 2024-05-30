#pragma once
#include "UIEntity.hpp"
#include <string>

//-----------------------------------------------------------------------------------------------
class Texture;
class Widget;
class BitmapFont;

//-----------------------------------------------------------------------------------------------
class UIButton: public UIEntity
{
public:
	UIButton(BitmapFont* font, std::string onClickEventName, std::string text, float textHeight, Widget* parentWidget, AABB2 const& bounds,
		Vec2 pivot = Vec2(), Vec2 widgetAlignment = Vec2(), Vec2 localAlignment = Vec2(), Vec2 scale = Vec2(), float orientation = 0.0f, bool isFocusable = false,
		IntVec2 widgetOrder = IntVec2(), Rgba8 color = Rgba8::WHITE, Rgba8 backgroundColor = Rgba8::BLACK, Rgba8 borderColor = Rgba8::BLACK, float borderTickness = 0.0f,
		Rgba8 focusTextColor = Rgba8::BLACK, Rgba8 focusBackgroundColor = Rgba8::WHITE, Rgba8 focusBorderColor = Rgba8::BLACK, float focusBorderThickness = 0.0f,
		bool isKeyboardControlled = true);
	~UIButton();

	virtual void Update() override;
	virtual void Render() const override;

	//Getters
	std::string GetText() const;
	float		GetTextHeight() const;
	BitmapFont*	GetFont() const;
	bool		GetFocus() const;
	Rgba8		GetFocusTextColor() const;
	Rgba8		GetFocusBackgroundColor() const;
	std::string GetEventOnClickName() const;
	bool		GetIsKeyboardControlled() const;

	//Setters
	void		SetText(std::string const& text);
	void		SetTextHeight(float textHeight);
	void		SetFont(BitmapFont* font);
	void		SetFocusTextColor(Rgba8 const& textColor);
	void		SetFocusBackgroundColor(Rgba8 const& backgroundColor);
	void		SetEventOnClickName(std::string const& name);
	void		SetIsKeyboardControlled(bool const& isKeyboardControlled);

private:
	void		OnClick();

private:
	std::string m_text;
	std::string m_onClickEventName;
	BitmapFont* m_font = nullptr;
	float		m_textHeight = 0.0f;
	float		m_focusBorderThickness = 0.0f;
	Rgba8		m_focusTextColor;
	Rgba8		m_focusBackgroundColor;
	Rgba8		m_focusBorderColor;
	bool		m_isKeyboardControlled = true;
};