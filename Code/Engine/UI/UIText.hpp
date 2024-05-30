#pragma once
#include "UIEntity.hpp"
#include <string>

//-----------------------------------------------------------------------------------------------
class Texture;
class Widget;
class BitmapFont;

//-----------------------------------------------------------------------------------------------
class UIText: public UIEntity
{
public:
	UIText(BitmapFont* font, std::string text, float textHeight, Widget* parentWidget, AABB2 const& bounds, Vec2 pivot = Vec2(), Vec2 widgetAlignment = Vec2(),
		Vec2 localAlignment = Vec2(), Vec2 scale = Vec2(), float orientation = 0.0f, bool isFocusable = false, IntVec2 widgetOrder = IntVec2(), 
		Rgba8 color = Rgba8::WHITE, Rgba8 backgroundColor = Rgba8(255, 255, 255, 0), Rgba8 borderColor = Rgba8(0, 0, 0, 0), float borderTickness = 0.0f);
	~UIText();

	virtual void Update() override;
	virtual void Render() const override;

	//Getters
	std::string GetText() const;
	float		GetTextHeight() const;
	BitmapFont* GetFont() const;

	//Setters
	void		SetText(std::string const& text);
	void		SetTextHeight(float textHeight);
	void		SetFont(BitmapFont* font);

private:
	std::string m_text;
	BitmapFont* m_font = nullptr;
	float		m_textHeight = 0.0f;
};