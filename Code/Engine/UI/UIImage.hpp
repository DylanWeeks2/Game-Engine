#pragma once
#include "UIEntity.hpp"

//-----------------------------------------------------------------------------------------------
class Texture;
class Widget;

//-----------------------------------------------------------------------------------------------
class UIImage: public UIEntity
{
public:
	UIImage(Texture* texture, Widget* parentWidget, AABB2 const& bounds, Vec2 pivot = Vec2(), Vec2 widgetAlignment = Vec2(),
		Vec2 localAlignment = Vec2(), Vec2 scale = Vec2(), float orientation = 0.0f, bool isFocusable = false, 
		IntVec2 widgetOrder = IntVec2(), Rgba8 color = Rgba8::WHITE, Rgba8 backgroundColor = Rgba8(255, 255, 255, 0),
		Rgba8 borderColor = Rgba8::BLACK, float borderTickness = 0.0f);
	~UIImage();

	virtual void Update() override;
	virtual void Render() const override;

	//Getters
	Texture*	GetTexture() const;

	//Setters
	void		SetTexture(Texture* texture);

private:
	Texture*	m_texture;
};