#pragma once
#include "Renderer.hpp"
#include "SpriteSheet.hpp"
#include "Engine/Core/Rgba8.hpp"
#include <string>

//-----------------------------------------------------------------------------------------------
struct Vec2;

//-----------------------------------------------------------------------------------------------
enum TextBoxMode
{
	SHRINK_TO_FIT,
	OVERRUN,
	NUM_TEXT_BOX_MODES = 2
};

//------------------------------------------------------------------------------------------------
class BitmapFont
{
	friend class Renderer; // Only the Renderer can create new BitmapFont objects!

private:
	BitmapFont(char const* fontFilePathNameWithNoExtension, Texture& fontTexture);

public:
	Texture& GetTexture();

	void AddVertsForText2D(std::vector<Vertex_PCU>& vertexArray, Vec2 const& textMins,
		float cellHeight, std::string const& text, Rgba8 const& tint = Rgba8(255, 255, 255, 255), float cellAspect = 1.0f);
	void AddVertsForText3D(std::vector<Vertex_PCU>& vertexArray, Vec2 const& textMins,
		float cellHeight, std::string const& text, Rgba8 const& tint = Rgba8(255, 255, 255, 255), float cellAspect = 1.0f,
		Vec2 const& alignment = Vec2(.5f, .5f), int maxGlyphsToDraw = 99999999);
	void AddVertsForTextInBox2D(std::vector<Vertex_PCU>& vertexArray, AABB2 const& box, float cellHeight,
		std::string const& text, Rgba8 const& tint = Rgba8(255, 255, 255, 255), float cellAspect = 1.0f,
		Vec2 const& alignment = Vec2(.5f, .5f), TextBoxMode mode = TextBoxMode::SHRINK_TO_FIT, int maxGlyphsToDraw = 99999999);

	float GetTextWidth(float cellHeight, std::string const& text, float cellAspect = 1.f);

protected:
	float GetGlyphAspect(int glyphUnicode) const; // For now this will always return 1.0f!!!

protected:
	std::string	m_fontFilePathNameWithNoExtension;
	SpriteSheet	m_fontGlyphsSpriteSheet;
};
