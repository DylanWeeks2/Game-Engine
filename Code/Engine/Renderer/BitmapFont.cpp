#include "BitmapFont.hpp"
#include "Texture.hpp"
#include "Engine/Core/VertexUtils.hpp"
#include "Engine/Core/StringUtils.hpp"

//-----------------------------------------------------------------------------------------------
BitmapFont::BitmapFont(char const* fontFilePathNameWithNoExtension, Texture& fontTexture)
	:m_fontFilePathNameWithNoExtension(fontFilePathNameWithNoExtension)
	,m_fontGlyphsSpriteSheet(fontTexture, IntVec2(16, 16))
{
}

//-----------------------------------------------------------------------------------------------
Texture& BitmapFont::GetTexture()
{
	return m_fontGlyphsSpriteSheet.GetTexture();
}

//-----------------------------------------------------------------------------------------------
void BitmapFont::AddVertsForText2D(std::vector<Vertex_PCU>& vertexArray, Vec2 const& textMins, float cellHeight, std::string const& text, Rgba8 const& tint, float cellAspect)
{
	Vec2 cellMins = textMins;
	Vec2 uvAtMins, uvAtMaxs;
	AABB2 glyph;
	for (int textIndex = 0; textIndex < text.length(); textIndex++)
	{
		float cellWidth = cellHeight * GetGlyphAspect(1) * cellAspect;
		glyph.m_mins = cellMins;
		glyph.m_maxs = Vec2(cellMins.x + cellWidth, cellMins.y + cellHeight);
		int spriteIndex = text[textIndex];
		const SpriteDefinition& tileSprite = m_fontGlyphsSpriteSheet.GetSpriteDef(spriteIndex);
		tileSprite.GetUVs(uvAtMins, uvAtMaxs);

		AddVertsForAABB2D(vertexArray, glyph, tint, uvAtMins, uvAtMaxs);
		cellMins = Vec2(glyph.m_maxs.x, cellMins.y);
	}
}

//-----------------------------------------------------------------------------------------------
void BitmapFont::AddVertsForText3D(std::vector<Vertex_PCU>& vertexArray, Vec2 const& textMins, float cellHeight, std::string const& text, Rgba8 const& tint, float cellAspect, Vec2 const& alignment, int maxGlyphsToDraw)
{
	std::string glyphText = text;
	if (glyphText.length() > maxGlyphsToDraw)
	{
		glyphText = glyphText.substr(0, maxGlyphsToDraw);
	}
	AddVertsForText2D(vertexArray, Vec2(alignment.x * GetTextWidth(cellHeight, glyphText, cellAspect) * -1.0f, alignment.y * cellHeight * -1.0f) + textMins, cellHeight, glyphText, tint, cellAspect);
}

//-----------------------------------------------------------------------------------------------
void BitmapFont::AddVertsForTextInBox2D(std::vector<Vertex_PCU>& vertexArray, AABB2 const& box, float cellHeight, std::string const& text, Rgba8 const& tint, float cellAspect, Vec2 const& alignment, TextBoxMode mode, int maxGlyphsToDraw)
{
	Vec2 textMins;
	float textWidth = 0.0;
	float textHeight = cellHeight;
	Strings splitString = SplitStringOnDelimiter(text, '\n');
	float totalTextHeight = splitString.size() * textHeight;
	int glyphIndex = 0;

	if (mode == TextBoxMode::OVERRUN)
	{
		for (int textIndex = 0; textIndex < splitString.size(); textIndex++)
		{
			if (glyphIndex > maxGlyphsToDraw)
			{
				return;
			}

			textWidth = GetTextWidth(cellHeight, splitString[textIndex], cellAspect);
			textMins.x = ((box.GetDimensions().x - textWidth) * alignment.x) + box.m_mins.x;
			textMins.y = (((box.GetDimensions().y - totalTextHeight) * alignment.y) + box.m_mins.y) + (textHeight * (splitString.size() - (textIndex + 1)));

			if (splitString[textIndex].length() > maxGlyphsToDraw - glyphIndex)
			{
				splitString[textIndex] = splitString[textIndex].substr(0, maxGlyphsToDraw - glyphIndex);
			}

			glyphIndex += static_cast<int>(splitString[textIndex].length());
			AddVertsForText2D(vertexArray, textMins, cellHeight, splitString[textIndex], tint, cellAspect);
		}
	}
	else
	{
		float maxUnalterdTextWidth = 0.0f;
		float maxAlteredTextHeight = cellHeight;

		for (int textIndex = 0; textIndex < splitString.size(); textIndex++)
		{
			float currentTextWidth = GetTextWidth(cellHeight, splitString[textIndex], cellAspect);
			float oldTextWidth = currentTextWidth;
			if (currentTextWidth > box.GetDimensions().x && currentTextWidth > maxUnalterdTextWidth)
			{
				maxUnalterdTextWidth = currentTextWidth;
				textWidth = box.GetDimensions().x;
				maxAlteredTextHeight = textHeight * (textWidth / oldTextWidth);
				totalTextHeight = splitString.size() * textHeight;
			}
		}

		textHeight = maxAlteredTextHeight;

		for (int textIndex = 0; textIndex < splitString.size(); textIndex++)
		{
			if (glyphIndex > maxGlyphsToDraw)
			{
				return;
			}

			textWidth = GetTextWidth(textHeight, splitString[textIndex], cellAspect);
			textMins.x = ((box.GetDimensions().x - textWidth) * alignment.x) + box.m_mins.x;
			textMins.y = (((box.GetDimensions().y - totalTextHeight) * alignment.y) + box.m_mins.y) + (textHeight * (splitString.size() - (textIndex + 1)));

			if (splitString[textIndex].length() > maxGlyphsToDraw - glyphIndex)
			{
				splitString[textIndex] = splitString[textIndex].substr(0, maxGlyphsToDraw - glyphIndex);
			}

			glyphIndex += static_cast<int>(splitString[textIndex].length());
			AddVertsForText2D(vertexArray, textMins, textHeight, splitString[textIndex], tint, cellAspect);
		}
	}	
}

//-----------------------------------------------------------------------------------------------
float BitmapFont::GetTextWidth(float cellHeight, std::string const& text, float cellAspect)
{
	float textWidth = 0.0f;

	for (int textIndex = 0; textIndex < text.length(); textIndex++)
	{
		float cellWidth = cellHeight * GetGlyphAspect(1) * cellAspect;
		textWidth += cellWidth;
	}

	return textWidth;
}

//-----------------------------------------------------------------------------------------------
float BitmapFont::GetGlyphAspect(int glyphUnicode) const
{
	glyphUnicode;
	return 1.0f;
}
