#include "SpriteSheet.hpp"
#include "Engine/Math/IntVec2.hpp"
#include "SpriteDefinition.hpp"

//-----------------------------------------------------------------------------------------------
SpriteSheet::SpriteSheet(Texture& texture, IntVec2 const& simpleGridLayout)
	:m_texture(texture)
{
	float uPerSpriteX = 1.0f / (float)simpleGridLayout.x;
	float vPerSpriteY = 1.0f / (float)simpleGridLayout.y;
	int spriteIndex = 0;
	float uNudgeValue = 1.0f / ((float)simpleGridLayout.x * 128);
	float vNudgeValue = 1.0f / ((float)simpleGridLayout.y * 128);
	for (int spriteY = 0; spriteY < simpleGridLayout.y; spriteY++)
	{
		for (int spriteX = 0; spriteX < simpleGridLayout.x; spriteX++)
		{
			float minU = uPerSpriteX * (float) spriteX;
			float maxU = minU + uPerSpriteX;
			float maxV = 1.0f - (vPerSpriteY * (float)spriteY);
			float minV = maxV - vPerSpriteY;

			minU += uNudgeValue;
			minV += vNudgeValue;
			maxU -= uNudgeValue;
			maxV -= vNudgeValue;

			m_spriteDefs.push_back(SpriteDefinition(*this, spriteIndex, Vec2(minU, minV), Vec2(maxU, maxV)));
			spriteIndex++;
		}
	}
}

//-----------------------------------------------------------------------------------------------
Texture& SpriteSheet::GetTexture() const
{
	return m_texture;
}

//-----------------------------------------------------------------------------------------------
int SpriteSheet::GetNumSprites() const
{
	return static_cast<int>(m_spriteDefs.size());
}

//-----------------------------------------------------------------------------------------------
SpriteDefinition const& SpriteSheet::GetSpriteDef(int spriteIndex) const
{
	return m_spriteDefs[spriteIndex];
}

//-----------------------------------------------------------------------------------------------
void SpriteSheet::GetSpriteUVs(Vec2& out_uvAtMins, Vec2& out_uvAtMaxs, int spriteIndex) const
{
	m_spriteDefs[spriteIndex].GetUVs(out_uvAtMins, out_uvAtMaxs);
}

//-----------------------------------------------------------------------------------------------
AABB2 SpriteSheet::GetSpriteUVs(int spriteIndex) const
{
	return m_spriteDefs[spriteIndex].GetUVs();
}
