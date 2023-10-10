#include "SpriteDefinition.hpp"
#include "SpriteSheet.hpp"

//------------------------------------------------------------------------------------------------
SpriteDefinition::SpriteDefinition(SpriteSheet const& spriteSheet, int spriteIndex, Vec2 const& uvAtMins, Vec2 const& uvAtMaxs)
	:m_spriteSheet(spriteSheet)
	,m_spriteIndex(spriteIndex)
	,m_uvAtMins(uvAtMins)
	,m_uvAtMaxs(uvAtMaxs)
{
}

//------------------------------------------------------------------------------------------------
void SpriteDefinition::GetUVs(Vec2& out_uvAtMins, Vec2& out_uvAtMaxs) const
{
	out_uvAtMins = m_uvAtMins;
	out_uvAtMaxs = m_uvAtMaxs;
}

//------------------------------------------------------------------------------------------------
AABB2 SpriteDefinition::GetUVs() const
{
	AABB2 bounds;
	bounds.m_mins = m_uvAtMins;
	bounds.m_maxs = m_uvAtMaxs;
	return bounds;
}

//------------------------------------------------------------------------------------------------
SpriteSheet const& SpriteDefinition::GetSpriteSheet() const
{
	return m_spriteSheet;
}

//------------------------------------------------------------------------------------------------
Texture& SpriteDefinition::GetTexture() const
{
	return m_spriteSheet.GetTexture();
}

//------------------------------------------------------------------------------------------------
float SpriteDefinition::GetAspect() const
{
	return 0.0f;
}
