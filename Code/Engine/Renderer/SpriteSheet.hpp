#pragma once
#include "Engine/Math/AABB2.hpp"
#include "SpriteDefinition.hpp"
#include <vector>

//-----------------------------------------------------------------------------------------------
class	Texture;
struct	Vec2;
struct	IntVec2;
class	SpriteDefinition;

//------------------------------------------------------------------------------------------------
class SpriteSheet
{
public:
	explicit SpriteSheet(Texture& texture, IntVec2 const& simpleGridLayout);
	explicit SpriteSheet(Texture& texture, int& gridPosition);

	Texture&						GetTexture() const;
	int								GetNumSprites() const;
	SpriteDefinition const&			GetSpriteDef(int spriteIndex) const;
	void							GetSpriteUVs(Vec2& out_uvAtMins, Vec2& out_uvAtMaxs, int spriteIndex) const;
	AABB2							GetSpriteUVs(int spriteIndex) const;

protected:
	Texture&						m_texture;
	std::vector<SpriteDefinition>	m_spriteDefs;
};