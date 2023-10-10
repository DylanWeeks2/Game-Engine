#include "SpriteAnimDefinition.hpp"
#include "SpriteSheet.hpp"
#include "SpriteDefinition.hpp"
#include "Engine/Math/MathUtils.hpp"

//-----------------------------------------------------------------------------------------------
SpriteAnimDefinition::SpriteAnimDefinition(SpriteSheet const& sheet, int startSpriteIndex, int endSpriteIndex, float framesPerSecond, SpriteAnimPlaybackType playbackType)
	:m_spriteSheet(sheet)
	,m_startSpriteIndex(startSpriteIndex)
	,m_endSpriteIndex(endSpriteIndex)
	,m_secondsPerFrame(1.0f / framesPerSecond)
	,m_playbackType(playbackType)
{
	if (m_secondsPerFrame >= 1.0f)
	{
		m_secondsPerFrame = framesPerSecond;
	}
}

//-----------------------------------------------------------------------------------------------
SpriteDefinition const& SpriteAnimDefinition::GetSpriteDefAtTime(float seconds) const
{
	int frameIndex = static_cast<int>(seconds / m_secondsPerFrame);
	int currentIndex = frameIndex;

	if (m_playbackType == SpriteAnimPlaybackType::ONCE)
	{
		currentIndex = m_startSpriteIndex + frameIndex;
		if (currentIndex < m_startSpriteIndex)
		{
			return m_spriteSheet.GetSpriteDef(m_startSpriteIndex);
		}
		else if (currentIndex > m_endSpriteIndex)
		{
			return m_spriteSheet.GetSpriteDef(m_endSpriteIndex);
		}

		return m_spriteSheet.GetSpriteDef(currentIndex);
	}
	else if (m_playbackType == SpriteAnimPlaybackType::LOOP)
	{
		currentIndex = currentIndex % (m_endSpriteIndex - m_startSpriteIndex + 1);
		currentIndex += m_startSpriteIndex;
		return m_spriteSheet.GetSpriteDef(currentIndex);
	}
	else
	{
		int loopCount = currentIndex / (m_endSpriteIndex - m_startSpriteIndex);
		currentIndex = currentIndex % (m_endSpriteIndex - m_startSpriteIndex);

		if (loopCount % 2 == 1)
		{
			currentIndex = m_endSpriteIndex - currentIndex;
		}
		else
		{
			currentIndex = m_startSpriteIndex + currentIndex;
		}

		return m_spriteSheet.GetSpriteDef(currentIndex);
	}
}

//-----------------------------------------------------------------------------------------------
void SpriteAnimDefinition::SetFromText(char const* text)
{
	text;
}
