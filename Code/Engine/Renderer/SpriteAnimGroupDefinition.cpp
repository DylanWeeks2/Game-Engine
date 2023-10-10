#include "SpriteAnimDefinition.hpp"
#include "SpriteSheet.hpp"
#include "SpriteDefinition.hpp"
#include "Engine/Math/MathUtils.hpp"
#include "SpriteAnimGroupDefinition.hpp"

//-----------------------------------------------------------------------------------------------
SpriteAnimGroupDefinition::SpriteAnimGroupDefinition(XmlElement& actorDefElement, SpriteSheet* spriteSheet)
{
	m_name = ParseXmlAttribute(actorDefElement, "name", "");
	m_scaledBySpeed = ParseXmlAttribute(actorDefElement, "scaleBySpeed", false);
	m_secondsPerFrame = ParseXmlAttribute(actorDefElement, "secondsPerFrame", 1.0f);
	std::string playbackModeName = ParseXmlAttribute(actorDefElement, "playbackMode", "Once");
	if (playbackModeName == "Once" || playbackModeName == "ONCE")
	{
		m_playbackType = SpriteAnimPlaybackType::ONCE;
	}
	else if (playbackModeName == "Loop" || playbackModeName == "LOOP")
	{
		m_playbackType = SpriteAnimPlaybackType::LOOP;
	}
	else if (playbackModeName == "PingPong" || playbackModeName == "PINGPONG")
	{
		m_playbackType = SpriteAnimPlaybackType::PINGPONG;
	}

	XmlElement* directionDefElement = actorDefElement.FirstChildElement("Direction");
	while (directionDefElement)
	{
		SingleDirectionAnimation* newSingleAnim = new SingleDirectionAnimation();
		XmlElement* animDefElement = directionDefElement->FirstChildElement("Animation");
		newSingleAnim->m_direction = ParseXmlAttribute(*directionDefElement, "vector", Vec3(1, 0, 0)).GetNormalized();
		SpriteAnimDefinition* newAnimationDef = new SpriteAnimDefinition(*spriteSheet, ParseXmlAttribute(*animDefElement, "startFrame", 0), ParseXmlAttribute(*animDefElement, "endFrame", 0), m_secondsPerFrame, m_playbackType);
		newSingleAnim->m_definition = newAnimationDef;
		m_animaitons.push_back(newSingleAnim);
		directionDefElement = directionDefElement->NextSiblingElement();
	}
}

//-----------------------------------------------------------------------------------------------
SpriteAnimGroupDefinition::~SpriteAnimGroupDefinition()
{
}
