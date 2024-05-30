#pragma once
#include "Engine/Math/Vec3.hpp"
#include "Engine/Renderer/SpriteAnimDefinition.hpp"
#include "Engine/Core/XmlUtils.hpp"
#include <string>
#include <vector>

//------------------------------------------------------------------------------------------------
class SpriteAnimGroupDefinition
{
public:
	SpriteAnimGroupDefinition();
	SpriteAnimGroupDefinition(XmlElement& actorDefElement, SpriteSheet* spriteSheet);
	~SpriteAnimGroupDefinition();

	struct SingleDirectionAnimation
	{
		Vec3					m_direction;
		SpriteAnimDefinition*	m_definition;
	};

public:
	std::string									m_name = "";
	std::vector<SingleDirectionAnimation*>		m_animaitons;
	float										m_secondsPerFrame = 0.0f;
	SpriteAnimPlaybackType						m_playbackType = SpriteAnimPlaybackType::ONCE;
	bool										m_scaledBySpeed = false;
};
