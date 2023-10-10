#include "Clock.hpp"
#include "Time.hpp"
#include "Engine/Core/ErrorWarningAssert.hpp"

static Clock g_systemClock;

//------------------------------------------------------------------------------------------------
Clock::Clock()
{
	if (this != &g_systemClock)
	{
		m_parent = &g_systemClock;
		g_systemClock.AddChild(this);
	}
}

//------------------------------------------------------------------------------------------------
Clock::Clock(Clock& parent)
{
	m_parent = &parent;
	m_parent->AddChild(this);
}

//------------------------------------------------------------------------------------------------
void Clock::Reset()
{
	m_totalSeconds = 0.0f;
	m_lastUpdateTimeSeconds = 0.0f;
	m_deltaSeconds = 0.0f;
	m_frameCount = 0;
}

//------------------------------------------------------------------------------------------------
bool Clock::IsPaused() const
{
	return m_isPaused;
}

//------------------------------------------------------------------------------------------------
void Clock::Pause()
{
	m_isPaused = true;
}

//------------------------------------------------------------------------------------------------
void Clock::Unpause()
{
	m_isPaused = false;
}

//------------------------------------------------------------------------------------------------
void Clock::TogglePause()
{
	if (!m_isPaused)
	{
		Pause();
	}
	else
	{
		Unpause();
	}
}

//------------------------------------------------------------------------------------------------
void Clock::StepSingleFrame()
{
	Unpause();
	Advance(m_deltaSeconds);
	m_stepSingleFrame = true;
}

//------------------------------------------------------------------------------------------------
void Clock::SetTimeScale(float timeScale)
{
	m_timeScale = timeScale;
}

//------------------------------------------------------------------------------------------------
float Clock::GetTimeScale() const
{
	return m_timeScale;
}

//------------------------------------------------------------------------------------------------
float Clock::GetDeltaSeconds() const
{
	return m_deltaSeconds;
}

//------------------------------------------------------------------------------------------------
float Clock::GetTotalSeconds() const
{
	return m_totalSeconds;
}

//------------------------------------------------------------------------------------------------
size_t Clock::GetFrameCount() const
{
	return m_frameCount;
}

//------------------------------------------------------------------------------------------------
Clock& Clock::GetSystemClock()
{
	return g_systemClock;
}

//------------------------------------------------------------------------------------------------
void Clock::TickSystemClock()
{
	g_systemClock.Tick();
}

//------------------------------------------------------------------------------------------------
void Clock::Tick()
{
	m_totalSeconds = static_cast<float>(GetCurrentTimeSeconds());
	//DebuggerPrintf("Current time = %f, lastTime = %f, DeltaSec = %f\n", GetCurrentTimeSeconds(), m_lastUpdateTimeSeconds, static_cast<float>(GetCurrentTimeSeconds()) - m_lastUpdateTimeSeconds);
	m_deltaSeconds = static_cast<float>(GetCurrentTimeSeconds()) - m_lastUpdateTimeSeconds;
	m_lastUpdateTimeSeconds = m_totalSeconds;
	Advance(m_deltaSeconds);
}

//------------------------------------------------------------------------------------------------
void Clock::Advance(float deltaSeconds)
{
	//Calculates delta time based on pause and time scale
	if (deltaSeconds > maxDeltaSeconds)
	{
		deltaSeconds = maxDeltaSeconds;
	}
	if (IsPaused())
	{
		deltaSeconds = 0.0f;
	}
	m_deltaSeconds = deltaSeconds;
	m_deltaSeconds *= m_timeScale;

	//updates book keeping variables
	m_lastUpdateTimeSeconds = m_totalSeconds;
	m_totalSeconds += m_deltaSeconds;
	m_frameCount++;

	if (GetTotalSeconds() == 0.0f)
	{
		deltaSeconds;
	}

	//Advances all child clocks
	for (int clockIndex = 0; clockIndex < m_children.size(); clockIndex++)
	{
		if (m_children[clockIndex] != nullptr)
		{
			m_children[clockIndex]->Advance(m_deltaSeconds);
		}
	}

	//Logic for single step pausing after one frame
	if (m_stepSingleFrame == true)
	{
		Pause();
		m_stepSingleFrame = false;
	}
}

//------------------------------------------------------------------------------------------------
void Clock::AddChild(Clock* childClock)
{
	m_children.push_back(childClock);
	childClock->m_parent = this;
}

//------------------------------------------------------------------------------------------------
void Clock::RemoveChild(Clock* childClock)
{
	for (int clockIndex = 0; clockIndex < m_children.size(); clockIndex++)
	{
		if (m_children[clockIndex] != nullptr)
		{
			if (m_children[clockIndex] == childClock)
			{
				m_children[clockIndex]->m_parent = nullptr;
				m_children[clockIndex] = nullptr;
				m_children.erase(m_children.begin() + clockIndex);
			}
		}
	}
}

//------------------------------------------------------------------------------------------------
Clock::~Clock()
{
	if (m_parent != nullptr)
	{
		m_parent->RemoveChild(this);
	}

	for (int clockIndex = 0; clockIndex < m_children.size(); clockIndex++)
	{
		if (m_children[clockIndex] != nullptr)
		{
			m_children[clockIndex] = nullptr;
			m_children.erase(m_children.begin() + clockIndex);
		}
	}

	m_parent = nullptr;
}
