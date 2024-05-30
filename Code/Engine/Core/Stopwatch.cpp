#include "Stopwatch.hpp"
#include "Clock.hpp"
#include "Engine/Core/ErrorWarningAssert.hpp"

//-----------------------------------------------------------------------------------------------
Stopwatch::Stopwatch(float duration)
{
	m_clock = &Clock::GetSystemClock();
	m_duration = duration;
}

//-----------------------------------------------------------------------------------------------
Stopwatch::Stopwatch(const Clock* clock, float duration)
{
	m_clock = clock;
	m_duration = duration;
}

//-----------------------------------------------------------------------------------------------
void Stopwatch::Start()
{
	m_startTime = m_clock->GetTotalSeconds();
}

//-----------------------------------------------------------------------------------------------
void Stopwatch::Restart()
{
	if (!IsStopped())
	{
		m_startTime = m_clock->GetTotalSeconds();
	}
}

//-----------------------------------------------------------------------------------------------
void Stopwatch::Stop()
{
	m_startTime = -1.0f;
}

//-----------------------------------------------------------------------------------------------
float Stopwatch::GetElapsedTime() const
{
	if (IsStopped())
	{
		return 0.0f;
	}
	else
	{
		return m_clock->GetTotalSeconds() - m_startTime;
	}
}

//-----------------------------------------------------------------------------------------------
float Stopwatch::GetElapsedFraction() const
{
	return GetElapsedTime() / m_duration;
}

//-----------------------------------------------------------------------------------------------
bool Stopwatch::IsStopped() const
{
	if (m_startTime == -1.0f)
	{
		return true;
	}
	else
	{
		return false;
	}
}

//-----------------------------------------------------------------------------------------------
bool Stopwatch::HasDuraitonElapsed() const
{
	if (!IsStopped())
	{
		if (GetElapsedTime() >= m_duration)
		{
			return true;
		}
		else
		{
			return false;
		}
	}
	else
	{
		return false;
	}
}

//-----------------------------------------------------------------------------------------------
bool Stopwatch::DecrementDurationIfElapsed()
{
	if (HasDuraitonElapsed())
	{
		m_startTime += m_duration;
		return true;
	}
	else
	{
		return false;
	}
}
