#pragma once

//-----------------------------------------------------------------------------------------------
class Clock;

//-----------------------------------------------------------------------------------------------
class Stopwatch
{
public:
	explicit Stopwatch(float duration);
	Stopwatch(const Clock* clock, float duration);

	void			Start();
	void			Restart();
	void			Stop();
	float			GetElapsedTime() const;
	float			GetElapsedFraction() const;
	bool			IsStopped() const;
	bool			HasDuraitonElapsed() const;
	bool			DecrementDurationIfElapsed();

public:
	const Clock*	m_clock = nullptr;
	float			m_startTime = -1.0f;
	float			m_duration = 0.0f;
};