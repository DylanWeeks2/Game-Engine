#include "AnalogJoystick.hpp"
#include "Engine/Math/MathUtils.hpp"
#include "Engine/Math/Vec2.hpp"

Vec2 AnalogJoystick::GetPosition() const
{
	return m_correctedPosition;
}

float AnalogJoystick::GetMagnitude() const
{
	return m_correctedPosition.GetLength();
}

float AnalogJoystick::GetOrientationDegrees() const
{
	return m_correctedPosition.GetOrientationDegrees();
}

Vec2 AnalogJoystick::GetRawUncorrectedPosition() const
{
	return m_rawPosition;
}

float AnalogJoystick::GetInnerDeadZoneFraction() const
{
	return m_innerDeadZoneFraction;
}

float AnalogJoystick::GetOuterDeadZoneFraction() const
{
	return m_outerDeadZoneFraction;
}

void AnalogJoystick::Reset()
{
}

void AnalogJoystick::SetDeadZoneThresholds(float normalizedInnerDeadzoneThreshold, float normalizedOuterDeadzoneThreshold)
{
	m_innerDeadZoneFraction = normalizedInnerDeadzoneThreshold;
	m_outerDeadZoneFraction = normalizedOuterDeadzoneThreshold;
}

void AnalogJoystick::UpdatePosition(float rawNormalizedX, float rawNormalizedY)
{
	//sets raw positions
	m_rawPosition.x = rawNormalizedX;
	m_rawPosition.y = rawNormalizedY;

	//converts to polar raw
	float rawR = m_rawPosition.GetLength();
	float theta = m_rawPosition.GetOrientationDegrees();

	//range maps down to 0 to 1
	float correctR = RangeMapClamped(rawR, GetInnerDeadZoneFraction(), GetOuterDeadZoneFraction(), 0.f, 1.f);

	//converts back to cartesan
	m_correctedPosition.x = correctR * CosDegrees(theta);
	m_correctedPosition.y = correctR * SinDegrees(theta);
}
