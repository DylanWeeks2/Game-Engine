#pragma once

//-----------------------------------------------------------------------------------------------
struct FloatRange
{
public:
	float					m_min = 0.0f;
	float					m_max = 0.0f;

	static const FloatRange ZERO;
	static const FloatRange ONE;
	static const FloatRange ZERO_TO_ONE;

public:
	FloatRange() = default;
	~FloatRange();
	explicit FloatRange(float min, float max);

	void SetFromText(char const* text);
	bool operator==(FloatRange const& compare) const; //vec3==vec3
	bool operator!=(FloatRange const& compare) const; //vec3!=vec3
	void operator=(FloatRange const& copyFrom);
	bool IsOnRange(float value);
	bool IsOverlappingWith(FloatRange const& compare);
};