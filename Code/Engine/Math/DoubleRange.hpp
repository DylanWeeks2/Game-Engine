#pragma once

//-----------------------------------------------------------------------------------------------
struct DoubleRange
{
public:
	double						m_min = 0.0f;
	double						m_max = 0.0f;

	static const DoubleRange	ZERO;
	static const DoubleRange	ONE;
	static const DoubleRange	ZERO_TO_ONE;

public:
	DoubleRange() = default;
	~DoubleRange();
	explicit DoubleRange(double min, double max);

	void SetFromText(char const* text);

	bool operator==(DoubleRange const& compare) const; //vec3==vec3
	bool operator!=(DoubleRange const& compare) const; //vec3!=vec3
	void operator=(DoubleRange const& copyFrom);
	bool IsOnRange(double value);
	bool IsOverlappingWith(DoubleRange const& compare);
};