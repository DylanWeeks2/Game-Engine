#pragma once

struct IntRange
{
public:
	int m_min = 0;
	int m_max = 0;

	static const IntRange ZERO;
	static const IntRange ONE;
	static const IntRange ZERO_TO_ONE;

public:
	IntRange() = default;
	~IntRange();
	explicit IntRange(int min, int max);

	bool operator==(IntRange const& compare) const; //vec3==vec3
	bool operator!=(IntRange const& compare) const; //vec3!=vec3
	void operator=(IntRange const& copyFrom);
	bool IsOnRange(int value);
	bool IsOverlappingWith(IntRange const& compare);

};