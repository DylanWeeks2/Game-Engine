#pragma once

struct IntVec2
{
public:
	int x = 0;
	int y = 0;

public:
	~IntVec2() {}
	IntVec2() {}
	IntVec2(const IntVec2& copyFrom);
	explicit IntVec2(int initialX, int initialY);

	void SetFromText(char const* text);

	//Accessors (const methods)
	float GetLength() const;
	int GetTaxicabLength() const;
	int GetLengthSquared() const;
	float GetOrientationRadians() const;
	float GetOrientationDegrees() const;
	IntVec2 const GetRotated90Degrees() const;
	IntVec2 const GetRotatedMinus90Degrees() const;

	//mutators (non-const methods)
	void Rotate90Degrees();
	void RotateMinus90Degrees();

	//operators (self-mutating / non-const)
	void operator=(const IntVec2& copyFrom);
	bool operator<(const IntVec2& compareTo) const;
	bool operator==(const IntVec2& checkValue);
	const IntVec2	operator+(const IntVec2& vecToAdd) const;
	const IntVec2	operator-(const IntVec2& vecToSubtract) const;
};
