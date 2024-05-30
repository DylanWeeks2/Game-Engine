#pragma once

//-----------------------------------------------------------------------------------------------
struct IntVec3
{
public:
	int x = 0;
	int y = 0;
	int z = 0;

public:
	~IntVec3() {}
	IntVec3() {}
	IntVec3(const IntVec3& copyFrom);
	explicit IntVec3(int initialX, int initialY, int initialZ);

	void			SetFromText(char const* text);

	//Accessors (const methods)
	float			GetLength() const;
	int				GetTaxicabLength() const;
	int				GetLengthSquared() const;

	//operators (self-mutating / non-const)
	void			operator=(const IntVec3& copyFrom);
	bool			operator==(const IntVec3& compare) const;
	bool			operator!=(const IntVec3& compare) const;
	const IntVec3	operator+(const IntVec3& intVecToAdd) const;
	const IntVec3	operator-(const IntVec3& intVecToAdd) const;
	bool			operator<(const IntVec3& compareTo) const;
};
