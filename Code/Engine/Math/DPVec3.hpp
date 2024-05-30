#pragma once 
#include "Vec3.hpp"

//-----------------------------------------------------------------------------------------------
struct Vec3;

//-----------------------------------------------------------------------------------------------
struct DPVec3
{
public: // NOTE: this is one of the few cases where we break both the "m_" naming rule AND the avoid-public-members rule
	double x = 0.0f;
	double y = 0.0f;
	double z = 0.0f;

public:
	DPVec3() = default;
	DPVec3(Vec3 vec);
	explicit DPVec3(double initialX, double initialY, double initialZ);
	
	void				SetFromText(char const* text);

	//Accessors (const methods)
	double				GetLength() const;
	double				GetLengthXY() const;
	double				GetLengthSquared() const;
	double				GetLengthXYSquared() const;
	double				GetAngleAboutZRadians() const;
	double				GetAngleAboutZDegrees() const;
	DPVec3 const		GetRotatedAboutZRadians(double deltaRadians) const;
	DPVec3 const		GetRotatedAboutZDegrees(double deltaDegrees) const;
	DPVec3 const		GetClamped(double maxLength) const;
	DPVec3 const		GetNormalized() const;

	void				SetLength(double maxLength);

	//Operators (const)
	bool				operator==(DPVec3 const& compare) const; //DPVec3==DPVec3
	bool				operator!=(DPVec3 const& compare) const; //DPVec3!=DPVec3
	DPVec3 const		operator+(DPVec3 const& vecToAdd) const; //DPVec3+DPVec3
	DPVec3 const		operator-(DPVec3 const& vecToSubtract) const; //DPVec3 - DPVec3
	DPVec3 const		operator*(double uniformScale) const; //DPVec3*double
	DPVec3 const		operator/(double inverseScale) const; //DPVec3/double

	//Operator (self-mutating / non-const)
	void				operator+=(DPVec3 const& vecToAdd); //DPVec3 += DPVec3
	void				operator-=(DPVec3 const& vecToSubtract); //DPVec3-= DPVec3
	void				operator*=(double uniformScale); //DPVec3*double
	void				operator/=(double uniformDivisor); // DPVec3/double
	void				operator=(DPVec3 const& copyFrom); // DPVec3 = DPVec3
	DPVec3				operator*(const DPVec3& vecToMultiply);

	void				Normalize();
	void				ClampLength(double maxLength);

	//Standalone "friend" funcitons that are conceptually, but not actually, part of DPVec3::
	friend DPVec3 const operator*(double uniformScale, DPVec3 const& vecToScale); //double * DPVec3

	//Static methods (e.g. creation functions)
	static DPVec3 const MakeFromPolarRadians(double longitudeRadians, double latitudeRadians, double length = 1.f);
	static DPVec3 const MakeFromPolarDegrees(double longitudeDegrees, double latitudeDegrees, double length = 1.f);
};


