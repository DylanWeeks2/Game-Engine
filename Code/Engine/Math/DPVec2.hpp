#pragma once

//-----------------------------------------------------------------------------------------------
struct Vec2;

//-----------------------------------------------------------------------------------------------
struct DPVec2
{
public: // NOTE: this is one of the few cases where we break both the "m_" naming rule AND the avoid-public-members rule
	double x = 0.f;
	double y = 0.f;

public:
	// Construction/Destruction
	~DPVec2() {}												// destructor (do nothing)
	DPVec2() {}												// default constructor (do nothing)
	DPVec2( const DPVec2& copyFrom );							// copy constructor (from another DPVec2)
	explicit DPVec2( double initialX, double initialY );		// explicit constructor (from x, y)
	explicit DPVec2(Vec2 vec);

	void				SetFromText(char const* text);

	//Static methods (e.g. creation functions)
	static DPVec2 const MakeFromPolarRadians(double orientationRadians, double length = 1.f);
	static DPVec2 const MakeFromPolarDegrees(double orientationDegrees, double length = 1.f);

	//Accessors (const methods)
	double				GetLength() const;
	double				GetLengthSquared() const;
	double				GetOrientationRadians() const;
	double				GetOrientationDegrees() const;
	DPVec2 const		GetRotated90Degrees() const;
	DPVec2 const		GetRotatedMinus90Degrees() const;
	DPVec2 const		GetRotatedRadians(double deltaRadians) const;
	DPVec2 const		GetRotatedDegrees(double deltaDegrees) const;
	DPVec2 const		GetClamped(double maxLength) const;
	DPVec2 const		GetNormalized() const;
	DPVec2 const		GetReflected(DPVec2 const& impactSurfaceNormal) const;

	//Mutators (non-const methods)
	void				SetOrientationRadians(double newOrientationRadians);
	void				SetOrientationDegrees(double newOrientationDegrees);
	void				SetPolarRadians(double newOrientationRadians, double newLength);
	void				SetPolarDegrees(double newOrientaitonDegrees, double newLength);
	void				Rotate90Degrees();
	void				RotateMinus90Degrees();
	void				RotateRadians(double deltaRadians);
	void				RotateDegrees(double deltaDegrees);
	void				SetLength(double maxLength);
	void				ClampLength(double maxLength);
	void				Normalize();
	double				NormalizeAndGetPreviousLength();
	void				Reflect(DPVec2 const& impactSurfaceNormal);

	// Operators (const)
	bool				operator==(const DPVec2& compare) const;		// DPVec2 == DPVec2
	bool				operator!=(const DPVec2& compare) const;		// DPVec2 != DPVec2
	const DPVec2		operator+( const DPVec2& vecToAdd ) const;		// DPVec2 + DPVec2
	const DPVec2		operator-( const DPVec2& vecToSubtract ) const;	// DPVec2 - DPVec2
	const DPVec2		operator-() const;								// -DPVec2, i.e. "unary negation"
	const DPVec2		operator*( double uniformScale ) const;			// DPVec2 * double
	const DPVec2		operator*( const DPVec2& vecToMultiply ) const;	// DPVec2 * DPVec2
	const DPVec2		operator/( double inverseScale ) const;			// DPVec2 / double

	// Operators (self-mutating / non-const)
	void				operator+=( const DPVec2& vecToAdd );				// DPVec2 += DPVec2
	void				operator-=( const DPVec2& vecToSubtract );		// DPVec2 -= DPVec2
	void				operator*=( const double uniformScale );			// DPVec2 *= double
	void				operator/=( const double uniformDivisor );		// DPVec2 /= double
	void				operator=( const DPVec2& copyFrom );				// DPVec2 = DPVec2

	// Standalone "friend" functions that are conceptually, but not actually, part of DPVec2::
	friend const DPVec2 operator*( double uniformScale, const DPVec2& vecToScale );	// double * DPVec2
};


