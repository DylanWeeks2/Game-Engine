#pragma once 

//-----------------------------------------------------------------------------------------------
struct DPVec4
{
public: // NOTE: this is one of the few cases where we break both the "m_" naming rule AND the avoid-public-members rule
	double x = 0.0f;
	double y = 0.0f;
	double z = 0.0f;
	double w = 0.0f;

public:
	DPVec4() = default;
	explicit DPVec4(double initialX, double initialY, double initialZ, double initialW);

	DPVec4 const	operator-(DPVec4 const& vecToSubtract) const; //vec3 - vec3
	void			operator*=(double uniformScale); //vec3*double
};
