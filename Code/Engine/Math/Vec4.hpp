#pragma once 

//-----------------------------------------------------------------------------------------------
struct Vec4
{
public: // NOTE: this is one of the few cases where we break both the "m_" naming rule AND the avoid-public-members rule
	float x = 0.0f;
	float y = 0.0f;
	float z = 0.0f;
	float w = 0.0f;

public:
	Vec4() = default;
	explicit Vec4(float initialX, float initialY, float initialZ, float initialW);

	Vec4 const operator-(Vec4 const& vecToSubtract) const; //vec3 - vec3

	void operator*=(float uniformScale); //vec3*float
};
