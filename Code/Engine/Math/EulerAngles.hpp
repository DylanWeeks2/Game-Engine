#pragma once

//-----------------------------------------------------------------------------------------------
struct Vec3;
struct Mat44;

//-----------------------------------------------------------------------------------------------
struct EulerAngles
{
public:
	EulerAngles() = default;
	EulerAngles(float yawDegrees, float pitchDegrees, float rollDegrees);
	EulerAngles(Mat44 matrix);
	void	GetAsVectors_XFwd_YLeft_ZUp(Vec3& out_forwardIBasis, Vec3& out_leftJBasis, Vec3& out_upKBasis) const;
	Vec3	GetForwardDirection_XFwd_YLeft_ZUp();
	Mat44	GetAsMatrix_XFwd_YLeft_ZUp() const;
	void	SetFromText(char const* text);

public:
	float	m_yawDegrees = 0.0f;
	float	m_pitchDegrees = 0.0f;
	float	m_rollDegrees = 0.0f;
};