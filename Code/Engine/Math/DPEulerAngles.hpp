#pragma once

//-----------------------------------------------------------------------------------------------
struct DPVec3;
struct DPMat44;

//-----------------------------------------------------------------------------------------------
struct DPEulerAngles
{
public:
	DPEulerAngles() = default;
	DPEulerAngles(double yawDegrees, double pitchDegrees, double rollDegrees);
	DPEulerAngles(DPMat44 matrix);
	void	GetAsVectors_XFwd_YLeft_ZUp(DPVec3& out_forwardIBasis, DPVec3& out_leftJBasis, DPVec3& out_upKBasis) const;
	DPVec3	GetForwardDirection_XFwd_YLeft_ZUp();
	DPMat44 GetAsMatrix_XFwd_YLeft_ZUp() const;
	void	SetFromText(char const* text);

public:
	double	m_yawDegrees = 0.0f;
	double	m_pitchDegrees = 0.0f;
	double	m_rollDegrees = 0.0f;
};