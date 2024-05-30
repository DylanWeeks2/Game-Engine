#include "DPEulerAngles.hpp"
#include "DPVec3.hpp"
#include "DPMat44.hpp"
#include "MathUtils.hpp"
#include "Engine/Core/StringUtils.hpp"

//-----------------------------------------------------------------------------------------------
DPEulerAngles::DPEulerAngles(double yawDegrees, double pitchDegrees, double rollDegrees)
{
	m_yawDegrees = yawDegrees;
	m_pitchDegrees = pitchDegrees;
	m_rollDegrees = rollDegrees;
}

//-----------------------------------------------------------------------------------------------
DPEulerAngles::DPEulerAngles(DPMat44 matrix)
{
	DPVec3 iBasis = matrix.GetIBasis3D();
	DPVec3 jBasis = matrix.GetJBasis3D();
	DPVec3 kBasis = matrix.GetKBasis3D();

	m_yawDegrees = Atan2Degrees(iBasis.y, iBasis.x);
	if (m_yawDegrees < 0.0f)
	{
		m_yawDegrees = 360.0f + m_yawDegrees;
	}
	m_pitchDegrees =  -1.0f * Atan2Degrees(jBasis.z, kBasis.z);
}

//-----------------------------------------------------------------------------------------------
void DPEulerAngles::GetAsVectors_XFwd_YLeft_ZUp(DPVec3& out_forwardIBasis, DPVec3& out_leftJBasis, DPVec3& out_upKBasis) const
{
	//Easy code
	/*DPMat44 returnMat;
	returnMat.AppendZRotation(m_yawDegrees);
	returnMat.AppendYRotation(m_pitchDegrees);
	returnMat.AppendXRotation(m_rollDegrees);

	out_forwardIBasis = returnMat.GetIBasis3D();
	out_leftJBasis = returnMat.GetJBasis3D();
	out_upKBasis = returnMat.GetKBasis3D();*/

	//Optimized code
	out_forwardIBasis.x = CosDegrees(m_yawDegrees) * CosDegrees(m_pitchDegrees);
	out_forwardIBasis.y = SinDegrees(m_yawDegrees) * CosDegrees(m_pitchDegrees);
	out_forwardIBasis.z = -SinDegrees(m_pitchDegrees);

	out_leftJBasis.x = (-SinDegrees(m_yawDegrees) * CosDegrees(m_rollDegrees)) + (CosDegrees(m_yawDegrees) * SinDegrees(m_pitchDegrees) * SinDegrees(m_rollDegrees));
	out_leftJBasis.y = (CosDegrees(m_yawDegrees) * CosDegrees(m_rollDegrees)) + (SinDegrees(m_yawDegrees) * SinDegrees(m_pitchDegrees) * SinDegrees(m_rollDegrees));
	out_leftJBasis.z = CosDegrees(m_pitchDegrees) * SinDegrees(m_rollDegrees);

	out_upKBasis.x = (SinDegrees(m_yawDegrees) * SinDegrees(m_rollDegrees)) + (CosDegrees(m_yawDegrees) * SinDegrees(m_pitchDegrees) * CosDegrees(m_rollDegrees));
	out_upKBasis.y = (CosDegrees(m_yawDegrees) * -SinDegrees(m_rollDegrees)) + (SinDegrees(m_yawDegrees) * SinDegrees(m_pitchDegrees) * CosDegrees(m_rollDegrees));
	out_upKBasis.z = CosDegrees(m_pitchDegrees) * CosDegrees(m_rollDegrees);
}

//-----------------------------------------------------------------------------------------------
DPMat44 DPEulerAngles::GetAsMatrix_XFwd_YLeft_ZUp() const
{
	DPMat44 returnMat;
	
	//Easy to write code
	/*returnMat.AppendZRotation(m_yawDegrees);
	returnMat.AppendYRotation(m_pitchDegrees);
	returnMat.AppendXRotation(m_rollDegrees);*/

	//Optimized code
	DPVec3 out_forwardIBasis; DPVec3 out_leftJBasis; DPVec3 out_upKBasis;
	out_forwardIBasis.x = CosDegrees(m_yawDegrees) * CosDegrees(m_pitchDegrees);
	out_forwardIBasis.y = SinDegrees(m_yawDegrees) * CosDegrees(m_pitchDegrees);
	out_forwardIBasis.z = -SinDegrees(m_pitchDegrees);

	out_leftJBasis.x = (-SinDegrees(m_yawDegrees) * CosDegrees(m_rollDegrees)) + (CosDegrees(m_yawDegrees) * SinDegrees(m_pitchDegrees) * SinDegrees(m_rollDegrees));
	out_leftJBasis.y = (CosDegrees(m_yawDegrees) * CosDegrees(m_rollDegrees)) + (SinDegrees(m_yawDegrees) * SinDegrees(m_pitchDegrees) * SinDegrees(m_rollDegrees));
	out_leftJBasis.z = CosDegrees(m_pitchDegrees) * SinDegrees(m_rollDegrees);

	out_upKBasis.x = (SinDegrees(m_yawDegrees) * SinDegrees(m_rollDegrees)) + (CosDegrees(m_yawDegrees) * SinDegrees(m_pitchDegrees) * CosDegrees(m_rollDegrees));
	out_upKBasis.y = (CosDegrees(m_yawDegrees) * -SinDegrees(m_rollDegrees)) + (SinDegrees(m_yawDegrees) * SinDegrees(m_pitchDegrees) * CosDegrees(m_rollDegrees));
	out_upKBasis.z = CosDegrees(m_pitchDegrees) * CosDegrees(m_rollDegrees);
	returnMat.SetIJK3D(out_forwardIBasis, out_leftJBasis, out_upKBasis);

	return returnMat;
}

//-----------------------------------------------------------------------------------------------
void DPEulerAngles::SetFromText(char const* text)
{
	Strings stringEulerAngles;
	stringEulerAngles = SplitStringOnDelimiter(text, ',');

	m_yawDegrees = static_cast<double>(atof(stringEulerAngles[0].c_str()));
	m_pitchDegrees = static_cast<double>(atof(stringEulerAngles[1].c_str()));
	m_rollDegrees = static_cast<double>(atof(stringEulerAngles[2].c_str()));
}

//-----------------------------------------------------------------------------------------------
DPVec3 DPEulerAngles::GetForwardDirection_XFwd_YLeft_ZUp()
{
	DPVec3 out_forwardIBasis;
	out_forwardIBasis.x = CosDegrees(m_yawDegrees) * CosDegrees(m_pitchDegrees);
	out_forwardIBasis.y = SinDegrees(m_yawDegrees) * CosDegrees(m_pitchDegrees);
	out_forwardIBasis.z = -SinDegrees(m_pitchDegrees);

	return out_forwardIBasis;
}