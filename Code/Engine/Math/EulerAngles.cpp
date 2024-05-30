#include "EulerAngles.hpp"
#include "Vec3.hpp"
#include "Mat44.hpp"
#include "MathUtils.hpp"
#include "Engine/Core/StringUtils.hpp"

//-----------------------------------------------------------------------------------------------
EulerAngles::EulerAngles(float yawDegrees, float pitchDegrees, float rollDegrees)
{
	m_yawDegrees = yawDegrees;
	m_pitchDegrees = pitchDegrees;
	m_rollDegrees = rollDegrees;
}

//-----------------------------------------------------------------------------------------------
EulerAngles::EulerAngles(Mat44 matrix)
{
	Vec3 iBasis = matrix.GetIBasis3D();
	Vec3 jBasis = matrix.GetJBasis3D();
	Vec3 kBasis = matrix.GetKBasis3D();

	m_yawDegrees = Atan2Degrees(iBasis.y, iBasis.x);
	if (m_yawDegrees < 0.0f)
	{
		m_yawDegrees = 360.0f + m_yawDegrees;
	}
	m_pitchDegrees = ConvertRadiansToDegrees(asinf(-iBasis.z));
	m_rollDegrees = Atan2Degrees(jBasis.z, kBasis.z);
}

//-----------------------------------------------------------------------------------------------
void EulerAngles::GetAsVectors_XFwd_YLeft_ZUp(Vec3& out_forwardIBasis, Vec3& out_leftJBasis, Vec3& out_upKBasis) const
{
	//Easy code
	/*Mat44 returnMat;
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
Mat44 EulerAngles::GetAsMatrix_XFwd_YLeft_ZUp() const
{
	Mat44 returnMat;
	
	//Easy to write code
	/*returnMat.AppendZRotation(m_yawDegrees);
	returnMat.AppendYRotation(m_pitchDegrees);
	returnMat.AppendXRotation(m_rollDegrees);*/

	//Optimized code
	Vec3 out_forwardIBasis; Vec3 out_leftJBasis; Vec3 out_upKBasis;
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
void EulerAngles::SetFromText(char const* text)
{
	Strings stringEulerAngles;
	stringEulerAngles = SplitStringOnDelimiter(text, ',');

	m_yawDegrees = static_cast<float>(atof(stringEulerAngles[0].c_str()));
	m_pitchDegrees = static_cast<float>(atof(stringEulerAngles[1].c_str()));
	m_rollDegrees = static_cast<float>(atof(stringEulerAngles[2].c_str()));
}

//-----------------------------------------------------------------------------------------------
Vec3 EulerAngles::GetForwardDirection_XFwd_YLeft_ZUp()
{
	Vec3 out_forwardIBasis;
	out_forwardIBasis.x = CosDegrees(m_yawDegrees) * CosDegrees(m_pitchDegrees);
	out_forwardIBasis.y = SinDegrees(m_yawDegrees) * CosDegrees(m_pitchDegrees);
	out_forwardIBasis.z = -SinDegrees(m_pitchDegrees);

	return out_forwardIBasis;
}