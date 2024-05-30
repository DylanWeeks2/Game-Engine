#include "DPMat44.hpp"
#include "DPVec2.hpp"
#include "DPVec3.hpp"
#include "DPVec4.hpp"
#include "MathUtils.hpp"
#include "EulerAngles.hpp"
#include "Engine/Core/StringUtils.hpp"

//-----------------------------------------------------------------------------------------------
DPMat44::DPMat44()
{
	//Identity Matrix
	m_values[Ix] = 1.0f;
	m_values[Iy] = 0.0f;
	m_values[Iz] = 0.0f;
	m_values[Iw] = 0.0f;

	m_values[Jx] = 0.0f;
	m_values[Jy] = 1.0f;
	m_values[Jz] = 0.0f;
	m_values[Jw] = 0.0f;

	m_values[Kx] = 0.0f;
	m_values[Ky] = 0.0f;
	m_values[Kz] = 1.0f;
	m_values[Kw] = 0.0f;

	m_values[Tx] = 0.0f;
	m_values[Ty] = 0.0f;
	m_values[Tz] = 0.0f;
	m_values[Tw] = 1.0f;
}

//-----------------------------------------------------------------------------------------------
DPMat44::DPMat44(DPVec2 const& iBasis2D, DPVec2 const& jBasis2D, DPVec2 const& tranlation2D)
{
	m_values[Ix] = iBasis2D.x;
	m_values[Iy] = iBasis2D.y;
	m_values[Iz] = 0.0f;
	m_values[Iw] = 0.0f;

	m_values[Jx] = jBasis2D.x;
	m_values[Jy] = jBasis2D.y;
	m_values[Jz] = 0.0f;
	m_values[Jw] = 0.0f;

	m_values[Kx] = 0.0f;
	m_values[Ky] = 0.0f;
	m_values[Kz] = 1.0f;
	m_values[Kw] = 0.0f;

	m_values[Tx] = tranlation2D.x;
	m_values[Ty] = tranlation2D.y;
	m_values[Tz] = 0.0f;
	m_values[Tw] = 1.0f;
}

//-----------------------------------------------------------------------------------------------
DPMat44::DPMat44(DPVec3 const& iBasis3D, DPVec3 const& jBasis3D, DPVec3 const& kBasis3D, DPVec3 const& translation3D)
{
	m_values[Ix] = iBasis3D.x;
	m_values[Iy] = iBasis3D.y;
	m_values[Iz] = iBasis3D.z;
	m_values[Iw] = 0.0f;

	m_values[Jx] = jBasis3D.x;
	m_values[Jy] = jBasis3D.y;
	m_values[Jz] = jBasis3D.z;
	m_values[Jw] = 0.0f;

	m_values[Kx] = kBasis3D.x;
	m_values[Ky] = kBasis3D.y;
	m_values[Kz] = kBasis3D.z;
	m_values[Kw] = 0.0f;

	m_values[Tx] = translation3D.x;
	m_values[Ty] = translation3D.y;
	m_values[Tz] = translation3D.z;
	m_values[Tw] = 1.0f;
}

//-----------------------------------------------------------------------------------------------
DPMat44::DPMat44(DPVec4 const& iBasis4D, DPVec4 const& jBasis4D, DPVec4 const& kBasis4D, DPVec4 const& translation4D)
{
	m_values[Ix] = iBasis4D.x;
	m_values[Iy] = iBasis4D.y;
	m_values[Iz] = iBasis4D.z;
	m_values[Iw] = iBasis4D.w;

	m_values[Jx] = jBasis4D.x;
	m_values[Jy] = jBasis4D.y;
	m_values[Jz] = jBasis4D.z;
	m_values[Jw] = jBasis4D.w;

	m_values[Kx] = kBasis4D.x;
	m_values[Ky] = kBasis4D.y;
	m_values[Kz] = kBasis4D.z;
	m_values[Kw] = kBasis4D.w;

	m_values[Tx] = translation4D.x;
	m_values[Ty] = translation4D.y;
	m_values[Tz] = translation4D.z;
	m_values[Tw] = translation4D.w;
}

//-----------------------------------------------------------------------------------------------
DPMat44::DPMat44(double const* sixteenValeusBasisMajor)
{
	m_values[Ix] = sixteenValeusBasisMajor[Ix];
	m_values[Iy] = sixteenValeusBasisMajor[Iy];
	m_values[Iz] = sixteenValeusBasisMajor[Iz];
	m_values[Iw] = sixteenValeusBasisMajor[Iw];

	m_values[Jx] = sixteenValeusBasisMajor[Jx];
	m_values[Jy] = sixteenValeusBasisMajor[Jy];
	m_values[Jz] = sixteenValeusBasisMajor[Jz];
	m_values[Jw] = sixteenValeusBasisMajor[Jw];

	m_values[Kx] = sixteenValeusBasisMajor[Kx];
	m_values[Ky] = sixteenValeusBasisMajor[Ky];
	m_values[Kz] = sixteenValeusBasisMajor[Kz];
	m_values[Kw] = sixteenValeusBasisMajor[Kw];

	m_values[Tx] = sixteenValeusBasisMajor[Tx];
	m_values[Ty] = sixteenValeusBasisMajor[Ty];
	m_values[Tz] = sixteenValeusBasisMajor[Tz];
	m_values[Tw] = sixteenValeusBasisMajor[Tw];
}

//-----------------------------------------------------------------------------------------------
DPMat44 const DPMat44::CreateTranslation2D(DPVec2 const& translationXY)
{
	DPMat44 translation;
	translation.m_values[Tx] = translationXY.x;
	translation.m_values[Ty] = translationXY.y;

	return translation;
}

//-----------------------------------------------------------------------------------------------
DPMat44 const DPMat44::CreateTranslation3D(DPVec3 const& translationXYZ)
{
	DPMat44 translation;
	translation.m_values[Tx] = translationXYZ.x;
	translation.m_values[Ty] = translationXYZ.y;
	translation.m_values[Tz] = translationXYZ.z;

	return translation;
}

//-----------------------------------------------------------------------------------------------
DPMat44 const DPMat44::CreateUniformScale2D(double const& uniformScaleXY)
{
	DPMat44 scaledMatrix;
	scaledMatrix.m_values[Ix] *= uniformScaleXY;
	scaledMatrix.m_values[Iy] *= uniformScaleXY;
	scaledMatrix.m_values[Jx] *= uniformScaleXY;
	scaledMatrix.m_values[Jy] *= uniformScaleXY;

	return scaledMatrix;
}

//-----------------------------------------------------------------------------------------------
DPMat44 const DPMat44::CreateUniformScale3D(double const& uniformScaleXYZ)
{
	DPMat44 scaledMatrix;
	scaledMatrix.m_values[Ix] *= uniformScaleXYZ;
	scaledMatrix.m_values[Iy] *= uniformScaleXYZ;
	scaledMatrix.m_values[Iz] *= uniformScaleXYZ;
	scaledMatrix.m_values[Jx] *= uniformScaleXYZ;
	scaledMatrix.m_values[Jy] *= uniformScaleXYZ;
	scaledMatrix.m_values[Jz] *= uniformScaleXYZ;
	scaledMatrix.m_values[Kx] *= uniformScaleXYZ;
	scaledMatrix.m_values[Ky] *= uniformScaleXYZ;
	scaledMatrix.m_values[Kz] *= uniformScaleXYZ;

	return scaledMatrix;
}

//-----------------------------------------------------------------------------------------------
DPMat44 const DPMat44::CreateNonUniformScale2D(DPVec2 const& nonUniformScaleXY)
{
	DPMat44 scaledMatrix;
	scaledMatrix.m_values[Ix] *= nonUniformScaleXY.x;
	scaledMatrix.m_values[Iy] *= nonUniformScaleXY.y;
	scaledMatrix.m_values[Jx] *= nonUniformScaleXY.x;
	scaledMatrix.m_values[Jy] *= nonUniformScaleXY.y;

	return scaledMatrix;
}

//-----------------------------------------------------------------------------------------------
DPMat44 const DPMat44::CreateNonUniformScale3D(DPVec3 const& nonUniformScaleXYZ)
{
	DPMat44 scaledMatrix;
	scaledMatrix.m_values[Ix] *= nonUniformScaleXYZ.x;
	scaledMatrix.m_values[Iy] *= nonUniformScaleXYZ.y;
	scaledMatrix.m_values[Iz] *= nonUniformScaleXYZ.z;
	scaledMatrix.m_values[Jx] *= nonUniformScaleXYZ.x;
	scaledMatrix.m_values[Jy] *= nonUniformScaleXYZ.y;
	scaledMatrix.m_values[Jz] *= nonUniformScaleXYZ.z;
	scaledMatrix.m_values[Kx] *= nonUniformScaleXYZ.x;
	scaledMatrix.m_values[Ky] *= nonUniformScaleXYZ.y;
	scaledMatrix.m_values[Kz] *= nonUniformScaleXYZ.z;

	return scaledMatrix;
}

//-----------------------------------------------------------------------------------------------
DPMat44 const DPMat44::CreateZRotationDegrees(double rotationDegreesAboutZ)
{
	DPMat44 rotZ;
	double c = CosDegrees(rotationDegreesAboutZ);
	double s = SinDegrees(rotationDegreesAboutZ);
	rotZ.m_values[Ix] = c;
	rotZ.m_values[Iy] = s;
	rotZ.m_values[Jx] = -s;
	rotZ.m_values[Jy] = c;

	return rotZ;
}

//-----------------------------------------------------------------------------------------------
DPMat44 const DPMat44::CreateYRotationDegrees(double rotationDegreesAboutY)
{
	DPMat44 rotZ;
	double c = CosDegrees(rotationDegreesAboutY);
	double s = SinDegrees(rotationDegreesAboutY);
	rotZ.m_values[Ix] = c;
	rotZ.m_values[Iz] = -s;
	rotZ.m_values[Kx] = s;
	rotZ.m_values[Kz] = c;

	return rotZ;
}

//-----------------------------------------------------------------------------------------------
DPMat44 const DPMat44::CreateXRotationDegrees(double rotationDegreesAboutX)
{
	DPMat44 rotZ;
	double c = CosDegrees(rotationDegreesAboutX);
	double s = SinDegrees(rotationDegreesAboutX);
	rotZ.m_values[Jy] = c;
	rotZ.m_values[Jz] = s;
	rotZ.m_values[Ky] = -s;
	rotZ.m_values[Kz] = c;

	return rotZ;
}

//-----------------------------------------------------------------------------------------------
DPMat44 const DPMat44::CreateOrthoProjection(double left, double right, double bottom, double top, double zNear, double zFar)
{
	DPMat44 orthoProjection;
	orthoProjection.m_values[Ix] = 2.0f / (right - left);
	orthoProjection.m_values[Jy] = 2.0f / (top - bottom);
	orthoProjection.m_values[Kz] = 1.0f / (zFar - zNear);
	orthoProjection.m_values[Tx] = (left + right) / (left - right);
	orthoProjection.m_values[Ty] = (bottom + top) / (bottom - top);
	orthoProjection.m_values[Tz] = zNear / (zNear - zFar);
	return orthoProjection;
}

//-----------------------------------------------------------------------------------------------
DPMat44 const DPMat44::CreatePerspectiveProjection(double fovYDegrees, double aspect, double zNear, double zFar)
{
	DPMat44 prospectiveProjection;
	prospectiveProjection.m_values[Ix] = (1.0f / TanDegrees(fovYDegrees / 2.0f)) / aspect;
	prospectiveProjection.m_values[Jy] = 1.0f / TanDegrees(fovYDegrees / 2.0f);
	prospectiveProjection.m_values[Kz] = zFar / (zFar - zNear);
	prospectiveProjection.m_values[Tz] = -(zNear * zFar) / (zFar - zNear);
	prospectiveProjection.m_values[Kw] = 1.0f;
	prospectiveProjection.m_values[Tw] = 0.0f;
	return prospectiveProjection;
}

//-----------------------------------------------------------------------------------------------
DPVec2 const DPMat44::TransformVectorQuantity2D(DPVec2 const& vectorQuantityXY) const
{
	DPVec2 returnVector;
	returnVector.x = (m_values[Ix] * vectorQuantityXY.x) + (m_values[Jx] * vectorQuantityXY.y);
	returnVector.y = (m_values[Iy] * vectorQuantityXY.x) + (m_values[Jy] * vectorQuantityXY.y);

	return returnVector;
}

//-----------------------------------------------------------------------------------------------
DPVec3 const DPMat44::TransformVectorQuantity3D(DPVec3 const& vectorQuantityXYZ) const
{
	DPVec3 returnVector;
	returnVector.x = (m_values[Ix] * vectorQuantityXYZ.x) + (m_values[Jx] * vectorQuantityXYZ.y) + (m_values[Kx] * vectorQuantityXYZ.z);
	returnVector.y = (m_values[Iy] * vectorQuantityXYZ.x) + (m_values[Jy] * vectorQuantityXYZ.y) + (m_values[Ky] * vectorQuantityXYZ.z);
	returnVector.z = (m_values[Iz] * vectorQuantityXYZ.x) + (m_values[Jz] * vectorQuantityXYZ.y) + (m_values[Kz] * vectorQuantityXYZ.z);

	return returnVector;
}

//-----------------------------------------------------------------------------------------------
DPVec2 const DPMat44::TransformPosition2D(DPVec2 const& positionXY) const
{
	DPVec2 returnVector;
	returnVector.x = (m_values[Ix] * positionXY.x) + (m_values[Jx] * positionXY.y) + m_values[Tx];
	returnVector.y = (m_values[Iy] * positionXY.x) + (m_values[Jy] * positionXY.y) + m_values[Ty];

	return returnVector;
}

//-----------------------------------------------------------------------------------------------
DPVec3 const DPMat44::TransformPosition3D(DPVec3 const& positionXYZ) const
{
	DPVec3 returnVector;
	returnVector.x = (m_values[Ix] * positionXYZ.x) + (m_values[Jx] * positionXYZ.y) + (m_values[Kx] * positionXYZ.z) + m_values[Tx];
	returnVector.y = (m_values[Iy] * positionXYZ.x) + (m_values[Jy] * positionXYZ.y) + (m_values[Ky] * positionXYZ.z) + m_values[Ty];
	returnVector.z = (m_values[Iz] * positionXYZ.x) + (m_values[Jz] * positionXYZ.y) + (m_values[Kz] * positionXYZ.z) + m_values[Tz];

	return returnVector;
}

//-----------------------------------------------------------------------------------------------
DPVec4 const DPMat44::TransformHomogeneous3D(DPVec4 const& homogeneousPoint3D) const
{
	DPVec4 returnVector;
	returnVector.x = (m_values[Ix] * homogeneousPoint3D.x) + (m_values[Jx] * homogeneousPoint3D.y) + (m_values[Kx] * homogeneousPoint3D.z) + (m_values[Tx] * homogeneousPoint3D.w);
	returnVector.y = (m_values[Iy] * homogeneousPoint3D.x) + (m_values[Jy] * homogeneousPoint3D.y) + (m_values[Ky] * homogeneousPoint3D.z) + (m_values[Ty] * homogeneousPoint3D.w);
	returnVector.z = (m_values[Iz] * homogeneousPoint3D.x) + (m_values[Jz] * homogeneousPoint3D.y) + (m_values[Kz] * homogeneousPoint3D.z) + (m_values[Tz] * homogeneousPoint3D.w);
	returnVector.w = (m_values[Iw] * homogeneousPoint3D.x) + (m_values[Jw] * homogeneousPoint3D.y) + (m_values[Kw] * homogeneousPoint3D.z) + (m_values[Tw] * homogeneousPoint3D.w);

	return returnVector;
}

//-----------------------------------------------------------------------------------------------
double* DPMat44::GetAsFloatArray()
{
	return m_values;
}

//-----------------------------------------------------------------------------------------------
double const* DPMat44::GetAsFloatArray() const
{
	return m_values;
}

//-----------------------------------------------------------------------------------------------
DPVec2 const DPMat44::GetIBasis2D() const
{
	DPVec2 iBasis;
	iBasis.x = m_values[Ix];
	iBasis.y = m_values[Iy];

	return iBasis;
}

//-----------------------------------------------------------------------------------------------
DPVec2 const DPMat44::GetJBasis2D() const
{
	DPVec2 jBasis;
	jBasis.x = m_values[Jx];
	jBasis.y = m_values[Jy];

	return jBasis;
}

//-----------------------------------------------------------------------------------------------
DPVec2 const DPMat44::GetTranslation2D() const
{
	DPVec2 translation;
	translation.x = m_values[Tx];
	translation.y = m_values[Ty];

	return translation;
}

//-----------------------------------------------------------------------------------------------
DPVec3 const DPMat44::GetIBasis3D() const
{
	DPVec3 iBasis;
	iBasis.x = m_values[Ix];
	iBasis.y = m_values[Iy];
	iBasis.z = m_values[Iz];

	return iBasis;
}

//-----------------------------------------------------------------------------------------------
DPVec3 const DPMat44::GetJBasis3D() const
{
	DPVec3 jBasis;
	jBasis.x = m_values[Jx];
	jBasis.y = m_values[Jy];
	jBasis.z = m_values[Jz];

	return jBasis;
}

//-----------------------------------------------------------------------------------------------
DPVec3 const DPMat44::GetKBasis3D() const
{
	DPVec3 kBasis;
	kBasis.x = m_values[Kx];
	kBasis.y = m_values[Ky];
	kBasis.z = m_values[Kz];

	return kBasis;
}

//-----------------------------------------------------------------------------------------------
DPVec3 const DPMat44::GetTranslation3D() const
{
	DPVec3 translation;
	translation.x = m_values[Tx];
	translation.y = m_values[Ty];
	translation.z = m_values[Tz];

	return translation;
}

//-----------------------------------------------------------------------------------------------
DPVec4 const DPMat44::GetIBasis4D() const
{
	DPVec4 iBasis;
	iBasis.x = m_values[Ix];
	iBasis.y = m_values[Iy];
	iBasis.z = m_values[Iz];
	iBasis.w = m_values[Iw];

	return iBasis;
}

//-----------------------------------------------------------------------------------------------
DPVec4 const DPMat44::GetJBasis4D() const
{
	DPVec4 jBasis;
	jBasis.x = m_values[Jx];
	jBasis.y = m_values[Jy];
	jBasis.z = m_values[Jz];
	jBasis.w = m_values[Jw];

	return jBasis;
}

//-----------------------------------------------------------------------------------------------
DPVec4 const DPMat44::GetKBasis4D() const
{
	DPVec4 kBasis;
	kBasis.x = m_values[Kx];
	kBasis.y = m_values[Ky];
	kBasis.z = m_values[Kz];
	kBasis.w = m_values[Kw];

	return kBasis;
}

//-----------------------------------------------------------------------------------------------
DPVec4 const DPMat44::GetTranslation4D() const
{
	DPVec4 translation;
	translation.x = m_values[Tx];
	translation.y = m_values[Ty];
	translation.z = m_values[Tz];
	translation.w = m_values[Tw];

	return translation;
}

//-----------------------------------------------------------------------------------------------
DPMat44 const DPMat44::GetOrthonormalInverse() const
{
	DPMat44 rotation;
	rotation.SetIJK3D(this->GetIBasis3D(), this->GetJBasis3D(), this->GetKBasis3D());
	rotation.Transpose();

	DPVec3 negativeT = GetTranslation3D();
	negativeT.x = -negativeT.x;
	negativeT.y = -negativeT.y;
	negativeT.z = -negativeT.z;

	DPMat44 translation;
	translation.SetTranslation3D(negativeT);

	rotation.Append(translation);
	return rotation;
}

//-----------------------------------------------------------------------------------------------
void DPMat44::SetTranslation2D(DPVec2 const& translationXY)
{
	m_values[Tx] = translationXY.x;
	m_values[Ty] = translationXY.y;
	m_values[Tz] = 0.0f;
	m_values[Tw] = 1.0f;
}

//-----------------------------------------------------------------------------------------------
void DPMat44::SetTranslation3D(DPVec3 const& translationXYZ)
{
	m_values[Tx] = translationXYZ.x;
	m_values[Ty] = translationXYZ.y;
	m_values[Tz] = translationXYZ.z;
	m_values[Tw] = 1.0f;
}

//-----------------------------------------------------------------------------------------------
void DPMat44::SetIJ2D(DPVec2 const& iBasis2D, DPVec2 const& jBasis2D)
{
	m_values[Ix] = iBasis2D.x;
	m_values[Iy] = iBasis2D.y;
	m_values[Iz] = 0.0f;
	m_values[Iw] = 0.0f;

	m_values[Jx] = jBasis2D.x;
	m_values[Jy] = jBasis2D.y;
	m_values[Jz] = 0.0f;
	m_values[Jw] = 0.0f;
}

//-----------------------------------------------------------------------------------------------
void DPMat44::SetIJT2D(DPVec2 const& iBasis2D, DPVec2 const& jBasis2D, DPVec2 const& translationXY)
{
	m_values[Ix] = iBasis2D.x;
	m_values[Iy] = iBasis2D.y;
	m_values[Iz] = 0.0f;
	m_values[Iw] = 0.0f;

	m_values[Jx] = jBasis2D.x;
	m_values[Jy] = jBasis2D.y;
	m_values[Jz] = 0.0f;
	m_values[Jw] = 0.0f;

	m_values[Tx] = translationXY.x;
	m_values[Ty] = translationXY.y;
	m_values[Tz] = 0.0f;
	m_values[Tw] = 1.0f;
}

//-----------------------------------------------------------------------------------------------
void DPMat44::SetIJK3D(DPVec3 const& iBasis3D, DPVec3 const& jBasis3D, DPVec3 const& kBasis3D)
{
	m_values[Ix] = iBasis3D.x;
	m_values[Iy] = iBasis3D.y;
	m_values[Iz] = iBasis3D.z;
	m_values[Iw] = 0.0f;

	m_values[Jx] = jBasis3D.x;
	m_values[Jy] = jBasis3D.y;
	m_values[Jz] = jBasis3D.z;
	m_values[Jw] = 0.0f;

	m_values[Kx] = kBasis3D.x;
	m_values[Ky] = kBasis3D.y;
	m_values[Kz] = kBasis3D.z;
	m_values[Kw] = 0.0f;
}

//-----------------------------------------------------------------------------------------------
void DPMat44::SetIJKT3D(DPVec3 const& iBasis3D, DPVec3 const& jBasis3D, DPVec3 const& kBasis3D, DPVec3 const& translationXYZ)
{
	m_values[Ix] = iBasis3D.x;
	m_values[Iy] = iBasis3D.y;
	m_values[Iz] = iBasis3D.z;
	m_values[Iw] = 0.0f;

	m_values[Jx] = jBasis3D.x;
	m_values[Jy] = jBasis3D.y;
	m_values[Jz] = jBasis3D.z;
	m_values[Jw] = 0.0f;

	m_values[Kx] = kBasis3D.x;
	m_values[Ky] = kBasis3D.y;
	m_values[Kz] = kBasis3D.z;
	m_values[Kw] = 0.0f;

	m_values[Tx] = translationXYZ.x;
	m_values[Ty] = translationXYZ.y;
	m_values[Tz] = translationXYZ.z;
	m_values[Tw] = 1.0f;
}

//-----------------------------------------------------------------------------------------------
void DPMat44::SetIJKT4D(DPVec4 const& iBasis4D, DPVec4 const& jBasis4D, DPVec4 const& kBasis4D, DPVec4 const& translation4D)
{
	m_values[Ix] = iBasis4D.x;
	m_values[Iy] = iBasis4D.y;
	m_values[Iz] = iBasis4D.z;
	m_values[Iw] = iBasis4D.w;

	m_values[Jx] = jBasis4D.x;
	m_values[Jy] = jBasis4D.y;
	m_values[Jz] = jBasis4D.z;
	m_values[Jw] = jBasis4D.w;

	m_values[Kx] = kBasis4D.x;
	m_values[Ky] = kBasis4D.y;
	m_values[Kz] = kBasis4D.z;
	m_values[Kw] = kBasis4D.w;

	m_values[Tx] = translation4D.x;
	m_values[Ty] = translation4D.y;
	m_values[Tz] = translation4D.z;
	m_values[Tw] = translation4D.w;
}

//-----------------------------------------------------------------------------------------------
void DPMat44::Transpose()
{
	DPMat44 transposeMatrix;
	transposeMatrix.m_values[Ix] = m_values[Ix];
	transposeMatrix.m_values[Iy] = m_values[Jx];
	transposeMatrix.m_values[Iz] = m_values[Kx];
	transposeMatrix.m_values[Iw] = m_values[Tx];

	transposeMatrix.m_values[Jx] = m_values[Iy];
	transposeMatrix.m_values[Jy] = m_values[Jy];
	transposeMatrix.m_values[Jz] = m_values[Ky];
	transposeMatrix.m_values[Jw] = m_values[Ty];

	transposeMatrix.m_values[Kx] = m_values[Iz];
	transposeMatrix.m_values[Ky] = m_values[Jz];
	transposeMatrix.m_values[Kz] = m_values[Kz];
	transposeMatrix.m_values[Kw] = m_values[Tz];

	transposeMatrix.m_values[Tx] = m_values[Iw];
	transposeMatrix.m_values[Ty] = m_values[Jw];
	transposeMatrix.m_values[Tz] = m_values[Kw];
	transposeMatrix.m_values[Tw] = m_values[Tw];

	for (int matrixIndex = 0; matrixIndex < 16; matrixIndex++)
	{
		m_values[matrixIndex] = transposeMatrix.m_values[matrixIndex];
	}
}

//-----------------------------------------------------------------------------------------------
void DPMat44::Orthonormalize_XFwd_YLeft_ZUp()
{
	DPVec3 IBasis = GetIBasis3D();
	DPVec3 JBasis = GetJBasis3D();
	DPVec3 KBasis = GetKBasis3D();

	//I
	IBasis.Normalize();

	//K
	double KDotI = DotProduct3D(KBasis, IBasis);
	DPVec3 KIDisplacement = KDotI * IBasis;
	KBasis -= KIDisplacement;
	KBasis.Normalize();

	//J
	double JDotI = DotProduct3D(JBasis, IBasis);
	DPVec3 JIDisplacement = JDotI * IBasis;
	JBasis -= JIDisplacement;
	double JDotK = DotProduct3D(JBasis, KBasis);
	DPVec3 JKDisplacement = JDotK * KBasis;
	JBasis -= JKDisplacement;
	JBasis.Normalize();

	SetIJK3D(IBasis, JBasis, KBasis);
}

//-----------------------------------------------------------------------------------------------
void DPMat44::Append(DPMat44 const& appendThis)
{
	DPMat44 copyOfSelf = *this;
	double const* left = &copyOfSelf.m_values[0];
	double const* right = &appendThis.m_values[0];

	m_values[Ix] = (left[Ix] * right[Ix]) + (left[Jx] * right[Iy]) + (left[Kx] * right[Iz]) + (left[Tx] * right[Iw]);
	m_values[Jx] = (left[Ix] * right[Jx]) + (left[Jx] * right[Jy]) + (left[Kx] * right[Jz]) + (left[Tx] * right[Jw]);
	m_values[Kx] = (left[Ix] * right[Kx]) + (left[Jx] * right[Ky]) + (left[Kx] * right[Kz]) + (left[Tx] * right[Kw]);
	m_values[Tx] = (left[Ix] * right[Tx]) + (left[Jx] * right[Ty]) + (left[Kx] * right[Tz]) + (left[Tx] * right[Tw]);

	m_values[Iy] = (left[Iy] * right[Ix]) + (left[Jy] * right[Iy]) + (left[Ky] * right[Iz]) + (left[Ty] * right[Iw]);
	m_values[Jy] = (left[Iy] * right[Jx]) + (left[Jy] * right[Jy]) + (left[Ky] * right[Jz]) + (left[Ty] * right[Jw]);
	m_values[Ky] = (left[Iy] * right[Kx]) + (left[Jy] * right[Ky]) + (left[Ky] * right[Kz]) + (left[Ty] * right[Kw]);
	m_values[Ty] = (left[Iy] * right[Tx]) + (left[Jy] * right[Ty]) + (left[Ky] * right[Tz]) + (left[Ty] * right[Tw]);

	m_values[Iz] = (left[Iz] * right[Ix]) + (left[Jz] * right[Iy]) + (left[Kz] * right[Iz]) + (left[Tz] * right[Iw]);
	m_values[Jz] = (left[Iz] * right[Jx]) + (left[Jz] * right[Jy]) + (left[Kz] * right[Jz]) + (left[Tz] * right[Jw]);
	m_values[Kz] = (left[Iz] * right[Kx]) + (left[Jz] * right[Ky]) + (left[Kz] * right[Kz]) + (left[Tz] * right[Kw]);
	m_values[Tz] = (left[Iz] * right[Tx]) + (left[Jz] * right[Ty]) + (left[Kz] * right[Tz]) + (left[Tz] * right[Tw]);

	m_values[Iw] = (left[Iw] * right[Ix]) + (left[Jw] * right[Iy]) + (left[Kw] * right[Iz]) + (left[Tw] * right[Iw]);
	m_values[Jw] = (left[Iw] * right[Jx]) + (left[Jw] * right[Jy]) + (left[Kw] * right[Jz]) + (left[Tw] * right[Jw]);
	m_values[Kw] = (left[Iw] * right[Kx]) + (left[Jw] * right[Ky]) + (left[Kw] * right[Kz]) + (left[Tw] * right[Kw]);
	m_values[Tw] = (left[Iw] * right[Tx]) + (left[Jw] * right[Ty]) + (left[Kw] * right[Tz]) + (left[Tw] * right[Tw]);
}

//-----------------------------------------------------------------------------------------------
void DPMat44::AppendZRotation(double degreesRotationAboutZ)
{
	DPMat44 rotZ = CreateZRotationDegrees(degreesRotationAboutZ);
	Append(rotZ);
}

//-----------------------------------------------------------------------------------------------
void DPMat44::AppendYRotation(double degreesRotationAboutY)
{
	DPMat44 rotY = CreateYRotationDegrees(degreesRotationAboutY);
	Append(rotY);
}

//-----------------------------------------------------------------------------------------------
void DPMat44::AppendXRotation(double degreesRotationAboutX)
{
	DPMat44 rotX = CreateXRotationDegrees(degreesRotationAboutX);
	Append(rotX);
}

//-----------------------------------------------------------------------------------------------
void DPMat44::AppendTranslation2D(DPVec2 const& translationXY)
{
	DPMat44 rotX = CreateTranslation2D(translationXY);
	Append(rotX);
}

//-----------------------------------------------------------------------------------------------
void DPMat44::AppendTranslation3D(DPVec3 const& translationXYZ)
{
	DPMat44 rotX = CreateTranslation3D(translationXYZ);
	Append(rotX);
}

//-----------------------------------------------------------------------------------------------
void DPMat44::AppendScaleUniform2D(double uniformScaleXY)
{
	DPMat44 scaleMatrix = CreateUniformScale2D(uniformScaleXY);
	Append(scaleMatrix);
}

//-----------------------------------------------------------------------------------------------
void DPMat44::AppendScaleUniform3D(double uniformScaleXYZ)
{
	DPMat44 scaleMatrix = CreateUniformScale3D(uniformScaleXYZ);
	Append(scaleMatrix);
}

//-----------------------------------------------------------------------------------------------
void DPMat44::AppendScaleNonUniform2D(DPVec2 const& nonUniformScaleXY)
{
	DPMat44 scaleMatrix = CreateNonUniformScale2D(nonUniformScaleXY);
	Append(scaleMatrix);
}

//-----------------------------------------------------------------------------------------------
void DPMat44::AppendScaleNonUniform3D(DPVec3 const& nonUniformScaleXYZ)
{
	DPMat44 scaleMatrix = CreateNonUniformScale3D(nonUniformScaleXYZ);
	Append(scaleMatrix);
}
