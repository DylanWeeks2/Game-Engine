#include "Mat33.hpp"
#include "Vec2.hpp"
#include "Vec3.hpp"
#include "Vec4.hpp"
#include "MathUtils.hpp"
#include "EulerAngles.hpp"
#include "Engine/Core/StringUtils.hpp"

//-----------------------------------------------------------------------------------------------
Mat33::Mat33()
{
	//Identity Matrix
	m_values[Ix] = 1.0f;
	m_values[Iy] = 0.0f;
	m_values[Iz] = 0.0f;

	m_values[Jx] = 0.0f;
	m_values[Jy] = 1.0f;
	m_values[Jz] = 0.0f;

	m_values[Kx] = 0.0f;
	m_values[Ky] = 0.0f;
	m_values[Kz] = 1.0f;
}

//-----------------------------------------------------------------------------------------------
Mat33::Mat33(Vec2 const& iBasis2D, Vec2 const& jBasis2D)
{
	m_values[Ix] = iBasis2D.x;
	m_values[Iy] = iBasis2D.y;
	m_values[Iz] = 0.0f;

	m_values[Jx] = jBasis2D.x;
	m_values[Jy] = jBasis2D.y;
	m_values[Jz] = 0.0f;

	m_values[Kx] = 0.0f;
	m_values[Ky] = 0.0f;
	m_values[Kz] = 1.0f;
}

//-----------------------------------------------------------------------------------------------
Mat33::Mat33(Vec3 const& iBasis3D, Vec3 const& jBasis3D, Vec3 const& kBasis3D)
{
	m_values[Ix] = iBasis3D.x;
	m_values[Iy] = iBasis3D.y;
	m_values[Iz] = iBasis3D.z;

	m_values[Jx] = jBasis3D.x;
	m_values[Jy] = jBasis3D.y;
	m_values[Jz] = jBasis3D.z;

	m_values[Kx] = kBasis3D.x;
	m_values[Ky] = kBasis3D.y;
	m_values[Kz] = kBasis3D.z;
}

//-----------------------------------------------------------------------------------------------
Mat33::Mat33(float const* sixteenValeusBasisMajor)
{
	m_values[Ix] = sixteenValeusBasisMajor[Ix];
	m_values[Iy] = sixteenValeusBasisMajor[Iy];
	m_values[Iz] = sixteenValeusBasisMajor[Iz];

	m_values[Jx] = sixteenValeusBasisMajor[Jx];
	m_values[Jy] = sixteenValeusBasisMajor[Jy];
	m_values[Jz] = sixteenValeusBasisMajor[Jz];

	m_values[Kx] = sixteenValeusBasisMajor[Kx];
	m_values[Ky] = sixteenValeusBasisMajor[Ky];
	m_values[Kz] = sixteenValeusBasisMajor[Kz];
}

//-----------------------------------------------------------------------------------------------
Mat33 const Mat33::CreateUniformScale2D(float const& uniformScaleXY)
{
	Mat33 scaledMatrix;
	scaledMatrix.m_values[Ix] *= uniformScaleXY;
	scaledMatrix.m_values[Iy] *= uniformScaleXY;
	scaledMatrix.m_values[Jx] *= uniformScaleXY;
	scaledMatrix.m_values[Jy] *= uniformScaleXY;

	return scaledMatrix;
}

//-----------------------------------------------------------------------------------------------
Mat33 const Mat33::CreateUniformScale3D(float const& uniformScaleXYZ)
{
	Mat33 scaledMatrix;
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
Mat33 const Mat33::CreateNonUniformScale2D(Vec2 const& nonUniformScaleXY)
{
	Mat33 scaledMatrix;
	scaledMatrix.m_values[Ix] *= nonUniformScaleXY.x;
	scaledMatrix.m_values[Iy] *= nonUniformScaleXY.y;
	scaledMatrix.m_values[Jx] *= nonUniformScaleXY.x;
	scaledMatrix.m_values[Jy] *= nonUniformScaleXY.y;

	return scaledMatrix;
}

//-----------------------------------------------------------------------------------------------
Mat33 const Mat33::CreateNonUniformScale3D(Vec3 const& nonUniformScaleXYZ)
{
	Mat33 scaledMatrix;
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
Mat33 const Mat33::CreateZRotationDegrees(float rotationDegreesAboutZ)
{
	Mat33 rotZ;
	float c = CosDegrees(rotationDegreesAboutZ);
	float s = SinDegrees(rotationDegreesAboutZ);
	rotZ.m_values[Ix] = c;
	rotZ.m_values[Iy] = s;
	rotZ.m_values[Jx] = -s;
	rotZ.m_values[Jy] = c;

	return rotZ;
}

//-----------------------------------------------------------------------------------------------
Mat33 const Mat33::CreateYRotationDegrees(float rotationDegreesAboutY)
{
	Mat33 rotZ;
	float c = CosDegrees(rotationDegreesAboutY);
	float s = SinDegrees(rotationDegreesAboutY);
	rotZ.m_values[Ix] = c;
	rotZ.m_values[Iz] = -s;
	rotZ.m_values[Kx] = s;
	rotZ.m_values[Kz] = c;

	return rotZ;
}

//-----------------------------------------------------------------------------------------------
Mat33 const Mat33::CreateXRotationDegrees(float rotationDegreesAboutX)
{
	Mat33 rotZ;
	float c = CosDegrees(rotationDegreesAboutX);
	float s = SinDegrees(rotationDegreesAboutX);
	rotZ.m_values[Jy] = c;
	rotZ.m_values[Jz] = s;
	rotZ.m_values[Ky] = -s;
	rotZ.m_values[Kz] = c;

	return rotZ;
}

//-----------------------------------------------------------------------------------------------
Mat33 const Mat33::CreateOrthoProjection(float left, float right, float bottom, float top, float zNear, float zFar)
{
	Mat33 orthoProjection;
	orthoProjection.m_values[Ix] = 2.0f / (right - left);
	orthoProjection.m_values[Jy] = 2.0f / (top - bottom);
	orthoProjection.m_values[Kz] = 1.0f / (zFar - zNear);
	return orthoProjection;
}

//-----------------------------------------------------------------------------------------------
Mat33 const Mat33::CreatePerspectiveProjection(float fovYDegrees, float aspect, float zNear, float zFar)
{
	Mat33 prospectiveProjection;
	prospectiveProjection.m_values[Ix] = (1.0f / TanDegrees(fovYDegrees / 2.0f)) / aspect;
	prospectiveProjection.m_values[Jy] = 1.0f / TanDegrees(fovYDegrees / 2.0f);
	prospectiveProjection.m_values[Kz] = zFar / (zFar - zNear);
	return prospectiveProjection;
}

//-----------------------------------------------------------------------------------------------
Vec2 const Mat33::TransformVectorQuantity2D(Vec2 const& vectorQuantityXY) const
{
	Vec2 returnVector;
	returnVector.x = (m_values[Ix] * vectorQuantityXY.x) + (m_values[Jx] * vectorQuantityXY.y);
	returnVector.y = (m_values[Iy] * vectorQuantityXY.x) + (m_values[Jy] * vectorQuantityXY.y);

	return returnVector;
}

//-----------------------------------------------------------------------------------------------
Vec3 const Mat33::TransformVectorQuantity3D(Vec3 const& vectorQuantityXYZ) const
{
	Vec3 returnVector;
	returnVector.x = (m_values[Ix] * vectorQuantityXYZ.x) + (m_values[Jx] * vectorQuantityXYZ.y) + (m_values[Kx] * vectorQuantityXYZ.z);
	returnVector.y = (m_values[Iy] * vectorQuantityXYZ.x) + (m_values[Jy] * vectorQuantityXYZ.y) + (m_values[Ky] * vectorQuantityXYZ.z);
	returnVector.z = (m_values[Iz] * vectorQuantityXYZ.x) + (m_values[Jz] * vectorQuantityXYZ.y) + (m_values[Kz] * vectorQuantityXYZ.z);

	return returnVector;
}

//-----------------------------------------------------------------------------------------------
Vec2 const Mat33::TransformPosition2D(Vec2 const& positionXY) const
{
	Vec2 returnVector;
	returnVector.x = (m_values[Ix] * positionXY.x) + (m_values[Jx] * positionXY.y);
	returnVector.y = (m_values[Iy] * positionXY.x) + (m_values[Jy] * positionXY.y);

	return returnVector;
}

//-----------------------------------------------------------------------------------------------
Vec3 const Mat33::TransformPosition3D(Vec3 const& positionXYZ) const
{
	Vec3 returnVector;
	returnVector.x = (m_values[Ix] * positionXYZ.x) + (m_values[Jx] * positionXYZ.y) + (m_values[Kx] * positionXYZ.z);
	returnVector.y = (m_values[Iy] * positionXYZ.x) + (m_values[Jy] * positionXYZ.y) + (m_values[Ky] * positionXYZ.z);
	returnVector.z = (m_values[Iz] * positionXYZ.x) + (m_values[Jz] * positionXYZ.y) + (m_values[Kz] * positionXYZ.z);

	return returnVector;
}

//-----------------------------------------------------------------------------------------------
float* Mat33::GetAsFloatArray()
{
	return m_values;
}

//-----------------------------------------------------------------------------------------------
float const* Mat33::GetAsFloatArray() const
{
	return m_values;
}

//-----------------------------------------------------------------------------------------------
Vec2 const Mat33::GetIBasis2D() const
{
	Vec2 iBasis;
	iBasis.x = m_values[Ix];
	iBasis.y = m_values[Iy];

	return iBasis;
}

//-----------------------------------------------------------------------------------------------
Vec2 const Mat33::GetJBasis2D() const
{
	Vec2 jBasis;
	jBasis.x = m_values[Jx];
	jBasis.y = m_values[Jy];

	return jBasis;
}

//-----------------------------------------------------------------------------------------------
Vec3 const Mat33::GetIBasis3D() const
{
	Vec3 iBasis;
	iBasis.x = m_values[Ix];
	iBasis.y = m_values[Iy];
	iBasis.z = m_values[Iz];

	return iBasis;
}

//-----------------------------------------------------------------------------------------------
Vec3 const Mat33::GetJBasis3D() const
{
	Vec3 jBasis;
	jBasis.x = m_values[Jx];
	jBasis.y = m_values[Jy];
	jBasis.z = m_values[Jz];

	return jBasis;
}

//-----------------------------------------------------------------------------------------------
Vec3 const Mat33::GetKBasis3D() const
{
	Vec3 kBasis;
	kBasis.x = m_values[Kx];
	kBasis.y = m_values[Ky];
	kBasis.z = m_values[Kz];

	return kBasis;
}

//-----------------------------------------------------------------------------------------------
Mat33 const Mat33::GetOrthonormalInverse() const
{
	Mat33 rotation;
	rotation.SetIJK3D(this->GetIBasis3D(), this->GetJBasis3D(), this->GetKBasis3D());
	rotation.Transpose();
	return rotation;
}

//-----------------------------------------------------------------------------------------------
float const Mat33::GetDeterminant() const
{
	float determinant =
		(m_values[Ix] * m_values[Jy] * m_values[Kz]) -
		(m_values[Ix] * m_values[Jz] * m_values[Ky]) +
		(m_values[Iy] * m_values[Jz] * m_values[Kx]) - 
		(m_values[Iy] * m_values[Jx] * m_values[Kz]) + 
		(m_values[Iz] * m_values[Jx] * m_values[Ky]) - 
		(m_values[Iz] * m_values[Jy] * m_values[Kx]);
	
	return determinant;
}

//-----------------------------------------------------------------------------------------------
Mat33 const Mat33::GetInverse() const
{
	float determinant = GetDeterminant();
	if (determinant == 0.0f)
		determinant = 1.0f;

	float values[9] = {
		(m_values[Jy] * m_values[Kz] - m_values[Ky] * m_values[Jz]) / determinant,
		-(m_values[Iy] * m_values[Kz] - m_values[Ky] * m_values[Jz]) / determinant,
		(m_values[Iy] * m_values[Jz] - m_values[Jy] * m_values[Iz]) / determinant,
		-(m_values[Jx] * m_values[Kz] - m_values[Kx] * m_values[Jz]) / determinant,
		(m_values[Ix] * m_values[Kz] - m_values[Kx] * m_values[Iz]) / determinant,
		-(m_values[Ix] * m_values[Jz] - m_values[Jx] * m_values[Iz]) / determinant,
		(m_values[Jx] * m_values[Ky] - m_values[Kx] * m_values[Jy]) / determinant,
		-(m_values[Ix] * m_values[Ky] - m_values[Kx] * m_values[Iy]) / determinant,
		(m_values[Ix] * m_values[Jy] - m_values[Jx] * m_values[Iy]) / determinant
	};

	Mat33 inverseMatrix(values);
	return inverseMatrix;
}

//-----------------------------------------------------------------------------------------------
void Mat33::SetIJ2D(Vec2 const& iBasis2D, Vec2 const& jBasis2D)
{
	m_values[Ix] = iBasis2D.x;
	m_values[Iy] = iBasis2D.y;
	m_values[Iz] = 0.0f;

	m_values[Jx] = jBasis2D.x;
	m_values[Jy] = jBasis2D.y;
	m_values[Jz] = 0.0f;
}

//-----------------------------------------------------------------------------------------------
void Mat33::SetIJK3D(Vec3 const& iBasis3D, Vec3 const& jBasis3D, Vec3 const& kBasis3D)
{
	m_values[Ix] = iBasis3D.x;
	m_values[Iy] = iBasis3D.y;
	m_values[Iz] = iBasis3D.z;

	m_values[Jx] = jBasis3D.x;
	m_values[Jy] = jBasis3D.y;
	m_values[Jz] = jBasis3D.z;

	m_values[Kx] = kBasis3D.x;
	m_values[Ky] = kBasis3D.y;
	m_values[Kz] = kBasis3D.z;
}

//-----------------------------------------------------------------------------------------------
void Mat33::Transpose()
{
	Mat33 transposeMatrix;
	transposeMatrix.m_values[Ix] = m_values[Ix];
	transposeMatrix.m_values[Iy] = m_values[Jx];
	transposeMatrix.m_values[Iz] = m_values[Kx];

	transposeMatrix.m_values[Jx] = m_values[Iy];
	transposeMatrix.m_values[Jy] = m_values[Jy];
	transposeMatrix.m_values[Jz] = m_values[Ky];

	transposeMatrix.m_values[Kx] = m_values[Iz];
	transposeMatrix.m_values[Ky] = m_values[Jz];
	transposeMatrix.m_values[Kz] = m_values[Kz];

	for (int matrixIndex = 0; matrixIndex < 9; matrixIndex++)
	{
		m_values[matrixIndex] = transposeMatrix.m_values[matrixIndex];
	}
}

//-----------------------------------------------------------------------------------------------
void Mat33::Orthonormalize_XFwd_YLeft_ZUp()
{
	Vec3 IBasis = GetIBasis3D();
	Vec3 JBasis = GetJBasis3D();
	Vec3 KBasis = GetKBasis3D();

	//I
	IBasis.Normalize();

	//K
	float KDotI = DotProduct3D(KBasis, IBasis);
	Vec3 KIDisplacement = KDotI * IBasis;
	KBasis -= KIDisplacement;
	KBasis.Normalize();

	//J
	float JDotI = DotProduct3D(JBasis, IBasis);
	Vec3 JIDisplacement = JDotI * IBasis;
	JBasis -= JIDisplacement;
	float JDotK = DotProduct3D(JBasis, KBasis);
	Vec3 JKDisplacement = JDotK * KBasis;
	JBasis -= JKDisplacement;
	JBasis.Normalize();

	SetIJK3D(IBasis, JBasis, KBasis);
}

//-----------------------------------------------------------------------------------------------
void Mat33::Append(Mat33 const& appendThis)
{
	Mat33 copyOfSelf = *this;
	float const* left = &copyOfSelf.m_values[0];
	float const* right = &appendThis.m_values[0];

	m_values[Ix] = (left[Ix] * right[Ix]) + (left[Jx] * right[Iy]) + (left[Kx] * right[Iz]);
	m_values[Jx] = (left[Ix] * right[Jx]) + (left[Jx] * right[Jy]) + (left[Kx] * right[Jz]);
	m_values[Kx] = (left[Ix] * right[Kx]) + (left[Jx] * right[Ky]) + (left[Kx] * right[Kz]);

	m_values[Iy] = (left[Iy] * right[Ix]) + (left[Jy] * right[Iy]) + (left[Ky] * right[Iz]);
	m_values[Jy] = (left[Iy] * right[Jx]) + (left[Jy] * right[Jy]) + (left[Ky] * right[Jz]);
	m_values[Ky] = (left[Iy] * right[Kx]) + (left[Jy] * right[Ky]) + (left[Ky] * right[Kz]);

	m_values[Iz] = (left[Iz] * right[Ix]) + (left[Jz] * right[Iy]) + (left[Kz] * right[Iz]);
	m_values[Jz] = (left[Iz] * right[Jx]) + (left[Jz] * right[Jy]) + (left[Kz] * right[Jz]);
	m_values[Kz] = (left[Iz] * right[Kx]) + (left[Jz] * right[Ky]) + (left[Kz] * right[Kz]);
}

//-----------------------------------------------------------------------------------------------
void Mat33::AppendZRotation(float degreesRotationAboutZ)
{
	Mat33 rotZ = CreateZRotationDegrees(degreesRotationAboutZ);
	Append(rotZ);
}

//-----------------------------------------------------------------------------------------------
void Mat33::AppendYRotation(float degreesRotationAboutY)
{
	Mat33 rotY = CreateYRotationDegrees(degreesRotationAboutY);
	Append(rotY);
}

//-----------------------------------------------------------------------------------------------
void Mat33::AppendXRotation(float degreesRotationAboutX)
{
	Mat33 rotX = CreateXRotationDegrees(degreesRotationAboutX);
	Append(rotX);
}

//-----------------------------------------------------------------------------------------------
void Mat33::AppendScaleUniform2D(float uniformScaleXY)
{
	Mat33 scaleMatrix = CreateUniformScale2D(uniformScaleXY);
	Append(scaleMatrix);
}

//-----------------------------------------------------------------------------------------------
void Mat33::AppendScaleUniform3D(float uniformScaleXYZ)
{
	Mat33 scaleMatrix = CreateUniformScale3D(uniformScaleXYZ);
	Append(scaleMatrix);
}

//-----------------------------------------------------------------------------------------------
void Mat33::AppendScaleNonUniform2D(Vec2 const& nonUniformScaleXY)
{
	Mat33 scaleMatrix = CreateNonUniformScale2D(nonUniformScaleXY);
	Append(scaleMatrix);
}

//-----------------------------------------------------------------------------------------------
void Mat33::AppendScaleNonUniform3D(Vec3 const& nonUniformScaleXYZ)
{
	Mat33 scaleMatrix = CreateNonUniformScale3D(nonUniformScaleXYZ);
	Append(scaleMatrix);
}
