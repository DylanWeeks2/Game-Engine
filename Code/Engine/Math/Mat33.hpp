#pragma once
#include <string>

//-----------------------------------------------------------------------------------------------
struct EulerAngles;
struct Vec2;	
struct Vec3;
struct Vec4;

//-----------------------------------------------------------------------------------------------
struct Mat33
{
	enum {Ix, Iy, Iz,	Jx, Jy, Jz,		Kx, Ky, Kz}; //index nicknames [0] thru [8]
	float m_values[9];

	Mat33();
	explicit Mat33(Vec2 const& iBasis2D, Vec2 const& jBasis2D);
	explicit Mat33(Vec3 const& iBasis3D, Vec3 const& jBasis3D, Vec3 const& kBasis3D);
	explicit Mat33(float const* sixteenValeusBasisMajor); 

	static	Mat33 const CreateUniformScale2D(float const& uniformScaleXY);
	static	Mat33 const CreateUniformScale3D(float const& uniformScaleXYZ);
	static	Mat33 const CreateNonUniformScale2D(Vec2 const& nonUniformScaleXY);
	static	Mat33 const CreateNonUniformScale3D(Vec3 const& nonUniformScaleXYZ);
	static	Mat33 const CreateZRotationDegrees(float rotationDegreesAboutZ);
	static	Mat33 const CreateYRotationDegrees(float rotationDegreesAboutY);
	static	Mat33 const CreateXRotationDegrees(float rotationDegreesAboutX);
	static	Mat33 const CreateOrthoProjection(float left, float right, float bottom, float top, float zNear, float zFar);
	static	Mat33 const CreatePerspectiveProjection(float fovYDegrees, float aspect, float zNear, float zFar);

	Vec2 const			TransformVectorQuantity2D(Vec2 const& vectorQuantityXY) const;
	Vec3 const			TransformVectorQuantity3D(Vec3 const& vectorQuantityXYZ) const;
	Vec2 const			TransformPosition2D(Vec2 const& positionXY) const;
	Vec3 const			TransformPosition3D(Vec3 const& positionXYZ) const;

	float*				GetAsFloatArray();
	float const*		GetAsFloatArray() const;
	Vec2 const			GetIBasis2D() const;
	Vec2 const			GetJBasis2D() const;
	Vec3 const			GetIBasis3D() const;
	Vec3 const			GetJBasis3D() const;
	Vec3 const			GetKBasis3D() const;
	Mat33 const			GetOrthonormalInverse() const;
	float const			GetDeterminant() const;
	Mat33 const			GetInverse() const;

	void				SetIJ2D(Vec2 const& iBasis2D, Vec2 const& jBasis2D);
	void				SetIJK3D(Vec3 const& iBasis3D, Vec3 const& jBasis3D, Vec3 const& kBasis3D);
	void				Transpose();
	void				Orthonormalize_XFwd_YLeft_ZUp();

	void				Append(Mat33 const& appendThis);
	void				AppendZRotation(float degreesRotationAboutZ);
	void				AppendYRotation(float degreesRotationAboutY);
	void				AppendXRotation(float degreesRotationAboutX);
	void				AppendScaleUniform2D(float uniformScaleXY);
	void				AppendScaleUniform3D(float uniformScaleXYZ);
	void				AppendScaleNonUniform2D(Vec2 const& nonUniformScaleXY);
	void				AppendScaleNonUniform3D(Vec3 const& nonUniformScaleXYZ);
};
