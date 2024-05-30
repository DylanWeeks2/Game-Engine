#pragma once
#include <string>

//-----------------------------------------------------------------------------------------------
struct EulerAngles;
struct DPVec2;	
struct DPVec3;
struct DPVec4;

//-----------------------------------------------------------------------------------------------
struct DPMat44
{
	enum {Ix, Iy, Iz, Iw,	Jx, Jy, Jz, Jw,		Kx, Ky, Kz, Kw,		Tx, Ty, Tz, Tw}; //index nicknames [0] thru [15]
	double m_values[16];

	DPMat44();
	explicit DPMat44(DPVec2 const& iBasis2D, DPVec2 const& jBasis2D, DPVec2 const& tranlation2D);
	explicit DPMat44(DPVec3 const& iBasis3D, DPVec3 const& jBasis3D, DPVec3 const& kBasis3D, DPVec3 const& translation3D);
	explicit DPMat44(DPVec4 const& iBasis4D, DPVec4 const& jBasis4D, DPVec4 const& kBasis4D, DPVec4 const& translation4D);
	explicit DPMat44(double const* sixteenValeusBasisMajor); 

	static DPMat44 const	CreateTranslation2D(DPVec2 const& translationXY);
	static DPMat44 const	CreateTranslation3D(DPVec3 const& translationXYZ);
	static DPMat44 const	CreateUniformScale2D(double const& uniformScaleXY);
	static DPMat44 const	CreateUniformScale3D(double const& uniformScaleXYZ);
	static DPMat44 const	CreateNonUniformScale2D(DPVec2 const& nonUniformScaleXY);
	static DPMat44 const	CreateNonUniformScale3D(DPVec3 const& nonUniformScaleXYZ);
	static DPMat44 const	CreateZRotationDegrees(double rotationDegreesAboutZ);
	static DPMat44 const	CreateYRotationDegrees(double rotationDegreesAboutY);
	static DPMat44 const	CreateXRotationDegrees(double rotationDegreesAboutX);
	static DPMat44 const	CreateOrthoProjection(double left, double right, double bottom, double top, double zNear, double zFar);
	static DPMat44 const	CreatePerspectiveProjection(double fovYDegrees, double aspect, double zNear, double zFar);

	DPVec2 const			TransformVectorQuantity2D(DPVec2 const& vectorQuantityXY) const;
	DPVec3 const			TransformVectorQuantity3D(DPVec3 const& vectorQuantityXYZ) const;
	DPVec2 const			TransformPosition2D(DPVec2 const& positionXY) const;
	DPVec3 const			TransformPosition3D(DPVec3 const& positionXYZ) const;
	DPVec4 const			TransformHomogeneous3D(DPVec4 const& homogeneousPoint3D) const;

	double*					GetAsFloatArray();
	double const*			GetAsFloatArray() const;
	DPVec2 const			GetIBasis2D() const;
	DPVec2 const			GetJBasis2D() const;
	DPVec2 const			GetTranslation2D() const;
	DPVec3 const			GetIBasis3D() const;
	DPVec3 const			GetJBasis3D() const;
	DPVec3 const			GetKBasis3D() const;
	DPVec3 const			GetTranslation3D() const;
	DPVec4 const			GetIBasis4D() const;
	DPVec4 const			GetJBasis4D() const;
	DPVec4 const			GetKBasis4D() const;
	DPVec4 const			GetTranslation4D() const;
	DPMat44 const			GetOrthonormalInverse() const;

	void					SetTranslation2D(DPVec2 const& translationXY);
	void					SetTranslation3D(DPVec3 const& translationXYZ);
	void					SetIJ2D(DPVec2 const& iBasis2D, DPVec2 const& jBasis2D);
	void					SetIJT2D(DPVec2 const& iBasis2D, DPVec2 const& jBasis2D, DPVec2 const& translationXY);
	void					SetIJK3D(DPVec3 const& iBasis3D, DPVec3 const& jBasis3D, DPVec3 const& kBasis3D);
	void					SetIJKT3D(DPVec3 const& iBasis3D, DPVec3 const& jBasis3D, DPVec3 const& kBasis3D, DPVec3 const& translationXYZ);
	void					SetIJKT4D(DPVec4 const& iBasis4D, DPVec4 const& jBasis4D, DPVec4 const& kBasis4D, DPVec4 const& translation4D);
	void					Transpose();
	void					Orthonormalize_XFwd_YLeft_ZUp();

	void					Append(DPMat44 const& appendThis);
	void					AppendZRotation(double degreesRotationAboutZ);
	void					AppendYRotation(double degreesRotationAboutY);
	void					AppendXRotation(double degreesRotationAboutX);
	void					AppendTranslation2D(DPVec2 const& translationXY);
	void					AppendTranslation3D(DPVec3 const& translationXYZ);
	void					AppendScaleUniform2D(double uniformScaleXY);
	void					AppendScaleUniform3D(double uniformScaleXYZ);
	void					AppendScaleNonUniform2D(DPVec2 const& nonUniformScaleXY);
	void					AppendScaleNonUniform3D(DPVec3 const& nonUniformScaleXYZ);
};
