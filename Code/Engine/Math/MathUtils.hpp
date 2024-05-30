#pragma once
#include "Vec2.hpp"
#include "Vec3.hpp"
#include "DPVec3.hpp"
#include <vector>
#include <Engine/Math/ConvexHull2D.hpp>

//-----------------------------------------------------------------------------------------------
enum class BillboardType
{
	NONE = -1,
	WORLD_UP_CAMERA_FACING,
	WORLD_UP_CAMERA_OPPOSING,
	FULL_CAMERA_FACING,
	FULL_CAMERA_OPPOSING,
	COUNT
};

//-----------------------------------------------------------------------------------------------
struct RaycastResult2D
{
public :
	bool	m_didImpact = false;
	float	m_impactDist = 0.0f;
	Vec2	m_impactPos;
	Vec2	m_impactNormal;
	Vec2	m_rayFwdNormal;
	Vec2	m_rayStartPos;
	float	m_rayMaxLength = 1.0f;
};

//-----------------------------------------------------------------------------------------------
struct RaycastResult3D
{
public:
	bool	m_didImpact = false;
	float	m_impactDist = 0.0f;
	Vec3	m_impactPos;
	Vec3	m_impactNormal;
	Vec3	m_rayFwdNormal;
	Vec3	m_rayStartPos;
	float	m_rayMaxLength = 1.0f;
};

//-----------------------------------------------------------------------------------------------
//Forward Type Declarations
struct Vec2;
struct Vec3;
struct IntVec2;
struct AABB2;
struct LineSegment2;
struct LineSegment3;
struct Capsule2;
struct Capsule3;
struct Cylinder3;
struct DPCapsule3;
struct DPCylinder3;
struct OBB2;
struct OBB3;
struct DPOBB3;
struct Vec4;
struct DPVec4;
struct Mat44;
struct DPMat44;
struct AABB3;
struct DPAABB3;
struct Plane2D;
struct Plane3D;
struct ConvexHull2D;
struct DPVec2;

//-----------------------------------------------------------------------------------------------
//Clamp and Lerp
float				GetClamped(float value, float minValue, float maxValue);
double				GetClamped(double value, double minValue, double maxValue);
float				GetClampedZeroToOne(float value);
float				Interpolate(float start, float end, float fractionTowardEnd);
float				GetFractionWithinRange(float value, float rangeStart, float rangeEnd);
float				RangeMap(float inValue, float inStart, float inEnd, float outStart, float outEnd);
float				RangeMapClamped(float inValue, float inStart, float inEnd, float outStart, float outEnd);
float				RoundDownToInt(float value);

//-----------------------------------------------------------------------------------------------
//Angle functions
float				ConvertDegreesToRadians(float degrees);
float				ConvertRadiansToDegrees(float radians);
double				ConvertDegreesToRadians(double degrees);
double				ConvertRadiansToDegrees(double radians);
float				CosDegrees(float degrees);
float				SinDegrees(float degrees);
float				TanDegrees(float degrees);
double				CosDegrees(double degrees);
double				SinDegrees(double degrees);
double				TanDegrees(double degrees);
float				Atan2Degrees(float y, float x);
double				Atan2Degrees(double y, double x);
float				GetShortestAngularDispDegrees(float startDegrees, float endDegrees);
float				GetTurnedTowardDegrees(float currentDegrees, float goalDegrees, float maxDeltaDegrees);
float				GetAngleDegreesBetweenVectors2D(Vec2 const& a, Vec2 const& b);
float				GetAngleDegreesBetweenVectors3D(Vec3 const& a, Vec3 const& b);

//-----------------------------------------------------------------------------------------------
//Dot and cross
float				DotProduct2D(Vec2 const& a, Vec2 const& b);
double				DotProduct2D(DPVec2 const& a, DPVec2 const& b);
float				DotProduct3D(Vec3 const& a, Vec3 const& b);
double				DotProduct3D(DPVec3 const& a, DPVec3 const& b);
float				DotProduct4D(Vec4 const& a, Vec4 const& b);
float				CrossProduct2D(Vec2 const& a, Vec2 const& b);
Vec3				CrossProduct3D(Vec3 const& a, Vec3 const& b);
DPVec3				CrossProduct3D(DPVec3 const& a, DPVec3 const& b);

//-----------------------------------------------------------------------------------------------
//Basic 2D and 3D utilities
float				GetDistance2D(Vec2 const& positionA, Vec2 const& positionB);
float				GetDistanceSquared2D(Vec2 const& positionA, Vec2 const& positionB);
float				GetDistance3D(Vec3 const& positionA, Vec3 const& positionB);
float				GetDistanceSquared3D(Vec3 const& positionA, Vec3 const& positionB);
double				GetDistanceSquared3D(DPVec3 const& positionA, DPVec3 const& positionB);
float				GetDistanceXY3D(Vec3 const& positionA, Vec3 const& positionB);
float				GetDistanceXYSquared3D(Vec3 const& positionA, Vec3 const& positionB);
int					GetTaxicabDistance2D(IntVec2 const& pointA, IntVec2 const& pointB);
int					GetTaxicabDistanceHex2D(IntVec2 const& pointA, IntVec2 const& pointB);
float				GetProjectedLength2D(Vec2 const& vectorToProject, Vec2 const& vectorToProjectOnto);
Vec2 const			GetProjectedOnto2D(Vec2 const& vectorToProject, Vec2 const& vectorToProjectOnto);
float				GetProjectedLength3D(Vec3 const& vectorToProject, Vec3 const& vectorToProjectOnto);
Vec3 const			GetProjectedOnto3D(Vec3 const& vectorToProject, Vec3 const& vectorToProjectOnto);
double				GetProjectedLength3D(DPVec3 const& vectorToProject, DPVec3 const& vectorToProjectOnto);
DPVec3 const		GetProjectedOnto3D(DPVec3 const& vectorToProject, DPVec3 const& vectorToProjectOnto);

//-----------------------------------------------------------------------------------------------
//Geometric query utilities
bool				IsPointInsideDisc2D(Vec2 const& point, Vec2 const& discCenter, float discRadius);
bool				IsPointInsideDisc2D(DPVec2 const& point, DPVec2 const& discCenter, double discRadius);
bool				IsPointInsideSphere3D(Vec3 const& point, Vec3 const& sphereCenter, float sphereRadius);
bool				IsPointInsideAABB2D(Vec2 const& point, AABB2 const& box);
bool				IsPointInsideAABB3D(Vec3 const& point, AABB3 const& box);
bool				IsPointInsideDPAABB3D(DPVec3 const& point, DPAABB3 const& box);
bool				IsPointInsideOBB2D(Vec2 const& point, OBB2 const& box);
bool				IsPointInsideHex3D(Vec3 const& point, Vec3 const& hexCenter, float hexCirumradius);
bool				IsPointInsideCapsule2D(Vec2 const& point, Capsule2 const& capsule);
bool				IsPointInsideCapsule3D(Vec3 const& point, Capsule3 const& capsule);
bool				IsPointInsideOrientedSector2D(Vec2 const& point, Vec2 const& sectorTip, float sectorForwardDegrees, float sectorApertureDegrees, float sectorRadius);
bool				IsPointInsideDirectedSector2D(Vec2 const& point, Vec2 const& sectorTip, Vec2 const& sectorForwardNormal, float sectorApertureDegrees, float sectorRadius);
bool				IsPointInsideCylinder3D(Vec3 const& point, Vec3 const& discCenter, float discRadius, float min, float max);
bool				DoDiscsOverlap(Vec2 const& centerA, float radiusA, Vec2 const& centerB, float radiusB);
bool				DoSpheresOverlap(Vec3 const& centerA, float radiusA, Vec3 const& centerB, float radiusB);
bool				DoSpheresOverlap(DPVec3 const& centerA, double radiusA, DPVec3 const& centerB, double radiusB);
bool				DoAABB3sOverlap(AABB3 const& aabb1, AABB3 const& aabb2);
Vec2				GetNearestPointOnDisc2D(Vec2 const& referencePosition, Vec2 const& discCenter, float discRadius);
DPVec2				GetNearestPointOnDisc2D(DPVec2 const& referencePosition, DPVec2 const& discCenter, double discRadius);
Vec3				GetNearestPointOnSphere3D(Vec3 const& referencePosition, Vec3 const& sphereCenter, float sphereRadius);
Vec2				GetNearestPointOnAABB2D(Vec2 const& referencePosition, AABB2 const& box);
Vec3				GetNearestPointOnAABB3D(Vec3 const& referencePosition, AABB3 const& box);
Vec2				GetNearestPointOnOBB2D(Vec2 const& referencePosition, OBB2 const& box);
Vec3				GetNearestPointOnOBB3D(Vec3 const& referencePosition, OBB3 const& box);
std::vector<Vec3>	GetNearestPointsBetweenLines3D(LineSegment3 const& lineA, LineSegment3 const& lineB);
Vec2				GetNearestPointOnInfiniteLine2D(Vec2 const& referencePosition, LineSegment2 const& lineSegment);
Vec2				GetNearestPointOnLineSegment2D(Vec2 const& referencePosition, LineSegment2 const& lineSegment);
Vec3				GetNearestPointOnLineSegment3D(Vec3 const& referencePosition, LineSegment3 const& lineSegment);
Vec2				GetNearestPointOnCapsule2D(Vec2 const& referencePosition, Capsule2 const& capsule);
Vec3				GetNearestPointOnCapsule3D(Vec3 const& referencePosition, Capsule3 const& capsule);
DPVec3				GetNearestPointOnCapsule3D(DPVec3 const& referencePosition, DPCapsule3 const& capsule);
Vec3				GetNearestPointOnCylinderZ3D(Vec3 const& referencePosition, Cylinder3 const& cylinder);
DPVec3				GetNearestPointOnCylinderZ3D(DPVec3 const& referencePosition, DPCylinder3 const& cylinder);
bool				PushDiscOutOfFixedPoint2D(Vec2& mobileDiscCenter, float discRadius, Vec2 const& fixedPoint);
bool				PushDiscOutOfFixedDisc2D(Vec2& mobileDiscCenter, float mobileDiscRadius, Vec2 const& fixedDiscCenter, float fixedDiscRadius);
bool				PushDiscOutOfFixedDisc2D(DPVec2& mobileDiscCenter, double mobileDiscRadius, DPVec2 const& fixedDiscCenter, double fixedDiscRadius);
bool				PushDiscsOutOfEachOther2D(Vec2& aCenter, float aRadius, Vec2& bCenter, float bRadius);
bool				PushDiscOutOfFixedAABB2D(Vec2& mobileDiscCenter, float discRadius, AABB2 const& fixedBox);
bool				PushDiscOutOfFixedOBB2D(Vec2& mobileDiscCenter, float discRadius, OBB2 const& fixedBox);
bool				PushDiscOutOfFixedOBB3D(Vec3& mobileSphereCenter, float sphereRadius, OBB3 const& fixedBox);
bool				PushDiscOutOfFixedOBB3D(DPVec3& mobileSphereCenter, double sphereRadius, DPOBB3 const& fixedOBB);
bool				PushDiscOutOfFixedCapsule2D(Vec2& mobileDiscCenter, float discRadius, Capsule2 const& fixedCapsule);
bool				PushDiscOutOfMobileCapsule2D(Vec2& mobileDiscCenter, const float& discRadius, const float& discInverseMass, Capsule2& mobileCapsule);
bool				PushSphereOutOfFixedAABB3D(Vec3& mobileSphereCenter, float sphereRadius, AABB3 const& fixedBox);
bool				PushSphereOutOfFixedAABB3D(DPVec3& mobileSphereCenter, double sphereRadius, DPAABB3 const& fixedBox);
bool				PushSphereOutOfFixedSphere3D(Vec3& mobileSphereCenter, float mobileSphereRadius, Vec3 const& fixedSphereCenter, float fixedSphereRadius);
bool				PushSphereOutOfFixedSphere3D(DPVec3& mobileSphereCenter, double mobileSphereRadius, DPVec3 const& fixedSphereCenter, double fixedSphereRadius);
bool				PushSphereOutOfSphere3D(Vec3& sphereCenterA, float sphereRadiusA, Vec3& sphereCenterB, float sphereRadiusB);
bool				PushSphereOutOfSphere3D(DPVec3& sphereCenterA, double sphereRadiusA, DPVec3& sphereCenterB, double sphereRadiusB);
bool				PushSphereOutOfFixedCylinderZ3D(Vec3& mobileSphereCenter, float mobileSphereRadius, Cylinder3 const& cylinder);
bool				PushSphereOutOfFixedCylinderZ3D(DPVec3& mobileSphereCenter, double mobileSphereRadius, DPCylinder3 const& cylinder);
bool				PushSphereOutOfFixedCylinder3D(Vec3& mobileSphereCenter, float mobileSphereRadius, Cylinder3 const& cylinder);
bool				PushSphereOutOfFixedCylinder3D(DPVec3& mobileSphereCenter, double mobileSphereRadius, DPCylinder3 const& cylinder);
bool				PushSphereOutOfFixedCapsule3D(Vec3& mobileSphereCenter, float mobileSphereRadius, Capsule3 const& capsule);
bool				PushSphereOutOfFixedCapsule3D(DPVec3& mobileSphereCenter, double mobileSphereRadius, DPCapsule3 const& capsule);
bool				PushCapsuleOutOfFixedAABB2D(Capsule2& capsule, AABB2 const& aabb);
bool				PushCapsuleOutOfFixedAABB3D(Capsule3& capsule, AABB3 const& aabb);
bool				PushCapsuleOutOfFixedAABB3D(DPCapsule3& capsule, DPAABB3 const& aabb);
bool				PushCapsuleOutOfFixedOBB3D(Capsule3& capsule, OBB3 const& obb);
bool				PushCapsuleOutOfFixedOBB3D(DPCapsule3& capsule, DPOBB3 const& obb);
bool				PushCapsuleOutOfFixedSphere3D(Capsule3& capsule, Vec3 const& fixedSphereCenter, float fixedSphereRadius);
bool				PushCapsuleOutOfFixedSphere3D(DPCapsule3& capsule, DPVec3 const& fixedSphereCenter, double fixedSphereRadius);
bool				PushCapsuleOutOfFixedCapsule3D(Capsule3& mobileCapsule, Capsule3 const& fixedCapsule);
bool				PushCapsuleOutOfFixedCapsule3D(DPCapsule3& mobileCapsule, DPCapsule3 const& fixedCapsule);
bool				PushCapsuleOutOfCapsule3D(Capsule3& capsuleA, Capsule3& capsuleB);
bool				PushCapsuleOutOfCapsule3D(DPCapsule3& capsuleA, DPCapsule3& capsuleB);
bool				PushCapsuleOutOfCapsule2D(Capsule2& capsuleA, Capsule2& capsuleB);
bool				PushCapsuleOutOfFixedCylinderZ3D(Capsule3& mobileCapsule, Cylinder3 const& fixedCapsule);
bool				PushCapsuleOutOfFixedCylinderZ3D(DPCapsule3& mobileCapsule, DPCylinder3 const& fixedCylinder);
bool				PushCapsuleOutOfFixedCylinder3D(Capsule3& mobileCapsule, Cylinder3 const& fixedCapsule);
bool				PushCapsuleOutOfFixedCylinder3D(DPCapsule3& mobileCapsule, DPCylinder3 const& fixedCylinder);
RaycastResult2D		RaycastVsDisc2D(Vec2 startPos, Vec2 fwdNormal, float maxDist, Vec2 discCenter, float discRadius);
void				BounceDiscsOffEachOther2D(Vec2& positionA, float radiusA, Vec2& velocityA, float elasticityA, Vec2& positionB, float radiusB, Vec2& velocityB, float elasticityB);
void				BounceDiscOffFixedDisc2D(Vec2& mobileCenter, float mobileRadius, Vec2& mobileVelocity, float mobileElasticity, Vec2 fixedCenter, float fixedRadius, float fixedElasticity);
void				BounceDiscOffFixedOBB2D(Vec2& mobileCenter, float mobileRadius, Vec2& mobileVelocity, float mobileElasticity, OBB2 fixedOBB, float fixedElasticity);
void				BounceDiscOffFixedCapsule2D(Vec2& mobileCenter, float mobileRadius, Vec2& mobileVelocity, float mobileElasticity, Capsule2 fixedCapsule, float fixedElasticity);
RaycastResult3D		RaycastVsCylinderZ3D(const Vec3& start, const Vec3& direction, float distance, const Vec2& center, float minZ, float maxZ, float radius);
RaycastResult2D		RaycastVsLineSegment2D(Vec2 startPos, Vec2 fwdNormal, float maxDist, LineSegment2 const& lineSegment);
RaycastResult2D		RaycastVsAABB2D(Vec2 startPos, Vec2 fwdNormal, float maxDist, AABB2 const& aabb);
RaycastResult3D		RaycastVsSphere3D(Vec3 startPos, Mat44 const& rotationMatrix, float maxDist, Vec3 sphereCenter, float sphereRadius);
RaycastResult2D		RaycastVsPlane2D(Vec2 startPos, Vec2 fwdNormal, float maxDist, Plane2D& plane);
RaycastResult2D		RaycastVsConvexHull2D(Vec2 startPos, Vec2 fwdNormal, float maxDist, ConvexHull2D& convexHull);
RaycastResult3D		RaycastVsPlane3D(Vec3 startPos, Vec3 fwdNormal, float maxDist, Plane3D& plane);
RaycastResult3D		RaycastVsAABB3D(Vec3 startPos, Vec3 fwdNormal, float maxDist, AABB3 const& aabb);

//-----------------------------------------------------------------------------------------------
//Transform utilities
void				TransformPosition2D(Vec2& posToTransform, float uniformScale, float rotationDegrees, Vec2 const& translation);
void				TransformPosition2D(Vec2& posToTransform, Vec2 const& iBasis, Vec2 const& jBasis, Vec2 const& translation);
void				TransformPositionXY3D(Vec3& positionToTransform, float scaleXY, float zRotationDegrees, Vec2 const& translationXY);
void				TransformPositionXY3D(Vec3& posToTransform, Vec2 const& iBasis, Vec2 const& jBasis, Vec2 const& translation);
void				TransformPosition3D(Vec3& posToTransform, Mat44 modelMatrix);
void				TransformPosition3D(DPVec3& posToTransform, DPMat44 modelMatrix);

//-----------------------------------------------------------------------------------------------
//Byte Normalization
float				NormalizeByte(unsigned char byteValue);
unsigned char		DenormalizeByte(float zeroToOne);

//-----------------------------------------------------------------------------------------------
//Mat44
Mat44				GetBillboardMatrix(BillboardType billboardType, Mat44 const& cameraMatrix, const Vec3& billboardPosition, const Vec2& billboardScale = Vec2(1.0f, 1.0f));

//-----------------------------------------------------------------------------------------------
//Easing
float				ComputeCubicBezier1D(float A, float B, float C, float D, float t);
float				ComputeQuinticBezier1D(float A, float B, float C, float D, float E, float F, float t);
float				SmoothStart2(float t);
float				SmoothStart3(float t);
float				SmoothStart4(float t);
float				SmoothStart5(float t);
float				SmoothStop2(float t);
float				SmoothStop3(float t);
float				SmoothStop4(float t);
float				SmoothStop5(float t);
float				SmoothStep3(float t);
float				SmoothStep5(float t);
float				Hesitate3(float t);
float				Hesitate5(float t);
float				CustomFunkyEasingFunction(float t);
