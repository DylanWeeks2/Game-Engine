#pragma once
#include "Vec2.hpp"
#include "Vec3.hpp"

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
struct OBB2;
struct Vec4;
struct Mat44;
struct AABB3;

//-----------------------------------------------------------------------------------------------
//Clamp and Lerp
float GetClamped(float value, float minValue, float maxValue);
float GetClampedZeroToOne(float value);
float Interpolate(float start, float end, float fractionTowardEnd);
float GetFractionWithinRange(float value, float rangeStart, float rangeEnd);
float RangeMap(float inValue, float inStart, float inEnd, float outStart, float outEnd);
float RangeMapClamped(float inValue, float inStart, float inEnd, float outStart, float outEnd);
float RoundDownToInt(float value);

//-----------------------------------------------------------------------------------------------
//Angle functions
float ConvertDegreesToRadians(float degrees);
float ConvertRadiansToDegrees(float radians);
float CosDegrees(float degrees);
float SinDegrees(float degrees);
float TanDegrees(float degrees);
float Atan2Degrees(float y, float x);
float GetShortestAngularDispDegrees(float startDegrees, float endDegrees);
float GetTurnedTowardDegrees(float currentDegrees, float goalDegrees, float maxDeltaDegrees);
float GetAngleDegreesBetweenVectors2D(Vec2 const& a, Vec2 const& b);

//-----------------------------------------------------------------------------------------------
//Dot and cross
float DotProduct2D(Vec2 const& a, Vec2 const& b);
float DotProduct3D(Vec3 const& a, Vec3 const& b);
float DotProduct4D(Vec4 const& a, Vec4 const& b);
float CrossProduct2D(Vec2 const& a, Vec2 const& b);
Vec3 CrossProduct3D(Vec3 const& a, Vec3 const& b);

//-----------------------------------------------------------------------------------------------
//Basic 2D and 3D utilities
float GetDistance2D(Vec2 const& positionA, Vec2 const& positionB);
float GetDistanceSquared2D(Vec2 const& positionA, Vec2 const& positionB);
float GetDistance3D(Vec3 const& positionA, Vec3 const& positionB);
float GetDistanceSquared3D(Vec3 const& positionA, Vec3 const& positionB);
float GetDistanceXY3D(Vec3 const& positionA, Vec3 const& positionB);
float GetDistanceXYSquared3D(Vec3 const& positionA, Vec3 const& positionB);
int GetTaxicabDistance2D(IntVec2 const& pointA, IntVec2 const& pointB);
float GetProjectedLength2D(Vec2 const& vectorToProject, Vec2 const& vectorToProjectOnto);
Vec2 const GetProjectedOnto2D(Vec2 const& vectorToProject, Vec2 const& vectorToProjectOnto);
float GetProjectedLength3D(Vec3 const& vectorToProject, Vec3 const& vectorToProjectOnto);
Vec3 const GetProjectedOnto3D(Vec3 const& vectorToProject, Vec3 const& vectorToProjectOnto);

//-----------------------------------------------------------------------------------------------
//Geometric query utilities
bool IsPointInsideDisc2D(Vec2 const& point, Vec2 const& discCenter, float discRadius);
bool IsPointInsideSphere3D(Vec3 const& point, Vec3 const& sphereCenter, float sphereRadius);
bool IsPointInsideAABB2D(Vec2 const& point, AABB2 const& box);
bool IsPointInsideAABB3D(Vec3 const& point, AABB3 const& box);
bool IsPointInsideOBB2D(Vec2 const& point, OBB2 const& box);
bool IsPointInsideHex3D(Vec3 const& point, Vec3 const& hexCenter, float hexCirumradius);
bool IsPointInsideCapsule2D(Vec2 const& point, Capsule2 const& capsule);
bool IsPointInsideCapsule3D(Vec3 const& point, Capsule3 const& capsule);
bool IsPointInsideOrientedSector2D(Vec2 const& point, Vec2 const& sectorTip, float sectorForwardDegrees, float sectorApertureDegrees, float sectorRadius);
bool IsPointInsideDirectedSector2D(Vec2 const& point, Vec2 const& sectorTip, Vec2 const& sectorForwardNormal, float sectorApertureDegrees, float sectorRadius);
bool IsPointInsideCylinder3D(Vec3 const& point, Vec3 const& discCenter, float discRadius, float min, float max);
bool DoDiscsOverlap(Vec2 const& centerA, float radiusA, Vec2 const& centerB, float radiusB);
bool DoSpheresOverlap(Vec3 const& centerA, float radiusA, Vec3 const& centerB, float radiusB);
Vec2 GetNearestPointOnDisc2D(Vec2 const& referencePosition, Vec2 const& discCenter, float discRadius);
Vec2 GetNearestPointOnAABB2D(Vec2 const& referencePosition, AABB2 const& box);
Vec3 GetNearestPointOnAABB3D(Vec3 const& referencePosition, AABB3 const& box);
Vec2 GetNearestPointOnOBB2D(Vec2 const& referencePosition, OBB2 const& box);
Vec2 GetNearestPointOnInfiniteLine2D(Vec2 const& referencePosition, LineSegment2 const& lineSegment);
Vec2 GetNearestPointOnLineSegment2D(Vec2 const& referencePosition, LineSegment2 const& lineSegment);
Vec3 GetNearestPointOnLineSegment3D(Vec3 const& referencePosition, LineSegment3 const& lineSegment);
Vec2 GetNearestPointOnCapsule2D(Vec2 const& referencePosition, Capsule2 const& capsule);
Vec3 GetNearestPointOnCapsule3D(Vec3 const& referencePosition, Capsule3 const& capsule);
bool PushDiscOutOfFixedPoint2D(Vec2& mobileDiscCenter, float discRadius, Vec2 const& fixedPoint);
bool PushDiscOutOfFixedDisc2D(Vec2& mobileDiscCenter, float mobileDiscRadius, Vec2 const& fixedDiscCenter, float fixedDiscRadius);
bool PushDiscsOutOfEachOther2D(Vec2& aCenter, float aRadius, Vec2& bCenter, float bRadius);
bool PushDiscOutOfFixedAABB2D(Vec2& mobileDiscCenter, float discRadius, AABB2 const& fixedBox);
bool PushDiscOutOfFixedOBB2D(Vec2& mobileDiscCenter, float discRadius, OBB2 const& fixedBox);
bool PushDiscOutOfFixedCapsule2D(Vec2& mobileDiscCenter, float discRadius, Capsule2 const& fixedCapsule);
bool PushDiscOutOfMobileCapsule2D(Vec2& mobileDiscCenter, const float& discRadius, const float& discInverseMass, Capsule2& mobileCapsule);
bool PushSphereOutOfFixedAABB3D(Vec3& mobileSphereCenter, float sphereRadius, AABB3 const& fixedBox);
bool PushSphereOutOfFixedSphere3D(Vec3& mobileSphereCenter, float mobileSphereRadius, Vec3 const& fixedSphereCenter, float fixedSphereRadius);
RaycastResult2D RaycastVsDisc2D(Vec2 startPos, Vec2 fwdNormal, float maxDist, Vec2 discCenter, float discRadius);
void BounceDiscsOffEachOther2D(Vec2& positionA, float radiusA, Vec2& velocityA, float elasticityA, Vec2& positionB, float radiusB, Vec2& velocityB, float elasticityB);
void BounceDiscOffFixedDisc2D(Vec2& mobileCenter, float mobileRadius, Vec2& mobileVelocity, float mobileElasticity, Vec2 fixedCenter, float fixedRadius, float fixedElasticity);
void BounceDiscOffFixedOBB2D(Vec2& mobileCenter, float mobileRadius, Vec2& mobileVelocity, float mobileElasticity, OBB2 fixedOBB, float fixedElasticity);
void BounceDiscOffFixedCapsule2D(Vec2& mobileCenter, float mobileRadius, Vec2& mobileVelocity, float mobileElasticity, Capsule2 fixedCapsule, float fixedElasticity);
RaycastResult3D RaycastVsCylinderZ3D(const Vec3& start, const Vec3& direction, float distance, const Vec2& center, float minZ, float maxZ, float radius);
RaycastResult2D RaycastVsLineSegment2D(Vec2 startPos, Vec2 fwdNormal, float maxDist, LineSegment2 const& lineSegment);
RaycastResult2D RaycastVsAABB2D(Vec2 startPos, Vec2 fwdNormal, float maxDist, AABB2 const& aabb);
RaycastResult3D RaycastVsSphere3D(Vec3 startPos, Mat44 const& rotationMatrix, float maxDist, Vec3 sphereCenter, float sphereRadius);

//-----------------------------------------------------------------------------------------------
//Transform utilities
void TransformPosition2D(Vec2& posToTransform, float uniformScale, float rotationDegrees, Vec2 const& translation);
void TransformPosition2D(Vec2& posToTransform, Vec2 const& iBasis, Vec2 const& jBasis, Vec2 const& translation);
void TransformPositionXY3D(Vec3& positionToTransform, float scaleXY, float zRotationDegrees, Vec2 const& translationXY);
void TransformPositionXY3D(Vec3& posToTransform, Vec2 const& iBasis, Vec2 const& jBasis, Vec2 const& translation);
void TransformPosition3D(Vec3& posToTransform, Mat44 modelMatrix);

//-----------------------------------------------------------------------------------------------
//Byte Normalization
float NormalizeByte(unsigned char byteValue);
unsigned char DenormalizeByte(float zeroToOne);

//-----------------------------------------------------------------------------------------------
//Mat44
Mat44 GetBillboardMatrix(BillboardType billboardType, Mat44 const& cameraMatrix, const Vec3& billboardPosition, const Vec2& billboardScale = Vec2(1.0f, 1.0f));

//-----------------------------------------------------------------------------------------------
//Easing
float ComputeCubicBezier1D(float A, float B, float C, float D, float t);
float ComputeQuinticBezier1D(float A, float B, float C, float D, float E, float F, float t);
float SmoothStart2(float t);
float SmoothStart3(float t);
float SmoothStart4(float t);
float SmoothStart5(float t);
float SmoothStop2(float t);
float SmoothStop3(float t);
float SmoothStop4(float t);
float SmoothStop5(float t);
float SmoothStep3(float t);
float SmoothStep5(float t);
float Hesitate3(float t);
float Hesitate5(float t);
float CustomFunkyEasingFunction(float t);
