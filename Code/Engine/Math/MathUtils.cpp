#include "MathUtils.hpp"
#include "Vec2.hpp"
#include "DPVec2.hpp"
#include "Vec3.hpp"
#include "IntVec2.hpp"
#include "AABB2.hpp"
#include "AABB3.hpp"
#include "DPAABB3.hpp"
#include "LineSegment2.hpp"
#include "LineSegment3.hpp"
#include "DPLineSegment3.hpp"
#include "Capsule2.hpp"
#include "Capsule3.hpp"
#include "DPCapsule3.hpp"
#include "OBB2.hpp"
#include "OBB3.hpp"
#include "DPOBB3.hpp"
#include "Vec4.hpp"
#include "Mat44.hpp"
#include "DPMat44.hpp"
#include "Cylinder3.hpp"
#include "Plane2D.hpp"
#include "Plane3D.hpp"
#include "ConvexHull2D.hpp"
#include "Engine/Core/ErrorWarningAssert.hpp"
#include <vector>
#include <limits>
#define _USE_MATH_DEFINES
#include <math.h>
#include "Engine/Core/DebugRender.hpp"

//-----------------------------------------------------------------------------------------------
float ConvertDegreesToRadians(float degrees)
{
	return degrees * float((M_PI / 180.0f));
}

//-----------------------------------------------------------------------------------------------
float ConvertRadiansToDegrees(float radians)
{
	return radians * float((180.0f / M_PI));
}

//-----------------------------------------------------------------------------------------------
double ConvertDegreesToRadians(double degrees)
{
	return degrees * double((M_PI / 180.0));
}

//-----------------------------------------------------------------------------------------------
double ConvertRadiansToDegrees(double radians)
{
	return radians * double((180.0 / M_PI));
}

//-----------------------------------------------------------------------------------------------
float CosDegrees(float degrees)
{
	return cosf(ConvertDegreesToRadians(degrees));
}

//-----------------------------------------------------------------------------------------------
float SinDegrees(float degrees)
{
	return sinf(ConvertDegreesToRadians(degrees));
}

//-----------------------------------------------------------------------------------------------
float TanDegrees(float degrees)
{
	return tanf(ConvertDegreesToRadians(degrees));
}

//-----------------------------------------------------------------------------------------------
double CosDegrees(double degrees)
{
	return cos(ConvertDegreesToRadians(degrees));
}

//-----------------------------------------------------------------------------------------------
double SinDegrees(double degrees)
{
	return sin(ConvertDegreesToRadians(degrees));
}

//-----------------------------------------------------------------------------------------------
double TanDegrees(double degrees)
{
	return tan(ConvertDegreesToRadians(degrees));
}

//-----------------------------------------------------------------------------------------------
float Atan2Degrees(float y, float x)
{
	return ConvertRadiansToDegrees(atan2f(y, x));
}

//-----------------------------------------------------------------------------------------------
double Atan2Degrees(double y, double x)
{
	return ConvertRadiansToDegrees(atan2(y, x));
}

//-----------------------------------------------------------------------------------------------
float GetShortestAngularDispDegrees(float startDegrees, float endDegrees)
{
	float displacement = endDegrees - startDegrees;
	while (displacement > 180.f)
	{
		displacement -= 360.f;
	}

	while (displacement < -180.f)
	{
		displacement += 360.f;
	}

	return displacement;

}

//-----------------------------------------------------------------------------------------------
float GetTurnedTowardDegrees(float currentDegrees, float goalDegrees, float maxDeltaDegrees)
{
	float degrees = GetShortestAngularDispDegrees(currentDegrees, goalDegrees);

	if(fabsf(degrees) < maxDeltaDegrees)
	{
		return goalDegrees;
	}

	if(degrees > 0)
	{
		return currentDegrees + maxDeltaDegrees;
	}
	else
	{
		return currentDegrees - maxDeltaDegrees;
	}

}

//-----------------------------------------------------------------------------------------------
float GetAngleDegreesBetweenVectors2D(Vec2 const& a, Vec2 const& b)
{
	Vec2 newA = a / a.GetLength();
	Vec2 newB = b / b.GetLength();
	float dot = DotProduct2D(newA, newB);
	if (dot >= 1.0f)
	{
		return 0.0f;
	}

	return ConvertRadiansToDegrees(acosf(dot));
}

//-----------------------------------------------------------------------------------------------
float GetAngleDegreesBetweenVectors3D(Vec3 const& a, Vec3 const& b)
{
	Vec3 newA = a / a.GetLength();
	Vec3 newB = b / b.GetLength();
	float dot = DotProduct3D(newA, newB);
	if (dot >= 1.0f)
	{
		return 0.0f;
	}

	return ConvertRadiansToDegrees(acosf(dot));
}

//-----------------------------------------------------------------------------------------------
float DotProduct2D(Vec2 const& a, Vec2 const& b)
{
	return (a.x * b.x) + (a.y * b.y);
}

//-----------------------------------------------------------------------------------------------
double DotProduct2D(DPVec2 const& a, DPVec2 const& b)
{
	return (a.x * b.x) + (a.y * b.y);
}

//-----------------------------------------------------------------------------------------------
float DotProduct3D(Vec3 const& a, Vec3 const& b)
{
	return (a.x * b.x) + (a.y * b.y) + (a.z * b.z);
}

//-----------------------------------------------------------------------------------------------
double DotProduct3D(DPVec3 const& a, DPVec3 const& b)
{
	return (a.x * b.x) + (a.y * b.y) + (a.z * b.z);
}

//-----------------------------------------------------------------------------------------------
float DotProduct4D(Vec4 const& a, Vec4 const& b)
{
	return (a.x * b.x) + (a.y * b.y) + (a.z * b.z) + (a.w * b.w);
}

//-----------------------------------------------------------------------------------------------
float CrossProduct2D(Vec2 const& a, Vec2 const& b)
{
	return (a.x * b.y - a.y * b.x);
}

//-----------------------------------------------------------------------------------------------
Vec3 CrossProduct3D(Vec3 const& a, Vec3 const& b)
{
	return Vec3((a.y * b.z - a.z * b.y), (a.z * b.x - a.x * b.z), (a.x * b.y - a.y * b.x));
}

//-----------------------------------------------------------------------------------------------
DPVec3 CrossProduct3D(DPVec3 const& a, DPVec3 const& b)
{
	return DPVec3((a.y * b.z - a.z * b.y), (a.z * b.x - a.x * b.z), (a.x * b.y - a.y * b.x));
}

//-----------------------------------------------------------------------------------------------
float GetDistance2D(Vec2 const& positionA, Vec2 const& positionB)
{
	float x = positionA.x - positionB.x;
	float y = positionA.y - positionB.y;
	float xSquared = x * x;
	float ySquared = y * y;
	float sqrtValue = xSquared + ySquared;
	return sqrtf(sqrtValue);
}

//-----------------------------------------------------------------------------------------------
float GetDistanceSquared2D(Vec2 const& positionA, Vec2 const& positionB)
{
	float x = positionA.x - positionB.x;
	float y = positionA.y - positionB.y;
	float xSquared = x * x;
	float ySquared = y * y;
	float lengthSquared = xSquared + ySquared;
	return lengthSquared;
}

//-----------------------------------------------------------------------------------------------
float GetDistance3D(Vec3 const& positionA, Vec3 const& positionB)
{
	float x = positionA.x - positionB.x;
	float y = positionA.y - positionB.y;
	float z = positionA.z - positionB.z;
	float xSquared = x * x;
	float ySquared = y * y;
	float zSquared = z * z;
	float sqrtValue = xSquared + ySquared + zSquared;
	return sqrtf(sqrtValue);
}

//-----------------------------------------------------------------------------------------------
float GetDistanceSquared3D(Vec3 const& positionA, Vec3 const& positionB)
{
	float x = positionA.x - positionB.x;
	float y = positionA.y - positionB.y;
	float z = positionA.z - positionB.z;
	float xSquared = x * x;
	float ySquared = y * y;
	float zSquared = z * z;
	float lengthSquared = xSquared + ySquared + zSquared;
	return lengthSquared;
}

//-----------------------------------------------------------------------------------------------
double GetDistanceSquared3D(DPVec3 const& positionA, DPVec3 const& positionB)
{
	double x = positionA.x - positionB.x;
	double y = positionA.y - positionB.y;
	double z = positionA.z - positionB.z;
	double xSquared = x * x;
	double ySquared = y * y;
	double zSquared = z * z;
	double lengthSquared = xSquared + ySquared + zSquared;
	return lengthSquared;
}

//-----------------------------------------------------------------------------------------------
float GetDistanceXY3D(Vec3 const& positionA, Vec3 const& positionB)
{
	float x = positionA.x - positionB.x;
	float y = positionA.y - positionB.y;
	float xSquared = x * x;
	float ySquared = y * y;
	float sqrtValue = xSquared + ySquared;
	return sqrtf(sqrtValue);
}

//-----------------------------------------------------------------------------------------------
float GetDistanceXYSquared3D(Vec3 const& positionA, Vec3 const& positionB)
{
	float x = positionA.x - positionB.x;
	float y = positionA.y - positionB.y;
	float xSquared = x * x;
	float ySquared = y * y;
	float sqrtValue = xSquared + ySquared;
	float squareRoot = sqrtf(sqrtValue);
	return squareRoot * squareRoot;
}

//-----------------------------------------------------------------------------------------------
int GetTaxicabDistance2D(IntVec2 const& pointA, IntVec2 const& pointB)
{
	return abs(pointB.x - pointA.x) + abs(pointB.y - pointA.y);
}

//-----------------------------------------------------------------------------------------------
int GetTaxicabDistanceHex2D(IntVec2 const& pointA, IntVec2 const& pointB)
{
	return (abs(pointB.x - pointA.x) + abs(pointA.x + pointA.y - pointB.x - pointB.y) + abs(pointA.y - pointB.y)) / 2;
}

//-----------------------------------------------------------------------------------------------
float GetProjectedLength2D(Vec2 const& vectorToProject, Vec2 const& vectorToProjectOnto)
{
	return DotProduct2D(vectorToProject, vectorToProjectOnto.GetNormalized());
}

//-----------------------------------------------------------------------------------------------
Vec2 const GetProjectedOnto2D(Vec2 const& vectorToProject, Vec2 const& vectorToProjectOnto)
{
	float projectedLength = GetProjectedLength2D(vectorToProject, vectorToProjectOnto);
	return projectedLength * vectorToProjectOnto.GetNormalized();
}

//-----------------------------------------------------------------------------------------------
float GetProjectedLength3D(Vec3 const& vectorToProject, Vec3 const& vectorToProjectOnto)
{
	return DotProduct3D(vectorToProject, vectorToProjectOnto.GetNormalized());
}

//-----------------------------------------------------------------------------------------------
double GetProjectedLength3D(DPVec3 const& vectorToProject, DPVec3 const& vectorToProjectOnto)
{
	return DotProduct3D(vectorToProject, vectorToProjectOnto.GetNormalized());
}

//-----------------------------------------------------------------------------------------------
Vec3 const GetProjectedOnto3D(Vec3 const& vectorToProject, Vec3 const& vectorToProjectOnto)
{
	float projectedLength = GetProjectedLength3D(vectorToProject, vectorToProjectOnto);
	return projectedLength * vectorToProjectOnto.GetNormalized();
}

//-----------------------------------------------------------------------------------------------
DPVec3 const GetProjectedOnto3D(DPVec3 const& vectorToProject, DPVec3 const& vectorToProjectOnto)
{
	double projectedLength = GetProjectedLength3D(vectorToProject, vectorToProjectOnto);
	return projectedLength * vectorToProjectOnto.GetNormalized();
}

//-----------------------------------------------------------------------------------------------
bool IsPointInsideDisc2D(Vec2 const& point, Vec2 const& discCenter, float discRadius)
{
	Vec2 difference = (discCenter - point);
	float distance = difference.GetLength();
	if (discRadius > distance)
	{
		return true;
	}
	return false;
}

//-----------------------------------------------------------------------------------------------
bool IsPointInsideDisc2D(DPVec2 const& point, DPVec2 const& discCenter, double discRadius)
{
	DPVec2 difference = (discCenter - point);
	double distance = difference.GetLength();
	if (discRadius > distance)
	{
		return true;
	}
	return false;
}


//-----------------------------------------------------------------------------------------------
bool IsPointInsideSphere3D(Vec3 const& point, Vec3 const& sphereCenter, float sphereRadius)
{
	Vec3 difference = (sphereCenter - point);
	float distance = difference.GetLength();
	if (sphereRadius > distance)
	{
		return true;
	}
	return false;
}

//-----------------------------------------------------------------------------------------------
bool IsPointInsideAABB2D(Vec2 const& point, AABB2 const& box)
{
	return box.IsPointInside(point);
}

//-----------------------------------------------------------------------------------------------
bool IsPointInsideAABB3D(Vec3 const& point, AABB3 const& box)
{
	return box.IsPointInside(point);
}

//-----------------------------------------------------------------------------------------------
bool IsPointInsideDPAABB3D(DPVec3 const& point, DPAABB3 const& box)
{
	return box.IsPointInside(point);
}

//-----------------------------------------------------------------------------------------------
bool IsPointInsideOBB2D(Vec2 const& point, OBB2 const& box)
{
	Vec2 displacement = point - box.m_center;
	float displacementIBasis = DotProduct2D(displacement, box.m_iBasisNormal);
	float displacementJBasis = DotProduct2D(displacement, box.m_iBasisNormal.GetRotated90Degrees());

	if (fabsf(displacementIBasis) >= box.m_halfDimensions.x)
	{
		return false;
	}


	if (fabsf(displacementJBasis) >= box.m_halfDimensions.y)
	{
		return false;
	}

	return true;
}

//-----------------------------------------------------------------------------------------------
bool IsPointInsideHex3D(Vec3 const& point, Vec3 const& hexCenter, float hexCirumradius)
{
	float degreesPerDebrisSide = 360.0f / static_cast<float>(6);
	std::vector<Vec3> localVertPositions;
	localVertPositions.resize(6);

	for (int sideIndex = 0; sideIndex < 6; sideIndex++)
	{
		float degrees = degreesPerDebrisSide * static_cast<float>(sideIndex);

		localVertPositions[sideIndex].x = hexCenter.x + hexCirumradius * CosDegrees(degrees);
		localVertPositions[sideIndex].y = hexCenter.y + hexCirumradius * SinDegrees(degrees);
	}

	Vec3 edge1 = localVertPositions[1] - localVertPositions[0];
	Vec3 edge2 = localVertPositions[2] - localVertPositions[1];
	Vec3 edge3 = localVertPositions[3] - localVertPositions[2];
	Vec3 edge4 = localVertPositions[4] - localVertPositions[3];
	Vec3 edge5 = localVertPositions[5] - localVertPositions[4];
	Vec3 edge6 = localVertPositions[0] - localVertPositions[5];
	Vec3 displacementToPointFromV1 = point - localVertPositions[0];
	Vec3 displacementToPointFromV2 = point - localVertPositions[1];
	Vec3 displacementToPointFromV3 = point - localVertPositions[2];
	Vec3 displacementToPointFromV4 = point - localVertPositions[3];
	Vec3 displacementToPointFromV5 = point - localVertPositions[4];
	Vec3 displacementToPointFromV6 = point - localVertPositions[5];
	Vec3 normal1 = CrossProduct3D(edge1, displacementToPointFromV1);
	Vec3 normal2 = CrossProduct3D(edge2, displacementToPointFromV2);
	Vec3 normal3 = CrossProduct3D(edge3, displacementToPointFromV3);
	Vec3 normal4 = CrossProduct3D(edge4, displacementToPointFromV4);
	Vec3 normal5 = CrossProduct3D(edge5, displacementToPointFromV5);
	Vec3 normal6 = CrossProduct3D(edge6, displacementToPointFromV6);

	if (normal1.z > 0.0f && normal2.z > 0.0f && normal3.z > 0.0f
		&& normal4.z > 0.0f && normal5.z > 0.0f && normal6.z > 0.0f)
	{
		return true;
	}
	return false; //IsPointInsideDisc2D(point, hexCenter, hexCirumradius);
}

//-----------------------------------------------------------------------------------------------
bool IsPointInsideCapsule2D(Vec2 const& point, Capsule2 const& capsule)
{
	Vec2 nearestPointBone = GetNearestPointOnLineSegment2D(point, capsule.m_bone);
	Vec2 displacementToNearestPointBone = point - nearestPointBone;
	float distance = displacementToNearestPointBone.GetLength();
	if (distance < capsule.m_radius)
	{
		return true;
	}

	return false;
}

//-----------------------------------------------------------------------------------------------
bool IsPointInsideCapsule3D(Vec3 const& point, Capsule3 const& capsule)
{
	LineSegment3 bone(capsule.m_bone.m_start, capsule.m_bone.m_end);
	Vec3 nearestPointBone = GetNearestPointOnLineSegment3D(point, bone);
	Vec3 displacementToNearestPointBone = point - nearestPointBone;
	float distance = displacementToNearestPointBone.GetLength();
	if (distance < capsule.m_radius)
	{
		return true;
	}

	return false;
}

//-----------------------------------------------------------------------------------------------
bool IsPointInsideOrientedSector2D(Vec2 const& point, Vec2 const& sectorTip, float sectorForwardDegrees, float sectorApertureDegrees, float sectorRadius)
{
	Vec2 distance = point - sectorTip;
	if (distance.GetLength() > sectorRadius)
	{
		return false;
	}

	float degreesOffCenter = GetAngleDegreesBetweenVectors2D(distance, Vec2(CosDegrees(sectorForwardDegrees), SinDegrees(sectorForwardDegrees)));
	if (degreesOffCenter <= sectorApertureDegrees * 0.5f)
	{
		return true;
	}

	return false;
}

//-----------------------------------------------------------------------------------------------
bool IsPointInsideDirectedSector2D(Vec2 const& point, Vec2 const& sectorTip, Vec2 const& sectorForwardNormal, float sectorApertureDegrees, float sectorRadius)
{
	Vec2 distance = point - sectorTip;
	if (distance.GetLength() > sectorRadius)
	{
		return false;
	}

	float degreesOffCenter = GetAngleDegreesBetweenVectors2D(distance, sectorForwardNormal);
	if (degreesOffCenter <= sectorApertureDegrees * 0.5f)
	{
		return true;
	}

	return false;
}

//-----------------------------------------------------------------------------------------------
bool IsPointInsideCylinder3D(Vec3 const& point, Vec3 const& center, float radius, float min, float max)
{
	if (IsPointInsideDisc2D(Vec2(point.x, point.y), Vec2(center.x, center.y), radius))
	{
		if (point.z < max && point.z > min)
		{
			return true;
		}
	}

	return false;
}

//-----------------------------------------------------------------------------------------------
bool DoDiscsOverlap(Vec2 const& centerA, float radiusA, Vec2 const& centerB, float radiusB)
{
	if (GetDistanceSquared2D(centerA, centerB) < (radiusA + radiusB) * (radiusA + radiusB))
	{
		return true;
	}

	return false;
}

//-----------------------------------------------------------------------------------------------
bool DoSpheresOverlap(Vec3 const& centerA, float radiusA, Vec3 const& centerB, float radiusB)
{
	if (GetDistanceSquared3D(centerA, centerB) < (radiusA + radiusB) * (radiusA + radiusB) )
	{
		return true;
	}

	return false;
}

//-----------------------------------------------------------------------------------------------
bool DoSpheresOverlap(DPVec3 const& centerA, double radiusA, DPVec3 const& centerB, double radiusB)
{
	if (GetDistanceSquared3D(centerA, centerB) < (radiusA + radiusB) * (radiusA + radiusB))
	{
		return true;
	}

	return false;
}

//-----------------------------------------------------------------------------------------------
bool DoAABB3sOverlap(AABB3 const& aabb1, AABB3 const& aabb2)
{
	bool returnValue = 
		aabb1.m_mins.x <= aabb2.m_maxs.x &&
		aabb1.m_maxs.x >= aabb2.m_mins.x &&
		aabb1.m_mins.y <= aabb2.m_maxs.y &&
		aabb1.m_maxs.y >= aabb2.m_mins.y &&
		aabb1.m_mins.z <= aabb2.m_maxs.z &&
		aabb1.m_maxs.z >= aabb2.m_mins.z;

	return returnValue;
}

//-----------------------------------------------------------------------------------------------
Vec2 GetNearestPointOnDisc2D(Vec2 const& referencePosition, Vec2 const& discCenter, float discRadius)
{
	Vec2 distance = referencePosition - discCenter;
	distance.ClampLength(discRadius);
	return discCenter + distance;
}

//-----------------------------------------------------------------------------------------------
DPVec2 GetNearestPointOnDisc2D(DPVec2 const& referencePosition, DPVec2 const& discCenter, double discRadius)
{
	DPVec2 distance = referencePosition - discCenter;
	distance.ClampLength(discRadius);
	return discCenter + distance;
}

//-----------------------------------------------------------------------------------------------
Vec3 GetNearestPointOnSphere3D(Vec3 const& referencePosition, Vec3 const& sphereCenter, float sphereRadius)
{
	Vec3 distance = referencePosition - sphereCenter;
	distance.ClampLength(sphereRadius);
	return sphereCenter + distance;
}

//-----------------------------------------------------------------------------------------------
Vec2 GetNearestPointOnAABB2D(Vec2 const& referencePosition, AABB2 const& box)
{
	return box.GetNearestPoint(referencePosition);
}

//-----------------------------------------------------------------------------------------------
Vec3 GetNearestPointOnAABB3D(Vec3 const& referencePosition, AABB3 const& box)
{
	return box.GetNearestPoint(referencePosition);
}

//-----------------------------------------------------------------------------------------------
Vec2 GetNearestPointOnOBB2D(Vec2 const& referencePosition, OBB2 const& box)
{
	Vec2 JBasisNormal = box.m_iBasisNormal.GetRotated90Degrees();
	Vec2 displacement = referencePosition - box.m_center;
	float displacementIBasis = DotProduct2D(displacement, box.m_iBasisNormal);
	float displacementJBasis = DotProduct2D(displacement, JBasisNormal);
	float nearestPointIBasis = GetClamped(displacementIBasis, -box.m_halfDimensions.x, box.m_halfDimensions.x);
	float nearestPointJBasis = GetClamped(displacementJBasis, -box.m_halfDimensions.y, box.m_halfDimensions.y);

	Vec2 nearestPoint = box.m_center + (nearestPointIBasis * box.m_iBasisNormal) + (nearestPointJBasis * JBasisNormal);
	return nearestPoint;
}

//-----------------------------------------------------------------------------------------------
Vec3 GetNearestPointOnOBB3D(Vec3 const& referencePosition, OBB3 const& box)
{
	Vec3 displacement = referencePosition - box.m_center;
	float displacementIBasis = DotProduct3D(displacement, box.m_iBasisNormal);
	float displacementJBasis = DotProduct3D(displacement, box.m_jBasisNormal);
	float displacementKBasis = DotProduct3D(displacement, box.m_kBasisNormal);
	float nearestPointIBasis = GetClamped(displacementIBasis, -box.m_halfDimensions.x, box.m_halfDimensions.x);
	float nearestPointJBasis = GetClamped(displacementJBasis, -box.m_halfDimensions.y, box.m_halfDimensions.y);
	float nearestPointKBasis = GetClamped(displacementKBasis, -box.m_halfDimensions.z, box.m_halfDimensions.z);

	Vec3 nearestPoint = box.m_center + (nearestPointIBasis * box.m_iBasisNormal) + 
		(nearestPointJBasis * box.m_jBasisNormal) + (nearestPointKBasis * box.m_kBasisNormal);
	return nearestPoint;
}

//-----------------------------------------------------------------------------------------------
std::vector<Vec3> GetNearestPointsBetweenLines3D(LineSegment3 const& lineA, LineSegment3 const& lineB)
{
	Vec3 nearestPointLineA; 
	Vec3 nearestPointLineB;
	std::vector<Vec3> nearestPoints;
	Vec3 directionA = lineA.m_end - lineA.m_start;
	Vec3 directionB = lineB.m_end - lineB.m_start;

	float b = DotProduct3D(directionA, directionB);
	if (b == 1 || b == -1)
	{
		nearestPointLineA = lineA.m_start;
		nearestPointLineB = lineB.m_start;
		nearestPoints.push_back(nearestPointLineA);
		nearestPoints.push_back(nearestPointLineB);
		return nearestPoints;
	}

	Vec3 r = lineA.m_start - lineB.m_start;
	float a = DotProduct3D(directionA, directionA);
	float c = DotProduct3D(directionA, r);
	float e = DotProduct3D(directionB, directionB);
	float f = DotProduct3D(directionB, r);
	float s = 0.0f;
	float t = 0.0f;
	float denom = a * e - b * b;

	if (denom != 0.0f)
	{
		s = GetClamped((b * f - c * e) / denom, 0.0f, 1.0f);
	}
	else
	{
		s = 0.0f;
	}

	t = (b * s + f) / e;

	if (t < 0.0f) 
	{
		t = 0.0f;
		s = GetClamped(-c / a, 0.0f, 1.0f);
	}
	else if (t > 1.0f) 
	{
		t = 1.0f;
		s = GetClamped((b - c) / a, 0.0f, 1.0f);
	}

	nearestPointLineA = lineA.m_start + s * directionA;
	nearestPointLineB = lineB.m_start + t * directionB;
	nearestPoints.push_back(nearestPointLineA);
	nearestPoints.push_back(nearestPointLineB);
	return nearestPoints;
}

//-----------------------------------------------------------------------------------------------
std::vector<DPVec3> GetNearestPointsBetweenLines3D(DPLineSegment3 const& lineA, DPLineSegment3 const& lineB)
{
	DPVec3 nearestPointLineA;
	DPVec3 nearestPointLineB;
	std::vector<DPVec3> nearestPoints;
	DPVec3 directionA = lineA.m_end - lineA.m_start;
	DPVec3 directionB = lineB.m_end - lineB.m_start;

	double b = DotProduct3D(directionA, directionB);
	if (b == 1 || b == -1)
	{
		nearestPointLineA = lineA.m_start;
		nearestPointLineB = lineB.m_start;
		nearestPoints.push_back(nearestPointLineA);
		nearestPoints.push_back(nearestPointLineB);
		return nearestPoints;
	}

	DPVec3 r = lineA.m_start - lineB.m_start;
	double a = DotProduct3D(directionA, directionA);
	double c = DotProduct3D(directionA, r);
	double e = DotProduct3D(directionB, directionB);
	double f = DotProduct3D(directionB, r);
	double s = 0.0f;
	double t = 0.0f;
	double denom = a * e - b * b;

	if (denom != 0.0f)
	{
		s = GetClamped((b * f - c * e) / denom, 0.0, 1.0);
	}
	else
	{
		s = 0.0f;
	}

	t = (b * s + f) / e;

	if (t < 0.0f)
	{
		t = 0.0f;
		s = GetClamped(-c / a, 0.0, 1.0);
	}
	else if (t > 1.0f)
	{
		t = 1.0f;
		s = GetClamped((b - c) / a, 0.0, 1.0);
	}

	nearestPointLineA = lineA.m_start + s * directionA;
	nearestPointLineB = lineB.m_start + t * directionB;
	nearestPoints.push_back(nearestPointLineA);
	nearestPoints.push_back(nearestPointLineB);
	return nearestPoints;
}

//-----------------------------------------------------------------------------------------------
Vec2 GetNearestPointOnInfiniteLine2D(Vec2 const& referencePosition, LineSegment2 const& lineSegment)
{
	Vec2 startDisplacement = referencePosition - lineSegment.m_start;
	Vec2 endDisplacement = referencePosition - lineSegment.m_end;
	Vec2 lineSegmentDisplacement = lineSegment.m_end - lineSegment.m_start;

	Vec2 nearestPoint = GetProjectedOnto2D(startDisplacement, lineSegmentDisplacement);
	return lineSegment.m_start + nearestPoint;
}

//-----------------------------------------------------------------------------------------------
Vec2 GetNearestPointOnLineSegment2D(Vec2 const& referencePosition, LineSegment2 const& lineSegment)
{
	Vec2 startDisplacement = referencePosition - lineSegment.m_start;
	Vec2 endDisplacement = referencePosition - lineSegment.m_end;
	Vec2 lineSegmentDisplacement = lineSegment.m_end - lineSegment.m_start;

	if (DotProduct2D(lineSegmentDisplacement, startDisplacement) < 0)
	{
		return lineSegment.m_start;
	}

	if (DotProduct2D(lineSegmentDisplacement, endDisplacement) > 0)
	{
		return lineSegment.m_end;
	}

	Vec2 nearestPoint = GetProjectedOnto2D(startDisplacement, lineSegmentDisplacement);
	return lineSegment.m_start + nearestPoint;
}

//-----------------------------------------------------------------------------------------------
Vec3 GetNearestPointOnLineSegment3D(Vec3 const& referencePosition, LineSegment3 const& lineSegment)
{
	Vec3 startDisplacement = referencePosition - lineSegment.m_start;
	Vec3 endDisplacement = referencePosition - lineSegment.m_end;
	Vec3 lineSegmentDisplacement = lineSegment.m_end - lineSegment.m_start;

	if (DotProduct3D(lineSegmentDisplacement, startDisplacement) < 0.0f)
	{
		return lineSegment.m_start;
	}

	if (DotProduct3D(lineSegmentDisplacement, endDisplacement) > 0.0f)
	{
		return lineSegment.m_end;
	}

	Vec3 nearestPoint = GetProjectedOnto3D(startDisplacement, lineSegmentDisplacement);
	return lineSegment.m_start + nearestPoint;
}

//-----------------------------------------------------------------------------------------------
DPVec3 GetNearestPointOnLineSegment3D(DPVec3 const& referencePosition, DPLineSegment3 const& lineSegment)
{
	DPVec3 startDisplacement = referencePosition - lineSegment.m_start;
	DPVec3 endDisplacement = referencePosition - lineSegment.m_end;
	DPVec3 lineSegmentDisplacement = lineSegment.m_end - lineSegment.m_start;

	if (DotProduct3D(lineSegmentDisplacement, startDisplacement) < 0.0f)
	{
		return lineSegment.m_start;
	}

	if (DotProduct3D(lineSegmentDisplacement, endDisplacement) > 0.0f)
	{
		return lineSegment.m_end;
	}

	DPVec3 nearestPoint = GetProjectedOnto3D(startDisplacement, lineSegmentDisplacement);
	return lineSegment.m_start + nearestPoint;
}

//-----------------------------------------------------------------------------------------------
Vec2 GetNearestPointOnCapsule2D(Vec2 const& referencePosition, Capsule2 const& capsule)
{
	Vec2 nearestPointBone = GetNearestPointOnLineSegment2D(referencePosition, capsule.m_bone);
	Vec2 displacementToNearestPointBone = referencePosition - nearestPointBone;
	displacementToNearestPointBone.ClampLength(capsule.m_radius);

	return nearestPointBone + displacementToNearestPointBone;
}

//-----------------------------------------------------------------------------------------------
Vec3 GetNearestPointOnCapsule3D(Vec3 const& referencePosition, Capsule3 const& capsule)
{
	LineSegment3 bone(capsule.m_bone.m_start, capsule.m_bone.m_end);
	Vec3 nearestPointBone = GetNearestPointOnLineSegment3D(referencePosition, bone);
	Vec3 displacementToNearestPointBone = referencePosition - nearestPointBone;
	displacementToNearestPointBone.ClampLength(capsule.m_radius);

	return nearestPointBone + displacementToNearestPointBone;
}

//-----------------------------------------------------------------------------------------------
DPVec3 GetNearestPointOnCapsule3D(DPVec3 const& referencePosition, DPCapsule3 const& capsule)
{
	DPLineSegment3 bone(capsule.m_bone.m_start, capsule.m_bone.m_end);
	DPVec3 nearestPointBone = GetNearestPointOnLineSegment3D(referencePosition, bone);
	DPVec3 displacementToNearestPointBone = referencePosition - nearestPointBone;
	displacementToNearestPointBone.ClampLength(capsule.m_radius);

	return nearestPointBone + displacementToNearestPointBone;
}

//-----------------------------------------------------------------------------------------------
Vec3 GetNearestPointOnCylinderZ3D(Vec3 const& referencePosition, Cylinder3 const& cylinder)
{
	Vec3 nearestPoint;
	Vec2 referencePositionXY = Vec2(referencePosition.x, referencePosition.y);
	Vec2 cylinderCenterXY = Vec2(cylinder.m_start.x, cylinder.m_start.y);
	if (referencePosition.z < cylinder.m_start.z)
	{
		Vec2 nearestPointXY = GetNearestPointOnDisc2D(referencePositionXY, cylinderCenterXY, cylinder.m_radius);
		nearestPoint = Vec3(nearestPointXY.x, nearestPointXY.y, cylinder.m_start.z);
	}
	else if (referencePosition.z > cylinder.m_end.z)
	{
		Vec2 nearestPointXY = GetNearestPointOnDisc2D(referencePositionXY, cylinderCenterXY, cylinder.m_radius);
		nearestPoint = Vec3(nearestPointXY.x, nearestPointXY.y, cylinder.m_end.z);
	}
	else
	{
		Vec2 nearestPointXY = GetNearestPointOnDisc2D(referencePositionXY, cylinderCenterXY, cylinder.m_radius);
		nearestPoint = Vec3(nearestPointXY.x, nearestPointXY.y, referencePosition.z);
	}
	return nearestPoint;
}

//-----------------------------------------------------------------------------------------------
DPVec3 GetNearestPointOnCylinderZ3D(DPVec3 const& referencePosition, DPCylinder3 const& cylinder)
{
	DPVec3 nearestPoint;
	DPVec2 referencePositionXY = DPVec2(referencePosition.x, referencePosition.y);
	DPVec2 cylinderCenterXY = DPVec2(cylinder.m_start.x, cylinder.m_start.y);
	if (referencePosition.z < cylinder.m_start.z)
	{
		DPVec2 nearestPointXY = GetNearestPointOnDisc2D(referencePositionXY, cylinderCenterXY, cylinder.m_radius);
		nearestPoint = DPVec3(nearestPointXY.x, nearestPointXY.y, cylinder.m_start.z);
	}
	else if (referencePosition.z > cylinder.m_end.z)
	{
		DPVec2 nearestPointXY = GetNearestPointOnDisc2D(referencePositionXY, cylinderCenterXY, cylinder.m_radius);
		nearestPoint = DPVec3(nearestPointXY.x, nearestPointXY.y, cylinder.m_end.z);
	}
	else
	{
		DPVec2 nearestPointXY = GetNearestPointOnDisc2D(referencePositionXY, cylinderCenterXY, cylinder.m_radius);
		nearestPoint = DPVec3(nearestPointXY.x, nearestPointXY.y, referencePosition.z);
	}
	return nearestPoint;
}

//-----------------------------------------------------------------------------------------------
bool PushDiscOutOfFixedPoint2D(Vec2& mobileDiscCenter, float discRadius, Vec2 const& fixedPoint)
{
	Vec2 difference = mobileDiscCenter - fixedPoint;
	float distance = difference.GetLength();
	if (discRadius >= distance)
	{
		float offset = discRadius - distance;
		difference.SetLength(offset);
		mobileDiscCenter += difference;
		return true;
	}
	return false;
}

//-----------------------------------------------------------------------------------------------
bool PushDiscOutOfFixedDisc2D(Vec2& mobileDiscCenter, float mobileDiscRadius, Vec2 const& fixedDiscCenter, float fixedDiscRadius)
{
	Vec2 difference = (mobileDiscCenter - fixedDiscCenter);
	float distance = difference.GetLength();
	if (fixedDiscRadius + mobileDiscRadius >= distance)
	{
		float offset = fixedDiscRadius + mobileDiscRadius - distance;
		difference.SetLength(offset);
		mobileDiscCenter += difference;
		return true;
	}

	return false;
}

//-----------------------------------------------------------------------------------------------
bool PushDiscOutOfFixedDisc2D(DPVec2& mobileDiscCenter, double mobileDiscRadius, DPVec2 const& fixedDiscCenter, double fixedDiscRadius)
{
	DPVec2 difference = (mobileDiscCenter - fixedDiscCenter);
	double distanceSquared = difference.GetLengthSquared();
	if ((fixedDiscRadius + mobileDiscRadius) * (fixedDiscRadius + mobileDiscRadius) >= distanceSquared)
	{
		double offset = fixedDiscRadius + mobileDiscRadius - sqrt(distanceSquared);
		difference.SetLength(offset);
		mobileDiscCenter += difference;
		return true;
	}

	return false;
}

//-----------------------------------------------------------------------------------------------
bool PushDiscsOutOfEachOther2D(Vec2& aCenter, float aRadius, Vec2& bCenter, float bRadius)
{
	Vec2 difference = (aCenter - bCenter);
	float distance = difference.GetLength();
	if (aRadius + bRadius >= distance)
	{
		float offset = aRadius + bRadius - distance;
		difference.SetLength(offset * 0.5f);
		aCenter += difference;
		bCenter -= difference;
		return true;
	}

	return false;
}

//-----------------------------------------------------------------------------------------------
bool PushDiscOutOfFixedAABB2D(Vec2& mobileDiscCenter, float discRadius, AABB2 const& fixedBox)
{
	//Bounding Disc Rejection test
	Vec2 nearestPoint = fixedBox.GetNearestPoint(mobileDiscCenter);
	Vec2 differenceFromPointToCenter = mobileDiscCenter - nearestPoint;
	float distance = differenceFromPointToCenter.GetLength();
	if (distance == 0.0f)
	{
		Vec2 halfDimensions = fixedBox.GetDimensions() * 0.5f;
		Vec2 aabbCenter = fixedBox.GetCenter();
		float distanceToTop = (aabbCenter.y + halfDimensions.y) - mobileDiscCenter.y;
		float distanceToBottom = mobileDiscCenter.y - (aabbCenter.y - halfDimensions.y);
		float distanceToRight = (aabbCenter.x + halfDimensions.x) - mobileDiscCenter.x;
		float distanceToLeft = mobileDiscCenter.x - (aabbCenter.x - halfDimensions.x);

		if (distanceToTop < distanceToBottom && distanceToTop < distanceToLeft && distanceToTop < distanceToRight)
		{
			mobileDiscCenter.y += distanceToTop + discRadius;
			return true;
		}
		else if (distanceToBottom < distanceToTop && distanceToBottom < distanceToLeft && distanceToBottom < distanceToRight)
		{
			mobileDiscCenter.y -= distanceToBottom + discRadius;
			return true;
		}
		else if (distanceToRight < distanceToBottom && distanceToRight < distanceToLeft && distanceToRight < distanceToTop)
		{
			mobileDiscCenter.x += distanceToRight + discRadius;
			return true;
		}
		else if (distanceToLeft < distanceToTop && distanceToLeft < distanceToBottom && distanceToLeft < distanceToRight)
		{
			mobileDiscCenter.x -= distanceToLeft + discRadius;
			return true;
		}
		return false;
	}
	else if (distance < discRadius)
	{
		float offset = discRadius - distance;
		differenceFromPointToCenter.SetLength(offset);
		mobileDiscCenter += differenceFromPointToCenter;
		return true;
	}

	return false;
}

//-----------------------------------------------------------------------------------------------
bool PushDiscOutOfFixedOBB2D(Vec2& mobileDiscCenter, float discRadius, OBB2 const& fixedOBB)
{
	//Bounding Disc Rejection test
	float radius = sqrtf((fixedOBB.m_halfDimensions.x * fixedOBB.m_halfDimensions.x) + (fixedOBB.m_halfDimensions.y * fixedOBB.m_halfDimensions.y));
	if (DoDiscsOverlap(mobileDiscCenter, discRadius, fixedOBB.m_center, radius) == false)
	{
		return false;
	}

	//Check if overlapping the OBB2
	Vec2 nearestPoint = GetNearestPointOnOBB2D(mobileDiscCenter, fixedOBB);
	Vec2 differenceFromPointToCenter = mobileDiscCenter - nearestPoint;
	float distance = differenceFromPointToCenter.GetLength();
	if (distance == 0.0f)
	{
		Vec2 iBasis = fixedOBB.m_iBasisNormal;
		Vec2 jBasis = iBasis.GetRotated90Degrees();
		Vec2 displacementToCenter = mobileDiscCenter - fixedOBB.m_center;
		float projectionLengthIBasis = GetProjectedLength2D(displacementToCenter, iBasis);
		float projectionLengthJBasis = GetProjectedLength2D(displacementToCenter, jBasis);
		
		bool isIBasisProjectionNegative = false;
		bool isJBasisProjectionNegative = false;
		if (projectionLengthIBasis < 0.0f)
		{
			projectionLengthIBasis *= -1.0f;
			isIBasisProjectionNegative = true;
		}
		if (projectionLengthJBasis < 0.0f)
		{
			projectionLengthJBasis *= -1.0f;
			isJBasisProjectionNegative = true;
		}

		if (fixedOBB.m_halfDimensions.x - projectionLengthIBasis < fixedOBB.m_halfDimensions.y - projectionLengthJBasis)
		{
			if (isIBasisProjectionNegative)
			{
				mobileDiscCenter += iBasis * (-fixedOBB.m_halfDimensions.x + projectionLengthIBasis - discRadius);
				return true;
			}
			else
			{
				mobileDiscCenter += iBasis * (fixedOBB.m_halfDimensions.x - projectionLengthIBasis + discRadius);
				return true;
			}
		}
		else
		{
			if (isJBasisProjectionNegative)
			{
				mobileDiscCenter += jBasis * (-fixedOBB.m_halfDimensions.y + projectionLengthJBasis - discRadius);
				return true;
			}
			else
			{
				mobileDiscCenter += jBasis * (fixedOBB.m_halfDimensions.y - projectionLengthJBasis + discRadius);
				return true;
			}
		}
	}
	else if (distance < discRadius)
	{
		//Push Logic
		float offset = discRadius - distance;
		differenceFromPointToCenter.SetLength(offset);
		mobileDiscCenter += differenceFromPointToCenter;
		return true;
	}

	return false;
}

//-----------------------------------------------------------------------------------------------
bool PushDiscOutOfFixedOBB3D(Vec3& mobileSphereCenter, float sphereRadius, OBB3 const& fixedOBB)
{
	Mat44 modelMatrix;
	modelMatrix.SetIJKT3D(fixedOBB.m_iBasisNormal, fixedOBB.m_jBasisNormal, fixedOBB.m_kBasisNormal, fixedOBB.m_center);

	AABB3 aabb;
	Vec3 dimensions;
	dimensions.x = fixedOBB.m_halfDimensions.x + fixedOBB.m_halfDimensions.x;
	dimensions.y = fixedOBB.m_halfDimensions.y + fixedOBB.m_halfDimensions.y;
	dimensions.z = fixedOBB.m_halfDimensions.z + fixedOBB.m_halfDimensions.z;
	aabb.SetDimensions(dimensions);

	Vec3 center = Vec3();
	Vec3 displacementToSphere = mobileSphereCenter - fixedOBB.m_center;
	float projectionIBasis = DotProduct3D(displacementToSphere, fixedOBB.m_iBasisNormal);
	float projectionJBasis = DotProduct3D(displacementToSphere, fixedOBB.m_jBasisNormal);
	float projectionKBasis = DotProduct3D(displacementToSphere, fixedOBB.m_kBasisNormal);
	Vec3 localSphereCenter = center + Vec3(1.0f, 0.0f, 0.0f) * projectionIBasis +
		Vec3(0.0f, 1.0f, 0.0f) * projectionJBasis + Vec3(0.0f, 0.0f, 1.0f) * projectionKBasis;

	if (PushSphereOutOfFixedAABB3D(localSphereCenter, sphereRadius, aabb))
	{
		TransformPosition3D(localSphereCenter, modelMatrix);
		mobileSphereCenter = localSphereCenter;
		return true;
	}

	return false;
}

//-----------------------------------------------------------------------------------------------
bool PushDiscOutOfFixedOBB3D(DPVec3& mobileSphereCenter, double sphereRadius, DPOBB3 const& fixedOBB)
{
	DPMat44 modelMatrix;
	modelMatrix.SetIJKT3D(fixedOBB.m_iBasisNormal, fixedOBB.m_jBasisNormal, fixedOBB.m_kBasisNormal, fixedOBB.m_center);

	DPAABB3 aabb;
	DPVec3 dimensions;
	dimensions.x = fixedOBB.m_halfDimensions.x + fixedOBB.m_halfDimensions.x;
	dimensions.y = fixedOBB.m_halfDimensions.y + fixedOBB.m_halfDimensions.y;
	dimensions.z = fixedOBB.m_halfDimensions.z + fixedOBB.m_halfDimensions.z;
	aabb.SetDimensions(dimensions);

	DPVec3 center = DPVec3();
	DPVec3 displacementToSphere = mobileSphereCenter - fixedOBB.m_center;
	double projectionIBasis = DotProduct3D(displacementToSphere, fixedOBB.m_iBasisNormal);
	double projectionJBasis = DotProduct3D(displacementToSphere, fixedOBB.m_jBasisNormal);
	double projectionKBasis = DotProduct3D(displacementToSphere, fixedOBB.m_kBasisNormal);
	DPVec3 localSphereCenter = center + DPVec3(1.0f, 0.0f, 0.0f) * projectionIBasis + DPVec3(0.0f, 1.0f, 0.0f) * projectionJBasis + DPVec3(0.0f, 0.0f, 1.0f) * projectionKBasis;

	if (PushSphereOutOfFixedAABB3D(localSphereCenter, sphereRadius, aabb))
	{
		TransformPosition3D(localSphereCenter, modelMatrix);
		mobileSphereCenter = localSphereCenter;
		return true;
	}

	return false;
}

//-----------------------------------------------------------------------------------------------
bool PushDiscOutOfFixedCapsule2D(Vec2& mobileDiscCenter, float discRadius, Capsule2 const& fixedCapsule)
{
	//Bounding Disc Rejection test
	Vec2 center = (fixedCapsule.m_bone.m_start + fixedCapsule.m_bone.m_end) * 0.5;
	float radius = center.GetLength() + fixedCapsule.m_radius;
	if (DoDiscsOverlap(mobileDiscCenter, discRadius, center, radius) == false)
	{
		return false;
	}

	bool success = false;
	OBB2 obb;
	obb.m_center = center;
	obb.m_halfDimensions = Vec2(fixedCapsule.m_radius, (fixedCapsule.m_bone.m_end - fixedCapsule.m_bone.m_start).GetLength() * 0.5f);
	obb.m_iBasisNormal = (fixedCapsule.m_bone.m_start - fixedCapsule.m_bone.m_end).GetRotatedMinus90Degrees().GetNormalized();
	if (PushDiscOutOfFixedOBB2D(mobileDiscCenter, discRadius, obb))
	{
		success = true;
	}
	if (PushDiscOutOfFixedDisc2D(mobileDiscCenter, discRadius, fixedCapsule.m_bone.m_start, fixedCapsule.m_radius))
	{
		success = true;
	}
	if (PushDiscOutOfFixedDisc2D(mobileDiscCenter, discRadius, fixedCapsule.m_bone.m_end, fixedCapsule.m_radius))
	{
		success = true;
	}

	return success;
}

//-----------------------------------------------------------------------------------------------
bool PushDiscOutOfMobileCapsule2D(Vec2& mobileDiscCenter, const float& discRadius, const float& discInverseMass, Capsule2& mobileCapsule)
{
	//Bounding Disc Rejection test
	Vec2 center = (mobileCapsule.m_bone.m_start + mobileCapsule.m_bone.m_end) * 0.5;
	float radius = center.GetLength() + mobileCapsule.m_radius;
	if (DoDiscsOverlap(mobileDiscCenter, discRadius, center, radius) == false)
	{
		return false;
	}
	
	Vec2 nearestPointOnBone = GetNearestPointOnLineSegment2D(mobileDiscCenter, mobileCapsule.m_bone);
	Vec2 displacement = mobileDiscCenter - nearestPointOnBone;
	float distance = (displacement.GetLength() - (discRadius + mobileCapsule.m_radius));
	if (distance < 0.0f)
	{
		Vec2 gradient = displacement.GetNormalized();
		float discWeightConstant = (discInverseMass / (discInverseMass + mobileCapsule.m_inverseMass));
		float capsuleWeightConstant = (mobileCapsule.m_inverseMass / (discInverseMass + mobileCapsule.m_inverseMass));

		Vec2 deltaDiscLocation = discWeightConstant * distance * gradient;
		Vec2 deltaCapsuleLocation = capsuleWeightConstant * distance * gradient;
		mobileDiscCenter += deltaDiscLocation;
		mobileCapsule.m_bone.m_start -= deltaCapsuleLocation;
		mobileCapsule.m_bone.m_end -= deltaCapsuleLocation;
		return true;
	}
	return false;
}

//-----------------------------------------------------------------------------------------------
bool PushSphereOutOfFixedAABB3D(Vec3& mobileSphereCenter, float sphereRadius, AABB3 const& fixedBox)
{
	Vec3 nearestPoint = fixedBox.GetNearestPoint(mobileSphereCenter);
	if (mobileSphereCenter == nearestPoint)
	{
		Vec3 halfDimensions = fixedBox.GetDimensions() * 0.5f;
		Vec3 aabbCenter = fixedBox.GetCenter();
		float distancePositiveZ = (aabbCenter.z + halfDimensions.z) - mobileSphereCenter.z;
		float distanceNegativeZ = mobileSphereCenter.z - (aabbCenter.z - halfDimensions.z);
		float distancePositiveY = (aabbCenter.y + halfDimensions.y) - mobileSphereCenter.y;
		float distanceNegativeY = mobileSphereCenter.y - (aabbCenter.y - halfDimensions.y);
		float distancePositiveX = (aabbCenter.x + halfDimensions.x) - mobileSphereCenter.x;
		float distanceNegativeX = mobileSphereCenter.x - (aabbCenter.x - halfDimensions.x);

		if (distancePositiveZ < distanceNegativeZ && distancePositiveZ < distanceNegativeY && distancePositiveZ < distanceNegativeX
			&& distancePositiveZ < distancePositiveX && distancePositiveZ < distancePositiveY)
		{
			mobileSphereCenter.z += distancePositiveZ + sphereRadius;
		}
		else if (distanceNegativeZ < distancePositiveZ && distanceNegativeZ < distanceNegativeY && distanceNegativeZ < distanceNegativeX
			&& distanceNegativeZ < distancePositiveX && distanceNegativeZ < distancePositiveY)
		{
			mobileSphereCenter.z -= distanceNegativeZ + sphereRadius;
		}
		else if (distancePositiveY < distancePositiveZ && distancePositiveY < distanceNegativeY && distancePositiveY < distanceNegativeX
			&& distancePositiveY < distancePositiveX && distancePositiveY < distanceNegativeZ)
		{
			mobileSphereCenter.y += distancePositiveY + sphereRadius;
		}
		else if (distanceNegativeY < distancePositiveZ && distanceNegativeY < distancePositiveY && distanceNegativeY < distanceNegativeX
			&& distanceNegativeY < distancePositiveX && distanceNegativeY < distanceNegativeZ)
		{
			mobileSphereCenter.y -= distanceNegativeY + sphereRadius;
		}
		else if (distancePositiveX < distancePositiveZ && distancePositiveX < distancePositiveY && distancePositiveX < distanceNegativeX
			&& distancePositiveX < distanceNegativeY && distancePositiveX < distanceNegativeZ)
		{
			mobileSphereCenter.x += distancePositiveX + sphereRadius;
		}
		else if (distanceNegativeX < distancePositiveZ && distanceNegativeX < distancePositiveY && distanceNegativeX < distancePositiveX
			&& distanceNegativeX < distanceNegativeY && distanceNegativeX < distanceNegativeZ)
		{
			mobileSphereCenter.x -= distanceNegativeX + sphereRadius;
		}
		return true;
	}
	else
	{
		Vec3 displacementFromPointToCenter = mobileSphereCenter - nearestPoint;
		float distSquaredPointToCenter = displacementFromPointToCenter.GetLengthSquared();
		if (distSquaredPointToCenter < sphereRadius * sphereRadius)
		{
			float offset = sphereRadius - sqrtf(distSquaredPointToCenter);
			displacementFromPointToCenter.SetLength(offset);
			mobileSphereCenter += displacementFromPointToCenter;
			return true;
		}
	}

	return false;
}

//-----------------------------------------------------------------------------------------------
bool PushSphereOutOfFixedAABB3D(DPVec3& mobileSphereCenter, double sphereRadius, DPAABB3 const& fixedBox)
{
	DPVec3 nearestPoint = fixedBox.GetNearestPoint(mobileSphereCenter);
	if (mobileSphereCenter == nearestPoint)
	{
		DPVec3 halfDimensions = fixedBox.GetDimensions() * 0.5f;
		DPVec3 aabbCenter = fixedBox.GetCenter();
		double distancePositiveZ = (aabbCenter.z + halfDimensions.z) - mobileSphereCenter.z;
		double distanceNegativeZ = mobileSphereCenter.z - (aabbCenter.z - halfDimensions.z);
		double distancePositiveY = (aabbCenter.y + halfDimensions.y) - mobileSphereCenter.y;
		double distanceNegativeY = mobileSphereCenter.y - (aabbCenter.y - halfDimensions.y);
		double distancePositiveX = (aabbCenter.x + halfDimensions.x) - mobileSphereCenter.x;
		double distanceNegativeX = mobileSphereCenter.x - (aabbCenter.x - halfDimensions.x);

		if (distancePositiveZ < distanceNegativeZ && distancePositiveZ < distanceNegativeY && distancePositiveZ < distanceNegativeX
			&& distancePositiveZ < distancePositiveX && distancePositiveZ < distancePositiveY)
		{
			mobileSphereCenter.z += distancePositiveZ + sphereRadius;
		}
		else if (distanceNegativeZ < distancePositiveZ && distanceNegativeZ < distanceNegativeY && distanceNegativeZ < distanceNegativeX
			&& distanceNegativeZ < distancePositiveX && distanceNegativeZ < distancePositiveY)
		{
			mobileSphereCenter.z -= distanceNegativeZ + sphereRadius;
		}
		else if (distancePositiveY < distancePositiveZ && distancePositiveY < distanceNegativeY && distancePositiveY < distanceNegativeX
			&& distancePositiveY < distancePositiveX && distancePositiveY < distanceNegativeZ)
		{
			mobileSphereCenter.y += distancePositiveY + sphereRadius;
		}
		else if (distanceNegativeY < distancePositiveZ && distanceNegativeY < distancePositiveY && distanceNegativeY < distanceNegativeX
			&& distanceNegativeY < distancePositiveX && distanceNegativeY < distanceNegativeZ)
		{
			mobileSphereCenter.y -= distanceNegativeY + sphereRadius;
		}
		else if (distancePositiveX < distancePositiveZ && distancePositiveX < distancePositiveY && distancePositiveX < distanceNegativeX
			&& distancePositiveX < distanceNegativeY && distancePositiveX < distanceNegativeZ)
		{
			mobileSphereCenter.x += distancePositiveX + sphereRadius;
		}
		else if (distanceNegativeX < distancePositiveZ && distanceNegativeX < distancePositiveY && distanceNegativeX < distancePositiveX
			&& distanceNegativeX < distanceNegativeY && distanceNegativeX < distanceNegativeZ)
		{
			mobileSphereCenter.x -= distanceNegativeX + sphereRadius;
		}
		return true;
	}
	else
	{
		DPVec3 displacementFromPointToCenter = mobileSphereCenter - nearestPoint;
		double distSquaredPointToCenter = displacementFromPointToCenter.GetLengthSquared();
		if (distSquaredPointToCenter < sphereRadius * sphereRadius)
		{
			double offset = sphereRadius - sqrt(distSquaredPointToCenter);
			displacementFromPointToCenter.SetLength(offset);
			mobileSphereCenter += displacementFromPointToCenter;
			return true;
		}
	}

	return false;
}

//-----------------------------------------------------------------------------------------------
bool PushSphereOutOfFixedSphere3D(Vec3& mobileSphereCenter, float mobileSphereRadius, Vec3 const& fixedSphereCenter, float fixedSphereRadius)
{
	Vec3 difference = (mobileSphereCenter - fixedSphereCenter);
	float distanceSquared = difference.GetLengthSquared();
	if ((fixedSphereRadius + mobileSphereRadius) * (fixedSphereRadius + mobileSphereRadius) >= distanceSquared)
	{
		float offset = fixedSphereRadius + mobileSphereRadius - sqrtf(distanceSquared);
		difference.SetLength(offset);
		mobileSphereCenter += difference;
		return true;
	}

	return false;
}

//-----------------------------------------------------------------------------------------------
bool PushSphereOutOfFixedSphere3D(DPVec3& mobileSphereCenter, double mobileSphereRadius, DPVec3 const& fixedSphereCenter, double fixedSphereRadius)
{
	DPVec3 difference = (mobileSphereCenter - fixedSphereCenter);
	double distanceSquared = difference.GetLengthSquared();
	if ((fixedSphereRadius + mobileSphereRadius) * (fixedSphereRadius + mobileSphereRadius) >= distanceSquared)
	{
		double offset = fixedSphereRadius + mobileSphereRadius - sqrt(distanceSquared);
		difference.SetLength(offset);
		mobileSphereCenter += difference;
		return true;
	}

	return false;
}

//-----------------------------------------------------------------------------------------------
bool PushSphereOutOfSphere3D(Vec3& sphereCenterA, float sphereRadiusA, Vec3& sphereCenterB, float sphereRadiusB)
{
	Vec3 difference = (sphereCenterA - sphereCenterB);
	float distanceSquared = difference.GetLengthSquared();
	if ((sphereRadiusA + sphereRadiusB) * (sphereRadiusA + sphereRadiusB) >= distanceSquared)
	{
		float offset = sphereRadiusA + sphereRadiusB - sqrtf(distanceSquared);
		difference.SetLength(offset);
		sphereCenterA += difference * 0.5f;
		sphereCenterB -= difference * 0.5f;
		return true;
	}

	return false;
}

//-----------------------------------------------------------------------------------------------
bool PushSphereOutOfSphere3D(DPVec3& sphereCenterA, double sphereRadiusA, DPVec3& sphereCenterB, double sphereRadiusB)
{
	DPVec3 difference = (sphereCenterA - sphereCenterB);
	double distanceSquared = difference.GetLengthSquared();
	if ((sphereRadiusA + sphereRadiusB) * (sphereRadiusA + sphereRadiusB) >= distanceSquared)
	{
		double offset = sphereRadiusA + sphereRadiusB - sqrt(distanceSquared);
		difference.SetLength(offset);
		sphereCenterA += difference * 0.5;
		sphereCenterB -= difference * 0.5;
		return true;
	}

	return false;
}

//-----------------------------------------------------------------------------------------------
bool PushSphereOutOfFixedCylinderZ3D(Vec3& mobileSphereCenter, float mobileSphereRadius, Cylinder3 const& cylinder)
{
	Vec3 nearestPoint = GetNearestPointOnCylinderZ3D(mobileSphereCenter, cylinder);
	if (mobileSphereCenter == nearestPoint)
	{
		Vec2 displacementFromCylinderCenterXY = Vec2(mobileSphereCenter.x, mobileSphereCenter.y) - Vec2(cylinder.m_start.x, cylinder.m_start.y);
		float distanceToOutside = cylinder.m_radius - (displacementFromCylinderCenterXY).GetLength();
		float distanceToTop = fabsf(mobileSphereCenter.z - cylinder.m_end.z);
		float distanceToBottom = fabsf(mobileSphereCenter.z - cylinder.m_start.z);

		if (distanceToOutside < distanceToTop)
		{
			if (distanceToOutside < distanceToBottom)
			{
				Vec2 cylinderCenterXY = Vec2(cylinder.m_start.x, cylinder.m_start.y);
				Vec2 newCenter = cylinderCenterXY + displacementFromCylinderCenterXY.GetNormalized() * (cylinder.m_radius + mobileSphereRadius);
				mobileSphereCenter.x = newCenter.x;
				mobileSphereCenter.y = newCenter.y;
			}
			else
			{
				mobileSphereCenter.z -= distanceToBottom + mobileSphereRadius;
			}
		}
		else if (distanceToTop < distanceToOutside)
		{
			if (distanceToTop < distanceToBottom)
			{
				mobileSphereCenter.z += distanceToTop + mobileSphereRadius;
			}
			else
			{
				mobileSphereCenter.z -= distanceToBottom + mobileSphereRadius;
			}
		}
		return true;
	}
	else
	{
		Vec3 differenceFromPointToCenter = mobileSphereCenter - nearestPoint;
		float distance = differenceFromPointToCenter.GetLengthSquared();
		if (distance < mobileSphereRadius * mobileSphereRadius)
		{
			float offset = mobileSphereRadius - sqrtf(distance);
			differenceFromPointToCenter.SetLength(offset);
			mobileSphereCenter += differenceFromPointToCenter;
			return true;
		}
	}

	return false;
}

//-----------------------------------------------------------------------------------------------
bool PushSphereOutOfFixedCylinderZ3D(DPVec3& mobileSphereCenter, double mobileSphereRadius, DPCylinder3 const& cylinder)
{
	DPVec3 nearestPoint = GetNearestPointOnCylinderZ3D(mobileSphereCenter, cylinder);
	if (mobileSphereCenter == nearestPoint)
	{
		DPVec2 displacementFromCylinderCenterXY = DPVec2(mobileSphereCenter.x, mobileSphereCenter.y) - DPVec2(cylinder.m_start.x, cylinder.m_start.y);
		double distanceToOutside = cylinder.m_radius - (displacementFromCylinderCenterXY).GetLength();
		double distanceToTop = abs(mobileSphereCenter.z - cylinder.m_end.z);
		double distanceToBottom = abs(mobileSphereCenter.z - cylinder.m_start.z);

		if (distanceToOutside < distanceToTop)
		{
			if (distanceToOutside < distanceToBottom)
			{
				DPVec2 cylinderCenterXY = DPVec2(cylinder.m_start.x, cylinder.m_start.y);
				DPVec2 newCenter = cylinderCenterXY + displacementFromCylinderCenterXY.GetNormalized() * (cylinder.m_radius + mobileSphereRadius);
				mobileSphereCenter.x = newCenter.x;
				mobileSphereCenter.y = newCenter.y;
			}
			else
			{
				mobileSphereCenter.z -= distanceToBottom + mobileSphereRadius;
			}
		}
		else if (distanceToTop < distanceToOutside)
		{
			if (distanceToTop < distanceToBottom)
			{
				mobileSphereCenter.z += distanceToTop + mobileSphereRadius;
			}
			else
			{
				mobileSphereCenter.z -= distanceToBottom + mobileSphereRadius;
			}
		}
		return true;
	}
	else
	{
		DPVec3 differenceFromPointToCenter = mobileSphereCenter - nearestPoint;
		double distance = differenceFromPointToCenter.GetLengthSquared();
		if (distance < mobileSphereRadius * mobileSphereRadius)
		{
			double offset = mobileSphereRadius - sqrt(distance);
			differenceFromPointToCenter.SetLength(offset);
			mobileSphereCenter += differenceFromPointToCenter;
			return true;
		}
	}

	return false;
}

//-----------------------------------------------------------------------------------------------
bool PushSphereOutOfFixedCylinder3D(Vec3& mobileSphereCenter, float mobileSphereRadius, Cylinder3 const& cylinder)
{
	Mat44 modelMatrix;
	modelMatrix.SetIJKT3D(cylinder.m_iBasis, cylinder.m_jBasis, cylinder.m_kBasis, cylinder.m_start);

	float length = (cylinder.m_end - cylinder.m_start).GetLength();
	Cylinder3 localCylinder;
	localCylinder.m_radius = cylinder.m_radius;
	localCylinder.m_start = Vec3();
	localCylinder.m_end = localCylinder.m_start + Vec3(0.0f, 0.0f, length);
	localCylinder.m_iBasis = Vec3(1.0f, 0.0f, 0.0f);
	localCylinder.m_jBasis = Vec3(0.0f, 1.0f, 0.0f);
	localCylinder.m_kBasis = Vec3(0.0f, 0.0f, 1.0f);

	Vec3 displacementToSphere = mobileSphereCenter - cylinder.m_start;
	float projectionIBasis = DotProduct3D(displacementToSphere, cylinder.m_iBasis);
	float projectionJBasis = DotProduct3D(displacementToSphere, cylinder.m_jBasis);
	float projectionKBasis = DotProduct3D(displacementToSphere, cylinder.m_kBasis);
	Vec3 localSphereCenter = localCylinder.m_start + localCylinder.m_iBasis * projectionIBasis +
		localCylinder.m_jBasis * projectionJBasis + localCylinder.m_kBasis * projectionKBasis;

	if (PushSphereOutOfFixedCylinderZ3D(localSphereCenter, mobileSphereRadius, localCylinder))
	{
		TransformPosition3D(localSphereCenter, modelMatrix);
		mobileSphereCenter = localSphereCenter;
		return true;
	}

	return false;
}

//-----------------------------------------------------------------------------------------------
bool PushSphereOutOfFixedCylinder3D(DPVec3& mobileSphereCenter, double mobileSphereRadius, DPCylinder3 const& cylinder)
{
	DPMat44 modelMatrix;
	modelMatrix.SetIJKT3D(cylinder.m_iBasis, cylinder.m_jBasis, cylinder.m_kBasis, cylinder.m_start);

	double length = (cylinder.m_end - cylinder.m_start).GetLength();
	DPCylinder3 localCylinder;
	localCylinder.m_radius = cylinder.m_radius;
	localCylinder.m_start = DPVec3();
	localCylinder.m_end = localCylinder.m_start + DPVec3(0.0f, 0.0f, length);
	localCylinder.m_iBasis = DPVec3(1.0f, 0.0f, 0.0f);
	localCylinder.m_jBasis = DPVec3(0.0f, 1.0f, 0.0f);
	localCylinder.m_kBasis = DPVec3(0.0f, 0.0f, 1.0f);

	DPVec3 displacementToSphere = mobileSphereCenter - cylinder.m_start;
	double projectionIBasis = DotProduct3D(displacementToSphere, cylinder.m_iBasis);
	double projectionJBasis = DotProduct3D(displacementToSphere, cylinder.m_jBasis);
	double projectionKBasis = DotProduct3D(displacementToSphere, cylinder.m_kBasis);
	DPVec3 localSphereCenter = localCylinder.m_start + localCylinder.m_iBasis * projectionIBasis + localCylinder.m_jBasis * projectionJBasis + localCylinder.m_kBasis * projectionKBasis;

	if (PushSphereOutOfFixedCylinderZ3D(localSphereCenter, mobileSphereRadius, localCylinder))
	{
		TransformPosition3D(localSphereCenter, modelMatrix);
		mobileSphereCenter = localSphereCenter;
		return true;
	}

	return false;
}

//-----------------------------------------------------------------------------------------------
bool PushSphereOutOfFixedCapsule3D(Vec3& mobileSphereCenter, float mobileSphereRadius, Capsule3 const& capsule)
{
	LineSegment3 line = LineSegment3(capsule.m_bone.m_start, capsule.m_bone.m_end);
	Vec3 nearestPointBone = GetNearestPointOnLineSegment3D(mobileSphereCenter, line);
	Vec3 differenceFromPointToCenter = mobileSphereCenter - nearestPointBone;
	float distance = differenceFromPointToCenter.GetLengthSquared();
	if (distance < ((mobileSphereRadius + capsule.m_radius) * (mobileSphereRadius + capsule.m_radius)))
	{
		float offset = mobileSphereRadius + capsule.m_radius - sqrtf(distance);
		differenceFromPointToCenter.SetLength(offset);
		mobileSphereCenter += differenceFromPointToCenter;
		return true;
	}

	return false;
}

//-----------------------------------------------------------------------------------------------
bool PushSphereOutOfFixedCapsule3D(DPVec3& mobileSphereCenter, double mobileSphereRadius, DPCapsule3 const& capsule)
{
	DPLineSegment3 line = DPLineSegment3(capsule.m_bone.m_start, capsule.m_bone.m_end);
	DPVec3 nearestPointBone = GetNearestPointOnLineSegment3D(mobileSphereCenter, line);
	DPVec3 differenceFromPointToCenter = mobileSphereCenter - nearestPointBone;
	double distance = differenceFromPointToCenter.GetLengthSquared();
	if (distance < ((mobileSphereRadius + capsule.m_radius) * (mobileSphereRadius + capsule.m_radius)))
	{
		double offset = mobileSphereRadius + capsule.m_radius - sqrt(distance);
		differenceFromPointToCenter.SetLength(offset);
		mobileSphereCenter += differenceFromPointToCenter;
		return true;
	}

	return false;
}

//-----------------------------------------------------------------------------------------------
bool PushCapsuleOutOfFixedAABB2D(Capsule2& capsule, AABB2 const& aabb)
{
	//Do Initial Push
	bool returnValue = false;
	returnValue = PushDiscOutOfFixedAABB2D(capsule.m_bone.m_start, capsule.m_radius, aabb);
	returnValue |= PushDiscOutOfFixedAABB2D(capsule.m_bone.m_end, capsule.m_radius, aabb);
	
	//Calculate intersects and get best intersect
	std::vector<float> tValues;
	float bestTValue = 1000000000000.0f;
	float bestDistanceSquared = 1000000000000.0f;
	Vec2 bestIntersectPoint = Vec2(-1000.0f, -1000.0f);
	Vec2 bestNearestPointAABB = Vec2(-1000.0f, -1000.0f);
	Vec2 capsuleRayStartToEnd = (capsule.m_bone.m_end - capsule.m_bone.m_start);
	float rayYScale = 1.0f / capsuleRayStartToEnd.y;
	float rayXScale = 1.0f / capsuleRayStartToEnd.x;
	tValues.push_back((aabb.m_maxs.y - capsule.m_bone.m_start.y) * rayYScale);
	tValues.push_back((aabb.m_mins.y - capsule.m_bone.m_start.y) * rayYScale);
	tValues.push_back((aabb.m_maxs.x - capsule.m_bone.m_start.x) * rayXScale);
	tValues.push_back((aabb.m_mins.x - capsule.m_bone.m_start.x) * rayXScale);
	for (int tIndex = 0; tIndex < tValues.size(); tIndex++)
	{
		if (tValues[tIndex] >= 0.0f && tValues[tIndex] <= 1.0f)
		{
			Vec2 intersectPoint = capsule.m_bone.m_start + (tValues[tIndex] * capsuleRayStartToEnd);
			Vec2 nearestPointAABB = aabb.GetNearestPoint(intersectPoint);
			float distanceSquared = GetDistanceSquared2D(intersectPoint, nearestPointAABB);
			if (distanceSquared < bestDistanceSquared)
			{
				bestDistanceSquared = distanceSquared;
				bestIntersectPoint = intersectPoint;
				bestNearestPointAABB = nearestPointAABB;
				bestTValue = tValues[tIndex];
			}
		}
	}

	//Out Check
	if (bestTValue < 0.0f || bestTValue > 1.0f)
	{
		return returnValue;
	}	

	//Calculate nearest points and get displacements
	Vec2 nearestPointCapsule = GetNearestPointOnLineSegment2D(bestNearestPointAABB, capsule.m_bone);
	Vec2 originalNearestPointCapsule = nearestPointCapsule;
	PushDiscOutOfFixedAABB2D(nearestPointCapsule, capsule.m_radius, aabb);

	Vec2 capulseDisplacement = nearestPointCapsule - originalNearestPointCapsule;
	capsule.m_bone.m_start += capulseDisplacement;
	capsule.m_bone.m_end += capulseDisplacement;
	return true;
}

//-----------------------------------------------------------------------------------------------
bool PushCapsuleOutOfFixedAABB3D(Capsule3& capsule, AABB3 const& aabb)
{
	//Initial Sphere Pushes
	bool returnValue = false;
	returnValue = PushSphereOutOfFixedAABB3D(capsule.m_bone.m_start, capsule.m_radius, aabb);
	returnValue |= PushSphereOutOfFixedAABB3D(capsule.m_bone.m_end, capsule.m_radius, aabb);

	//Calculate intersects and get best intersect
	std::vector<float> tValues;
	float bestTValue = 1000000000000.0f;
	float bestDistanceSquared = 1000000000000.0f;
	Vec3 bestIntersectPoint = Vec3(-1000.0f, -1000.0f, -1000.0f);
	Vec3 bestNearestPointAABB = Vec3(-1000.0f, -1000.0f, -1000.0f);
	Vec3 capsuleRayStartToEnd = (capsule.m_bone.m_end - capsule.m_bone.m_start);
	float rayZScale = 1.0f / capsuleRayStartToEnd.z;
	float rayYScale = 1.0f / capsuleRayStartToEnd.y;
	float rayXScale = 1.0f / capsuleRayStartToEnd.x;
	tValues.push_back((aabb.m_maxs.z - capsule.m_bone.m_start.z) * rayZScale);
	tValues.push_back((aabb.m_mins.z - capsule.m_bone.m_start.z) * rayZScale);
	tValues.push_back((aabb.m_maxs.y - capsule.m_bone.m_start.y) * rayYScale);
	tValues.push_back((aabb.m_mins.y - capsule.m_bone.m_start.y) * rayYScale);
	tValues.push_back((aabb.m_maxs.x - capsule.m_bone.m_start.x) * rayXScale);
	tValues.push_back((aabb.m_mins.x - capsule.m_bone.m_start.x) * rayXScale);
	for (int tIndex = 0; tIndex < tValues.size(); tIndex++)
	{
		if (tValues[tIndex] >= 0.0f && tValues[tIndex] <= 1.0f)
		{
			Vec3 intersectPoint = capsule.m_bone.m_start + (tValues[tIndex] * capsuleRayStartToEnd);
			Vec3 nearestPointAABB = aabb.GetNearestEdgePosition(intersectPoint);
			float distanceSquared = GetDistanceSquared3D(intersectPoint, nearestPointAABB);
			if (distanceSquared < bestDistanceSquared)
			{
				bestDistanceSquared = distanceSquared;
				bestIntersectPoint = intersectPoint;
				bestNearestPointAABB = nearestPointAABB;
				bestTValue = tValues[tIndex];
			}
		}
	}

	//Out Check
	if (bestTValue < -1.0f || bestTValue > 2.0f)
	{
		//DebugAddWorldPoint(capsule.m_bone.m_end, capsule.m_radius, 0.1f, Rgba8::GREEN, Rgba8::GREEN);
		return returnValue;
	}

	//Calculate nearest points and get displacements
	Vec3 center = aabb.GetCenter();
	LineSegment3 bone(capsule.m_bone.m_start, capsule.m_bone.m_end);
	Vec3 nearestPointBone = GetNearestPointOnLineSegment3D(bestNearestPointAABB, bone);
	bestNearestPointAABB = aabb.GetNearestEdgePosition(nearestPointBone);
	nearestPointBone = GetNearestPointOnLineSegment3D(bestNearestPointAABB, bone);
	if (IsPointInsideAABB3D(nearestPointBone, aabb))
	{
		Vec3 displacementCenterToBone = nearestPointBone - center;
		Vec3 displacementCenterToCorner = bestNearestPointAABB - center;
		if (displacementCenterToBone.GetLengthSquared() < displacementCenterToCorner.GetLengthSquared())
		{
			//DebugAddWorldPoint(bestIntersectPoint, capsule.m_radius, 0.1f, Rgba8::CYAN, Rgba8::CYAN);
			//DebugAddWorldPoint(bestNearestPointAABB, capsule.m_radius, 0.1f, Rgba8::GREEN, Rgba8::GREEN);
			//DebugAddWorldPoint(nearestPointBone, capsule.m_radius, 0.1f, Rgba8::MAGENTA, Rgba8::MAGENTA);
			Vec3 displacement = bestNearestPointAABB - nearestPointBone;
			displacement.SetLength(displacement.GetLength() + capsule.m_radius);
			capsule.m_bone.m_start += displacement;
			capsule.m_bone.m_end += displacement;
			return true;
		}
	}
	else
	{
		Vec3 displacement = nearestPointBone - bestNearestPointAABB;
		float distanceSquared = displacement.GetLengthSquared();
		if (distanceSquared < capsule.m_radius * capsule.m_radius)
		{
			float offset = capsule.m_radius - sqrtf(distanceSquared);
			displacement.SetLength(offset);
			capsule.m_bone.m_start += displacement;
			capsule.m_bone.m_end += displacement;
			//DebugAddWorldPoint(bestIntersectPoint, capsule.m_radius, 0.1f, Rgba8::YELLOW, Rgba8::YELLOW);
			//DebugAddWorldPoint(bestNearestPointAABB, capsule.m_radius, 0.1f, Rgba8::RED, Rgba8::RED);
			//DebugAddWorldPoint(nearestPointBone, capsule.m_radius, 0.1f, Rgba8::BLACK, Rgba8::BLACK);
			return true;
		}
	}
	return returnValue;
}

//-----------------------------------------------------------------------------------------------
bool PushCapsuleOutOfFixedAABB3D(DPCapsule3& capsule, DPAABB3 const& aabb)
{
	//Initial Sphere Pushes
	bool returnValue = false;
	returnValue = PushSphereOutOfFixedAABB3D(capsule.m_bone.m_start, capsule.m_radius, aabb);
	returnValue |= PushSphereOutOfFixedAABB3D(capsule.m_bone.m_end, capsule.m_radius, aabb);
	
	//Calculate intersects and get best intersect
	std::vector<double> tValues;
	double bestTValue = 1000000000000.0f;
	double bestDistanceSquared = 1000000000000.0f;
	DPVec3 bestIntersectPoint = Vec3(-1000.0f, -1000.0f, -1000.0f);
	DPVec3 bestNearestPointAABB = Vec3(-1000.0f, -1000.0f, -1000.0f);
	DPVec3 capsuleRayStartToEnd = (capsule.m_bone.m_end - capsule.m_bone.m_start);
	double rayZScale = 1.0 / capsuleRayStartToEnd.z;
	double rayYScale = 1.0 / capsuleRayStartToEnd.y;
	double rayXScale = 1.0 / capsuleRayStartToEnd.x;
	tValues.push_back((aabb.m_maxs.z - capsule.m_bone.m_start.z) * rayZScale);
	tValues.push_back((aabb.m_mins.z - capsule.m_bone.m_start.z) * rayZScale);
	tValues.push_back((aabb.m_maxs.y - capsule.m_bone.m_start.y) * rayYScale);
	tValues.push_back((aabb.m_mins.y - capsule.m_bone.m_start.y) * rayYScale);
	tValues.push_back((aabb.m_maxs.x - capsule.m_bone.m_start.x) * rayXScale);
	tValues.push_back((aabb.m_mins.x - capsule.m_bone.m_start.x) * rayXScale);
	for (int tIndex = 0; tIndex < tValues.size(); tIndex++)
	{
		if (tValues[tIndex] >= 0.0f && tValues[tIndex] <= 1.0f)
		{
			DPVec3 intersectPoint = capsule.m_bone.m_start + (tValues[tIndex] * capsuleRayStartToEnd);
			DPVec3 nearestPointAABB = aabb.GetNearestEdgePosition(intersectPoint);
			double distanceSquared = GetDistanceSquared3D(intersectPoint, nearestPointAABB);
			if (distanceSquared < bestDistanceSquared)
			{
				bestDistanceSquared = distanceSquared;
				bestIntersectPoint = intersectPoint;
				bestNearestPointAABB = nearestPointAABB;
				bestTValue = tValues[tIndex];
			}
		}
	}

	//Out Check
	if (bestTValue < -1.0f || bestTValue > 2.0f)
	{
		return returnValue;
	}

	//Calculate nearest points and get displacements
	DPVec3 center = aabb.GetCenter();
	DPLineSegment3 bone(capsule.m_bone.m_start, capsule.m_bone.m_end);
	DPVec3 nearestPointBone = GetNearestPointOnLineSegment3D(bestNearestPointAABB, bone);
	bestNearestPointAABB = aabb.GetNearestEdgePosition(nearestPointBone);
	nearestPointBone = GetNearestPointOnLineSegment3D(bestNearestPointAABB, bone);
	if (IsPointInsideDPAABB3D(nearestPointBone, aabb))
	{
		DPVec3 displacementCenterToBone = nearestPointBone - center;
		DPVec3 displacementCenterToCorner = bestNearestPointAABB - center;
		if (displacementCenterToBone.GetLengthSquared() < displacementCenterToCorner.GetLengthSquared())
		{
			/*DebugAddWorldPoint(bestIntersectPoint, capsule.m_radius, 0.1f, Rgba8::CYAN, Rgba8::CYAN);
			DebugAddWorldPoint(bestNearestPointAABB, capsule.m_radius, 0.1f, Rgba8::GREEN, Rgba8::GREEN);
			DebugAddWorldPoint(nearestPointBone, capsule.m_radius, 0.1f, Rgba8::MAGENTA, Rgba8::MAGENTA);*/
			DPVec3 displacement = bestNearestPointAABB - nearestPointBone;
			displacement.SetLength(displacement.GetLength() + capsule.m_radius);

			/*if (nearestPointBone == capsule.m_bone.m_start)
			{
				capsule.m_bone.m_start += displacement;
			}
			else if (nearestPointBone == capsule.m_bone.m_end)
			{
				capsule.m_bone.m_end += displacement;
			}
			else
			{w
				Vec3 startToEndOfCapsuleDisplacement = capsule.m_bone.m_end - capsule.m_bone.m_start;
				Vec3 capsuleStartToLinePointDisplacement = (nearestPointBone - capsule.m_bone.m_end);
				float startToEndOfCapsuleDistance = startToEndOfCapsuleDisplacement.GetLength();
				float capsuleStartToLinePointDistance = capsuleStartToLinePointDisplacement.GetLength();
				float percentToPushStart = capsuleStartToLinePointDistance / startToEndOfCapsuleDistance;
				float percentToPushEnd = (startToEndOfCapsuleDistance - capsuleStartToLinePointDistance) / startToEndOfCapsuleDistance;
				capsule.m_bone.m_start += displacement * percentToPushStart;
				capsule.m_bone.m_end += displacement * percentToPushEnd;
			}*/
			capsule.m_bone.m_start += displacement;
			capsule.m_bone.m_end += displacement;

			/*DebugAddWorldPoint(capsule.m_bone.m_start, capsule.m_radius, 0.1f, Rgba8::CYAN, Rgba8::CYAN);
			DebugAddWorldPoint(capsule.m_bone.m_end, capsule.m_radius, 0.1f, Rgba8::CYAN, Rgba8::CYAN);*/
			return true;
		}
	}
	else
	{
		DPVec3 displacement = nearestPointBone - bestNearestPointAABB;
		double distanceSquared = displacement.GetLengthSquared();
		if (distanceSquared < capsule.m_radius * capsule.m_radius)
		{
			double offset = capsule.m_radius - sqrt(distanceSquared);
			displacement.SetLength(offset);
			/*if (nearestPointBone == capsule.m_bone.m_start)
			{
				capsule.m_bone.m_start += displacement;
			}
			else if (nearestPointBone == capsule.m_bone.m_end)
			{
				capsule.m_bone.m_end += displacement;
			}
			else
			{
				Vec3 startToEndOfCapsuleDisplacement = capsule.m_bone.m_end - capsule.m_bone.m_start;
				Vec3 capsuleStartToLinePointDisplacement = (nearestPointBone - capsule.m_bone.m_end);
				float startToEndOfCapsuleDistance = startToEndOfCapsuleDisplacement.GetLength();
				float capsuleStartToLinePointDistance = capsuleStartToLinePointDisplacement.GetLength();
				float percentToPushStart = capsuleStartToLinePointDistance / startToEndOfCapsuleDistance;
				float percentToPushEnd = (startToEndOfCapsuleDistance - capsuleStartToLinePointDistance) / startToEndOfCapsuleDistance;
				capsule.m_bone.m_start += displacement * percentToPushStart;
				capsule.m_bone.m_end += displacement * percentToPushEnd;
			}*/
			capsule.m_bone.m_start += displacement;
			capsule.m_bone.m_end += displacement;
			/*DebugAddWorldPoint(capsule.m_bone.m_start, capsule.m_radius, 0.1f, Rgba8::YELLOW, Rgba8::YELLOW);
			DebugAddWorldPoint(capsule.m_bone.m_end, capsule.m_radius, 0.1f, Rgba8::YELLOW, Rgba8::YELLOW);*/
			/*DebugAddWorldPoint(bestIntersectPoint, capsule.m_radius, 0.1f, Rgba8::YELLOW, Rgba8::YELLOW);
			DebugAddWorldPoint(bestNearestPointAABB, capsule.m_radius, 0.1f, Rgba8::RED, Rgba8::RED);
			DebugAddWorldPoint(nearestPointBone, capsule.m_radius, 0.1f, Rgba8::BLACK, Rgba8::BLACK);*/
			return true;
		}
	}

	return returnValue;
}

//-----------------------------------------------------------------------------------------------
bool PushCapsuleOutOfFixedOBB3D(Capsule3& capsule, OBB3 const& obb)
{
	Mat44 modelMatrix;
	modelMatrix.SetIJKT3D(obb.m_iBasisNormal, obb.m_jBasisNormal, obb.m_kBasisNormal, obb.m_center);

	AABB3 aabb;
	Vec3 dimensions;
	dimensions.x = obb.m_halfDimensions.x + obb.m_halfDimensions.x;
	dimensions.y = obb.m_halfDimensions.y + obb.m_halfDimensions.y;
	dimensions.z = obb.m_halfDimensions.z + obb.m_halfDimensions.z;
	aabb.SetDimensions(dimensions);

	Vec3 center = Vec3();
	Vec3 displacementToCapsuleStart = capsule.m_bone.m_start - obb.m_center;
	float projectionIBasis = DotProduct3D(displacementToCapsuleStart, obb.m_iBasisNormal);
	float projectionJBasis = DotProduct3D(displacementToCapsuleStart, obb.m_jBasisNormal);
	float projectionKBasis = DotProduct3D(displacementToCapsuleStart, obb.m_kBasisNormal);
	Vec3 localCapsuleStart = center + Vec3(1.0f, 0.0f, 0.0f) * projectionIBasis +
		Vec3(0.0f, 1.0f, 0.0f) * projectionJBasis + Vec3(0.0f, 0.0f, 1.0f) * projectionKBasis;
	Vec3 displacementToCapsuleEnd = capsule.m_bone.m_end - obb.m_center;
	projectionIBasis = DotProduct3D(displacementToCapsuleEnd, obb.m_iBasisNormal);
	projectionJBasis = DotProduct3D(displacementToCapsuleEnd, obb.m_jBasisNormal);
	projectionKBasis = DotProduct3D(displacementToCapsuleEnd, obb.m_kBasisNormal);
	Vec3 localCapsuleEnd = center + Vec3(1.0f, 0.0f, 0.0f) * projectionIBasis +
		Vec3(0.0f, 1.0f, 0.0f) * projectionJBasis + Vec3(0.0f, 0.0f, 1.0f) * projectionKBasis;
	
	Capsule3 localCapsule;
	localCapsule.m_bone.m_start = localCapsuleStart;
	localCapsule.m_bone.m_end = localCapsuleEnd;
	localCapsule.m_radius = capsule.m_radius;
	localCapsule.m_bone.m_radius = capsule.m_bone.m_radius;
	localCapsule.m_bone.m_iBasis = capsule.m_bone.m_iBasis;
	localCapsule.m_bone.m_jBasis = capsule.m_bone.m_jBasis;
	localCapsule.m_bone.m_kBasis = capsule.m_bone.m_kBasis;

	if (PushCapsuleOutOfFixedAABB3D(localCapsule, aabb))
	{
		TransformPosition3D(localCapsule.m_bone.m_start, modelMatrix);
		TransformPosition3D(localCapsule.m_bone.m_end, modelMatrix);
		capsule.m_bone.m_start = localCapsule.m_bone.m_start;
		capsule.m_bone.m_end = localCapsule.m_bone.m_end;
		return true;
	}

	return false;
}

//-----------------------------------------------------------------------------------------------
bool PushCapsuleOutOfFixedOBB3D(DPCapsule3& capsule, DPOBB3 const& obb)
{
	DPMat44 modelMatrix;
	modelMatrix.SetIJKT3D(obb.m_iBasisNormal, obb.m_jBasisNormal, obb.m_kBasisNormal, obb.m_center);

	DPAABB3 aabb;
	DPVec3 dimensions;
	dimensions.x = obb.m_halfDimensions.x + obb.m_halfDimensions.x;
	dimensions.y = obb.m_halfDimensions.y + obb.m_halfDimensions.y;
	dimensions.z = obb.m_halfDimensions.z + obb.m_halfDimensions.z;
	aabb.SetDimensions(dimensions);

	DPVec3 center = Vec3();
	DPVec3 displacementToCapsuleStart = capsule.m_bone.m_start - obb.m_center;
	double projectionIBasis = DotProduct3D(displacementToCapsuleStart, obb.m_iBasisNormal);
	double projectionJBasis = DotProduct3D(displacementToCapsuleStart, obb.m_jBasisNormal);
	double projectionKBasis = DotProduct3D(displacementToCapsuleStart, obb.m_kBasisNormal);
	DPVec3 localCapsuleStart = center + DPVec3(1.0f, 0.0f, 0.0f) * projectionIBasis + DPVec3(0.0f, 1.0f, 0.0f) * projectionJBasis + DPVec3(0.0f, 0.0f, 1.0f) * projectionKBasis;
	DPVec3 displacementToCapsuleEnd = capsule.m_bone.m_end - obb.m_center;
	projectionIBasis = DotProduct3D(displacementToCapsuleEnd, obb.m_iBasisNormal);
	projectionJBasis = DotProduct3D(displacementToCapsuleEnd, obb.m_jBasisNormal);
	projectionKBasis = DotProduct3D(displacementToCapsuleEnd, obb.m_kBasisNormal);
	DPVec3 localCapsuleEnd = center + DPVec3(1.0f, 0.0f, 0.0f) * projectionIBasis + DPVec3(0.0f, 1.0f, 0.0f) * projectionJBasis + DPVec3(0.0f, 0.0f, 1.0f) * projectionKBasis;

	DPCapsule3 localCapsule;
	localCapsule.m_bone.m_start = localCapsuleStart;
	localCapsule.m_bone.m_end = localCapsuleEnd;
	localCapsule.m_radius = capsule.m_radius;
	localCapsule.m_bone.m_radius = capsule.m_bone.m_radius;
	localCapsule.m_bone.m_iBasis = capsule.m_bone.m_iBasis;
	localCapsule.m_bone.m_jBasis = capsule.m_bone.m_jBasis;
	localCapsule.m_bone.m_kBasis = capsule.m_bone.m_kBasis;

	if (PushCapsuleOutOfFixedAABB3D(localCapsule, aabb))
	{
		TransformPosition3D(localCapsule.m_bone.m_start, modelMatrix);
		TransformPosition3D(localCapsule.m_bone.m_end, modelMatrix);
		capsule.m_bone.m_start = localCapsule.m_bone.m_start;
		capsule.m_bone.m_end = localCapsule.m_bone.m_end;
		return true;
	}

	return false;
}

//-----------------------------------------------------------------------------------------------
bool PushCapsuleOutOfFixedSphere3D(Capsule3& capsule, Vec3 const& fixedSphereCenter, float fixedSphereRadius)
{
	LineSegment3 bone(capsule.m_bone.m_start, capsule.m_bone.m_end);
	Vec3 nearestPointOnBone = GetNearestPointOnLineSegment3D(fixedSphereCenter, bone);
	Vec3 originalNearestPointOnBone = nearestPointOnBone;
	Vec3 diplacementFromCenterToBone = nearestPointOnBone - fixedSphereCenter;
	float distanceFromCenterToBone = diplacementFromCenterToBone.GetLengthSquared();
	if (distanceFromCenterToBone < (fixedSphereRadius + capsule.m_radius) * (fixedSphereRadius + capsule.m_radius))
	{
		//Do Sphere Push
		float offset = (fixedSphereRadius + capsule.m_radius) - sqrtf(distanceFromCenterToBone);
		diplacementFromCenterToBone.SetLength(offset);
		nearestPointOnBone += diplacementFromCenterToBone;

		//Get Push Difference and spread accurately to capsule start and end
		Vec3 nearestPointDisplacement = nearestPointOnBone - originalNearestPointOnBone;
		if (originalNearestPointOnBone == capsule.m_bone.m_start)
		{
			capsule.m_bone.m_start += nearestPointDisplacement;
		}
		else if (originalNearestPointOnBone == capsule.m_bone.m_end)
		{
			capsule.m_bone.m_end += nearestPointDisplacement;
		}
		else
		{
			Vec3 startToEndOfCapsuleDisplacement = capsule.m_bone.m_end - capsule.m_bone.m_start;
			Vec3 capsuleStartToLinePointDisplacement = (originalNearestPointOnBone - capsule.m_bone.m_end);
			float startToEndOfCapsuleDistance = startToEndOfCapsuleDisplacement.GetLength();
			float capsuleStartToLinePointDistance = capsuleStartToLinePointDisplacement.GetLength();
			float percentToPushStart = capsuleStartToLinePointDistance / startToEndOfCapsuleDistance;
			float percentToPushEnd = (startToEndOfCapsuleDistance - capsuleStartToLinePointDistance) / startToEndOfCapsuleDistance;
			capsule.m_bone.m_start += nearestPointDisplacement * percentToPushStart;
			capsule.m_bone.m_end += nearestPointDisplacement * percentToPushEnd;
		}
		return true;
	}
	return false;
}

//-----------------------------------------------------------------------------------------------
bool PushCapsuleOutOfFixedSphere3D(DPCapsule3& capsule, DPVec3 const& fixedSphereCenter, double fixedSphereRadius)
{
	DPLineSegment3 bone(capsule.m_bone.m_start, capsule.m_bone.m_end);
	DPVec3 nearestPointOnBone = GetNearestPointOnLineSegment3D(fixedSphereCenter, bone);
	DPVec3 originalNearestPointOnBone = nearestPointOnBone;
	DPVec3 diplacementFromCenterToBone = nearestPointOnBone - fixedSphereCenter;
	double distanceFromCenterToBone = diplacementFromCenterToBone.GetLengthSquared();
	if (distanceFromCenterToBone < (fixedSphereRadius + capsule.m_radius) * (fixedSphereRadius + capsule.m_radius))
	{
		//Do Sphere Push
		double offset = (fixedSphereRadius + capsule.m_radius) - sqrt(distanceFromCenterToBone);
		diplacementFromCenterToBone.SetLength(offset);
		nearestPointOnBone += diplacementFromCenterToBone;

		//Get Push Difference and spread accurately to capsule start and end
		DPVec3 nearestPointDisplacement = nearestPointOnBone - originalNearestPointOnBone;
		if (originalNearestPointOnBone == capsule.m_bone.m_start)
		{
			capsule.m_bone.m_start += nearestPointDisplacement;
		}
		else if (originalNearestPointOnBone == capsule.m_bone.m_end)
		{
			capsule.m_bone.m_end += nearestPointDisplacement;
		}
		else
		{
			DPVec3 startToEndOfCapsuleDisplacement = capsule.m_bone.m_end - capsule.m_bone.m_start;
			DPVec3 capsuleStartToLinePointDisplacement = (originalNearestPointOnBone - capsule.m_bone.m_end);
			double startToEndOfCapsuleDistance = startToEndOfCapsuleDisplacement.GetLength();
			double capsuleStartToLinePointDistance = capsuleStartToLinePointDisplacement.GetLength();
			double percentToPushStart = capsuleStartToLinePointDistance / startToEndOfCapsuleDistance;
			double percentToPushEnd = (startToEndOfCapsuleDistance - capsuleStartToLinePointDistance) / startToEndOfCapsuleDistance;
			capsule.m_bone.m_start += nearestPointDisplacement * percentToPushStart;
			capsule.m_bone.m_end += nearestPointDisplacement * percentToPushEnd;
		}
		return true;
	}
	return false;
}

//-----------------------------------------------------------------------------------------------
bool PushCapsuleOutOfFixedCapsule3D(Capsule3& mobileCapsule, Capsule3 const& fixedCapsule)
{
	LineSegment3 mobileLine = LineSegment3(mobileCapsule.m_bone.m_start, mobileCapsule.m_bone.m_end);
	LineSegment3 fixedLine = LineSegment3(fixedCapsule.m_bone.m_start, fixedCapsule.m_bone.m_end);
	std::vector<Vec3> nearestPoints = GetNearestPointsBetweenLines3D(mobileLine, fixedLine);

	Vec3 displacementNearestPoints = nearestPoints[0] - nearestPoints[1];
	float distanceNearestPoints = displacementNearestPoints.GetLength();
	if (distanceNearestPoints < mobileCapsule.m_radius + fixedCapsule.m_radius)
	{
		float offset = mobileCapsule.m_radius + fixedCapsule.m_radius - distanceNearestPoints;
		displacementNearestPoints.SetLength(offset);
		mobileCapsule.m_bone.m_start += displacementNearestPoints;
		mobileCapsule.m_bone.m_end += displacementNearestPoints;
		return true;
	}

	return false;
}

//-----------------------------------------------------------------------------------------------
bool PushCapsuleOutOfFixedCapsule3D(DPCapsule3& mobileCapsule, DPCapsule3 const& fixedCapsule)
{
	DPLineSegment3 mobileLine = DPLineSegment3(mobileCapsule.m_bone.m_start, mobileCapsule.m_bone.m_end);
	DPLineSegment3 fixedLine = DPLineSegment3(fixedCapsule.m_bone.m_start, fixedCapsule.m_bone.m_end);
	std::vector<DPVec3> nearestPoints = GetNearestPointsBetweenLines3D(mobileLine, fixedLine);

	DPVec3 displacementNearestPoints = nearestPoints[0] - nearestPoints[1];
	double distanceNearestPoints = displacementNearestPoints.GetLength();
	if (distanceNearestPoints < mobileCapsule.m_radius + fixedCapsule.m_radius)
	{
		double offset = mobileCapsule.m_radius + fixedCapsule.m_radius - distanceNearestPoints;
		displacementNearestPoints.SetLength(offset);
		mobileCapsule.m_bone.m_start += displacementNearestPoints;
		mobileCapsule.m_bone.m_end += displacementNearestPoints;
		return true;
	}

	return false;
}

//-----------------------------------------------------------------------------------------------
bool PushCapsuleOutOfCapsule3D(Capsule3& capsuleA, Capsule3& capsuleB)
{
	LineSegment3 lineA = LineSegment3(capsuleA.m_bone.m_start, capsuleA.m_bone.m_end);
	LineSegment3 lineB = LineSegment3(capsuleB.m_bone.m_start, capsuleB.m_bone.m_end);
	std::vector<Vec3> nearestPoints = GetNearestPointsBetweenLines3D(lineA, lineB);

	Vec3 displacementNearestPoints = nearestPoints[0] - nearestPoints[1];
	float distanceNearestPoints = displacementNearestPoints.GetLength();
	if (distanceNearestPoints < capsuleA.m_radius + capsuleB.m_radius)
	{
		float offset = (capsuleA.m_radius + capsuleB.m_radius - distanceNearestPoints) * 0.5f;
		displacementNearestPoints.SetLength(offset);
		capsuleA.m_bone.m_start += displacementNearestPoints;
		capsuleA.m_bone.m_end += displacementNearestPoints;
		capsuleB.m_bone.m_start -= displacementNearestPoints;
		capsuleB.m_bone.m_end -= displacementNearestPoints;
		return true;
	}

	return false;
}

//-----------------------------------------------------------------------------------------------
bool PushCapsuleOutOfCapsule3D(DPCapsule3& capsuleA, DPCapsule3& capsuleB)
{
	DPLineSegment3 lineA = DPLineSegment3(capsuleA.m_bone.m_start, capsuleA.m_bone.m_end);
	DPLineSegment3 lineB = DPLineSegment3(capsuleB.m_bone.m_start, capsuleB.m_bone.m_end);
	std::vector<DPVec3> nearestPoints = GetNearestPointsBetweenLines3D(lineA, lineB);

	DPVec3 displacementNearestPoints = nearestPoints[0] - nearestPoints[1];
	double distanceNearestPoints = displacementNearestPoints.GetLength();
	if (distanceNearestPoints < capsuleA.m_radius + capsuleB.m_radius)
	{
		double offset = (capsuleA.m_radius + capsuleB.m_radius - distanceNearestPoints) * 0.5f;
		displacementNearestPoints.SetLength(offset);
		capsuleA.m_bone.m_start += displacementNearestPoints;
		capsuleA.m_bone.m_end += displacementNearestPoints;
		capsuleB.m_bone.m_start -= displacementNearestPoints;
		capsuleB.m_bone.m_end -= displacementNearestPoints;
		return true;
	}

	return false;
}

//-----------------------------------------------------------------------------------------------
bool PushCapsuleOutOfCapsule2D(Capsule2& capsuleA, Capsule2& capsuleB)
{
	Vec2 nearestPointCapsuleB;
	Vec2 nearestPointCapsuleA;
	LineSegment2 lineA(capsuleA.m_bone.m_start, capsuleA.m_bone.m_end);
	LineSegment2 lineB(capsuleB.m_bone.m_start, capsuleB.m_bone.m_end);

	std::vector<float> distances;
	Vec2 nearestPointBFromAStart = GetNearestPointOnLineSegment2D(capsuleA.m_bone.m_start, lineB);
	Vec2 nearestPointBFromAEnd = GetNearestPointOnLineSegment2D(capsuleA.m_bone.m_end, lineB);
	Vec2 nearestPointAFromBStart = GetNearestPointOnLineSegment2D(capsuleB.m_bone.m_start, lineA);
	Vec2 nearestPointAFromBEnd = GetNearestPointOnLineSegment2D(capsuleB.m_bone.m_end, lineA);
	distances.push_back(GetDistanceSquared2D(capsuleA.m_bone.m_start, nearestPointBFromAStart));
	distances.push_back(GetDistanceSquared2D(capsuleA.m_bone.m_end, nearestPointBFromAEnd));
	distances.push_back(GetDistanceSquared2D(capsuleB.m_bone.m_start, nearestPointAFromBStart));
	distances.push_back(GetDistanceSquared2D(capsuleB.m_bone.m_end, nearestPointAFromBEnd));

	int bestDistanceIndex = -1;
	float bestDistance = 10000000000.0f;
	for (int distanceIndex = 0; distanceIndex < distances.size(); distanceIndex++)
	{
		if (distances[distanceIndex] < bestDistance)
		{
			bestDistance = distances[distanceIndex];
			bestDistanceIndex = distanceIndex;
		}
	}

	if (bestDistanceIndex == 0)
	{
		nearestPointCapsuleA = capsuleA.m_bone.m_start;
		nearestPointCapsuleB = nearestPointBFromAStart;
	}
	else if (bestDistanceIndex == 1)
	{
		nearestPointCapsuleA = capsuleA.m_bone.m_end;
		nearestPointCapsuleB = nearestPointBFromAEnd;
	}
	if (bestDistanceIndex == 2)
	{
		nearestPointCapsuleB = capsuleB.m_bone.m_start;
		nearestPointCapsuleA = nearestPointAFromBStart;
	}
	else if (bestDistanceIndex == 3)
	{
		nearestPointCapsuleB = capsuleB.m_bone.m_end;
		nearestPointCapsuleA = nearestPointAFromBEnd;
	}

	Vec2 originalNearestPointCapsuleA = nearestPointCapsuleA;
	Vec2 originalNearestPointCapsuleB = nearestPointCapsuleB;
	if (PushDiscsOutOfEachOther2D(nearestPointCapsuleA, capsuleA.m_radius, nearestPointCapsuleB, capsuleB.m_radius))
	{
		Vec2 displacementA = nearestPointCapsuleA - originalNearestPointCapsuleA;
		Vec2 displacementB = nearestPointCapsuleB - originalNearestPointCapsuleB;

		//Capsule A
		if (nearestPointCapsuleA == capsuleA.m_bone.m_start)
		{
			capsuleA.m_bone.m_start += displacementA;
		}
		else if (nearestPointCapsuleA == capsuleA.m_bone.m_end)
		{
			capsuleA.m_bone.m_end += displacementA;
		}
		else
		{
			Vec2 startToEndOfCapsuleDisplacement = capsuleA.m_bone.m_end - capsuleA.m_bone.m_start;
			Vec2 capsuleStartToLinePointDisplacement = (nearestPointCapsuleA - capsuleA.m_bone.m_end);
			float startToEndOfCapsuleDistance = startToEndOfCapsuleDisplacement.GetLength();
			float capsuleStartToLinePointDistance = capsuleStartToLinePointDisplacement.GetLength();
			float percentToPushStart = capsuleStartToLinePointDistance / startToEndOfCapsuleDistance;
			float percentToPushEnd = (startToEndOfCapsuleDistance - capsuleStartToLinePointDistance) / startToEndOfCapsuleDistance;
			capsuleA.m_bone.m_start += displacementA * percentToPushStart;
			capsuleA.m_bone.m_end += displacementA * percentToPushEnd;
		}

		//Capsule B
		if (nearestPointCapsuleB == capsuleB.m_bone.m_start)
		{
			capsuleB.m_bone.m_start -= displacementB;
		}
		else if (nearestPointCapsuleB == capsuleB.m_bone.m_end)
		{
			capsuleB.m_bone.m_end -= displacementB;
		}
		else
		{
			Vec2 startToEndOfCapsuleDisplacement = capsuleB.m_bone.m_end - capsuleB.m_bone.m_start;
			Vec2 capsuleStartToLinePointDisplacement = (nearestPointCapsuleB - capsuleB.m_bone.m_end);
			float startToEndOfCapsuleDistance = startToEndOfCapsuleDisplacement.GetLength();
			float capsuleStartToLinePointDistance = capsuleStartToLinePointDisplacement.GetLength();
			float percentToPushStart = capsuleStartToLinePointDistance / startToEndOfCapsuleDistance;
			float percentToPushEnd = (startToEndOfCapsuleDistance - capsuleStartToLinePointDistance) / startToEndOfCapsuleDistance;
			capsuleB.m_bone.m_start += displacementB * percentToPushStart;
			capsuleB.m_bone.m_end += displacementB * percentToPushEnd;
		}
		return true;
	}

	return false;
}

//-----------------------------------------------------------------------------------------------
bool PushCapsuleOutOfFixedCylinderZ3D(Capsule3& mobileCapsule, Cylinder3 const& fixedCylinder)
{
	//Evaluate top and bottom intersections
	LineSegment3 mobileLine = LineSegment3(mobileCapsule.m_bone.m_start, mobileCapsule.m_bone.m_end);
	LineSegment3 fixedLine = LineSegment3(fixedCylinder.m_start, fixedCylinder.m_end);
	Vec3 topOfCylinder;
	Vec3 bottomOfCylinder;
	if (fixedCylinder.m_start.z > fixedCylinder.m_end.z)
	{
		topOfCylinder = fixedCylinder.m_start;
		bottomOfCylinder = fixedCylinder.m_end;
	}
	else
	{
		topOfCylinder = fixedCylinder.m_end;
		bottomOfCylinder = fixedCylinder.m_start;
	}

	//Evaluate Side intersections
	if (mobileCapsule.m_bone.m_start.z < topOfCylinder.z && mobileCapsule.m_bone.m_end.z < topOfCylinder.z
		&& mobileCapsule.m_bone.m_start.z > bottomOfCylinder.z && mobileCapsule.m_bone.m_end.z > bottomOfCylinder.z)
	{
		std::vector<Vec3> nearestPoints = GetNearestPointsBetweenLines3D(mobileLine, fixedLine);
		Vec3 displacementNearestPoints = nearestPoints[0] - nearestPoints[1];
		float distanceNearestPointsSquared = displacementNearestPoints.GetLengthSquared();
		if (distanceNearestPointsSquared < (mobileCapsule.m_radius + fixedCylinder.m_radius) * (mobileCapsule.m_radius + fixedCylinder.m_radius))
		{
			float offset = (mobileCapsule.m_radius + fixedCylinder.m_radius - sqrtf(distanceNearestPointsSquared)) * 0.5f;
			displacementNearestPoints.SetLength(offset);
			mobileCapsule.m_bone.m_start += displacementNearestPoints;
			mobileCapsule.m_bone.m_end += displacementNearestPoints;
			return true;
		}
	}
	//Top
	else if (mobileCapsule.m_bone.m_start.z >= fixedCylinder.m_end.z || mobileCapsule.m_bone.m_end.z >= fixedCylinder.m_end.z)
	{
		bool returnValue = false;
		Vec3 nearestPointOnBone = mobileCapsule.m_bone.m_end;
		Vec3 nearestPointCylinder = GetNearestPointOnCylinderZ3D(nearestPointOnBone, fixedCylinder);
		Vec3 displacement = nearestPointOnBone - nearestPointCylinder;
		float distanceSquared = displacement.GetLengthSquared();
		if (nearestPointOnBone == nearestPointCylinder)
		{
			float distanceToTop = fixedCylinder.m_end.z - nearestPointCylinder.z;
			Vec3 deltaPos = DPVec3(0.0, 0.0, 1.0) * (distanceToTop + mobileCapsule.m_radius);
			mobileCapsule.m_bone.m_end += deltaPos;
			returnValue = true;
		}
		else if (distanceSquared < (mobileCapsule.m_radius * mobileCapsule.m_radius))
		{
			float offset = mobileCapsule.m_radius - sqrtf(distanceSquared);
			displacement.SetLength(offset);
			mobileCapsule.m_bone.m_end += displacement;
			returnValue = true;
		}

		nearestPointOnBone = mobileCapsule.m_bone.m_start;
		nearestPointCylinder = GetNearestPointOnCylinderZ3D(nearestPointOnBone, fixedCylinder);
		displacement = nearestPointOnBone - nearestPointCylinder;
		distanceSquared = displacement.GetLengthSquared();
		if (nearestPointOnBone == nearestPointCylinder)
		{
			float distanceToTop = fixedCylinder.m_end.z - nearestPointCylinder.z;
			Vec3 deltaPos = DPVec3(0.0, 0.0, 1.0) * (distanceToTop + mobileCapsule.m_radius);
			mobileCapsule.m_bone.m_start += deltaPos;
			returnValue = true;
		}
		else if (distanceSquared < (mobileCapsule.m_radius * mobileCapsule.m_radius))
		{
			float offset = mobileCapsule.m_radius - sqrtf(distanceSquared);
			displacement.SetLength(offset);
			mobileCapsule.m_bone.m_start += displacement;
			returnValue = true;
		}

		Vec3 mobileCapsuleDireciton = mobileCapsule.m_bone.m_end - mobileCapsule.m_bone.m_start;
		float tTop = (topOfCylinder.z - mobileCapsule.m_bone.m_start.z) / mobileCapsuleDireciton.z;
		if (mobileCapsuleDireciton.z != 0.0)
		{
			mobileLine = LineSegment3(mobileCapsule.m_bone.m_start, mobileCapsule.m_bone.m_end);
			Vec3 intersectPoint = mobileCapsule.m_bone.m_start + tTop * mobileCapsuleDireciton;
			nearestPointCylinder = GetNearestPointOnCylinderZ3D(intersectPoint, fixedCylinder);
			nearestPointOnBone = GetNearestPointOnLineSegment3D(nearestPointCylinder, mobileLine);
			nearestPointCylinder = GetNearestPointOnCylinderZ3D(nearestPointOnBone, fixedCylinder);
			nearestPointOnBone = GetNearestPointOnLineSegment3D(nearestPointCylinder, mobileLine);
			displacement = nearestPointOnBone - nearestPointCylinder;
			distanceSquared = displacement.GetLengthSquared();
			if (nearestPointOnBone == nearestPointCylinder)
			{
				Vec3 cylinderDisplacement = nearestPointCylinder - fixedCylinder.m_end;
				cylinderDisplacement.SetLength(fixedCylinder.m_radius);
				nearestPointCylinder = fixedCylinder.m_end + cylinderDisplacement;
				nearestPointOnBone = GetNearestPointOnLineSegment3D(nearestPointCylinder, mobileLine);
				//DebugAddWorldPoint(nearestPointOnBone, mobileCapsule.m_radius, 0.1f, Rgba8::MAGENTA, Rgba8::MAGENTA);
				//DebugAddWorldPoint(nearestPointCylinder, mobileCapsule.m_radius, 0.1f, Rgba8::CYAN, Rgba8::CYAN);
				displacement = nearestPointCylinder - nearestPointOnBone;
				displacement.SetLength(displacement.GetLength() + mobileCapsule.m_radius);
				//DebugAddWorldPoint(nearestPointOnBone + displacement, mobileCapsule.m_radius, 0.1f, Rgba8::YELLOW, Rgba8::YELLOW);
				mobileCapsule.m_bone.m_start += displacement;
				mobileCapsule.m_bone.m_end += displacement;
				return true;
			}
			else if (distanceSquared < mobileCapsule.m_radius * mobileCapsule.m_radius)
			{
				float offset = mobileCapsule.m_radius - sqrtf(distanceSquared);
				displacement.SetLength(offset);
				mobileCapsule.m_bone.m_start += displacement;
				mobileCapsule.m_bone.m_end += displacement;

				//DebugAddWorldPoint(mobileCapsule.m_bone.m_end, mobileCapsule.m_radius, 0.1f, Rgba8::YELLOW, Rgba8::YELLOW);
				return true;
			}


		}
		if (returnValue)
		{
			//DebugAddWorldPoint(mobileCapsule.m_bone.m_end, mobileCapsule.m_radius, 0.1f, Rgba8::MEDIUM_GREEN, Rgba8::MEDIUM_GREEN);
			return true;
		}
	}
	//Bottom
	else if (mobileCapsule.m_bone.m_start.z <= bottomOfCylinder.z || mobileCapsule.m_bone.m_end.z <= bottomOfCylinder.z)
	{
		bool returnValue = false;
		Vec3 nearestPointOnBone = mobileCapsule.m_bone.m_end;
		Vec3 nearestPointCylinder = GetNearestPointOnCylinderZ3D(nearestPointOnBone, fixedCylinder);
		Vec3 displacement = nearestPointOnBone - nearestPointCylinder;
		float distanceSquared = displacement.GetLengthSquared();
		if (nearestPointOnBone == nearestPointCylinder)
		{
			float distanceToBottom = nearestPointCylinder.z - fixedCylinder.m_start.z;
			Vec3 deltaPos = Vec3(0.0, 0.0, -1.0) * (distanceToBottom + mobileCapsule.m_radius);
			mobileCapsule.m_bone.m_end += deltaPos;
			returnValue = true;
		}
		else if (distanceSquared < (mobileCapsule.m_radius * mobileCapsule.m_radius))
		{
			float offset = mobileCapsule.m_radius - sqrtf(distanceSquared);
			displacement.SetLength(offset);
			mobileCapsule.m_bone.m_end += displacement;
			returnValue = true;
		}

		nearestPointOnBone = mobileCapsule.m_bone.m_start;
		nearestPointCylinder = GetNearestPointOnCylinderZ3D(nearestPointOnBone, fixedCylinder);
		displacement = nearestPointOnBone - nearestPointCylinder;
		distanceSquared = displacement.GetLengthSquared();
		if (nearestPointOnBone == nearestPointCylinder)
		{
			float distanceToBottom = nearestPointCylinder.z - fixedCylinder.m_start.z;
			Vec3 deltaPos = Vec3(0.0, 0.0, -1.0) * (distanceToBottom + mobileCapsule.m_radius);
			mobileCapsule.m_bone.m_start += deltaPos;
			returnValue = true;
		}
		else if (distanceSquared < (mobileCapsule.m_radius * mobileCapsule.m_radius))
		{
			float offset = mobileCapsule.m_radius - sqrtf(distanceSquared);
			displacement.SetLength(offset);
			mobileCapsule.m_bone.m_start += displacement;
			returnValue = true;
		}

		Vec3 mobileCapsuleDireciton = mobileCapsule.m_bone.m_end - mobileCapsule.m_bone.m_start;
		float tBottom = (bottomOfCylinder.z - mobileCapsule.m_bone.m_start.z) / mobileCapsuleDireciton.z;
		if (mobileCapsuleDireciton.z != 0.0)
		{
			mobileLine = LineSegment3(mobileCapsule.m_bone.m_start, mobileCapsule.m_bone.m_end);
			Vec3 intersectPoint = mobileCapsule.m_bone.m_start + tBottom * mobileCapsuleDireciton;
			nearestPointCylinder = GetNearestPointOnCylinderZ3D(intersectPoint, fixedCylinder);
			nearestPointOnBone = GetNearestPointOnLineSegment3D(nearestPointCylinder, mobileLine);
			nearestPointCylinder = GetNearestPointOnCylinderZ3D(nearestPointOnBone, fixedCylinder);
			nearestPointOnBone = GetNearestPointOnLineSegment3D(nearestPointCylinder, mobileLine);
			displacement = nearestPointOnBone - nearestPointCylinder;
			distanceSquared = displacement.GetLengthSquared();
			if (nearestPointOnBone == nearestPointCylinder)
			{
				Vec3 cylinderDisplacement = nearestPointCylinder - fixedCylinder.m_end;
				cylinderDisplacement.SetLength(fixedCylinder.m_radius);
				nearestPointCylinder = fixedCylinder.m_end + cylinderDisplacement;
				nearestPointOnBone = GetNearestPointOnLineSegment3D(nearestPointCylinder, mobileLine);
				//DebugAddWorldPoint(nearestPointOnBone, mobileCapsule.m_radius, 0.1f, Rgba8::MAGENTA, Rgba8::MAGENTA);
				//DebugAddWorldPoint(nearestPointCylinder, mobileCapsule.m_radius, 0.1f, Rgba8::CYAN, Rgba8::CYAN);
				displacement = nearestPointCylinder - nearestPointOnBone;
				displacement.SetLength(displacement.GetLength() + mobileCapsule.m_radius);
				//DebugAddWorldPoint(nearestPointOnBone + displacement, mobileCapsule.m_radius, 0.1f, Rgba8::YELLOW, Rgba8::YELLOW);
				mobileCapsule.m_bone.m_start += displacement;
				mobileCapsule.m_bone.m_end += displacement;
				return true;
			}
			else if (distanceSquared < mobileCapsule.m_radius * mobileCapsule.m_radius)
			{
				float offset = mobileCapsule.m_radius - sqrtf(distanceSquared);
				displacement.SetLength(offset);
				mobileCapsule.m_bone.m_start += displacement;
				mobileCapsule.m_bone.m_end += displacement;

				//DebugAddWorldPoint(mobileCapsule.m_bone.m_end, mobileCapsule.m_radius, 0.1f, Rgba8::YELLOW, Rgba8::YELLOW);
				return true;
			}


		}
		if (returnValue)
		{
			//DebugAddWorldPoint(mobileCapsule.m_bone.m_end, mobileCapsule.m_radius, 0.1f, Rgba8::MEDIUM_GREEN, Rgba8::MEDIUM_GREEN);
			return true;
		}
	}

	return false;
}

//-----------------------------------------------------------------------------------------------
bool PushCapsuleOutOfFixedCylinderZ3D(DPCapsule3& mobileCapsule, DPCylinder3 const& fixedCylinder)
{
	//Evaluate top and bottom intersections
	DPLineSegment3 mobileLine = DPLineSegment3(mobileCapsule.m_bone.m_start, mobileCapsule.m_bone.m_end);
	DPLineSegment3 fixedLine = DPLineSegment3(fixedCylinder.m_start, fixedCylinder.m_end);
	DPVec3 topOfCylinder;
	DPVec3 bottomOfCylinder;
	if (fixedCylinder.m_start.z > fixedCylinder.m_end.z)
	{
		topOfCylinder = fixedCylinder.m_start;
		bottomOfCylinder = fixedCylinder.m_end;
	}
	else
	{
		topOfCylinder = fixedCylinder.m_end;
		bottomOfCylinder = fixedCylinder.m_start;
	}

	//Evaluate Side intersections
	if (mobileCapsule.m_bone.m_start.z < topOfCylinder.z && mobileCapsule.m_bone.m_end.z < topOfCylinder.z
		&& mobileCapsule.m_bone.m_start.z > bottomOfCylinder.z && mobileCapsule.m_bone.m_end.z > bottomOfCylinder.z)
	{
		std::vector<DPVec3> nearestPoints = GetNearestPointsBetweenLines3D(mobileLine, fixedLine);
		DPVec3 displacementNearestPoints = nearestPoints[0] - nearestPoints[1];
		double distanceNearestPointsSquared = displacementNearestPoints.GetLengthSquared();
		if (distanceNearestPointsSquared < (mobileCapsule.m_radius + fixedCylinder.m_radius) * (mobileCapsule.m_radius + fixedCylinder.m_radius))
		{
			double offset = (mobileCapsule.m_radius + fixedCylinder.m_radius - sqrt(distanceNearestPointsSquared)) * 0.5f;
			displacementNearestPoints.SetLength(offset);
			mobileCapsule.m_bone.m_start += displacementNearestPoints;
			mobileCapsule.m_bone.m_end += displacementNearestPoints;
			return true;
		}
	}
	//Top
	else if(mobileCapsule.m_bone.m_start.z >= fixedCylinder.m_end.z || mobileCapsule.m_bone.m_end.z >= fixedCylinder.m_end.z)
	{
		bool returnValue = false;
		DPVec3 nearestPointOnBone = mobileCapsule.m_bone.m_end;
		DPVec3 nearestPointCylinder = GetNearestPointOnCylinderZ3D(nearestPointOnBone, fixedCylinder);
		DPVec3 displacement = nearestPointOnBone - nearestPointCylinder;
		double distanceSquared = displacement.GetLengthSquared();
		if (nearestPointOnBone == nearestPointCylinder)
		{
			double distanceToTop = fixedCylinder.m_end.z - nearestPointCylinder.z;
			DPVec3 deltaPos = DPVec3(0.0, 0.0, 1.0) * (distanceToTop + mobileCapsule.m_radius);
			mobileCapsule.m_bone.m_end += deltaPos;
			returnValue = true;
		}
		else if (distanceSquared < (mobileCapsule.m_radius * mobileCapsule.m_radius))
		{
			double offset = mobileCapsule.m_radius - sqrt(distanceSquared);
			displacement.SetLength(offset);
			mobileCapsule.m_bone.m_end += displacement;
			returnValue = true;
		}

		nearestPointOnBone = mobileCapsule.m_bone.m_start;
		nearestPointCylinder = GetNearestPointOnCylinderZ3D(nearestPointOnBone, fixedCylinder);
		displacement = nearestPointOnBone - nearestPointCylinder;
		distanceSquared = displacement.GetLengthSquared();
		if (nearestPointOnBone == nearestPointCylinder)
		{
			double distanceToTop = fixedCylinder.m_end.z - nearestPointCylinder.z;
			DPVec3 deltaPos = DPVec3(0.0, 0.0, 1.0) * (distanceToTop + mobileCapsule.m_radius);
			mobileCapsule.m_bone.m_start += deltaPos;
			returnValue = true;
		}
		else if (distanceSquared < (mobileCapsule.m_radius * mobileCapsule.m_radius))
		{
			double offset = mobileCapsule.m_radius - sqrt(distanceSquared);
			displacement.SetLength(offset);
			mobileCapsule.m_bone.m_start += displacement;
			returnValue = true;
		}

		DPVec3 mobileCapsuleDireciton = mobileCapsule.m_bone.m_end - mobileCapsule.m_bone.m_start;
		double tTop = (topOfCylinder.z - mobileCapsule.m_bone.m_start.z) / mobileCapsuleDireciton.z;
		if (mobileCapsuleDireciton.z != 0.0)
		{
			mobileLine = DPLineSegment3(mobileCapsule.m_bone.m_start, mobileCapsule.m_bone.m_end);
			DPVec3 intersectPoint = mobileCapsule.m_bone.m_start + tTop * mobileCapsuleDireciton;
			nearestPointCylinder = GetNearestPointOnCylinderZ3D(intersectPoint, fixedCylinder);
			nearestPointOnBone = GetNearestPointOnLineSegment3D(nearestPointCylinder, mobileLine);
			nearestPointCylinder = GetNearestPointOnCylinderZ3D(nearestPointOnBone, fixedCylinder);
			nearestPointOnBone = GetNearestPointOnLineSegment3D(nearestPointCylinder, mobileLine);
			displacement = nearestPointOnBone - nearestPointCylinder;
			distanceSquared = displacement.GetLengthSquared();
			if(nearestPointOnBone == nearestPointCylinder)
			{
				DPVec3 cylinderDisplacement = nearestPointCylinder - fixedCylinder.m_end;
				cylinderDisplacement.SetLength(fixedCylinder.m_radius);
				nearestPointCylinder = fixedCylinder.m_end + cylinderDisplacement;
				nearestPointOnBone = GetNearestPointOnLineSegment3D(nearestPointCylinder, mobileLine);
				//DebugAddWorldPoint(nearestPointOnBone, mobileCapsule.m_radius, 0.1f, Rgba8::MAGENTA, Rgba8::MAGENTA);
				//DebugAddWorldPoint(nearestPointCylinder, mobileCapsule.m_radius, 0.1f, Rgba8::CYAN, Rgba8::CYAN);
				displacement = nearestPointCylinder - nearestPointOnBone;
				displacement.SetLength(displacement.GetLength() + mobileCapsule.m_radius);
				//DebugAddWorldPoint(nearestPointOnBone + displacement, mobileCapsule.m_radius, 0.1f, Rgba8::YELLOW, Rgba8::YELLOW);
				mobileCapsule.m_bone.m_start += displacement;
				mobileCapsule.m_bone.m_end += displacement;
				return true;
			}
			else if (distanceSquared < mobileCapsule.m_radius * mobileCapsule.m_radius)
			{
				double offset = mobileCapsule.m_radius - sqrt(distanceSquared);
				displacement.SetLength(offset);
				mobileCapsule.m_bone.m_start += displacement;
				mobileCapsule.m_bone.m_end += displacement;

				//DebugAddWorldPoint(mobileCapsule.m_bone.m_end, mobileCapsule.m_radius, 0.1f, Rgba8::YELLOW, Rgba8::YELLOW);
				return true;
			}
			
			
		}	
		if (returnValue)
		{
			//DebugAddWorldPoint(mobileCapsule.m_bone.m_end, mobileCapsule.m_radius, 0.1f, Rgba8::MEDIUM_GREEN, Rgba8::MEDIUM_GREEN);
			return true;
		}
	}
	//Bottom
	else if (mobileCapsule.m_bone.m_start.z <= bottomOfCylinder.z || mobileCapsule.m_bone.m_end.z <= bottomOfCylinder.z)
	{
		bool returnValue = false;
		DPVec3 nearestPointOnBone = mobileCapsule.m_bone.m_end;
		DPVec3 nearestPointCylinder = GetNearestPointOnCylinderZ3D(nearestPointOnBone, fixedCylinder);
		DPVec3 displacement = nearestPointOnBone - nearestPointCylinder;
		double distanceSquared = displacement.GetLengthSquared();
		if (nearestPointOnBone == nearestPointCylinder)
		{
			double distanceToBottom = nearestPointCylinder.z - fixedCylinder.m_start.z;
			DPVec3 deltaPos = DPVec3(0.0, 0.0, -1.0) * (distanceToBottom + mobileCapsule.m_radius);
			mobileCapsule.m_bone.m_end += deltaPos;
			returnValue = true;
		}
		else if (distanceSquared < (mobileCapsule.m_radius * mobileCapsule.m_radius))
		{
			double offset = mobileCapsule.m_radius - sqrt(distanceSquared);
			displacement.SetLength(offset);
			mobileCapsule.m_bone.m_end += displacement;
			returnValue = true;
		}

		nearestPointOnBone = mobileCapsule.m_bone.m_start;
		nearestPointCylinder = GetNearestPointOnCylinderZ3D(nearestPointOnBone, fixedCylinder);
		displacement = nearestPointOnBone - nearestPointCylinder;
		distanceSquared = displacement.GetLengthSquared();
		if (nearestPointOnBone == nearestPointCylinder)
		{
			double distanceToBottom = nearestPointCylinder.z- fixedCylinder.m_start.z;
			DPVec3 deltaPos = DPVec3(0.0, 0.0, -1.0) * (distanceToBottom + mobileCapsule.m_radius);
			mobileCapsule.m_bone.m_start += deltaPos;
			returnValue = true;
		}
		else if (distanceSquared < (mobileCapsule.m_radius * mobileCapsule.m_radius))
		{
			double offset = mobileCapsule.m_radius - sqrt(distanceSquared);
			displacement.SetLength(offset);
			mobileCapsule.m_bone.m_start += displacement;
			returnValue = true;
		}

		DPVec3 mobileCapsuleDireciton = mobileCapsule.m_bone.m_end - mobileCapsule.m_bone.m_start;
		double tBottom = (bottomOfCylinder.z - mobileCapsule.m_bone.m_start.z) / mobileCapsuleDireciton.z;
		if (mobileCapsuleDireciton.z != 0.0)
		{
			mobileLine = DPLineSegment3(mobileCapsule.m_bone.m_start, mobileCapsule.m_bone.m_end);
			DPVec3 intersectPoint = mobileCapsule.m_bone.m_start + tBottom * mobileCapsuleDireciton;
			nearestPointCylinder = GetNearestPointOnCylinderZ3D(intersectPoint, fixedCylinder);
			nearestPointOnBone = GetNearestPointOnLineSegment3D(nearestPointCylinder, mobileLine);
			nearestPointCylinder = GetNearestPointOnCylinderZ3D(nearestPointOnBone, fixedCylinder);
			nearestPointOnBone = GetNearestPointOnLineSegment3D(nearestPointCylinder, mobileLine);
			displacement = nearestPointOnBone - nearestPointCylinder;
			distanceSquared = displacement.GetLengthSquared();
			if (nearestPointOnBone == nearestPointCylinder)
			{
				DPVec3 cylinderDisplacement = nearestPointCylinder - fixedCylinder.m_end;
				cylinderDisplacement.SetLength(fixedCylinder.m_radius);
				nearestPointCylinder = fixedCylinder.m_end + cylinderDisplacement;
				nearestPointOnBone = GetNearestPointOnLineSegment3D(nearestPointCylinder, mobileLine);
				//DebugAddWorldPoint(nearestPointOnBone, mobileCapsule.m_radius, 0.1f, Rgba8::MAGENTA, Rgba8::MAGENTA);
				//DebugAddWorldPoint(nearestPointCylinder, mobileCapsule.m_radius, 0.1f, Rgba8::CYAN, Rgba8::CYAN);
				displacement = nearestPointCylinder - nearestPointOnBone;
				displacement.SetLength(displacement.GetLength() + mobileCapsule.m_radius);
				//DebugAddWorldPoint(nearestPointOnBone + displacement, mobileCapsule.m_radius, 0.1f, Rgba8::YELLOW, Rgba8::YELLOW);
				mobileCapsule.m_bone.m_start += displacement;
				mobileCapsule.m_bone.m_end += displacement;
				return true;
			}
			else if (distanceSquared < mobileCapsule.m_radius * mobileCapsule.m_radius)
			{
				double offset = mobileCapsule.m_radius - sqrt(distanceSquared);
				displacement.SetLength(offset);
				mobileCapsule.m_bone.m_start += displacement;
				mobileCapsule.m_bone.m_end += displacement;

				//DebugAddWorldPoint(mobileCapsule.m_bone.m_end, mobileCapsule.m_radius, 0.1f, Rgba8::YELLOW, Rgba8::YELLOW);
				return true;
			}


		}
		if (returnValue)
		{
			//DebugAddWorldPoint(mobileCapsule.m_bone.m_end, mobileCapsule.m_radius, 0.1f, Rgba8::MEDIUM_GREEN, Rgba8::MEDIUM_GREEN);
			return true;
		}
	}

	return false;
}

//-----------------------------------------------------------------------------------------------
bool PushCapsuleOutOfFixedCylinder3D(Capsule3& mobileCapsule, Cylinder3 const& fixedCylinder)
{
	Mat44 modelMatrix;
	modelMatrix.SetIJKT3D(fixedCylinder.m_iBasis, fixedCylinder.m_jBasis, fixedCylinder.m_kBasis, fixedCylinder.m_start);

	float length = (fixedCylinder.m_end - fixedCylinder.m_start).GetLength();
	Cylinder3 localCylinder;
	localCylinder.m_radius = fixedCylinder.m_radius;
	localCylinder.m_start = Vec3();
	localCylinder.m_end = localCylinder.m_start + Vec3(0.0f, 0.0f, length);
	localCylinder.m_iBasis = Vec3(1.0f, 0.0f, 0.0f);
	localCylinder.m_jBasis = Vec3(0.0f, 1.0f, 0.0f);
	localCylinder.m_kBasis = Vec3(0.0f, 0.0f, 1.0f);

	Vec3 displacementToCapsuleStart = mobileCapsule.m_bone.m_start - fixedCylinder.m_start;
	float projectionIBasis = DotProduct3D(displacementToCapsuleStart, fixedCylinder.m_iBasis);
	float projectionJBasis = DotProduct3D(displacementToCapsuleStart, fixedCylinder.m_jBasis);
	float projectionKBasis = DotProduct3D(displacementToCapsuleStart, fixedCylinder.m_kBasis);
	Vec3 localCapsuleStart = localCylinder.m_start + localCylinder.m_iBasis * projectionIBasis +
		localCylinder.m_jBasis * projectionJBasis + localCylinder.m_kBasis * projectionKBasis;
	Vec3 displacementToCapsuleEnd = mobileCapsule.m_bone.m_end - fixedCylinder.m_start;
	projectionIBasis = DotProduct3D(displacementToCapsuleEnd, fixedCylinder.m_iBasis);
	projectionJBasis = DotProduct3D(displacementToCapsuleEnd, fixedCylinder.m_jBasis);
	projectionKBasis = DotProduct3D(displacementToCapsuleEnd, fixedCylinder.m_kBasis);
	Vec3 localCapsuleEnd = localCylinder.m_start + localCylinder.m_iBasis * projectionIBasis +
		localCylinder.m_jBasis * projectionJBasis + localCylinder.m_kBasis * projectionKBasis;

	Capsule3 localCapsule;
	localCapsule.m_bone.m_start = localCapsuleStart;
	localCapsule.m_bone.m_end = localCapsuleEnd;
	localCapsule.m_radius = mobileCapsule.m_radius;
	localCapsule.m_bone.m_radius = mobileCapsule.m_bone.m_radius;
	localCapsule.m_bone.m_iBasis = mobileCapsule.m_bone.m_iBasis;
	localCapsule.m_bone.m_jBasis = mobileCapsule.m_bone.m_jBasis;
	localCapsule.m_bone.m_kBasis = mobileCapsule.m_bone.m_kBasis;

	if (PushCapsuleOutOfFixedCylinderZ3D(localCapsule, localCylinder))
	{
		TransformPosition3D(localCapsule.m_bone.m_start, modelMatrix);
		TransformPosition3D(localCapsule.m_bone.m_end, modelMatrix);
		mobileCapsule.m_bone.m_start = localCapsule.m_bone.m_start;
		mobileCapsule.m_bone.m_end = localCapsule.m_bone.m_end;
		return true;
	}

	return false;
}

//-----------------------------------------------------------------------------------------------
bool PushCapsuleOutOfFixedCylinder3D(DPCapsule3& mobileCapsule, DPCylinder3 const& fixedCylinder)
{
	DPMat44 modelMatrix;
	modelMatrix.SetIJKT3D(fixedCylinder.m_iBasis, fixedCylinder.m_jBasis, fixedCylinder.m_kBasis, fixedCylinder.m_start);

	double length = (fixedCylinder.m_end - fixedCylinder.m_start).GetLength();
	DPCylinder3 localCylinder;
	localCylinder.m_radius = fixedCylinder.m_radius;
	localCylinder.m_start = DPVec3();
	localCylinder.m_end = localCylinder.m_start + DPVec3(0.0f, 0.0f, length);
	localCylinder.m_iBasis = DPVec3(1.0f, 0.0f, 0.0f);
	localCylinder.m_jBasis = DPVec3(0.0f, 1.0f, 0.0f);
	localCylinder.m_kBasis = DPVec3(0.0f, 0.0f, 1.0f);

	DPVec3 displacementToCapsuleStart = mobileCapsule.m_bone.m_start - fixedCylinder.m_start;
	double projectionIBasis = DotProduct3D(displacementToCapsuleStart, fixedCylinder.m_iBasis);
	double projectionJBasis = DotProduct3D(displacementToCapsuleStart, fixedCylinder.m_jBasis);
	double projectionKBasis = DotProduct3D(displacementToCapsuleStart, fixedCylinder.m_kBasis);
	DPVec3 localCapsuleStart = localCylinder.m_start + localCylinder.m_iBasis * projectionIBasis + localCylinder.m_jBasis * projectionJBasis + localCylinder.m_kBasis * projectionKBasis;
	DPVec3 displacementToCapsuleEnd = mobileCapsule.m_bone.m_end - fixedCylinder.m_start;
	projectionIBasis = DotProduct3D(displacementToCapsuleEnd, fixedCylinder.m_iBasis);
	projectionJBasis = DotProduct3D(displacementToCapsuleEnd, fixedCylinder.m_jBasis);
	projectionKBasis = DotProduct3D(displacementToCapsuleEnd, fixedCylinder.m_kBasis);
	DPVec3 localCapsuleEnd = localCylinder.m_start + localCylinder.m_iBasis * projectionIBasis + localCylinder.m_jBasis * projectionJBasis + localCylinder.m_kBasis * projectionKBasis;

	DPCapsule3 localCapsule;
	localCapsule.m_bone.m_start = localCapsuleStart;
	localCapsule.m_bone.m_end = localCapsuleEnd;
	localCapsule.m_radius = mobileCapsule.m_radius;
	localCapsule.m_bone.m_radius = mobileCapsule.m_bone.m_radius;
	localCapsule.m_bone.m_iBasis = mobileCapsule.m_bone.m_iBasis;
	localCapsule.m_bone.m_jBasis = mobileCapsule.m_bone.m_jBasis;
	localCapsule.m_bone.m_kBasis = mobileCapsule.m_bone.m_kBasis;

	if (PushCapsuleOutOfFixedCylinderZ3D(localCapsule, localCylinder))
	{
		TransformPosition3D(localCapsule.m_bone.m_start, modelMatrix);
		TransformPosition3D(localCapsule.m_bone.m_end, modelMatrix);
		mobileCapsule.m_bone.m_start = localCapsule.m_bone.m_start;
		mobileCapsule.m_bone.m_end = localCapsule.m_bone.m_end;
		return true;
	}

	return false;
}

//-----------------------------------------------------------------------------------------------
void TransformPosition2D(Vec2& posToTransform, float uniformScale, float rotationDegrees, Vec2 const& translation)
{
	posToTransform *= uniformScale;

	posToTransform.RotateDegrees(rotationDegrees);

	posToTransform += translation;
}

//-----------------------------------------------------------------------------------------------
void TransformPosition2D(Vec2& posToTransform, Vec2 const& iBasis, Vec2 const& jBasis, Vec2 const& translation)
{
	posToTransform = translation + (posToTransform.x * iBasis) + (posToTransform.y * jBasis);
}

//-----------------------------------------------------------------------------------------------
void TransformPositionXY3D(Vec3& positionToTransform, float scaleXY, float zRotationDegrees, Vec2 const& translationXY)
{
	positionToTransform.x *= scaleXY;
	positionToTransform.y *= scaleXY;

	positionToTransform = positionToTransform.GetRotatedAboutZDegrees(zRotationDegrees);

	positionToTransform.x += translationXY.x;
	positionToTransform.y += translationXY.y;

}

//-----------------------------------------------------------------------------------------------
void TransformPositionXY3D(Vec3& posToTransform, Vec2 const& iBasis, Vec2 const& jBasis, Vec2 const& translation)
{
	Vec2 pos2D = translation + (posToTransform.x * iBasis) + (posToTransform.y * jBasis);
	posToTransform.x = pos2D.x;
	posToTransform.y = pos2D.y;
}

//-----------------------------------------------------------------------------------------------
void TransformPosition3D(Vec3& posToTransform, Mat44 modelMatrix)
{
	posToTransform = modelMatrix.TransformPosition3D(posToTransform);
}

//-----------------------------------------------------------------------------------------------
void TransformPosition3D(DPVec3& posToTransform, DPMat44 modelMatrix)
{
	posToTransform = modelMatrix.TransformPosition3D(posToTransform);
}

//-----------------------------------------------------------------------------------------------
float NormalizeByte(unsigned char byteValue)
{
	float inValue = static_cast<float>(byteValue);

	return RangeMap(inValue, 0.0f, 255.0f, 0.0f, 1.0f);
}

//-----------------------------------------------------------------------------------------------
unsigned char DenormalizeByte(float zeroToOne)
{
	float outValue = zeroToOne * 256.0f;

	if (outValue == 256.0f)
	{
		return static_cast<unsigned char>(outValue) - 1;
	}

	return static_cast<unsigned char>(outValue);
}

//-----------------------------------------------------------------------------------------------
Mat44 GetBillboardMatrix(BillboardType billboardType, Mat44 const& cameraMatrix, const Vec3& billboardPosition, const Vec2& billboardScale)
{
	Mat44 billboardMatrix;
	billboardScale;
	Vec3 iBasis = Vec3(1.0f, 0.0f, 0.0f);
	Vec3 jBasis = Vec3(0.0f, 1.0f, 0.0f);
	Vec3 kBasis = Vec3(0.0f, 0.0f, 1.0f);
	Vec3 translation = billboardPosition;
	Vec3 forward = cameraMatrix.GetIBasis3D();
	Vec3 left = cameraMatrix.GetJBasis3D();
	Vec3 up = cameraMatrix.GetKBasis3D();
	Vec3 position = cameraMatrix.GetTranslation3D();
	Vec3 z = Vec3(0.0f, 0.0f, 1.0f);
	Vec3 y = Vec3(0.0f, 1.0f, 0.0f);
	
	if (billboardType == BillboardType::FULL_CAMERA_OPPOSING)
	{
		iBasis = forward * -1.0f;
		jBasis = left * -1.0f;
		kBasis = up;

		billboardMatrix.SetIJK3D(iBasis, jBasis, kBasis);
		billboardMatrix.SetTranslation3D(billboardPosition);
	}
	else if (billboardType == BillboardType::FULL_CAMERA_FACING)
	{
		iBasis = position - translation;
		iBasis.Normalize();
		if (abs(DotProduct3D(iBasis, z)) < 1.0f)
		{
			jBasis = CrossProduct3D(z, iBasis);
			jBasis.Normalize();
			kBasis = CrossProduct3D(iBasis, jBasis);
			kBasis.Normalize();
		}
		else
		{
			kBasis = CrossProduct3D(iBasis, y);
			kBasis.Normalize();
			jBasis = CrossProduct3D(kBasis, iBasis);
			jBasis.Normalize();
		}

		billboardMatrix.SetIJK3D(iBasis, jBasis, kBasis);
		billboardMatrix.SetTranslation3D(billboardPosition);
	}
	else if (billboardType == BillboardType::WORLD_UP_CAMERA_FACING)
	{
		kBasis = z;
		iBasis = position - translation;
		iBasis.z = 0.0f;
		iBasis.Normalize();
		jBasis = CrossProduct3D(kBasis, iBasis);
		jBasis.Normalize();

		billboardMatrix.SetIJK3D(iBasis, jBasis, kBasis);
		billboardMatrix.SetTranslation3D(billboardPosition);
	}
	else if (billboardType == BillboardType::WORLD_UP_CAMERA_OPPOSING)
	{
		iBasis = forward * -1.0f;
		iBasis.z = 0.0f;
		iBasis.Normalize();
		jBasis = left * -1.0f;
		kBasis = z;

		billboardMatrix.SetIJK3D(iBasis, jBasis, kBasis);
		billboardMatrix.SetTranslation3D(billboardPosition);
	}
	return billboardMatrix;
}

//-----------------------------------------------------------------------------------------------
float ComputeCubicBezier1D(float A, float B, float C, float D, float t)
{
	float s = 1 - t;
	float E = s * A + t * B;
	float F = s * B + t * C;
	float G = s * C + t * D;
	float H = s * E + t * F;
	float I = s * F + t * G;
	float P = s * H + t * I;
	return P;
}

//-----------------------------------------------------------------------------------------------
float ComputeQuinticBezier1D(float A, float B, float C, float D, float E, float F, float t)
{
	float s = 1 - t;
	float G = s * A + t * B;
	float H = s * B + t * C;
	float I = s * C + t * D;
	float J = s * D + t * E;
	float K = s * E + t * F;
	float L = s * G + t * H;
	float M = s * H + t * I;
	float N = s * I + t * J;
	float O = s * J + t * K;
	float P = s * L + t * M;
	float Q = s * M + t * N;
	float R = s * N + t * O;
	float S = s * P + t * Q;
	float T = s * Q + t * R;
	float U = s * S + t * T;

	return U;
}

//-----------------------------------------------------------------------------------------------
float SmoothStart2(float t)
{
	return t * t;
}

//-----------------------------------------------------------------------------------------------
float SmoothStart3(float t)
{
	return t * t * t;
}

//-----------------------------------------------------------------------------------------------
float SmoothStart4(float t)
{
	return t * t * t * t;
}

//-----------------------------------------------------------------------------------------------
float SmoothStart5(float t)
{
	return t * t * t * t * t;
}

//-----------------------------------------------------------------------------------------------
float SmoothStop2(float t)
{
	float s = 1.0f - t;
	return 1.0f - (s * s);
}

//-----------------------------------------------------------------------------------------------
float SmoothStop3(float t)
{
	float s = 1.0f - t;
	return 1.0f - (s * s * s);
}

//-----------------------------------------------------------------------------------------------
float SmoothStop4(float t)
{
	float s = 1.0f - t;
	return 1.0f - (s * s * s * s);
}

//-----------------------------------------------------------------------------------------------
float SmoothStop5(float t)
{
	float s = 1.0f - t;
	return 1.0f - (s * s * s * s * s);
}

//-----------------------------------------------------------------------------------------------
float SmoothStep3(float t)
{
	return ComputeCubicBezier1D(0.0f, 0.0f, 1.0f, 1.0f, t);
}

//-----------------------------------------------------------------------------------------------
float SmoothStep5(float t)
{
	return ComputeQuinticBezier1D(0.0f, 0.0f, 0.0f, 1.0f, 1.0f, 1.0f, t);
}

//-----------------------------------------------------------------------------------------------
float Hesitate3(float t)
{
	return ComputeCubicBezier1D(0.0f, 1.0f, 0.0f, 1.0f, t);
}

//-----------------------------------------------------------------------------------------------
float Hesitate5(float t)
{
	return ComputeQuinticBezier1D(0.0f, 1.0f, 0.0f, 1.0f, 0.0f, 1.0f, t);
}

//-----------------------------------------------------------------------------------------------
float CustomFunkyEasingFunction(float t)
{
	return ComputeCubicBezier1D(0.0f, 1.0f, 0.0f, 1.0f, ComputeQuinticBezier1D(0.0f, 1.0f, 0.9f, 1.0f, 0.01f, 1.0f, t));
}

//-----------------------------------------------------------------------------------------------
float GetClamped(float value, float minValue, float maxValue)
{
	if (value < minValue)
	{
		value = minValue;
	}
	else if (value > maxValue)
	{
		value = maxValue;
	}

	return value;
}

//-----------------------------------------------------------------------------------------------
double GetClamped(double value, double minValue, double maxValue)
{
	if (value < minValue)
	{
		value = minValue;
	}
	else if (value > maxValue)
	{
		value = maxValue;
	}

	return value;
}

//-----------------------------------------------------------------------------------------------
float GetClampedZeroToOne(float value)
{
	if (value < 0)
	{
		value = 0;
	}
	else if (value > 1)
	{
		value = 1;
	}

	return value;
}

//-----------------------------------------------------------------------------------------------
float Interpolate(float start, float end, float fractionTowardEnd)
{
	return start + fractionTowardEnd * (end - start);
}

//-----------------------------------------------------------------------------------------------
float GetFractionWithinRange(float value, float rangeStart, float rangeEnd)
{
	return (value - rangeStart) / (rangeEnd - rangeStart);
}

//-----------------------------------------------------------------------------------------------
float RangeMap(float inValue, float inStart, float inEnd, float outStart, float outEnd)
{
	float inRange = inEnd - inStart;
	float outRange = outEnd - outStart;
	float scale = outRange / inRange;

	float outValue = scale * (inValue - inStart) + outStart;
	return outValue;
}

//-----------------------------------------------------------------------------------------------
float RangeMapClamped(float inValue, float inStart, float inEnd, float outStart, float outEnd)
{
	float outValue = RangeMap(inValue, inStart, inEnd, outStart, outEnd);
	if (outEnd >= outStart)
	{
		outValue = GetClamped(outValue, outStart, outEnd);
	}
	else
	{
		outValue = GetClamped(outValue, outEnd, outStart);
	}
	return outValue;
}

//-----------------------------------------------------------------------------------------------
float RoundDownToInt(float value)
{
	if (value > 0)
	{
		return static_cast<float>(static_cast<int>(value));
	}
	else
	{
		return static_cast<float>(static_cast<int>(value) - 1);
	}
}

//-----------------------------------------------------------------------------------------------
RaycastResult2D RaycastVsDisc2D(Vec2 startPos, Vec2 fwdNormal, float maxDist, Vec2 discCenter, float discRadius)
{
	RaycastResult2D result;
	result.m_rayStartPos = startPos;
	result.m_rayFwdNormal = fwdNormal;
	result.m_rayMaxLength = maxDist;
	result.m_didImpact = false;

	Vec2 displacement = discCenter - startPos;
	Vec2 IBasis = fwdNormal;
	Vec2 JBasis = IBasis.GetRotated90Degrees();
	float displacementIBasis = DotProduct2D(displacement, IBasis);
	float displacementJBasis = DotProduct2D(displacement, JBasis);

	if (displacementJBasis >= discRadius)
	{
		return result;
	}
	else if (displacementIBasis <= -discRadius)
	{
		return result;
	}
	else if (displacementIBasis >= maxDist + discRadius)
	{
		return result;
	}

	float discRadiusSquared = discRadius * discRadius;
	float displacementJBasisSquared = displacementJBasis * displacementJBasis;

	if (discRadiusSquared - displacementJBasisSquared <= 0.0f)
	{
		return result;
	}

	float distanceToNearestEdgePoint = sqrtf(discRadiusSquared - displacementJBasisSquared);
	result.m_impactDist = displacementIBasis - distanceToNearestEdgePoint;

	if (result.m_impactDist >= maxDist)
	{
		return result;
	}

	if (IsPointInsideDisc2D(startPos, discCenter, discRadius) == true)
	{
		result.m_didImpact = true;
		result.m_impactPos = startPos;
		result.m_impactNormal = fwdNormal * -1.0f;
		return result;
	}
	
	if (result.m_impactDist < 0.0f)
	{
		return result;
	}

	result.m_didImpact = true;
	result.m_impactPos = startPos + (result.m_impactDist * IBasis);
	result.m_impactNormal = (result.m_impactPos - discCenter).GetNormalized();
	return result;
}

//-----------------------------------------------------------------------------------------------
void BounceDiscsOffEachOther2D(Vec2& positionA, float radiusA, Vec2& velocityA, float elasticityA, Vec2& positionB, float radiusB, Vec2& velocityB, float elasticityB)
{
	//Bounding Disc Rejection test
	if (DoDiscsOverlap(positionA, radiusA, positionB, radiusB) == false)
	{
		return;
	}

	PushDiscsOutOfEachOther2D(positionA, radiusA, positionB, radiusB);
	float elasticity = elasticityA * elasticityB;
	Vec2 impactNormal = (positionB - positionA).GetNormalized();

	Vec2 velocityAProjectedOntoN = DotProduct2D(velocityA, impactNormal) * impactNormal * elasticity;
	Vec2 velocityAT = velocityA - velocityAProjectedOntoN;
	Vec2 velocityBProjectedOntoN = DotProduct2D(velocityB, impactNormal) * impactNormal * elasticity;
	Vec2 velocityBT = velocityB - velocityBProjectedOntoN;

	//Divergence Check
	//Moving away from each other check
	if(DotProduct2D(velocityB, impactNormal) - DotProduct2D(velocityA, impactNormal) < 0.0f)
	{
		velocityA = velocityAT + velocityBProjectedOntoN;
		velocityB = velocityBT + velocityAProjectedOntoN;
	}
	else
	{
		velocityA = velocityAT + velocityAProjectedOntoN;
		velocityB = velocityBT + velocityBProjectedOntoN;
	}
}

//-----------------------------------------------------------------------------------------------
void BounceDiscOffFixedDisc2D(Vec2& mobileCenter, float mobileRadius, Vec2& mobileVelocity, float mobileElasticity, Vec2 fixedCenter, float fixedRadius, float fixedElasticity)
{
	//Bounding Disc Rejection test
	if (DoDiscsOverlap(mobileCenter, mobileRadius, fixedCenter, fixedRadius) == false)
	{
		return;
	}

	PushDiscOutOfFixedDisc2D(mobileCenter, mobileRadius, fixedCenter, fixedRadius);
	float elasticity = mobileElasticity * fixedElasticity;
	Vec2 impactNormal = mobileCenter - fixedCenter;
	impactNormal.Normalize();

	//Divergent Velocity Check
	if (DotProduct2D(mobileVelocity.GetNormalized(), (mobileCenter - fixedCenter).GetNormalized()) > 0)
	{
		Vec2 translationDown = DotProduct2D(mobileVelocity, impactNormal) * impactNormal;
		Vec2 tanslationOver = mobileVelocity - translationDown;
		Vec2 reflected = tanslationOver + (translationDown * elasticity);
		mobileVelocity = reflected;
		return;
	}

	Vec2 translationDown = DotProduct2D(mobileVelocity, impactNormal) * impactNormal;
	Vec2 tanslationOver = mobileVelocity - translationDown;
	Vec2 reflected = tanslationOver + (translationDown * elasticity) * -1.0f;
	mobileVelocity = reflected;
}

//-----------------------------------------------------------------------------------------------
void BounceDiscOffFixedOBB2D(Vec2& mobileCenter, float mobileRadius, Vec2& mobileVelocity, float mobileElasticity, OBB2 fixedOBB, float fixedElasticity)
{
	//Bounding Disc Rejection test
	float radius = sqrtf((fixedOBB.m_halfDimensions.x * fixedOBB.m_halfDimensions.x) + (fixedOBB.m_halfDimensions.y * fixedOBB.m_halfDimensions.y));
	if (DoDiscsOverlap(mobileCenter, mobileRadius, fixedOBB.m_center, radius) == false)
	{
		return;
	}

	//Check if overlapping the OBB2
	Vec2 nearestPoint = GetNearestPointOnOBB2D(mobileCenter, fixedOBB);
	Vec2 differenceFromPointToRadius = mobileCenter - nearestPoint;
	float distance = differenceFromPointToRadius.GetLength();
	if (distance > mobileRadius)
	{
		return;
	}

	//Push Logic
	float offset = mobileRadius - distance;
	differenceFromPointToRadius.SetLength(offset);
	mobileCenter += differenceFromPointToRadius;

	Vec2 impactNormal = Vec2();
	impactNormal = (mobileCenter - nearestPoint).GetNormalized();

	//Divergence Check
	if (DotProduct2D(mobileVelocity, (nearestPoint - mobileCenter)) < 0.0f)
	{
		float elasticity = mobileElasticity * fixedElasticity;
		Vec2 translationDown = DotProduct2D(mobileVelocity, impactNormal) * impactNormal;
		Vec2 tanslationOver = mobileVelocity - translationDown;
		Vec2 reflected = tanslationOver + (translationDown * elasticity);
		mobileVelocity = reflected;
		return;
	}

	float elasticity = mobileElasticity * fixedElasticity;
	Vec2 translationDown = DotProduct2D(mobileVelocity, impactNormal) * impactNormal;
	Vec2 tanslationOver = mobileVelocity - translationDown;
	Vec2 reflected = tanslationOver + (translationDown * elasticity) * -1.0f;
	mobileVelocity = reflected;
}

//-----------------------------------------------------------------------------------------------
void BounceDiscOffFixedCapsule2D(Vec2& mobileCenter, float mobileRadius, Vec2& mobileVelocity, float mobileElasticity, Capsule2 fixedCapsule, float fixedElasticity)
{
	//Bounding Disc Rejection test
	Vec2 center = (fixedCapsule.m_bone.m_start + fixedCapsule.m_bone.m_end) * 0.5;
	float radius = center.GetLength() + fixedCapsule.m_radius;
	if (DoDiscsOverlap(mobileCenter, mobileRadius, center, radius) == false)
	{
		return;
	}

	//Check if in capsule
	Vec2 nearestPoint = GetNearestPointOnCapsule2D(mobileCenter, fixedCapsule);
	Vec2 differenceFromPointToRadius = mobileCenter - nearestPoint;
	float distance = differenceFromPointToRadius.GetLength();
	if (distance > mobileRadius)
	{
		return;
	}

	//Push Logic
	float offset = mobileRadius - distance;
	differenceFromPointToRadius.SetLength(offset);
	mobileCenter += differenceFromPointToRadius;

	Vec2 impactNormal = Vec2();
	impactNormal = (mobileCenter - nearestPoint).GetNormalized();

	//Diverging
	if (DotProduct2D(mobileVelocity.GetNormalized(), (nearestPoint - mobileCenter).GetNormalized()) < 0.0f)
	{
		float elasticity = mobileElasticity * fixedElasticity;
		Vec2 translationDown = DotProduct2D(mobileVelocity, impactNormal) * impactNormal;
		Vec2 tanslationOver = mobileVelocity - translationDown;
		Vec2 reflected = tanslationOver + (translationDown * elasticity);
		return;
	}

	float elasticity = mobileElasticity * fixedElasticity;
	Vec2 translationDown = DotProduct2D(mobileVelocity, impactNormal) * impactNormal;
	Vec2 tanslationOver = mobileVelocity - translationDown;
	Vec2 reflected = tanslationOver + (translationDown * elasticity) * -1.0f;
	mobileVelocity = reflected;
}

//-----------------------------------------------------------------------------------------------
RaycastResult3D RaycastVsCylinderZ3D(const Vec3& start, const Vec3& direction, float distance, const Vec2& center, float minZ, float maxZ, float radius)
{
	Vec3 rayEndPos = direction * distance + start;
	Vec3 ray = direction * distance;

	RaycastResult3D rayResult;
	rayResult.m_rayFwdNormal = direction.GetNormalized();
	rayResult.m_rayMaxLength = distance;
	rayResult.m_rayStartPos = start;

	//Inside Cylinder
	if (IsPointInsideDisc2D(Vec2(start.x, start.y), center, radius))
	{
		if (start.z < maxZ && start.z > minZ)
		{
			rayResult.m_impactPos = start;
			rayResult.m_impactDist = 0.0f;
			rayResult.m_impactNormal = direction * -1.0f;
			rayResult.m_didImpact = true;
			return rayResult;
		}
	}

	//Max Z
	if (start.z >= maxZ && rayEndPos.z <= maxZ)
	{
		float t = (maxZ - start.z) / ray.z;
		Vec3 position = start + (t * ray);
		if (IsPointInsideDisc2D(Vec2(position.x, position.y), center, radius))
		{
			rayResult.m_impactPos = position;
			rayResult.m_impactDist = (rayResult.m_impactPos - start).GetLength();
			rayResult.m_impactNormal = Vec3(0.0f, 0.0f, 1.0f);
			rayResult.m_didImpact = true;
			return rayResult;
		}
	}

	//Min Z
	if (start.z <= minZ && rayEndPos.z >= minZ)
	{
		float t = (minZ - start.z) / ray.z;
		Vec3 position = start + (t * ray);
		if (IsPointInsideDisc2D(Vec2(position.x, position.y), center, radius))
		{
			rayResult.m_impactPos = position;
			rayResult.m_impactDist = (rayResult.m_impactPos - start).GetLength();
			rayResult.m_impactNormal = Vec3(0.0f, 0.0f, -1.0f);
			rayResult.m_didImpact = true;
			return rayResult;
		}
	}

	//X and Y
	RaycastResult2D xyResult;
	Vec2 xyFwdNormal = Vec2(rayResult.m_rayFwdNormal.x, rayResult.m_rayFwdNormal.y).GetNormalized();
	xyResult = RaycastVsDisc2D(Vec2(start.x, start.y), xyFwdNormal, GetProjectedLength2D(Vec2(ray.x, ray.y), xyFwdNormal), center, radius);
	if (xyResult.m_didImpact == true)
	{
		float t = (xyResult.m_impactPos.y - start.y) / ray.y;
		Vec3 position = start + (t * ray);
		if (position.z > minZ && position.z < maxZ)
		{
			rayResult.m_impactPos = position;
			rayResult.m_didImpact = xyResult.m_didImpact;
			rayResult.m_impactDist = (rayResult.m_impactPos - start).GetLength();
			rayResult.m_impactNormal = Vec3(xyResult.m_impactNormal.x, xyResult.m_impactNormal.y, 0.0f);
		}
	}

	return rayResult;
}

//-----------------------------------------------------------------------------------------------
RaycastResult2D RaycastVsLineSegment2D(Vec2 startPos, Vec2 fwdNormal, float maxDist, LineSegment2 const& lineSegment)
{
	RaycastResult2D rayResult;
	rayResult.m_rayStartPos = startPos;
	rayResult.m_rayMaxLength = maxDist;
	rayResult.m_rayFwdNormal = fwdNormal;

	Vec2 jBasis = fwdNormal.GetRotated90Degrees();
	Vec2 startToLineStart = lineSegment.m_start - startPos;
	Vec2 startToLineEnd = lineSegment.m_end - startPos;
	float startToLineStartJDisplacement = DotProduct2D(startToLineStart, jBasis);
	float startToLineEndJDisplacement = DotProduct2D(startToLineEnd, jBasis);

	if (startToLineStartJDisplacement <= 0.0f && startToLineEndJDisplacement <= 0.0f)
	{
		return rayResult;
	}
	if (startToLineStartJDisplacement >= 0.0f && startToLineEndJDisplacement >= 0.0f)
	{
		return rayResult;
	}

	float t = startToLineStartJDisplacement * -1.0f / (startToLineEndJDisplacement - startToLineStartJDisplacement);
	Vec2 impactPos = lineSegment.m_start + (t * (lineSegment.m_end - lineSegment.m_start));
	Vec2 impactLine = impactPos - startPos;
	float impactDist = DotProduct2D(impactLine, fwdNormal);
	if (impactDist > maxDist || impactDist <= 0.0f)
	{
		return rayResult;
	}
	Vec2 line = lineSegment.m_end - lineSegment.m_start;
	Vec2 impactNormal = (line.GetNormalized()).GetRotated90Degrees();
	if (DotProduct2D(fwdNormal, impactNormal) > 0)
	{
		impactNormal *= -1.0f;
	}
	rayResult.m_didImpact = true;
	rayResult.m_impactDist = impactDist;
	rayResult.m_impactNormal = impactNormal;
	rayResult.m_impactPos = impactPos;
	return rayResult;
}

//-----------------------------------------------------------------------------------------------
RaycastResult2D RaycastVsAABB2D(Vec2 startPos, Vec2 fwdNormal, float maxDist, AABB2 const& aabb)
{
	RaycastResult2D rayResult;
	rayResult.m_rayStartPos = startPos;
	rayResult.m_rayMaxLength = maxDist;
	rayResult.m_rayFwdNormal = fwdNormal;

	Vec2 ray = fwdNormal * maxDist;
	Vec2 currentPos = startPos;
	//Start Inside Box
	if (IsPointInsideAABB2D(currentPos, aabb) == true)
	{
		rayResult.m_didImpact = true;
		rayResult.m_impactDist = 0.0f;
		rayResult.m_impactNormal = fwdNormal * -1.0f;
		rayResult.m_impactPos = startPos;
		return rayResult;
	}
	
	std::vector<RaycastResult2D> totalRayResults;
	totalRayResults.push_back(RaycastVsLineSegment2D(startPos, fwdNormal, maxDist, LineSegment2(aabb.m_mins.x, aabb.m_mins.y, aabb.m_mins.x, aabb.m_maxs.y)));
	totalRayResults.push_back(RaycastVsLineSegment2D(startPos, fwdNormal, maxDist, LineSegment2(aabb.m_mins.x, aabb.m_mins.y, aabb.m_maxs.x, aabb.m_mins.y)));
	totalRayResults.push_back(RaycastVsLineSegment2D(startPos, fwdNormal, maxDist, LineSegment2(aabb.m_mins.x, aabb.m_maxs.y, aabb.m_maxs.x, aabb.m_maxs.y)));
	totalRayResults.push_back(RaycastVsLineSegment2D(startPos, fwdNormal, maxDist, LineSegment2(aabb.m_maxs.x, aabb.m_mins.y, aabb.m_maxs.x, aabb.m_maxs.y)));
	
	float minDist = FLT_MAX;
	for (int rayIndex = 0; rayIndex < totalRayResults.size(); rayIndex++)
	{
		if (totalRayResults[rayIndex].m_didImpact == true)
		{
			if (totalRayResults[rayIndex].m_impactDist < minDist)
			{
				minDist = totalRayResults[rayIndex].m_impactDist;
				rayResult = totalRayResults[rayIndex];
			}
		}
	}
	return rayResult;
}

//-----------------------------------------------------------------------------------------------
RaycastResult3D RaycastVsSphere3D(Vec3 startPos, Mat44 const& rotationMatrix, float maxDist, Vec3 sphereCenter, float sphereRadius)
{
	RaycastResult3D result;
	result.m_rayStartPos = startPos;
	result.m_rayFwdNormal = rotationMatrix.GetIBasis3D();
	result.m_rayMaxLength = maxDist;
	result.m_didImpact = false;

	Vec3 displacement = sphereCenter - startPos;
	Vec3 iBasis = rotationMatrix.GetIBasis3D();;
	Vec3 jBasis = rotationMatrix.GetJBasis3D();
	Vec3 kBasis = rotationMatrix.GetKBasis3D();
	float displacementIBasis = DotProduct3D(displacement, iBasis);
	float displacementJBasis = DotProduct3D(displacement, jBasis);
	float displacementKBasis = DotProduct3D(displacement, kBasis);

	if (displacementJBasis >= sphereRadius)
	{
		return result;
	}
	else if (displacementKBasis >= sphereRadius)
	{
		return result;
	}
	else if (displacementIBasis <= -sphereRadius)
	{
		return result;
	}
	else if (displacementIBasis >= maxDist + sphereRadius)
	{
		return result;
	}

	float discRadiusSquared = sphereRadius * sphereRadius;
	float displacementJBasisSquared = displacementJBasis * displacementJBasis;
	float displacementKBasisSquared = displacementKBasis * displacementKBasis;

	if (discRadiusSquared - displacementJBasisSquared <= 0.0f)
	{
		return result;
	}
	if (discRadiusSquared - displacementKBasisSquared <= 0.0f)
	{
		return result;
	}

	float distanceToNearestEdgePoint = sqrtf(discRadiusSquared - displacementJBasisSquared);
	result.m_impactDist = displacementIBasis - distanceToNearestEdgePoint;

	if (result.m_impactDist >= maxDist)
	{
		return result;
	}

	if (IsPointInsideSphere3D(startPos, sphereCenter, sphereRadius) == true)
	{
		result.m_didImpact = true;
		result.m_impactPos = startPos;
		result.m_impactNormal = iBasis * -1.0f;
		return result;
	}

	if (result.m_impactDist < 0.0f)
	{
		return result;
	}

	result.m_didImpact = true;
	result.m_impactPos = startPos + (result.m_impactDist * iBasis);
	result.m_impactNormal = (result.m_impactPos - sphereCenter).GetNormalized();
	return result;
}

//-----------------------------------------------------------------------------------------------
RaycastResult2D RaycastVsPlane2D(Vec2 startPos, Vec2 fwdNormal, float maxDist, Plane2D& plane)
{
	RaycastResult2D rayResult;
	rayResult.m_rayFwdNormal = fwdNormal;
	rayResult.m_rayStartPos = startPos;
	rayResult.m_rayMaxLength = maxDist;

	Vec2 endPos = startPos + (fwdNormal * maxDist);
	float altitudeS = DotProduct2D(startPos, plane.m_normal) - plane.m_distanceFromOrigin;
	float altitudeE = DotProduct2D(endPos, plane.m_normal) - plane.m_distanceFromOrigin; 
	if (altitudeS * altitudeE >= 0.0f)
	{
		return rayResult;
	}
	
	float impactDist = -(altitudeS / DotProduct2D(fwdNormal, plane.m_normal));
	Vec2 impactPoint = startPos + fwdNormal * impactDist;

	rayResult.m_didImpact = true;
	rayResult.m_impactDist = impactDist;
	rayResult.m_impactPos = impactPoint;
	if (DotProduct2D(fwdNormal, plane.m_normal) >= 0.0f)
		rayResult.m_impactNormal = plane.m_normal * -1.0f;
	else
		rayResult.m_impactNormal = plane.m_normal;
	
	return rayResult;
}

//-----------------------------------------------------------------------------------------------
RaycastResult2D RaycastVsConvexHull2D(Vec2 startPos, Vec2 fwdNormal, float maxDist, ConvexHull2D& convexHull)
{
	RaycastResult2D bestRayResult;
	if (convexHull.IsPointInside(startPos))
	{
		bestRayResult.m_didImpact = true;
		bestRayResult.m_impactDist = 0;
		bestRayResult.m_impactPos = startPos;
		bestRayResult.m_impactNormal = fwdNormal * -1.0f;
		bestRayResult.m_rayFwdNormal = fwdNormal;
		bestRayResult.m_rayMaxLength = maxDist;
		bestRayResult.m_rayStartPos = startPos;
		return bestRayResult;
	}

	bool wasTrueEntryFound = false;
	bool wasTrueExitFound = false;
	Vec2 trueEntry;
	Vec2 trueExit;
	std::vector<RaycastResult2D> entries;
	std::vector<RaycastResult2D> exits;
	for (int planeIndex = 0; planeIndex < convexHull.m_boundingPlanes.size(); planeIndex++)
	{
		Plane2D& plane = convexHull.m_boundingPlanes[planeIndex];
		RaycastResult2D currentRayResult = RaycastVsPlane2D(startPos, fwdNormal, maxDist, plane);
		if (currentRayResult.m_didImpact)
		{
			if (DotProduct2D(currentRayResult.m_rayStartPos, plane.m_normal) < plane.m_distanceFromOrigin)
			{
				//Exit
				exits.push_back(currentRayResult);
			}
			else
			{
				//Entry
				entries.push_back(currentRayResult);
			}
		}
	}

	//Miss
	if (entries.size() == 0)
	{
		return RaycastResult2D();
	}
	
	float trueExitDist = FLT_MAX;
	for (int exitIndex = 0; exitIndex < exits.size(); exitIndex++)
	{
		float distSquaredToImpact = (startPos - exits[exitIndex].m_impactPos).GetLengthSquared();
		if (distSquaredToImpact < trueExitDist)
		{
			trueExitDist = distSquaredToImpact;
			trueExit = exits[exitIndex].m_impactPos;
			wasTrueExitFound = true;
		}
	}

	float trueEntryDist = 0.0f;
	for (int entryIndex = 0; entryIndex < entries.size(); entryIndex++)
	{
		float distSquaredToImpact = (startPos - entries[entryIndex].m_impactPos).GetLengthSquared();
		if (distSquaredToImpact > trueEntryDist && distSquaredToImpact < trueExitDist)
		{
			trueEntryDist = distSquaredToImpact;
			trueEntry = entries[entryIndex].m_impactPos;
			bestRayResult = entries[entryIndex];
			wasTrueEntryFound = true;
		}
	}

	if (wasTrueExitFound == false)
	{
		trueExit = startPos + (fwdNormal * maxDist);
	}
	if (convexHull.IsPointInside((trueEntry + trueExit) * 0.5f))
	{
		return bestRayResult;
	}

	return RaycastResult2D();
}

//-----------------------------------------------------------------------------------------------
RaycastResult3D RaycastVsPlane3D(Vec3 startPos, Vec3 fwdNormal, float maxDist, Plane3D& plane)
{
	RaycastResult3D rayResult;
	rayResult.m_rayFwdNormal = fwdNormal;
	rayResult.m_rayStartPos = startPos;
	rayResult.m_rayMaxLength = maxDist;

	Vec3 endPos = startPos + (fwdNormal * maxDist);
	float altitudeS = DotProduct3D(startPos, plane.m_normal) - plane.m_distanceFromOrigin;
	float altitudeE = DotProduct3D(endPos, plane.m_normal) - plane.m_distanceFromOrigin;
	if (altitudeS * altitudeE >= 0.0f)
	{
		return rayResult;
	}

	float impactDist = -(altitudeS / DotProduct3D(fwdNormal, plane.m_normal));
	Vec3 impactPoint = startPos + fwdNormal * impactDist;

	rayResult.m_didImpact = true;
	rayResult.m_impactDist = impactDist;
	rayResult.m_impactPos = impactPoint;
	if (DotProduct3D(fwdNormal, plane.m_normal) >= 0.0f)
		rayResult.m_impactNormal = plane.m_normal * -1.0f;
	else
		rayResult.m_impactNormal = plane.m_normal;

	return rayResult;
}

//-----------------------------------------------------------------------------------------------
RaycastResult3D RaycastVsAABB3D(Vec3 startPos, Vec3 fwdNormal, float maxDist, AABB3 const& aabb)
{
	RaycastResult3D rayResult;
	rayResult.m_rayStartPos = startPos;
	rayResult.m_rayMaxLength = maxDist;
	rayResult.m_rayFwdNormal = fwdNormal;

	Vec3 ray = fwdNormal * maxDist;
	Vec3 currentPos = startPos;
	//Start Inside Box
	if (IsPointInsideAABB3D(currentPos, aabb) == true)
	{
		rayResult.m_didImpact = true;
		rayResult.m_impactDist = 0.0f;
		rayResult.m_impactNormal = fwdNormal * -1.0f;
		rayResult.m_impactPos = startPos;
		return rayResult;
	}

	std::vector<RaycastResult3D> totalRayResults;
	Plane3D zPositive = Plane3D(Vec3(0.0f, 0.0f, 1.0f), DotProduct3D(aabb.m_maxs, Vec3(0.0f, 0.0f, 1.0f)));
	Plane3D zNegative = Plane3D(Vec3(0.0f, 0.0f, -1.0f), DotProduct3D(aabb.m_mins, Vec3(0.0f, 0.0f, -1.0f)));
	Plane3D yPositive = Plane3D(Vec3(0.0f, 1.0f, 0.0f), DotProduct3D(aabb.m_maxs, Vec3(0.0f, 1.0f, 0.0f)));
	Plane3D yNegative = Plane3D(Vec3(0.0f, -1.0f, 0.0f), DotProduct3D(aabb.m_mins, Vec3(0.0f, -1.0f, 0.0f)));
	Plane3D xPositive = Plane3D(Vec3(1.0f, 0.0f, 0.0f), DotProduct3D(aabb.m_maxs, Vec3(1.0f, 0.0f, 0.0f)));
	Plane3D xNegative = Plane3D(Vec3(-1.0f, 0.0f, 0.0f), DotProduct3D(aabb.m_mins, Vec3(-1.0f, 0.0f, 0.0f)));
	totalRayResults.push_back(RaycastVsPlane3D(startPos, fwdNormal, maxDist, zPositive));
	totalRayResults.push_back(RaycastVsPlane3D(startPos, fwdNormal, maxDist, zNegative));
	totalRayResults.push_back(RaycastVsPlane3D(startPos, fwdNormal, maxDist, yPositive));
	totalRayResults.push_back(RaycastVsPlane3D(startPos, fwdNormal, maxDist, yNegative));
	totalRayResults.push_back(RaycastVsPlane3D(startPos, fwdNormal, maxDist, xPositive));
	totalRayResults.push_back(RaycastVsPlane3D(startPos, fwdNormal, maxDist, xNegative));

	float minDist = FLT_MAX;
	for (int rayIndex = 0; rayIndex < totalRayResults.size(); rayIndex++)
	{
		if (totalRayResults[rayIndex].m_didImpact == true)
		{
			if (totalRayResults[rayIndex].m_impactDist < minDist)
			{
				minDist = totalRayResults[rayIndex].m_impactDist;
				rayResult = totalRayResults[rayIndex];
			}
		}
	}
	return rayResult;
}
