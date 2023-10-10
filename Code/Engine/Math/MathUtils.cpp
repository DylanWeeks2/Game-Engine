#include "MathUtils.hpp"
#define _USE_MATH_DEFINES
#include <math.h>
#include "Vec2.hpp"
#include "Vec3.hpp"
#include "IntVec2.hpp"
#include "AABB2.hpp"
#include "AABB3.hpp"
#include "LineSegment2.hpp"
#include "LineSegment3.hpp"
#include "Capsule2.hpp"
#include "Capsule3.hpp"
#include "OBB2.hpp"
#include "Vec4.hpp"
#include "Mat44.hpp"
#include "Engine/Core/ErrorWarningAssert.hpp"
#include <vector>

//-----------------------------------------------------------------------------------------------
float ConvertDegreesToRadians(float degrees)
{
	return degrees * float((M_PI / 180));
}

//-----------------------------------------------------------------------------------------------
float ConvertRadiansToDegrees(float radians)
{
	return radians * float((180 / M_PI));
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
float Atan2Degrees(float y, float x)
{
	return ConvertRadiansToDegrees(atan2f(y, x));
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
float DotProduct2D(Vec2 const& a, Vec2 const& b)
{
	return (a.x * b.x) + (a.y * b.y);
}

//-----------------------------------------------------------------------------------------------
float DotProduct3D(Vec3 const& a, Vec3 const& b)
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
	float sqrtValue = xSquared + ySquared;
	float squareRoot = sqrtf(sqrtValue);
	return squareRoot * squareRoot;
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
	float sqrtValue = xSquared + ySquared + zSquared;
	float squareRoot = sqrtf(sqrtValue);
	return squareRoot * squareRoot;
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
Vec3 const GetProjectedOnto3D(Vec3 const& vectorToProject, Vec3 const& vectorToProjectOnto)
{
	float projectedLength = GetProjectedLength3D(vectorToProject, vectorToProjectOnto);
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

	if (displacementToNearestPointBone.GetLength() < capsule.m_radius)
	{
		return true;
	}

	return false;
}

//-----------------------------------------------------------------------------------------------
bool IsPointInsideCapsule3D(Vec3 const& point, Capsule3 const& capsule)
{
	Vec3 nearestPointBone = GetNearestPointOnLineSegment3D(point, capsule.m_bone);
	Vec3 displacementToNearestPointBone = point - nearestPointBone;

	if (displacementToNearestPointBone.GetLength() < capsule.m_radius)
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
	if (GetDistanceSquared2D(centerA, centerB) < radiusA * radiusA + radiusB * radiusB)
	{
		return true;
	}

	return false;
}

//-----------------------------------------------------------------------------------------------
bool DoSpheresOverlap(Vec3 const& centerA, float radiusA, Vec3 const& centerB, float radiusB)
{
	if (GetDistanceSquared3D(centerA, centerB) < radiusA * radiusA + radiusB * radiusB)
	{
		return true;
	}
	return false;
}

//-----------------------------------------------------------------------------------------------
Vec2 GetNearestPointOnDisc2D(Vec2 const& referencePosition, Vec2 const& discCenter, float discRadius)
{
	Vec2 distance = referencePosition - discCenter;
	distance.ClampLength(discRadius);
	return discCenter + distance;
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

	if (DotProduct3D(lineSegmentDisplacement, startDisplacement) < 0)
	{
		return lineSegment.m_start;
	}

	if (DotProduct3D(lineSegmentDisplacement, endDisplacement) > 0)
	{
		return lineSegment.m_end;
	}

	Vec3 nearestPoint = GetProjectedOnto3D(startDisplacement, lineSegmentDisplacement);
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
	Vec3 nearestPointBone = GetNearestPointOnLineSegment3D(referencePosition, capsule.m_bone);
	Vec3 displacementToNearestPointBone = referencePosition - nearestPointBone;
	displacementToNearestPointBone.ClampLength(capsule.m_radius);

	return nearestPointBone + displacementToNearestPointBone;
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
	//Bounding Disc Rejection test
	Vec3 nearestPoint = fixedBox.GetNearestPoint(mobileSphereCenter);
	Vec3 differenceFromPointToCenter = mobileSphereCenter - nearestPoint;
	float distance = differenceFromPointToCenter.GetLength();
	if (distance == 0.0f)
	{
		Vec3 halfDimensions = fixedBox.GetDimensions() * 0.5f;
		Vec3 aabbCenter = fixedBox.GetCenter();
		float distanceToTop = (aabbCenter.y + halfDimensions.y) - mobileSphereCenter.y;
		float distanceToBottom = mobileSphereCenter.y - (aabbCenter.y - halfDimensions.y);
		float distanceToRight = (aabbCenter.x + halfDimensions.x) - mobileSphereCenter.x;
		float distanceToLeft = mobileSphereCenter.x - (aabbCenter.x - halfDimensions.x);

		if (distanceToTop < distanceToBottom && distanceToTop < distanceToLeft && distanceToTop < distanceToRight)
		{
			mobileSphereCenter.y += distanceToTop + sphereRadius;
			return true;
		}
		else if (distanceToBottom < distanceToTop && distanceToBottom < distanceToLeft && distanceToBottom < distanceToRight)
		{
			mobileSphereCenter.y -= distanceToBottom + sphereRadius;
			return true;
		}
		else if (distanceToRight < distanceToBottom && distanceToRight < distanceToLeft && distanceToRight < distanceToTop)
		{
			mobileSphereCenter.x += distanceToRight + sphereRadius;
			return true;
		}
		else if (distanceToLeft < distanceToTop && distanceToLeft < distanceToBottom && distanceToLeft < distanceToRight)
		{
			mobileSphereCenter.x -= distanceToLeft + sphereRadius;
			return true;
		}
		return false;
	}
	else if (distance < sphereRadius)
	{
		float offset = sphereRadius - distance;
		differenceFromPointToCenter.SetLength(offset);
		mobileSphereCenter += differenceFromPointToCenter;
		return true;
	}

	return false;
}

//-----------------------------------------------------------------------------------------------
bool PushSphereOutOfFixedSphere3D(Vec3& mobileSphereCenter, float mobileSphereRadius, Vec3 const& fixedSphereCenter, float fixedSphereRadius)
{
	Vec3 difference = (mobileSphereCenter - fixedSphereCenter);
	float distance = difference.GetLength();
	if (fixedSphereRadius + mobileSphereRadius >= distance)
	{
		float offset = fixedSphereRadius + mobileSphereRadius - distance;
		difference.SetLength(offset);
		mobileSphereCenter += difference;
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
