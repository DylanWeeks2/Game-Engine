static const uint threadX = 1;
static const uint threadY = 1;
static const uint threadZ = 1;

//BUFFER AND STRUCT LOGIC
//------------------------------------------------------------------------------------------------
struct uint64_t
{
	uint m_lowBits;
	uint m_highBits;
};

//------------------------------------------------------------------------------------------------
struct AABB3
{
	float3 m_mins;
	float3 m_maxs;
};

//------------------------------------------------------------------------------------------------
struct AABBCollisionObject
{
	double		m_vptrOffset;
	uint64_t	m_macroBitRegions;
	uint64_t	m_microBitRegions;
	float3		m_boundingDiscCenter;
	float		m_boundingDiscRadius;
	AABB3		m_aabb;
};

//------------------------------------------------------------------------------------------------
struct OBB3
{
	float3 m_center;
	float3 m_iBasisNormal;
	float3 m_jBasisNormal;
	float3 m_kBasisNormal;
	float3 m_halfDimensions;
};

//------------------------------------------------------------------------------------------------
struct OBBCollisionObject
{
	double		m_vptrOffset;
	uint64_t	m_macroBitRegions;
	uint64_t	m_microBitRegions;
	float3		m_boundingDiscCenter;
	float		m_boundingDiscRadius;
	OBB3		m_obb;
};

//------------------------------------------------------------------------------------------------
struct Cylinder3
{
	float3 m_start;
	float3 m_end;
	float3 m_iBasis;
	float3 m_jBasis;
	float3 m_kBasis;
	float  m_radius;
};

//------------------------------------------------------------------------------------------------
struct CylinderCollisionObject
{
	double		m_vptrOffset;
	uint64_t	m_macroBitRegions;
	uint64_t	m_microBitRegions;
	float3		m_boundingDiscCenter;
	float		m_boundingDiscRadius;
	Cylinder3	m_cylinder;
};

//------------------------------------------------------------------------------------------------
struct Capsule3
{
	Cylinder3	m_bone;
	float		m_radius;
};

//------------------------------------------------------------------------------------------------
struct CapsuleCollisionObject
{
	double		m_vptrOffset;
	uint64_t	m_macroBitRegions;
	uint64_t	m_microBitRegions;
	float3		m_boundingDiscCenter;
	float		m_boundingDiscRadius;
	Capsule3	m_capsule;
	float3		m_collisionNormalStart;
	float3		m_jacobiCorrectionStart;
	int			m_jacobiConstraintTotalStart;
	int			m_isCollidingStart;
	float3		m_collisionNormalEnd;
	float3		m_jacobiCorrectionEnd;
	int			m_jacobiConstraintTotalEnd;
	int			m_isCollidingEnd;
};

//------------------------------------------------------------------------------------------------
struct Sphere3
{
	float3 m_center;
	float  m_radius;
};

//------------------------------------------------------------------------------------------------
struct SphereCollisionObject
{
	double		m_vptrOffset;
	uint64_t	m_macroBitRegions;
	uint64_t	m_microBitRegions;
	float3		m_boundingDiscCenter;
	float		m_boundingDiscRadius;
	Sphere3 m_sphere;
};

//------------------------------------------------------------------------------------------------
struct LineSegment3
{
	float3 m_start;
	float3 m_end;
};

//------------------------------------------------------------------------------------------------
cbuffer RopeConstantBuffer : register(b0)
{
	uint	m_totalParticles;
	float	m_ropeRadius;
	float	m_gravityCoefficient;
	float	m_physicsTimestep;
	float	m_dampingCoefficient;
	int		m_totalSolverIterations;
	float	m_desiredDistance;
	float	m_compressionCoefficient;
	float	m_stretchingCoefficient;
	float	m_kineticFrictionCoefficient;
	float	m_bendingConstraintDistance;
	float	m_bendingCoefficient;
	AABB3	m_worldBounds;
	int		m_totalCollisionObjects;
	int		m_totalAABBs;
	int		m_totalOBBs;
	int		m_totalCylinders;
	int		m_totalCapsules;
	int		m_totalSpheres;
	int		m_isSelfCollisionEnabled;
};

//------------------------------------------------------------------------------------------------
RWStructuredBuffer<float3>					m_particleProposedPositions : register(u2);
RWStructuredBuffer<float4>					m_particleJacobiCorrections : register(u3);
RWStructuredBuffer<float3>					m_particleCollisionNormals : register(u4);
RWStructuredBuffer<CapsuleCollisionObject>	m_ropeCapsules:register(u7);
StructuredBuffer<int>						m_particleIsAttached : register(t0);
StructuredBuffer<AABBCollisionObject>		m_aabbs : register(t3);
StructuredBuffer<OBBCollisionObject>		m_obbs : register(t4);
StructuredBuffer<CapsuleCollisionObject>	m_capsules : register(t5);
StructuredBuffer<CylinderCollisionObject>	m_cylinders : register(t6);
StructuredBuffer<SphereCollisionObject>		m_spheres : register(t7);

//MATH UTILITIES
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
float GetLength(float3 vec3)
{
	float xSquared = vec3.x * vec3.x;
	float ySquared = vec3.y * vec3.y;
	float zSquared = vec3.z * vec3.z;
	float sqrtValue = xSquared + ySquared + zSquared;
	return sqrt(sqrtValue);
}

//-----------------------------------------------------------------------------------------------
float GetLengthSquared(float3 vec3)
{
	float xSquared = vec3.x * vec3.x;
	float ySquared = vec3.y * vec3.y;
	float zSquared = vec3.z * vec3.z;
	return (xSquared + ySquared + zSquared);
}

//-----------------------------------------------------------------------------------------------
float GetDistanceSquared3D(float3 positionA, float3 positionB)
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
bool DoSpheresOverlap(float3 centerA, float radiusA, float3 centerB, float radiusB)
{
	if (GetDistanceSquared3D(centerA, centerB) < (radiusA + radiusB) * (radiusA + radiusB))
	{
		return true;
	}

	return false;
}

//-----------------------------------------------------------------------------------------------
float3 GetNormalized(float3 vec3)
{
	if (GetLength(vec3) == 0.0f)
	{
		return float3(0.0f, 0.0f, 0.0f);
	}

	float scale = 1.0f / GetLength(vec3);
	return float3(vec3.x * scale, vec3.y * scale, vec3.z * scale);
}

//-----------------------------------------------------------------------------------------------
float DotProduct3D(float3 a, float3 b)
{
	return (a.x * b.x) + (a.y * b.y) + (a.z * b.z);
}

//-----------------------------------------------------------------------------------------------
float GetProjectedLength3D(float3 vectorToProject, float3 vectorToProjectOnto)
{
	return DotProduct3D(vectorToProject, GetNormalized(vectorToProjectOnto));
}

//-----------------------------------------------------------------------------------------------
float3 GetProjectedOnto3D(float3 vectorToProject, float3 vectorToProjectOnto)
{
	float projectedLength = GetProjectedLength3D(vectorToProject, vectorToProjectOnto);
	return projectedLength * GetNormalized(vectorToProjectOnto);
}

//-----------------------------------------------------------------------------------------------
float3 SetLength(float3 vecToAlter, float maxLength)
{
	vecToAlter = GetNormalized(vecToAlter) * maxLength;
	return vecToAlter;
}

//-----------------------------------------------------------------------------------------------
float3 GetNearestPointAABB3(AABB3 aabb, float3 referencePosition)
{
	float3 nearestPoint;

	//inside the box
	if (referencePosition.x > aabb.m_mins.x && referencePosition.x < aabb.m_maxs.x &&
		referencePosition.y > aabb.m_mins.y && referencePosition.y < aabb.m_maxs.y &&
		referencePosition.z > aabb.m_mins.z && referencePosition.z < aabb.m_maxs.z)
	{
		nearestPoint = referencePosition;
	}

	//X
	if (referencePosition.x < aabb.m_maxs.x)
	{
		if (referencePosition.x > aabb.m_mins.x)
		{
			nearestPoint.x = referencePosition.x;
		}
		else
		{
			nearestPoint.x = aabb.m_mins.x;
		}
	}
	else
	{
		nearestPoint.x = aabb.m_maxs.x;
	}
	//Y
	if (referencePosition.y < aabb.m_maxs.y)
	{
		if (referencePosition.y > aabb.m_mins.y)
		{
			nearestPoint.y = referencePosition.y;
		}
		else
		{
			nearestPoint.y = aabb.m_mins.y;
		}
	}
	else
	{
		nearestPoint.y = aabb.m_maxs.y;
	}
	//Z
	if (referencePosition.z < aabb.m_maxs.z)
	{
		if (referencePosition.z > aabb.m_mins.z)
		{
			nearestPoint.z = referencePosition.z;
		}
		else
		{
			nearestPoint.z = aabb.m_mins.z;
		}
	}
	else
	{
		nearestPoint.z = aabb.m_maxs.z;
	}

	return nearestPoint;
}

//-----------------------------------------------------------------------------------------------
float3 GetDimensionsAABB3(AABB3 aabb)
{
	float3 dimensions = float3(aabb.m_maxs.x - aabb.m_mins.x, aabb.m_maxs.y - aabb.m_mins.y, aabb.m_maxs.z - aabb.m_mins.z);
	return dimensions;
}

//-----------------------------------------------------------------------------------------------
float3 GetCenterAABB3(AABB3 aabb)
{
	float3 center = (aabb.m_mins + aabb.m_maxs) * 0.5f;
	return center;
}

//-----------------------------------------------------------------------------------------------
float3 GetNearestEdgePositionAABB3(AABB3 aabb, float3 referencePosition)
{
	float3 edgePos = referencePosition;
	float3 center = GetCenterAABB3(aabb);
	float3 halfDimensions = GetDimensionsAABB3(aabb) * 0.5f;
	float distancesToSides[6];
	distancesToSides[0] = ((center.x + halfDimensions.x) - referencePosition.x);
	distancesToSides[1] = (referencePosition.x - (center.x - halfDimensions.x));
	distancesToSides[2] = ((center.y + halfDimensions.y) - referencePosition.y);
	distancesToSides[3] = (referencePosition.y - (center.y - halfDimensions.y));
	distancesToSides[4] = ((center.z + halfDimensions.z) - referencePosition.z);
	distancesToSides[5] = (referencePosition.z - (center.z - halfDimensions.z));

	float bestDistance = 10000000000000000000.0f;
	int bestDistanceIndex = -1;
	float secondBestDistance = 10000000000000000000.0f;
	int secondBestDistanceIndex = -1;
	for (int distanceIndex = 0; distanceIndex < 6; distanceIndex++)
	{
		if (distancesToSides[distanceIndex] < bestDistance)
		{
			secondBestDistance = bestDistance;
			secondBestDistanceIndex = bestDistanceIndex;
			bestDistance = distancesToSides[distanceIndex];
			bestDistanceIndex = distanceIndex;
		}
		else if (distancesToSides[distanceIndex] < secondBestDistance)
		{
			secondBestDistance = distancesToSides[distanceIndex];
			secondBestDistanceIndex = distanceIndex;
		}
	}

	//Side 1
	if (bestDistanceIndex == 0)
	{
		edgePos.x += distancesToSides[bestDistanceIndex];
	}
	else if (bestDistanceIndex == 1)
	{
		edgePos.x -= distancesToSides[bestDistanceIndex];
	}
	else if (bestDistanceIndex == 2)
	{
		edgePos.y += distancesToSides[bestDistanceIndex];
	}
	else if (bestDistanceIndex == 3)
	{
		edgePos.y -= distancesToSides[bestDistanceIndex];
	}
	else if (bestDistanceIndex == 4)
	{
		edgePos.z += distancesToSides[bestDistanceIndex];
	}
	else if (bestDistanceIndex == 5)
	{
		edgePos.z -= distancesToSides[bestDistanceIndex];
	}

	//Side 2
	if (secondBestDistanceIndex == 0)
	{
		edgePos.x += distancesToSides[secondBestDistanceIndex];
		return edgePos;
	}
	else if (secondBestDistanceIndex == 1)
	{
		edgePos.x -= distancesToSides[secondBestDistanceIndex];
		return edgePos;
	}
	else if (secondBestDistanceIndex == 2)
	{
		edgePos.y += distancesToSides[secondBestDistanceIndex];
		return edgePos;
	}
	else if (secondBestDistanceIndex == 3)
	{
		edgePos.y -= distancesToSides[secondBestDistanceIndex];
		return edgePos;
	}
	else if (secondBestDistanceIndex == 4)
	{
		edgePos.z += distancesToSides[secondBestDistanceIndex];
		return edgePos;
	}
	else if (secondBestDistanceIndex == 5)
	{
		edgePos.z -= distancesToSides[secondBestDistanceIndex];
		return edgePos;
	}

	return edgePos;
}

//-----------------------------------------------------------------------------------------------
AABB3 SetDimensionsAABB3(AABB3 aabb, float3 newDimensions)
{
	float3 center = GetCenterAABB3(aabb);
	float originalCenterX = center.x;
	float originalCenterY = center.y;
	float originalCenterZ = center.z;

	float halfOfX = newDimensions.x * 0.5f;
	float halfOfY = newDimensions.y * 0.5f;
	float halfOfZ = newDimensions.z * 0.5f;

	aabb.m_mins.x = originalCenterX - halfOfX;
	aabb.m_maxs.x = originalCenterX + halfOfX;
	aabb.m_mins.y = originalCenterY - halfOfY;
	aabb.m_maxs.y = originalCenterY + halfOfY;
	aabb.m_mins.z = originalCenterZ - halfOfZ;
	aabb.m_maxs.z = originalCenterZ + halfOfZ;
	return aabb;
}

//-----------------------------------------------------------------------------------------------
bool IsPointInsideAABB3D(float3 refPoint, AABB3 aabb)
{
	if (refPoint.x > aabb.m_mins.x && refPoint.x < aabb.m_maxs.x &&
		refPoint.y > aabb.m_mins.y && refPoint.y < aabb.m_maxs.y &&
		refPoint.z > aabb.m_mins.z && refPoint.z < aabb.m_maxs.z)
	{
		return true;
	}

	return false;
}

//-----------------------------------------------------------------------------------------------
float GetLengthfloat2(float2 vec2) 
{
	float xSquared = vec2.x * vec2.x;
	float ySquared = vec2.y * vec2.y;
	float sqrtValue = xSquared + ySquared;
	return sqrt(float(sqrtValue));
}

//-----------------------------------------------------------------------------------------------
float2 GetNormalizedfloat2(float2 vec2)
{
	if (vec2.x == 0.0f && vec2.y == 0.0f)
	{
		return float2(0.0f, 0.0f);
	}
	float scale = 1.0f / GetLengthfloat2(vec2);
	return float2(vec2.x * scale, vec2.y * scale);
}

//-----------------------------------------------------------------------------------------------
float2 SetLengthfloat2(float2 vecToAlter, float newLength)
{
	vecToAlter = GetNormalizedfloat2(vecToAlter) * newLength;
	return vecToAlter;
}

//-----------------------------------------------------------------------------------------------
float2 ClampLengthfloat2(float2 vectorToClamp, float maxLength)
{
	float currentLength = GetLengthfloat2(vectorToClamp);
	if (currentLength > maxLength)
	{
		float scale = 1 / (currentLength / maxLength);
		vectorToClamp.x *= scale;
		vectorToClamp.y *= scale;
	}
	return vectorToClamp;
}

//-----------------------------------------------------------------------------------------------
float3 PushSphereOutOfFixedAABB3D(float3 mobileSphereCenter, float sphereRadius, AABB3 fixedBox)
{
	float3 previousPos = mobileSphereCenter;
	float3 nearestPoint = GetNearestPointAABB3(fixedBox, mobileSphereCenter);
	if (mobileSphereCenter.x == nearestPoint.x && mobileSphereCenter.y == nearestPoint.y && mobileSphereCenter.z == nearestPoint.z)
	{
		float3 halfDimensions = GetDimensionsAABB3(fixedBox) * 0.5f;
		float3 aabbCenter = GetCenterAABB3(fixedBox);
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
		return mobileSphereCenter;
		//return true;
	}
	else
	{
		float3 displacementFromPointToCenter = mobileSphereCenter - nearestPoint;
		float distSquaredPointToCenter = GetLengthSquared(displacementFromPointToCenter);
		if (distSquaredPointToCenter < sphereRadius * sphereRadius)
		{
			float offset = sphereRadius - sqrt(distSquaredPointToCenter);
			displacementFromPointToCenter = SetLength(displacementFromPointToCenter, offset);
			mobileSphereCenter += displacementFromPointToCenter;
			return mobileSphereCenter;
			//return true;
		}
	}

	return mobileSphereCenter;
	//return false;
}

//-----------------------------------------------------------------------------------------------
float3 PushDiscOutOfFixedOBB3D(float3 mobileSphereCenter, float sphereRadius, OBB3 fixedOBB)
{
	//Transform to local space
	float4x4 modelMatrix;
	modelMatrix[0] = float4(fixedOBB.m_iBasisNormal, 0.0f);
	modelMatrix[1] = float4(fixedOBB.m_jBasisNormal, 0.0f);
	modelMatrix[2] = float4(fixedOBB.m_kBasisNormal, 0.0f);
	modelMatrix[3] = float4(fixedOBB.m_center, 0.0f);

	AABB3 aabb;
	aabb.m_mins = float3(0.0f, 0.0f, 0.0f);
	aabb.m_maxs = float3(0.0f, 0.0f, 0.0f);
	float3 dimensions;
	dimensions.x = fixedOBB.m_halfDimensions.x + fixedOBB.m_halfDimensions.x;
	dimensions.y = fixedOBB.m_halfDimensions.y + fixedOBB.m_halfDimensions.y;
	dimensions.z = fixedOBB.m_halfDimensions.z + fixedOBB.m_halfDimensions.z;
	aabb = SetDimensionsAABB3(aabb, dimensions);

	float3 center = float3(0.0f, 0.0f, 0.0f);
	float3 displacementToSphere = mobileSphereCenter - fixedOBB.m_center;
	float projectionIBasis = DotProduct3D(displacementToSphere, fixedOBB.m_iBasisNormal);
	float projectionJBasis = DotProduct3D(displacementToSphere, fixedOBB.m_jBasisNormal);
	float projectionKBasis = DotProduct3D(displacementToSphere, fixedOBB.m_kBasisNormal);
	float3 localSphereCenter = center + float3(1.0f, 0.0f, 0.0f) * projectionIBasis + float3(0.0f, 1.0f, 0.0f) * projectionJBasis + float3(0.0f, 0.0f, 1.0f) * projectionKBasis;

	float3 newPosition = PushSphereOutOfFixedAABB3D(localSphereCenter, sphereRadius, aabb);
	if (newPosition.x != localSphereCenter.x || newPosition.y != localSphereCenter.y || newPosition.z != localSphereCenter.z)
	{
		//Tranform back to world space
		float3 transformedPosition = float3(0.0f, 0.0f, 0.0f);
		transformedPosition.x = (modelMatrix[0].x * newPosition.x) + (modelMatrix[1].x * newPosition.y) + (modelMatrix[2].x * newPosition.z) + modelMatrix[3].x;
		transformedPosition.y = (modelMatrix[0].y * newPosition.x) + (modelMatrix[1].y * newPosition.y) + (modelMatrix[2].y * newPosition.z) + modelMatrix[3].y;
		transformedPosition.z = (modelMatrix[0].z * newPosition.x) + (modelMatrix[1].z * newPosition.y) + (modelMatrix[2].z * newPosition.z) + modelMatrix[3].z;
		mobileSphereCenter = transformedPosition;
		return mobileSphereCenter;
		//return true;
	}

	return mobileSphereCenter;
	//return false;
}

//-----------------------------------------------------------------------------------------------
float2 GetNearestPointOnDisc2D(float2 referencePosition, float2 discCenter, float discRadius)
{
	float2 distance = referencePosition - discCenter;
	distance = ClampLengthfloat2(distance, discRadius);
	return discCenter + distance;
}

//-----------------------------------------------------------------------------------------------
float3 GetNearestPointOnCylinderZ3D(float3 referencePosition, Cylinder3 cylinder)
{
	float3 nearestPoint;
	float2 referencePositionXY = float2(referencePosition.x, referencePosition.y);
	float2 cylinderCenterXY = float2(cylinder.m_start.x, cylinder.m_start.y);
	if (referencePosition.z < cylinder.m_start.z)
	{
		float2 nearestPointXY = GetNearestPointOnDisc2D(referencePositionXY, cylinderCenterXY, cylinder.m_radius);
		nearestPoint = float3(nearestPointXY.x, nearestPointXY.y, cylinder.m_start.z);
	}
	else if (referencePosition.z > cylinder.m_end.z)
	{
		float2 nearestPointXY = GetNearestPointOnDisc2D(referencePositionXY, cylinderCenterXY, cylinder.m_radius);
		nearestPoint = float3(nearestPointXY.x, nearestPointXY.y, cylinder.m_end.z);
	}
	else
	{
		float2 nearestPointXY = GetNearestPointOnDisc2D(referencePositionXY, cylinderCenterXY, cylinder.m_radius);
		nearestPoint = float3(nearestPointXY.x, nearestPointXY.y, referencePosition.z);
	}
	return nearestPoint;
}

//-----------------------------------------------------------------------------------------------
float3 PushSphereOutOfFixedCylinderZ3D(float3 mobileSphereCenter, float mobileSphereRadius, Cylinder3 cylinder)
{
	float3 nearestPoint = GetNearestPointOnCylinderZ3D(mobileSphereCenter, cylinder);
	if (mobileSphereCenter.x == nearestPoint.x && mobileSphereCenter.y == nearestPoint.y && mobileSphereCenter.z == nearestPoint.z)
	{
		float2 displacementFromCylinderCenterXY = float2(mobileSphereCenter.x, mobileSphereCenter.y) - float2(cylinder.m_start.x, cylinder.m_start.y);
		float distanceToOutside = cylinder.m_radius - GetLengthfloat2(displacementFromCylinderCenterXY);
		float distanceToTop = abs(mobileSphereCenter.z - cylinder.m_end.z);
		float distanceToBottom = abs(mobileSphereCenter.z - cylinder.m_start.z);

		if (distanceToOutside < distanceToTop)
		{
			if (distanceToOutside < distanceToBottom)
			{
				float2 cylinderCenterXY = float2(cylinder.m_start.x, cylinder.m_start.y);
				float2 newCenter = cylinderCenterXY + GetNormalizedfloat2(displacementFromCylinderCenterXY) * (cylinder.m_radius + mobileSphereRadius);
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
		return mobileSphereCenter;
		//return true;
	}
	else
	{
		float3 differenceFromPointToCenter = mobileSphereCenter - nearestPoint;
		float distance = GetLengthSquared(differenceFromPointToCenter);
		if (distance < mobileSphereRadius * mobileSphereRadius)
		{
			float offset = mobileSphereRadius - sqrt(distance);
			differenceFromPointToCenter = SetLength(differenceFromPointToCenter, offset);
			mobileSphereCenter += differenceFromPointToCenter;
			return mobileSphereCenter;
			//return true;
		}
	}

	return mobileSphereCenter;
	//return false;
}

//-----------------------------------------------------------------------------------------------
float3 PushSphereOutOfFixedCylinder3D(float3 mobileSphereCenter, float mobileSphereRadius, Cylinder3 cylinder)
{
	//Transform to local space
	float4x4 modelMatrix;
	modelMatrix[0] = float4(cylinder.m_iBasis, 0.0f);
	modelMatrix[1] = float4(cylinder.m_jBasis, 0.0f);
	modelMatrix[2] = float4(cylinder.m_kBasis, 0.0f);
	modelMatrix[3] = float4(cylinder.m_start, 0.0f);

	float length = GetLength(cylinder.m_end - cylinder.m_start);
	Cylinder3 localCylinder;
	localCylinder.m_radius = cylinder.m_radius;
	localCylinder.m_start = float3(0.0f, 0.0f, 0.0f);
	localCylinder.m_end = localCylinder.m_start + float3(0.0f, 0.0f, length);
	localCylinder.m_iBasis = float3(1.0f, 0.0f, 0.0f);
	localCylinder.m_jBasis = float3(0.0f, 1.0f, 0.0f);
	localCylinder.m_kBasis = float3(0.0f, 0.0f, 1.0f);

	float3 displacementToSphere = mobileSphereCenter - cylinder.m_start;
	float projectionIBasis = DotProduct3D(displacementToSphere, cylinder.m_iBasis);
	float projectionJBasis = DotProduct3D(displacementToSphere, cylinder.m_jBasis);
	float projectionKBasis = DotProduct3D(displacementToSphere, cylinder.m_kBasis);
	float3 localSphereCenter = localCylinder.m_start + localCylinder.m_iBasis * projectionIBasis + localCylinder.m_jBasis * projectionJBasis + localCylinder.m_kBasis * projectionKBasis;

	float3 newPosition = PushSphereOutOfFixedCylinderZ3D(localSphereCenter, mobileSphereRadius, localCylinder);
	if (newPosition.x != localSphereCenter.x || newPosition.y != localSphereCenter.y || newPosition.z != localSphereCenter.z)
	{
		float3 transformedPosition = float3(0.0f, 0.0f, 0.0f);
		transformedPosition.x = (modelMatrix[0].x * newPosition.x) + (modelMatrix[1].x * newPosition.y) + (modelMatrix[2].x * newPosition.z) + modelMatrix[3].x;
		transformedPosition.y = (modelMatrix[0].y * newPosition.x) + (modelMatrix[1].y * newPosition.y) + (modelMatrix[2].y * newPosition.z) + modelMatrix[3].y;
		transformedPosition.z = (modelMatrix[0].z * newPosition.x) + (modelMatrix[1].z * newPosition.y) + (modelMatrix[2].z * newPosition.z) + modelMatrix[3].z;
		mobileSphereCenter = transformedPosition;
		return mobileSphereCenter;
		//return true
	}

	return mobileSphereCenter;
	//return false;
}

//-----------------------------------------------------------------------------------------------
float3 GetNearestPointOnLineSegment3D(float3 referencePosition, LineSegment3 lineSegment)
{
	float3 startDisplacement = referencePosition - lineSegment.m_start;
	float3 endDisplacement = referencePosition - lineSegment.m_end;
	float3 lineSegmentDisplacement = lineSegment.m_end - lineSegment.m_start;

	if (DotProduct3D(lineSegmentDisplacement, startDisplacement) < 0.0f)
	{
		return lineSegment.m_start;
	}

	if (DotProduct3D(lineSegmentDisplacement, endDisplacement) > 0.0f)
	{
		return lineSegment.m_end;
	}

	float3 nearestPoint = GetProjectedOnto3D(startDisplacement, lineSegmentDisplacement);
	return lineSegment.m_start + nearestPoint;
}

//-----------------------------------------------------------------------------------------------
float3 PushSphereOutOfFixedCapsule3D(float3 mobileSphereCenter, float mobileSphereRadius, Capsule3 capsule)
{
	LineSegment3 bone;
	bone.m_start = capsule.m_bone.m_start;
	bone.m_end = capsule.m_bone.m_end;
	float3 nearestPointBone = GetNearestPointOnLineSegment3D(mobileSphereCenter, bone);
	float3 differenceFromPointToCenter = mobileSphereCenter - nearestPointBone;
	float distance = GetLengthSquared(differenceFromPointToCenter);
	if (distance < ((mobileSphereRadius + capsule.m_radius) * (mobileSphereRadius + capsule.m_radius)))
	{
		float offset = mobileSphereRadius + capsule.m_radius - sqrt(distance);
		differenceFromPointToCenter = SetLength(differenceFromPointToCenter, offset);
		mobileSphereCenter += differenceFromPointToCenter;
		return mobileSphereCenter;
		//return true;
	}

	return mobileSphereCenter;
	//return false;
}

//-----------------------------------------------------------------------------------------------
float3 PushSphereOutOfFixedSphere3D(float3 mobileSphereCenter, float mobileSphereRadius, float3 fixedSphereCenter, float fixedSphereRadius)
{
	float3 difference = (mobileSphereCenter - fixedSphereCenter);
	float distanceSquared = GetLengthSquared(difference);
	if ((fixedSphereRadius + mobileSphereRadius) * (fixedSphereRadius + mobileSphereRadius) >= distanceSquared)
	{
		float offset = fixedSphereRadius + mobileSphereRadius - sqrt(distanceSquared);
		difference = SetLength(difference, offset);
		mobileSphereCenter += difference;
		return mobileSphereCenter;
		//return true;
	}

	return mobileSphereCenter;
	//return false;
}

//-----------------------------------------------------------------------------------------------
void PushCapsuleOutOfFixedSphere3D(Capsule3 capsule, float3 fixedSphereCenter, float fixedSphereRadius, out float3 returnPosition1, out float3 returnPosition2)
{
	LineSegment3 bone;
	bone.m_start = capsule.m_bone.m_start;
	bone.m_end = capsule.m_bone.m_end;
	float3 nearestPointOnBone = GetNearestPointOnLineSegment3D(fixedSphereCenter, bone);
	float3 originalNearestPointOnBone = nearestPointOnBone;
	float3 diplacementFromCenterToBone = nearestPointOnBone - fixedSphereCenter;
	float distanceFromCenterToBone = GetLengthSquared(diplacementFromCenterToBone);
	float overallRadii = (fixedSphereRadius + capsule.m_radius);
	if (distanceFromCenterToBone < overallRadii * overallRadii)
	{
		//Do Sphere Push
		float offset = overallRadii - sqrt(distanceFromCenterToBone);
		diplacementFromCenterToBone = SetLength(diplacementFromCenterToBone, offset);
		nearestPointOnBone += diplacementFromCenterToBone;

		//Get Push Difference and spread accurately to capsule start and end
		float3 nearestPointDisplacement = nearestPointOnBone - originalNearestPointOnBone;
		if (originalNearestPointOnBone.x == capsule.m_bone.m_start.x && originalNearestPointOnBone.y == capsule.m_bone.m_start.y && originalNearestPointOnBone.z == capsule.m_bone.m_start.z)
		{
			capsule.m_bone.m_start += nearestPointDisplacement;
		}
		else if (originalNearestPointOnBone.x == capsule.m_bone.m_end.x && originalNearestPointOnBone.y == capsule.m_bone.m_end.y && originalNearestPointOnBone.z == capsule.m_bone.m_end.z)
		{
			capsule.m_bone.m_end += nearestPointDisplacement;
		}
		else
		{
			float3 startToEndOfCapsuleDisplacement = capsule.m_bone.m_end - capsule.m_bone.m_start;
			float3 capsuleStartToLinePointDisplacement = (originalNearestPointOnBone - capsule.m_bone.m_end);
			float startToEndOfCapsuleDistance = GetLength(startToEndOfCapsuleDisplacement);
			float capsuleStartToLinePointDistance = GetLength(capsuleStartToLinePointDisplacement);
			float percentToPushStart = capsuleStartToLinePointDistance / startToEndOfCapsuleDistance;
			float percentToPushEnd = (startToEndOfCapsuleDistance - capsuleStartToLinePointDistance) / startToEndOfCapsuleDistance;
			capsule.m_bone.m_start += nearestPointDisplacement * percentToPushStart;
			capsule.m_bone.m_end += nearestPointDisplacement * percentToPushEnd;
		}
	}
	returnPosition1 = capsule.m_bone.m_start;
	returnPosition2 = capsule.m_bone.m_end;
}

//-----------------------------------------------------------------------------------------------
void PushCapsuleOutOfFixedAABB3D(Capsule3 capsule, AABB3 aabb, out float3 returnPosition1, out float3 returnPosition2)
{
	//Initial Sphere Pushes
	bool returnValue = false;
	capsule.m_bone.m_start = PushSphereOutOfFixedAABB3D(capsule.m_bone.m_start, capsule.m_radius, aabb);
	capsule.m_bone.m_end = PushSphereOutOfFixedAABB3D(capsule.m_bone.m_end, capsule.m_radius, aabb);

	//Calculate intersects and get best intersect
	float tValues[6];
	float bestTValue = 1000000000000.0f;
	float bestDistanceSquared = 1000000000000.0f;
	float3 bestIntersectPoint = float3(-1000.0f, -1000.0f, -1000.0f);
	float3 bestNearestPointAABB = float3(-1000.0f, -1000.0f, -1000.0f);
	float3 capsuleRayStartToEnd = (capsule.m_bone.m_end - capsule.m_bone.m_start);
	float rayZScale = 1.0 / capsuleRayStartToEnd.z;
	float rayYScale = 1.0 / capsuleRayStartToEnd.y;
	float rayXScale = 1.0 / capsuleRayStartToEnd.x;
	tValues[0] = ((aabb.m_maxs.z - capsule.m_bone.m_start.z) * rayZScale);
	tValues[1] = ((aabb.m_mins.z - capsule.m_bone.m_start.z) * rayZScale);
	tValues[2] = ((aabb.m_maxs.y - capsule.m_bone.m_start.y) * rayYScale);
	tValues[3] = ((aabb.m_mins.y - capsule.m_bone.m_start.y) * rayYScale);
	tValues[4] = ((aabb.m_maxs.x - capsule.m_bone.m_start.x) * rayXScale);
	tValues[5] = ((aabb.m_mins.x - capsule.m_bone.m_start.x) * rayXScale);
	for (int tIndex = 0; tIndex < 6; tIndex++)
	{
		if (tValues[tIndex] >= 0.0f && tValues[tIndex] <= 1.0f)
		{
			float3 intersectPoint = capsule.m_bone.m_start + (tValues[tIndex] * capsuleRayStartToEnd);
			float3 nearestPointAABB = GetNearestEdgePositionAABB3(aabb, intersectPoint);
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
		returnPosition1 = capsule.m_bone.m_start;
		returnPosition2 = capsule.m_bone.m_end;
		return;
	}

	//Calculate nearest points and get displacements
	float3 center = GetCenterAABB3(aabb);
	LineSegment3 bone;
	bone.m_start = capsule.m_bone.m_start;
	bone.m_end = capsule.m_bone.m_end;
	float3 nearestPointBone = GetNearestPointOnLineSegment3D(bestNearestPointAABB, bone);
	bestNearestPointAABB = GetNearestEdgePositionAABB3(aabb, nearestPointBone);
	nearestPointBone = GetNearestPointOnLineSegment3D(bestNearestPointAABB, bone);
	if (IsPointInsideAABB3D(nearestPointBone, aabb))
	{
		float3 displacementCenterToBone = nearestPointBone - center;
		float3 displacementCenterToCorner = bestNearestPointAABB - center;
		if (GetLengthSquared(displacementCenterToBone) < GetLengthSquared(displacementCenterToCorner))
		{
			float3 displacement = bestNearestPointAABB - nearestPointBone;
			displacement = SetLength(displacement, GetLength(displacement) + capsule.m_radius);
			capsule.m_bone.m_start += displacement;
			capsule.m_bone.m_end += displacement;
		}
	}
	else
	{
		float3 displacement = nearestPointBone - bestNearestPointAABB;
		float distanceSquared = GetLengthSquared(displacement);
		if (distanceSquared < capsule.m_radius * capsule.m_radius)
		{
			float offset = capsule.m_radius - sqrt(float(distanceSquared));
			displacement = SetLength(displacement, offset);
			capsule.m_bone.m_start += displacement;
			capsule.m_bone.m_end += displacement;
		}
	}
	
	returnPosition1 = capsule.m_bone.m_start;
	returnPosition2 = capsule.m_bone.m_end;
}

//-----------------------------------------------------------------------------------------------
void PushCapsuleOutOfFixedOBB3D(Capsule3 capsule, OBB3 obb, out float3 returnPosition1, out float3 returnPosition2)
{
	//Transform to local space
	float4x4 modelMatrix;
	modelMatrix[0] = float4(obb.m_iBasisNormal, 0.0f);
	modelMatrix[1] = float4(obb.m_jBasisNormal, 0.0f);
	modelMatrix[2] = float4(obb.m_kBasisNormal, 0.0f);
	modelMatrix[3] = float4(obb.m_center, 0.0f);

	AABB3 aabb;
	aabb.m_mins = float3(0.0f, 0.0f, 0.0f);
	aabb.m_maxs = float3(0.0f, 0.0f, 0.0f);
	float3 dimensions;
	dimensions.x = obb.m_halfDimensions.x + obb.m_halfDimensions.x;
	dimensions.y = obb.m_halfDimensions.y + obb.m_halfDimensions.y;
	dimensions.z = obb.m_halfDimensions.z + obb.m_halfDimensions.z;
	aabb = SetDimensionsAABB3(aabb, dimensions);

	float3 center = float3(0.0f, 0.0f, 0.0f);
	float3 displacementToCapsuleStart = capsule.m_bone.m_start - obb.m_center;
	float projectionIBasis = DotProduct3D(displacementToCapsuleStart, obb.m_iBasisNormal);
	float projectionJBasis = DotProduct3D(displacementToCapsuleStart, obb.m_jBasisNormal);
	float projectionKBasis = DotProduct3D(displacementToCapsuleStart, obb.m_kBasisNormal);
	float3 localCapsuleStart = center + float3(1.0f, 0.0f, 0.0f) * projectionIBasis + float3(0.0f, 1.0f, 0.0f) * projectionJBasis + float3(0.0f, 0.0f, 1.0f) * projectionKBasis;
	float3 displacementToCapsuleEnd = capsule.m_bone.m_end - obb.m_center;
	projectionIBasis = DotProduct3D(displacementToCapsuleEnd, obb.m_iBasisNormal);
	projectionJBasis = DotProduct3D(displacementToCapsuleEnd, obb.m_jBasisNormal);
	projectionKBasis = DotProduct3D(displacementToCapsuleEnd, obb.m_kBasisNormal);
	float3 localCapsuleEnd = center + float3(1.0f, 0.0f, 0.0f) * projectionIBasis + float3(0.0f, 1.0f, 0.0f) * projectionJBasis + float3(0.0f, 0.0f, 1.0f) * projectionKBasis;

	Capsule3 localCapsule;
	localCapsule.m_bone.m_start = localCapsuleStart;
	localCapsule.m_bone.m_end = localCapsuleEnd;
	localCapsule.m_radius = capsule.m_radius;
	localCapsule.m_bone.m_radius = capsule.m_bone.m_radius;
	localCapsule.m_bone.m_iBasis = capsule.m_bone.m_iBasis;
	localCapsule.m_bone.m_jBasis = capsule.m_bone.m_jBasis;
	localCapsule.m_bone.m_kBasis = capsule.m_bone.m_kBasis;

	PushCapsuleOutOfFixedAABB3D(localCapsule, aabb, returnPosition1, returnPosition2);
	if (returnPosition1.x != localCapsuleStart.x ||
		returnPosition1.y != localCapsuleStart.y ||
		returnPosition1.z != localCapsuleStart.z ||
		returnPosition2.x != localCapsuleEnd.x ||
		returnPosition2.y != localCapsuleEnd.y ||
		returnPosition2.z != localCapsuleEnd.z)
	{
		float3 transformedPosition = float3(0.0f, 0.0f, 0.0f);
		transformedPosition.x = (modelMatrix[0].x * returnPosition1.x) + (modelMatrix[1].x * returnPosition1.y) + (modelMatrix[2].x * returnPosition1.z) + modelMatrix[3].x;
		transformedPosition.y = (modelMatrix[0].y * returnPosition1.x) + (modelMatrix[1].y * returnPosition1.y) + (modelMatrix[2].y * returnPosition1.z) + modelMatrix[3].y;
		transformedPosition.z = (modelMatrix[0].z * returnPosition1.x) + (modelMatrix[1].z * returnPosition1.y) + (modelMatrix[2].z * returnPosition1.z) + modelMatrix[3].z;
		returnPosition1 = transformedPosition;
		transformedPosition = float3(0.0f, 0.0f, 0.0f); 
		transformedPosition.x = (modelMatrix[0].x * returnPosition2.x) + (modelMatrix[1].x * returnPosition2.y) + (modelMatrix[2].x * returnPosition2.z) + modelMatrix[3].x;
		transformedPosition.y = (modelMatrix[0].y * returnPosition2.x) + (modelMatrix[1].y * returnPosition2.y) + (modelMatrix[2].y * returnPosition2.z) + modelMatrix[3].y;
		transformedPosition.z = (modelMatrix[0].z * returnPosition2.x) + (modelMatrix[1].z * returnPosition2.y) + (modelMatrix[2].z * returnPosition2.z) + modelMatrix[3].z;
		returnPosition2 = transformedPosition;
	}
	else
	{
		returnPosition1 = capsule.m_bone.m_start;
		returnPosition2 = capsule.m_bone.m_end;
	}
}

//-----------------------------------------------------------------------------------------------
void GetNearestPointsBetweenLines3D(LineSegment3 lineA, LineSegment3 lineB, out float3 nearestPointLineA, out float3 nearestPointLineB)
{
	float3 directionA = lineA.m_end - lineA.m_start;
	float3 directionB = lineB.m_end - lineB.m_start;

	float b = DotProduct3D(directionA, directionB);
	if (b == 1 || b == -1)
	{
		nearestPointLineA = lineA.m_start;
		nearestPointLineB = lineB.m_start;
		return;
	}

	float3 r = lineA.m_start - lineB.m_start;
	float a = DotProduct3D(directionA, directionA);
	float c = DotProduct3D(directionA, r);
	float e = DotProduct3D(directionB, directionB);
	float f = DotProduct3D(directionB, r);
	float s = 0.0f;
	float t = 0.0f;
	float denom = a * e - b * b;

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
}

//-----------------------------------------------------------------------------------------------
void PushCapsuleOutOfFixedCylinderZ3D(Capsule3 mobileCapsule, Cylinder3 fixedCylinder, out float3 returnPosition1, out float3 returnPosition2)
{
	//Evaluate top and bottom intersections
	LineSegment3 mobileLine;
	mobileLine.m_start = mobileCapsule.m_bone.m_start;
	mobileLine.m_end = mobileCapsule.m_bone.m_end;
	LineSegment3 fixedLine;
	fixedLine.m_start = fixedCylinder.m_start;
	fixedLine.m_end = fixedCylinder.m_end;
	float3 topOfCylinder = float3(0.0f, 0.0f, 0.0f);
	float3 bottomOfCylinder = float3(0.0f, 0.0f, 0.0f);
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
		float3 nearestPoint1 = float3(0.0f, 0.0f, 0.0f);
		float3 nearestPoint2 = float3(0.0f, 0.0f, 0.0f);
		GetNearestPointsBetweenLines3D(mobileLine, fixedLine, nearestPoint1, nearestPoint2);
		float3 displacementNearestPoints = nearestPoint1 - nearestPoint2;
		float distanceNearestPointsSquared = GetLengthSquared(displacementNearestPoints);
		if (distanceNearestPointsSquared < (mobileCapsule.m_radius + fixedCylinder.m_radius) * (mobileCapsule.m_radius + fixedCylinder.m_radius))
		{
			float offset = (mobileCapsule.m_radius + fixedCylinder.m_radius - sqrt(float(distanceNearestPointsSquared))) * 0.5f;
			displacementNearestPoints = SetLength(displacementNearestPoints, offset);
			if (nearestPoint1.x == mobileCapsule.m_bone.m_start.x && nearestPoint1.y == mobileCapsule.m_bone.m_start.y && nearestPoint1.z == mobileCapsule.m_bone.m_start.z)
			{
				mobileCapsule.m_bone.m_start += displacementNearestPoints;
			}
			else if (nearestPoint1.x == mobileCapsule.m_bone.m_end.x && nearestPoint1.y == mobileCapsule.m_bone.m_end.y && nearestPoint1.z == mobileCapsule.m_bone.m_end.z)
			{
				mobileCapsule.m_bone.m_end += displacementNearestPoints;
			}
			else
			{
				float3 startToEndOfCapsuleDisplacement = mobileCapsule.m_bone.m_end - mobileCapsule.m_bone.m_start;
				float3 capsuleStartToLinePointDisplacement = (nearestPoint1 - mobileCapsule.m_bone.m_end);
				float startToEndOfCapsuleDistance = GetLength(startToEndOfCapsuleDisplacement);
				float capsuleStartToLinePointDistance = GetLength(capsuleStartToLinePointDisplacement);
				float percentToPushStart = capsuleStartToLinePointDistance / startToEndOfCapsuleDistance;
				float percentToPushEnd = (startToEndOfCapsuleDistance - capsuleStartToLinePointDistance) / startToEndOfCapsuleDistance;
				mobileCapsule.m_bone.m_start += displacementNearestPoints * percentToPushStart;
				mobileCapsule.m_bone.m_end += displacementNearestPoints * percentToPushEnd;
			}
			/*mobileCapsule.m_bone.m_start += displacementNearestPoints;
			mobileCapsule.m_bone.m_end += displacementNearestPoints;*/
			returnPosition1 = mobileCapsule.m_bone.m_start;
			returnPosition2 = mobileCapsule.m_bone.m_end;
			return;
		}
	}
	//Top
	else if (mobileCapsule.m_bone.m_start.z >= fixedCylinder.m_end.z || mobileCapsule.m_bone.m_end.z >= fixedCylinder.m_end.z)
	{
		bool returnValue = false;
		float3 nearestPointOnBone = mobileCapsule.m_bone.m_end;
		float3 nearestPointCylinder = GetNearestPointOnCylinderZ3D(nearestPointOnBone, fixedCylinder);
		float3 displacement = nearestPointOnBone - nearestPointCylinder;
		float distanceSquared = GetLengthSquared(displacement);
		if (nearestPointOnBone.x == nearestPointCylinder.x && nearestPointOnBone.y == nearestPointCylinder.y && nearestPointOnBone.z == nearestPointCylinder.z)
		{
			float distanceToTop = fixedCylinder.m_end.z - nearestPointCylinder.z;
			float3 deltaPos = float3(0.0f, 0.0f, 1.0f) * (distanceToTop + mobileCapsule.m_radius);
			mobileCapsule.m_bone.m_end += deltaPos;
			returnValue = true;
		}
		else if (distanceSquared < (mobileCapsule.m_radius * mobileCapsule.m_radius))
		{
			float offset = mobileCapsule.m_radius - sqrt(float(distanceSquared));
			displacement = SetLength(displacement, offset);
			mobileCapsule.m_bone.m_end += displacement;
			returnValue = true;
		}

		nearestPointOnBone = mobileCapsule.m_bone.m_start;
		nearestPointCylinder = GetNearestPointOnCylinderZ3D(nearestPointOnBone, fixedCylinder);
		displacement = nearestPointOnBone - nearestPointCylinder;
		distanceSquared = GetLengthSquared(displacement);
		if (nearestPointOnBone.x == nearestPointCylinder.x && nearestPointOnBone.y == nearestPointCylinder.y && nearestPointOnBone.z == nearestPointCylinder.z)
		{
			float distanceToTop = fixedCylinder.m_end.z - nearestPointCylinder.z;
			float3 deltaPos = float3(0.0f, 0.0f, 1.0f) * (distanceToTop + mobileCapsule.m_radius);
			mobileCapsule.m_bone.m_start += deltaPos;
			returnValue = true;
		}
		else if (distanceSquared < (mobileCapsule.m_radius * mobileCapsule.m_radius))
		{
			float offset = mobileCapsule.m_radius - sqrt(float(distanceSquared));
			displacement = SetLength(displacement, offset);
			mobileCapsule.m_bone.m_start += displacement;
			returnValue = true;
		}

		float3 mobileCapsuleDireciton = mobileCapsule.m_bone.m_end - mobileCapsule.m_bone.m_start;
		float tTop = (topOfCylinder.z - mobileCapsule.m_bone.m_start.z) / mobileCapsuleDireciton.z;
		if (mobileCapsuleDireciton.z != 0.0f)
		{
			mobileLine.m_start = mobileCapsule.m_bone.m_start;
			mobileLine.m_end = mobileCapsule.m_bone.m_end;
			float3 intersectPoint = mobileCapsule.m_bone.m_start + tTop * mobileCapsuleDireciton;
			float3 nearestPointCylinder = GetNearestPointOnCylinderZ3D(intersectPoint, fixedCylinder);
			float3 nearestPointOnBone = GetNearestPointOnLineSegment3D(nearestPointCylinder, mobileLine);
			nearestPointCylinder = GetNearestPointOnCylinderZ3D(nearestPointOnBone, fixedCylinder);
			nearestPointOnBone = GetNearestPointOnLineSegment3D(nearestPointCylinder, mobileLine);
			float3 displacement = nearestPointOnBone - nearestPointCylinder;
			float distanceSquared = GetLengthSquared(displacement);
			if (nearestPointOnBone.x == nearestPointCylinder.x && nearestPointOnBone.y == nearestPointCylinder.y && nearestPointOnBone.z == nearestPointCylinder.z)
			{
				float3 cylinderDisplacement = nearestPointCylinder - fixedCylinder.m_end;
				cylinderDisplacement = SetLength(cylinderDisplacement, fixedCylinder.m_radius);
				nearestPointCylinder = fixedCylinder.m_end + cylinderDisplacement;
				nearestPointOnBone = GetNearestPointOnLineSegment3D(nearestPointCylinder, mobileLine);
				displacement = nearestPointCylinder - nearestPointOnBone;
				displacement = SetLength(displacement, GetLength(displacement) + mobileCapsule.m_radius);
				if (nearestPointOnBone.x == mobileCapsule.m_bone.m_start.x && nearestPointOnBone.y == mobileCapsule.m_bone.m_start.y && nearestPointOnBone.z == mobileCapsule.m_bone.m_start.z)
				{
					mobileCapsule.m_bone.m_start += displacement;
				}
				else if (nearestPointOnBone.x == mobileCapsule.m_bone.m_end.x && nearestPointOnBone.y == mobileCapsule.m_bone.m_end.y && nearestPointOnBone.z == mobileCapsule.m_bone.m_end.z)
				{
					mobileCapsule.m_bone.m_end += displacement;
				}
				else
				{
					float3 startToEndOfCapsuleDisplacement = mobileCapsule.m_bone.m_end - mobileCapsule.m_bone.m_start;
					float3 capsuleStartToLinePointDisplacement = (nearestPointOnBone - mobileCapsule.m_bone.m_end);
					float startToEndOfCapsuleDistance = GetLength(startToEndOfCapsuleDisplacement);
					float capsuleStartToLinePointDistance = GetLength(capsuleStartToLinePointDisplacement);
					float percentToPushStart = capsuleStartToLinePointDistance / startToEndOfCapsuleDistance;
					float percentToPushEnd = (startToEndOfCapsuleDistance - capsuleStartToLinePointDistance) / startToEndOfCapsuleDistance;
					mobileCapsule.m_bone.m_start += displacement * percentToPushStart;
					mobileCapsule.m_bone.m_end += displacement * percentToPushEnd;
				}
				/*mobileCapsule.m_bone.m_start += displacement;
				mobileCapsule.m_bone.m_end += displacement;*/
				returnPosition1 = mobileCapsule.m_bone.m_start;
				returnPosition2 = mobileCapsule.m_bone.m_end;
				return;
			}
			else if (distanceSquared < mobileCapsule.m_radius * mobileCapsule.m_radius)
			{
				float offset = mobileCapsule.m_radius - sqrt(float(distanceSquared));
				displacement = SetLength(displacement, offset);
				if (nearestPointOnBone.x == mobileCapsule.m_bone.m_start.x && nearestPointOnBone.y == mobileCapsule.m_bone.m_start.y && nearestPointOnBone.z == mobileCapsule.m_bone.m_start.z)
				{
					mobileCapsule.m_bone.m_start += displacement;
				}
				else if (nearestPointOnBone.x == mobileCapsule.m_bone.m_end.x && nearestPointOnBone.y == mobileCapsule.m_bone.m_end.y && nearestPointOnBone.z == mobileCapsule.m_bone.m_end.z)
				{
					mobileCapsule.m_bone.m_end += displacement;
				}
				else
				{
					float3 startToEndOfCapsuleDisplacement = mobileCapsule.m_bone.m_end - mobileCapsule.m_bone.m_start;
					float3 capsuleStartToLinePointDisplacement = (nearestPointOnBone - mobileCapsule.m_bone.m_end);
					float startToEndOfCapsuleDistance = GetLength(startToEndOfCapsuleDisplacement);
					float capsuleStartToLinePointDistance = GetLength(capsuleStartToLinePointDisplacement);
					float percentToPushStart = capsuleStartToLinePointDistance / startToEndOfCapsuleDistance;
					float percentToPushEnd = (startToEndOfCapsuleDistance - capsuleStartToLinePointDistance) / startToEndOfCapsuleDistance;
					mobileCapsule.m_bone.m_start += displacement * percentToPushStart;
					mobileCapsule.m_bone.m_end += displacement * percentToPushEnd;
				}
				/*mobileCapsule.m_bone.m_start += displacement;
				mobileCapsule.m_bone.m_end += displacement;*/
				returnPosition1 = mobileCapsule.m_bone.m_start;
				returnPosition2 = mobileCapsule.m_bone.m_end;
				return;
			}


		}
		if (returnValue)
		{
			returnPosition1 = mobileCapsule.m_bone.m_start;
			returnPosition2 = mobileCapsule.m_bone.m_end;
			return;
		}
	}
	//Bottom
	else if (mobileCapsule.m_bone.m_start.z <= bottomOfCylinder.z || mobileCapsule.m_bone.m_end.z <= bottomOfCylinder.z)
	{
		bool returnValue = false;
		float3 nearestPointOnBone = mobileCapsule.m_bone.m_end;
		float3 nearestPointCylinder = GetNearestPointOnCylinderZ3D(nearestPointOnBone, fixedCylinder);
		float3 displacement = nearestPointOnBone - nearestPointCylinder;
		float distanceSquared = GetLengthSquared(displacement);
		if (nearestPointOnBone.x == nearestPointCylinder.x && nearestPointOnBone.y == nearestPointCylinder.y && nearestPointOnBone.z == nearestPointCylinder.z)
		{
			float distanceToBottom = nearestPointCylinder.z - fixedCylinder.m_start.z;
			float3 deltaPos = float3(0.0f, 0.0f, -1.0f) * (distanceToBottom + mobileCapsule.m_radius);
			mobileCapsule.m_bone.m_end += deltaPos;
			returnValue = true;
		}
		else if (distanceSquared < (mobileCapsule.m_radius * mobileCapsule.m_radius))
		{
			float offset = mobileCapsule.m_radius - sqrt(float(distanceSquared));
			displacement = SetLength(displacement, offset);
			mobileCapsule.m_bone.m_end += displacement;
			returnValue = true;
		}

		nearestPointOnBone = mobileCapsule.m_bone.m_start;
		nearestPointCylinder = GetNearestPointOnCylinderZ3D(nearestPointOnBone, fixedCylinder);
		displacement = nearestPointOnBone - nearestPointCylinder;
		distanceSquared = GetLengthSquared(displacement);
		if (nearestPointOnBone.x == nearestPointCylinder.x && nearestPointOnBone.y == nearestPointCylinder.y && nearestPointOnBone.z == nearestPointCylinder.z)
		{
			float distanceToBottom = nearestPointCylinder.z - fixedCylinder.m_start.z;
			float3 deltaPos = float3(0.0f, 0.0f, -1.0f) * (distanceToBottom + mobileCapsule.m_radius);
			mobileCapsule.m_bone.m_start += deltaPos;
			returnValue = true;
		}
		else if (distanceSquared < (mobileCapsule.m_radius * mobileCapsule.m_radius))
		{
			float offset = mobileCapsule.m_radius - sqrt(float(distanceSquared));
			displacement = SetLength(displacement, offset);
			mobileCapsule.m_bone.m_start += displacement;
			returnValue = true;
		}

		float3 mobileCapsuleDireciton = mobileCapsule.m_bone.m_end - mobileCapsule.m_bone.m_start;
		float tBottom = (bottomOfCylinder.z - mobileCapsule.m_bone.m_start.z) / mobileCapsuleDireciton.z;
		if (mobileCapsuleDireciton.z != 0.0f)
		{
			mobileLine.m_start = mobileCapsule.m_bone.m_start;
			mobileLine.m_end = mobileCapsule.m_bone.m_end;
			float3 intersectPoint = mobileCapsule.m_bone.m_start + tBottom * mobileCapsuleDireciton;
			float3 nearestPointCylinder = GetNearestPointOnCylinderZ3D(intersectPoint, fixedCylinder);
			float3 nearestPointOnBone = GetNearestPointOnLineSegment3D(nearestPointCylinder, mobileLine);
			nearestPointCylinder = GetNearestPointOnCylinderZ3D(nearestPointOnBone, fixedCylinder);
			nearestPointOnBone = GetNearestPointOnLineSegment3D(nearestPointCylinder, mobileLine);
			float3 displacement = nearestPointOnBone - nearestPointCylinder;
			float distanceSquared = GetLengthSquared(displacement);
			if (nearestPointOnBone.x == nearestPointCylinder.x && nearestPointOnBone.y == nearestPointCylinder.y && nearestPointOnBone.z == nearestPointCylinder.z)
			{
				float3 cylinderDisplacement = nearestPointCylinder - fixedCylinder.m_end;
				cylinderDisplacement = SetLength(cylinderDisplacement, fixedCylinder.m_radius);
				nearestPointCylinder = fixedCylinder.m_end + cylinderDisplacement;
				nearestPointOnBone = GetNearestPointOnLineSegment3D(nearestPointCylinder, mobileLine);
				displacement = nearestPointCylinder - nearestPointOnBone;
				displacement = SetLength(displacement, GetLength(displacement) + mobileCapsule.m_radius);
				if (nearestPointOnBone.x == mobileCapsule.m_bone.m_start.x && nearestPointOnBone.y == mobileCapsule.m_bone.m_start.y && nearestPointOnBone.z == mobileCapsule.m_bone.m_start.z)
				{
					mobileCapsule.m_bone.m_start += displacement;
				}
				else if (nearestPointOnBone.x == mobileCapsule.m_bone.m_end.x && nearestPointOnBone.y == mobileCapsule.m_bone.m_end.y && nearestPointOnBone.z == mobileCapsule.m_bone.m_end.z)
				{
					mobileCapsule.m_bone.m_end += displacement;
				}
				else
				{
					float3 startToEndOfCapsuleDisplacement = mobileCapsule.m_bone.m_end - mobileCapsule.m_bone.m_start;
					float3 capsuleStartToLinePointDisplacement = (nearestPointOnBone - mobileCapsule.m_bone.m_end);
					float startToEndOfCapsuleDistance = GetLength(startToEndOfCapsuleDisplacement);
					float capsuleStartToLinePointDistance = GetLength(capsuleStartToLinePointDisplacement);
					float percentToPushStart = capsuleStartToLinePointDistance / startToEndOfCapsuleDistance;
					float percentToPushEnd = (startToEndOfCapsuleDistance - capsuleStartToLinePointDistance) / startToEndOfCapsuleDistance;
					mobileCapsule.m_bone.m_start += displacement * percentToPushStart;
					mobileCapsule.m_bone.m_end += displacement * percentToPushEnd;
				}
				/*mobileCapsule.m_bone.m_start += displacement;
				mobileCapsule.m_bone.m_end += displacement;*/
				returnPosition1 = mobileCapsule.m_bone.m_start;
				returnPosition2 = mobileCapsule.m_bone.m_end;
				return;
			}
			else if (distanceSquared < mobileCapsule.m_radius * mobileCapsule.m_radius)
			{
				float offset = mobileCapsule.m_radius - sqrt(float(distanceSquared));
				displacement = SetLength(displacement, offset);
				if (nearestPointOnBone.x == mobileCapsule.m_bone.m_start.x && nearestPointOnBone.y == mobileCapsule.m_bone.m_start.y && nearestPointOnBone.z == mobileCapsule.m_bone.m_start.z)
				{
					mobileCapsule.m_bone.m_start += displacement;
				}
				else if (nearestPointOnBone.x == mobileCapsule.m_bone.m_end.x && nearestPointOnBone.y == mobileCapsule.m_bone.m_end.y && nearestPointOnBone.z == mobileCapsule.m_bone.m_end.z)
				{
					mobileCapsule.m_bone.m_end += displacement;
				}
				else
				{
					float3 startToEndOfCapsuleDisplacement = mobileCapsule.m_bone.m_end - mobileCapsule.m_bone.m_start;
					float3 capsuleStartToLinePointDisplacement = (nearestPointOnBone - mobileCapsule.m_bone.m_end);
					float startToEndOfCapsuleDistance = GetLength(startToEndOfCapsuleDisplacement);
					float capsuleStartToLinePointDistance = GetLength(capsuleStartToLinePointDisplacement);
					float percentToPushStart = capsuleStartToLinePointDistance / startToEndOfCapsuleDistance;
					float percentToPushEnd = (startToEndOfCapsuleDistance - capsuleStartToLinePointDistance) / startToEndOfCapsuleDistance;
					mobileCapsule.m_bone.m_start += displacement * percentToPushStart;
					mobileCapsule.m_bone.m_end += displacement * percentToPushEnd;
				}
				/*mobileCapsule.m_bone.m_start += displacement;
				mobileCapsule.m_bone.m_end += displacement;*/
				returnPosition1 = mobileCapsule.m_bone.m_start;
				returnPosition2 = mobileCapsule.m_bone.m_end;
				return;
			}


		}
		if (returnValue)
		{
			returnPosition1 = mobileCapsule.m_bone.m_start;
			returnPosition2 = mobileCapsule.m_bone.m_end;
			return;
		}
	}

	returnPosition1 = mobileCapsule.m_bone.m_start;
	returnPosition2 = mobileCapsule.m_bone.m_end;
	return;
}

//-----------------------------------------------------------------------------------------------
void PushCapsuleOutOfFixedCylinder3D(Capsule3 mobileCapsule, Cylinder3 fixedCylinder, out float3 returnPosition1, out float3 returnPosition2)
{
	//Transform to local space
	float4x4 modelMatrix;
	modelMatrix[0] = float4(fixedCylinder.m_iBasis, 0.0f);
	modelMatrix[1] = float4(fixedCylinder.m_jBasis, 0.0f);
	modelMatrix[2] = float4(fixedCylinder.m_kBasis, 0.0f);
	modelMatrix[3] = float4(fixedCylinder.m_start, 0.0f);

	float length = GetLength(fixedCylinder.m_end - fixedCylinder.m_start);
	Cylinder3 localCylinder;
	localCylinder.m_radius = fixedCylinder.m_radius;
	localCylinder.m_start = float3(0.0f, 0.0f, 0.0f);
	localCylinder.m_end = localCylinder.m_start + float3(0.0f, 0.0f, length);
	localCylinder.m_iBasis = float3(1.0f, 0.0f, 0.0f);
	localCylinder.m_jBasis = float3(0.0f, 1.0f, 0.0f);
	localCylinder.m_kBasis = float3(0.0f, 0.0f, 1.0f);

	float3 displacementToCapsuleStart = mobileCapsule.m_bone.m_start - fixedCylinder.m_start;
	float projectionIBasis = DotProduct3D(displacementToCapsuleStart, fixedCylinder.m_iBasis);
	float projectionJBasis = DotProduct3D(displacementToCapsuleStart, fixedCylinder.m_jBasis);
	float projectionKBasis = DotProduct3D(displacementToCapsuleStart, fixedCylinder.m_kBasis);
	float3 localCapsuleStart = localCylinder.m_start + localCylinder.m_iBasis * projectionIBasis + localCylinder.m_jBasis * projectionJBasis + localCylinder.m_kBasis * projectionKBasis;
	float3 displacementToCapsuleEnd = mobileCapsule.m_bone.m_end - fixedCylinder.m_start;
	projectionIBasis = DotProduct3D(displacementToCapsuleEnd, fixedCylinder.m_iBasis);
	projectionJBasis = DotProduct3D(displacementToCapsuleEnd, fixedCylinder.m_jBasis);
	projectionKBasis = DotProduct3D(displacementToCapsuleEnd, fixedCylinder.m_kBasis);
	float3 localCapsuleEnd = localCylinder.m_start + localCylinder.m_iBasis * projectionIBasis + localCylinder.m_jBasis * projectionJBasis + localCylinder.m_kBasis * projectionKBasis;

	Capsule3 localCapsule;
	localCapsule.m_bone.m_start = localCapsuleStart;
	localCapsule.m_bone.m_end = localCapsuleEnd;
	localCapsule.m_radius = mobileCapsule.m_radius;
	localCapsule.m_bone.m_radius = mobileCapsule.m_bone.m_radius;
	localCapsule.m_bone.m_iBasis = mobileCapsule.m_bone.m_iBasis;
	localCapsule.m_bone.m_jBasis = mobileCapsule.m_bone.m_jBasis;
	localCapsule.m_bone.m_kBasis = mobileCapsule.m_bone.m_kBasis;

	PushCapsuleOutOfFixedCylinderZ3D(localCapsule, localCylinder, returnPosition1, returnPosition2);
	if (returnPosition1.x != localCapsuleStart.x ||
		returnPosition1.y != localCapsuleStart.y ||
		returnPosition1.z != localCapsuleStart.z ||
		returnPosition2.x != localCapsuleEnd.x ||
		returnPosition2.y != localCapsuleEnd.y ||
		returnPosition2.z != localCapsuleEnd.z)
	{
		float3 transformedPosition = float3(0.0f, 0.0f, 0.0f);
		transformedPosition.x = (modelMatrix[0].x * returnPosition1.x) + (modelMatrix[1].x * returnPosition1.y) + (modelMatrix[2].x * returnPosition1.z) + modelMatrix[3].x;
		transformedPosition.y = (modelMatrix[0].y * returnPosition1.x) + (modelMatrix[1].y * returnPosition1.y) + (modelMatrix[2].y * returnPosition1.z) + modelMatrix[3].y;
		transformedPosition.z = (modelMatrix[0].z * returnPosition1.x) + (modelMatrix[1].z * returnPosition1.y) + (modelMatrix[2].z * returnPosition1.z) + modelMatrix[3].z;
		returnPosition1 = transformedPosition;
		transformedPosition = float3(0.0f, 0.0f, 0.0f);
		transformedPosition.x = (modelMatrix[0].x * returnPosition2.x) + (modelMatrix[1].x * returnPosition2.y) + (modelMatrix[2].x * returnPosition2.z) + modelMatrix[3].x;
		transformedPosition.y = (modelMatrix[0].y * returnPosition2.x) + (modelMatrix[1].y * returnPosition2.y) + (modelMatrix[2].y * returnPosition2.z) + modelMatrix[3].y;
		transformedPosition.z = (modelMatrix[0].z * returnPosition2.x) + (modelMatrix[1].z * returnPosition2.y) + (modelMatrix[2].z * returnPosition2.z) + modelMatrix[3].z;
		returnPosition2 = transformedPosition;
	}
	else
	{
		returnPosition1 = mobileCapsule.m_bone.m_start;
		returnPosition2 = mobileCapsule.m_bone.m_end;
	}
}

//-----------------------------------------------------------------------------------------------
void PushCapsuleOutOfFixedCapsule3D(Capsule3 mobileCapsule, Capsule3 fixedCapsule, out float3 returnPosition1, out float3 returnPosition2)
{
	LineSegment3 mobileLine;
	mobileLine.m_start = mobileCapsule.m_bone.m_start;
	mobileLine.m_end = mobileCapsule.m_bone.m_end;
	LineSegment3 fixedLine;
	fixedLine.m_start = fixedCapsule.m_bone.m_start;
	fixedLine.m_end = fixedCapsule.m_bone.m_end;
	float3 nearestPoint1 = float3(0.0f, 0.0f, 0.0f);
	float3 nearestPoint2 = float3(0.0f, 0.0f, 0.0f);
	GetNearestPointsBetweenLines3D(mobileLine, fixedLine, nearestPoint1, nearestPoint2);

	float3 displacementNearestPoints = nearestPoint1 - nearestPoint2;
	float distanceNearestPoints = GetLengthSquared(displacementNearestPoints);
	float overallRadii = mobileCapsule.m_radius + fixedCapsule.m_radius;
	if (distanceNearestPoints < overallRadii * overallRadii)
	{
		float offset = overallRadii - sqrt(distanceNearestPoints);
		displacementNearestPoints = SetLength(displacementNearestPoints, offset);
		if (nearestPoint1.x == mobileCapsule.m_bone.m_start.x && nearestPoint1.y == mobileCapsule.m_bone.m_start.y && nearestPoint1.z == mobileCapsule.m_bone.m_start.z)
		{
			mobileCapsule.m_bone.m_start += displacementNearestPoints;
		}
		else if (nearestPoint1.x == mobileCapsule.m_bone.m_end.x && nearestPoint1.y == mobileCapsule.m_bone.m_end.y && nearestPoint1.z == mobileCapsule.m_bone.m_end.z)
		{
			mobileCapsule.m_bone.m_end += displacementNearestPoints;
		}
		else
		{
			float3 startToEndOfCapsuleDisplacement = mobileCapsule.m_bone.m_end - mobileCapsule.m_bone.m_start;
			float3 capsuleStartToLinePointDisplacement = (nearestPoint1 - mobileCapsule.m_bone.m_end);
			float startToEndOfCapsuleDistance = GetLength(startToEndOfCapsuleDisplacement);
			float capsuleStartToLinePointDistance = GetLength(capsuleStartToLinePointDisplacement);
			float percentToPushStart = capsuleStartToLinePointDistance / startToEndOfCapsuleDistance;
			float percentToPushEnd = (startToEndOfCapsuleDistance - capsuleStartToLinePointDistance) / startToEndOfCapsuleDistance;
			mobileCapsule.m_bone.m_start += displacementNearestPoints * percentToPushStart;
			mobileCapsule.m_bone.m_end += displacementNearestPoints * percentToPushEnd;
		}
		/*mobileCapsule.m_bone.m_start += displacementNearestPoints;
		mobileCapsule.m_bone.m_end += displacementNearestPoints;*/
	}

	returnPosition1 = mobileCapsule.m_bone.m_start;
	returnPosition2 = mobileCapsule.m_bone.m_end;
}

//-----------------------------------------------------------------------------------------------
void PushCapsuleOutOfCapsule3D(Capsule3 capsuleA, Capsule3 capsuleB, out float3 returnPosition1, out float3 returnPosition2)
{
	LineSegment3 lineA;
	lineA.m_start = capsuleA.m_bone.m_start;
	lineA.m_end = capsuleA.m_bone.m_end;
	LineSegment3 lineB;
	lineB.m_start = capsuleB.m_bone.m_start;
	lineB.m_end = capsuleB.m_bone.m_end;
	float3 nearestPoint1 = float3(0.0f, 0.0f, 0.0f);
	float3 nearestPoint2 = float3(0.0f, 0.0f, 0.0f);
	GetNearestPointsBetweenLines3D(lineA, lineB, nearestPoint1, nearestPoint2);

	float3 displacementNearestPoints = nearestPoint1 - nearestPoint2;
	float distanceNearestPoints = GetLengthSquared(displacementNearestPoints);
	float overallRadii = capsuleA.m_radius + capsuleB.m_radius;
	if (distanceNearestPoints < overallRadii * overallRadii)
	{
		float offset = (overallRadii - sqrt(distanceNearestPoints)) * 0.5f;
		displacementNearestPoints = SetLength(displacementNearestPoints, offset);
		if (nearestPoint1.x == capsuleA.m_bone.m_start.x && nearestPoint1.y == capsuleA.m_bone.m_start.y && nearestPoint1.z == capsuleA.m_bone.m_start.z)
		{
			capsuleA.m_bone.m_start += displacementNearestPoints;
		}
		else if (nearestPoint1.x == capsuleA.m_bone.m_end.x && nearestPoint1.y == capsuleA.m_bone.m_end.y && nearestPoint1.z == capsuleA.m_bone.m_end.z)
		{
			capsuleA.m_bone.m_end += displacementNearestPoints;
		}
		else
		{
			float3 startToEndOfCapsuleDisplacement = capsuleA.m_bone.m_end - capsuleA.m_bone.m_start;
			float3 capsuleStartToLinePointDisplacement = (nearestPoint1 - capsuleA.m_bone.m_end);
			float startToEndOfCapsuleDistance = GetLength(startToEndOfCapsuleDisplacement);
			float capsuleStartToLinePointDistance = GetLength(capsuleStartToLinePointDisplacement);
			float percentToPushStart = capsuleStartToLinePointDistance / startToEndOfCapsuleDistance;
			float percentToPushEnd = (startToEndOfCapsuleDistance - capsuleStartToLinePointDistance) / startToEndOfCapsuleDistance;
			capsuleA.m_bone.m_start += displacementNearestPoints * percentToPushStart;
			capsuleA.m_bone.m_end += displacementNearestPoints * percentToPushEnd;
		}
		/*capsuleA.m_bone.m_start += displacementNearestPoints;
		capsuleA.m_bone.m_end += displacementNearestPoints;*/
	}

	returnPosition1 = capsuleA.m_bone.m_start;
	returnPosition2 = capsuleA.m_bone.m_end;
}

//-----------------------------------------------------------------------------------------------
void ProjectWorldBoundsConstraintsJacobi(int sentParticleIndex)
{
	//Initializations
	float3 newPosition = m_particleProposedPositions[sentParticleIndex];

	//Min Z
	if (m_particleProposedPositions[sentParticleIndex].z < m_worldBounds.m_mins.z + m_ropeRadius)
	{
		newPosition.z = m_worldBounds.m_mins.z + m_ropeRadius;
		m_particleCollisionNormals[sentParticleIndex] = float3(0.0f, 0.0f, 1.0f);
		m_particleJacobiCorrections[sentParticleIndex].w++;
	}
	//Max Z
	if (m_particleProposedPositions[sentParticleIndex].z > m_worldBounds.m_maxs.z - m_ropeRadius)
	{
		newPosition.z = m_worldBounds.m_maxs.z - m_ropeRadius;
		m_particleCollisionNormals[sentParticleIndex] = float3(0.0f, 0.0f, -1.0f);
		m_particleJacobiCorrections[sentParticleIndex].w++;
	}
	//Min X
	if (m_particleProposedPositions[sentParticleIndex].x < m_worldBounds.m_mins.x + m_ropeRadius)
	{
		newPosition.x = m_worldBounds.m_mins.x + m_ropeRadius;
		m_particleCollisionNormals[sentParticleIndex] = float3(1.0f, 0.0f, 0.0f);
		m_particleJacobiCorrections[sentParticleIndex].w++;
	}
	//Max X
	if (m_particleProposedPositions[sentParticleIndex].x > m_worldBounds.m_maxs.x - m_ropeRadius)
	{
		newPosition.x = m_worldBounds.m_maxs.x - m_ropeRadius;
		m_particleCollisionNormals[sentParticleIndex] = float3(-1.0f, 0.0f, 0.0f);
		m_particleJacobiCorrections[sentParticleIndex].w++;
	}
	//Min Y
	if (m_particleProposedPositions[sentParticleIndex].y < m_worldBounds.m_mins.y + m_ropeRadius)
	{
		newPosition.y = m_worldBounds.m_mins.y + m_ropeRadius;
		m_particleCollisionNormals[sentParticleIndex] = float3(0.0f, 1.0f, 0.0f);
		m_particleJacobiCorrections[sentParticleIndex].w++;
	}
	//Max Y
	if (m_particleProposedPositions[sentParticleIndex].y > m_worldBounds.m_maxs.y - m_ropeRadius)
	{
		newPosition.y = m_worldBounds.m_maxs.y - m_ropeRadius;
		m_particleCollisionNormals[sentParticleIndex] = float3(0.0f, -1.0f, 0.0f);
		m_particleJacobiCorrections[sentParticleIndex].w++;
	}

	//Update back into buffer
	m_particleJacobiCorrections[sentParticleIndex].xyz += (newPosition - m_particleProposedPositions[sentParticleIndex]);
}

//COMPUTE SHADER LOGIC
//------------------------------------------------------------------------------------------------
[numthreads(threadX, threadY, threadZ)]
void CSMain(uint3 threadGroupId : SV_GroupID, uint threadIndex : SV_GroupIndex)
{
	//Initializations
	uint startIndex = threadGroupId.x * threadX;
	uint sentCapsuleIndex = startIndex + (threadIndex);
	sentCapsuleIndex *= 2;
	sentCapsuleIndex += 1;
	if (sentCapsuleIndex >= m_totalParticles)
		return;

	//Self Collision
	int totalCapsules = m_totalParticles - 1;
	if (m_isSelfCollisionEnabled == 1)
	{
		for (int capsuleIndex = 0; capsuleIndex < totalCapsules; capsuleIndex++)
		{
			//Early Out Check
			if (capsuleIndex == sentCapsuleIndex || capsuleIndex == sentCapsuleIndex + 1 || capsuleIndex == sentCapsuleIndex - 1 || capsuleIndex == sentCapsuleIndex + 2 || capsuleIndex == sentCapsuleIndex - 2)
				continue;

			//Initialization
			CapsuleCollisionObject currentCapsule = m_ropeCapsules[capsuleIndex];

			//Macro Check
			uint macroLowBitResult = (m_ropeCapsules[sentCapsuleIndex].m_macroBitRegions.m_lowBits & currentCapsule.m_macroBitRegions.m_lowBits);
			uint macroHighBitResult = (m_ropeCapsules[sentCapsuleIndex].m_macroBitRegions.m_highBits & currentCapsule.m_macroBitRegions.m_highBits);
			if (m_ropeCapsules[sentCapsuleIndex].m_macroBitRegions.m_highBits == 0 && currentCapsule.m_macroBitRegions.m_highBits == 0)
			{
				macroHighBitResult = 1;
			}
			if (m_ropeCapsules[sentCapsuleIndex].m_macroBitRegions.m_lowBits == 0 && currentCapsule.m_macroBitRegions.m_lowBits == 0)
			{
				macroLowBitResult = 1;
			}
			if (macroLowBitResult != 0 || macroHighBitResult != 0)
			{
				//Micro Check
				uint microLowBitResult = (m_ropeCapsules[sentCapsuleIndex].m_microBitRegions.m_lowBits & currentCapsule.m_microBitRegions.m_lowBits);
				uint microHighBitResult = (m_ropeCapsules[sentCapsuleIndex].m_microBitRegions.m_highBits & currentCapsule.m_microBitRegions.m_highBits);
				if (m_ropeCapsules[sentCapsuleIndex].m_microBitRegions.m_highBits == 0 && currentCapsule.m_microBitRegions.m_highBits == 0)
				{
					microHighBitResult = 1;
				}
				if (m_ropeCapsules[sentCapsuleIndex].m_microBitRegions.m_lowBits == 0 && currentCapsule.m_microBitRegions.m_lowBits == 0)
				{
					microLowBitResult = 1;
				}
				if (microLowBitResult != 0 || microHighBitResult != 0)
				{
					//Bounding Disc Check
					if (DoSpheresOverlap(m_ropeCapsules[sentCapsuleIndex].m_boundingDiscCenter, m_ropeCapsules[sentCapsuleIndex].m_boundingDiscRadius, currentCapsule.m_boundingDiscCenter, currentCapsule.m_boundingDiscRadius))
					{
						//Capsule vs Capsule
						float3 newStartPosition = float3(0.0f, 0.0f, 0.0f);
						float3 newEndPosition = float3(0.0f, 0.0f, 0.0f);
						PushCapsuleOutOfCapsule3D(m_ropeCapsules[sentCapsuleIndex].m_capsule, currentCapsule.m_capsule, newStartPosition, newEndPosition);
						if (newStartPosition.x != m_ropeCapsules[sentCapsuleIndex].m_capsule.m_bone.m_start.x ||
							newStartPosition.y != m_ropeCapsules[sentCapsuleIndex].m_capsule.m_bone.m_start.y ||
							newStartPosition.z != m_ropeCapsules[sentCapsuleIndex].m_capsule.m_bone.m_start.z)
						{
							float3 displacementStart = newStartPosition - m_ropeCapsules[sentCapsuleIndex].m_capsule.m_bone.m_start;
							m_ropeCapsules[sentCapsuleIndex].m_isCollidingStart = 1;
							m_ropeCapsules[sentCapsuleIndex].m_collisionNormalStart = GetNormalized(displacementStart);
							m_ropeCapsules[sentCapsuleIndex].m_jacobiCorrectionStart += (displacementStart);
							m_ropeCapsules[sentCapsuleIndex].m_jacobiConstraintTotalStart++;
						}
						if (newEndPosition.x != m_ropeCapsules[sentCapsuleIndex].m_capsule.m_bone.m_end.x ||
							newEndPosition.y != m_ropeCapsules[sentCapsuleIndex].m_capsule.m_bone.m_end.y ||
							newEndPosition.z != m_ropeCapsules[sentCapsuleIndex].m_capsule.m_bone.m_end.z)
						{
							float3 displacementEnd = newEndPosition - m_ropeCapsules[sentCapsuleIndex].m_capsule.m_bone.m_end;
							m_ropeCapsules[sentCapsuleIndex].m_isCollidingEnd = 1;
							m_ropeCapsules[sentCapsuleIndex].m_collisionNormalEnd = GetNormalized(displacementEnd);
							m_ropeCapsules[sentCapsuleIndex].m_jacobiCorrectionEnd += (displacementEnd);
							m_ropeCapsules[sentCapsuleIndex].m_jacobiConstraintTotalEnd++;
						}
					}
				}
			}
		}
	}

	//AABBs
	for (int aabbIndex = 0; aabbIndex < m_totalAABBs; aabbIndex++)
	{
		//Macro Check
		uint macroLowBitResult = (m_ropeCapsules[sentCapsuleIndex].m_macroBitRegions.m_lowBits & m_aabbs[aabbIndex].m_macroBitRegions.m_lowBits);
		uint macroHighBitResult = (m_ropeCapsules[sentCapsuleIndex].m_macroBitRegions.m_highBits & m_aabbs[aabbIndex].m_macroBitRegions.m_highBits);
		if (m_ropeCapsules[sentCapsuleIndex].m_macroBitRegions.m_highBits == 0 && m_aabbs[aabbIndex].m_macroBitRegions.m_highBits == 0)
		{
			macroHighBitResult = 1;
		}
		if (m_ropeCapsules[sentCapsuleIndex].m_macroBitRegions.m_lowBits == 0 && m_aabbs[aabbIndex].m_macroBitRegions.m_lowBits == 0)
		{
			macroLowBitResult = 1;
		}
		if (macroLowBitResult != 0 || macroHighBitResult != 0)
		{
			//Micro Check
			uint microLowBitResult = (m_ropeCapsules[sentCapsuleIndex].m_microBitRegions.m_lowBits & m_aabbs[aabbIndex].m_microBitRegions.m_lowBits);
			uint microHighBitResult = (m_ropeCapsules[sentCapsuleIndex].m_microBitRegions.m_highBits & m_aabbs[aabbIndex].m_microBitRegions.m_highBits);
			if (m_ropeCapsules[sentCapsuleIndex].m_microBitRegions.m_highBits == 0 && m_aabbs[aabbIndex].m_microBitRegions.m_highBits == 0)
			{
				microHighBitResult = 1;
			}
			if (m_ropeCapsules[sentCapsuleIndex].m_microBitRegions.m_lowBits == 0 && m_aabbs[aabbIndex].m_microBitRegions.m_lowBits == 0)
			{
				microLowBitResult = 1;
			}
			if (microLowBitResult != 0 || microHighBitResult != 0)
			{
				//Bounding Disc Check
				if (DoSpheresOverlap(m_ropeCapsules[sentCapsuleIndex].m_boundingDiscCenter, m_ropeCapsules[sentCapsuleIndex].m_boundingDiscRadius, m_aabbs[aabbIndex].m_boundingDiscCenter, m_aabbs[aabbIndex].m_boundingDiscRadius))
				{
					//Capsule vs AABB
					float3 newStartPosition = float3(0.0f, 0.0f, 0.0f);
					float3 newEndPosition = float3(0.0f, 0.0f, 0.0f);
					PushCapsuleOutOfFixedAABB3D(m_ropeCapsules[sentCapsuleIndex].m_capsule, m_aabbs[aabbIndex].m_aabb, newStartPosition, newEndPosition);
					if (newStartPosition.x != m_ropeCapsules[sentCapsuleIndex].m_capsule.m_bone.m_start.x ||
						newStartPosition.y != m_ropeCapsules[sentCapsuleIndex].m_capsule.m_bone.m_start.y ||
						newStartPosition.z != m_ropeCapsules[sentCapsuleIndex].m_capsule.m_bone.m_start.z)
					{
						float3 displacementStart = newStartPosition - m_ropeCapsules[sentCapsuleIndex].m_capsule.m_bone.m_start;
						m_ropeCapsules[sentCapsuleIndex].m_isCollidingStart = 1;
						m_ropeCapsules[sentCapsuleIndex].m_collisionNormalStart = GetNormalized(displacementStart);
						m_ropeCapsules[sentCapsuleIndex].m_jacobiCorrectionStart += (displacementStart);
						m_ropeCapsules[sentCapsuleIndex].m_jacobiConstraintTotalStart++;
					}
					if (newEndPosition.x != m_ropeCapsules[sentCapsuleIndex].m_capsule.m_bone.m_end.x ||
						newEndPosition.y != m_ropeCapsules[sentCapsuleIndex].m_capsule.m_bone.m_end.y ||
						newEndPosition.z != m_ropeCapsules[sentCapsuleIndex].m_capsule.m_bone.m_end.z)
					{
						float3 displacementEnd = newEndPosition - m_ropeCapsules[sentCapsuleIndex].m_capsule.m_bone.m_end;
						m_ropeCapsules[sentCapsuleIndex].m_isCollidingEnd = 1;
						m_ropeCapsules[sentCapsuleIndex].m_collisionNormalEnd = GetNormalized(displacementEnd);
						m_ropeCapsules[sentCapsuleIndex].m_jacobiCorrectionEnd += (displacementEnd);
						m_ropeCapsules[sentCapsuleIndex].m_jacobiConstraintTotalEnd++;
					}
				}
			}
		}
	}

	//OBBs
	for (int obbIndex = 0; obbIndex < m_totalOBBs; obbIndex++)
	{
		//Macro Check
		uint macroLowBitResult = (m_ropeCapsules[sentCapsuleIndex].m_macroBitRegions.m_lowBits & m_obbs[obbIndex].m_macroBitRegions.m_lowBits);
		uint macroHighBitResult = (m_ropeCapsules[sentCapsuleIndex].m_macroBitRegions.m_highBits & m_obbs[obbIndex].m_macroBitRegions.m_highBits);
		if (m_ropeCapsules[sentCapsuleIndex].m_macroBitRegions.m_highBits == 0 && m_obbs[obbIndex].m_macroBitRegions.m_highBits == 0)
		{
			macroHighBitResult = 1;
		}
		if (m_ropeCapsules[sentCapsuleIndex].m_macroBitRegions.m_lowBits == 0 && m_obbs[obbIndex].m_macroBitRegions.m_lowBits == 0)
		{
			macroLowBitResult = 1;
		}
		if (macroLowBitResult != 0 || macroHighBitResult != 0)
		{
			//Micro Check
			uint microLowBitResult = (m_ropeCapsules[sentCapsuleIndex].m_microBitRegions.m_lowBits & m_obbs[obbIndex].m_microBitRegions.m_lowBits);
			uint microHighBitResult = (m_ropeCapsules[sentCapsuleIndex].m_microBitRegions.m_highBits & m_obbs[obbIndex].m_microBitRegions.m_highBits);
			if (m_ropeCapsules[sentCapsuleIndex].m_microBitRegions.m_highBits == 0 && m_obbs[obbIndex].m_microBitRegions.m_highBits == 0)
			{
				microHighBitResult = 1;
			}
			if (m_ropeCapsules[sentCapsuleIndex].m_microBitRegions.m_lowBits == 0 && m_obbs[obbIndex].m_microBitRegions.m_lowBits == 0)
			{
				microLowBitResult = 1;
			}
			if (microLowBitResult != 0 || microHighBitResult != 0)
			{
				//Bounding Disc Check
				if (DoSpheresOverlap(m_ropeCapsules[sentCapsuleIndex].m_boundingDiscCenter, m_ropeCapsules[sentCapsuleIndex].m_boundingDiscRadius, m_obbs[obbIndex].m_boundingDiscCenter, m_obbs[obbIndex].m_boundingDiscRadius))
				{
					//Capsule vs OBB
					float3 newStartPosition = float3(0.0f, 0.0f, 0.0f);
					float3 newEndPosition = float3(0.0f, 0.0f, 0.0f);
					PushCapsuleOutOfFixedOBB3D(m_ropeCapsules[sentCapsuleIndex].m_capsule, m_obbs[obbIndex].m_obb, newStartPosition, newEndPosition);
					if (newStartPosition.x != m_ropeCapsules[sentCapsuleIndex].m_capsule.m_bone.m_start.x ||
						newStartPosition.y != m_ropeCapsules[sentCapsuleIndex].m_capsule.m_bone.m_start.y ||
						newStartPosition.z != m_ropeCapsules[sentCapsuleIndex].m_capsule.m_bone.m_start.z)
					{
						float3 displacementStart = newStartPosition - m_ropeCapsules[sentCapsuleIndex].m_capsule.m_bone.m_start;
						m_ropeCapsules[sentCapsuleIndex].m_isCollidingStart = 1;
						m_ropeCapsules[sentCapsuleIndex].m_collisionNormalStart = GetNormalized(displacementStart);
						m_ropeCapsules[sentCapsuleIndex].m_jacobiCorrectionStart += (displacementStart);
						m_ropeCapsules[sentCapsuleIndex].m_jacobiConstraintTotalStart++;
					}
					if (newEndPosition.x != m_ropeCapsules[sentCapsuleIndex].m_capsule.m_bone.m_end.x ||
						newEndPosition.y != m_ropeCapsules[sentCapsuleIndex].m_capsule.m_bone.m_end.y ||
						newEndPosition.z != m_ropeCapsules[sentCapsuleIndex].m_capsule.m_bone.m_end.z)
					{
						float3 displacementEnd = newEndPosition - m_ropeCapsules[sentCapsuleIndex].m_capsule.m_bone.m_end;
						m_ropeCapsules[sentCapsuleIndex].m_isCollidingEnd = 1;
						m_ropeCapsules[sentCapsuleIndex].m_collisionNormalEnd = GetNormalized(displacementEnd);
						m_ropeCapsules[sentCapsuleIndex].m_jacobiCorrectionEnd += (displacementEnd);
						m_ropeCapsules[sentCapsuleIndex].m_jacobiConstraintTotalEnd++;
					}
				}
			}
		}
	}

	//Cylinders
	for (int cylinderIndex = 0; cylinderIndex < m_totalCylinders; cylinderIndex++)
	{
		//Macro Check
		uint macroLowBitResult = (m_ropeCapsules[sentCapsuleIndex].m_macroBitRegions.m_lowBits & m_cylinders[cylinderIndex].m_macroBitRegions.m_lowBits);
		uint macroHighBitResult = (m_ropeCapsules[sentCapsuleIndex].m_macroBitRegions.m_highBits & m_cylinders[cylinderIndex].m_macroBitRegions.m_highBits);
		if (m_ropeCapsules[sentCapsuleIndex].m_macroBitRegions.m_highBits == 0 && m_cylinders[cylinderIndex].m_macroBitRegions.m_highBits == 0)
		{
			macroHighBitResult = 1;
		}
		if (m_ropeCapsules[sentCapsuleIndex].m_macroBitRegions.m_lowBits == 0 && m_cylinders[cylinderIndex].m_macroBitRegions.m_lowBits == 0)
		{
			macroLowBitResult = 1;
		}
		if (macroLowBitResult != 0 || macroHighBitResult != 0)
		{
			//Micro Check
			uint microLowBitResult = (m_ropeCapsules[sentCapsuleIndex].m_microBitRegions.m_lowBits & m_cylinders[cylinderIndex].m_microBitRegions.m_lowBits);
			uint microHighBitResult = (m_ropeCapsules[sentCapsuleIndex].m_microBitRegions.m_highBits & m_cylinders[cylinderIndex].m_microBitRegions.m_highBits);
			if (m_ropeCapsules[sentCapsuleIndex].m_microBitRegions.m_highBits == 0 && m_cylinders[cylinderIndex].m_microBitRegions.m_highBits == 0)
			{
				microHighBitResult = 1;
			}
			if (m_ropeCapsules[sentCapsuleIndex].m_microBitRegions.m_lowBits == 0 && m_cylinders[cylinderIndex].m_microBitRegions.m_lowBits == 0)
			{
				microLowBitResult = 1;
			}
			if (microLowBitResult != 0 || microHighBitResult != 0)
			{
				//Bounding Disc Check
				if (DoSpheresOverlap(m_ropeCapsules[sentCapsuleIndex].m_boundingDiscCenter, m_ropeCapsules[sentCapsuleIndex].m_boundingDiscRadius, m_cylinders[cylinderIndex].m_boundingDiscCenter, m_cylinders[cylinderIndex].m_boundingDiscRadius))
				{
					//Capsule vs Cylinder
					float3 newStartPosition = float3(0.0f, 0.0f, 0.0f);
					float3 newEndPosition = float3(0.0f, 0.0f, 0.0f);
					PushCapsuleOutOfFixedCylinder3D(m_ropeCapsules[sentCapsuleIndex].m_capsule, m_cylinders[cylinderIndex].m_cylinder, newStartPosition, newEndPosition);
					if (newStartPosition.x != m_ropeCapsules[sentCapsuleIndex].m_capsule.m_bone.m_start.x ||
						newStartPosition.y != m_ropeCapsules[sentCapsuleIndex].m_capsule.m_bone.m_start.y ||
						newStartPosition.z != m_ropeCapsules[sentCapsuleIndex].m_capsule.m_bone.m_start.z)
					{
						float3 displacementStart = newStartPosition - m_ropeCapsules[sentCapsuleIndex].m_capsule.m_bone.m_start;
						m_ropeCapsules[sentCapsuleIndex].m_isCollidingStart = 1;
						m_ropeCapsules[sentCapsuleIndex].m_collisionNormalStart = GetNormalized(displacementStart);
						m_ropeCapsules[sentCapsuleIndex].m_jacobiCorrectionStart += (displacementStart);
						m_ropeCapsules[sentCapsuleIndex].m_jacobiConstraintTotalStart++;
					}
					if (newEndPosition.x != m_ropeCapsules[sentCapsuleIndex].m_capsule.m_bone.m_end.x ||
						newEndPosition.y != m_ropeCapsules[sentCapsuleIndex].m_capsule.m_bone.m_end.y ||
						newEndPosition.z != m_ropeCapsules[sentCapsuleIndex].m_capsule.m_bone.m_end.z)
					{
						float3 displacementEnd = newEndPosition - m_ropeCapsules[sentCapsuleIndex].m_capsule.m_bone.m_end;
						m_ropeCapsules[sentCapsuleIndex].m_isCollidingEnd = 1;
						m_ropeCapsules[sentCapsuleIndex].m_collisionNormalEnd = GetNormalized(displacementEnd);
						m_ropeCapsules[sentCapsuleIndex].m_jacobiCorrectionEnd += (displacementEnd);
						m_ropeCapsules[sentCapsuleIndex].m_jacobiConstraintTotalEnd++;
					}
				}
			}
		}
	}

	//Capsules
	for (int capsuleIndex = 0; capsuleIndex < m_totalCapsules; capsuleIndex++)
	{
		//Macro Check
		uint macroLowBitResult = (m_ropeCapsules[sentCapsuleIndex].m_macroBitRegions.m_lowBits & m_capsules[capsuleIndex].m_macroBitRegions.m_lowBits);
		uint macroHighBitResult = (m_ropeCapsules[sentCapsuleIndex].m_macroBitRegions.m_highBits & m_capsules[capsuleIndex].m_macroBitRegions.m_highBits);
		if (m_ropeCapsules[sentCapsuleIndex].m_macroBitRegions.m_highBits == 0 && m_capsules[capsuleIndex].m_macroBitRegions.m_highBits == 0)
		{
			macroHighBitResult = 1;
		}
		if (m_ropeCapsules[sentCapsuleIndex].m_macroBitRegions.m_lowBits == 0 && m_capsules[capsuleIndex].m_macroBitRegions.m_lowBits == 0)
		{
			macroLowBitResult = 1;
		}
		if (macroLowBitResult != 0 || macroHighBitResult != 0)
		{
			//Micro Check
			uint microLowBitResult = (m_ropeCapsules[sentCapsuleIndex].m_microBitRegions.m_lowBits & m_capsules[capsuleIndex].m_microBitRegions.m_lowBits);
			uint microHighBitResult = (m_ropeCapsules[sentCapsuleIndex].m_microBitRegions.m_highBits & m_capsules[capsuleIndex].m_microBitRegions.m_highBits);
			if (m_ropeCapsules[sentCapsuleIndex].m_microBitRegions.m_highBits == 0 && m_capsules[capsuleIndex].m_microBitRegions.m_highBits == 0)
			{
				microHighBitResult = 1;
			}
			if (m_ropeCapsules[sentCapsuleIndex].m_microBitRegions.m_lowBits == 0 && m_capsules[capsuleIndex].m_microBitRegions.m_lowBits == 0)
			{
				microLowBitResult = 1;
			}
			if (microLowBitResult != 0 || microHighBitResult != 0)
			{
				//Bounding Disc Check
				if (DoSpheresOverlap(m_ropeCapsules[sentCapsuleIndex].m_boundingDiscCenter, m_ropeCapsules[sentCapsuleIndex].m_boundingDiscRadius, m_capsules[capsuleIndex].m_boundingDiscCenter, m_capsules[capsuleIndex].m_boundingDiscRadius))
				{
					//Capsule vs Fixed Capsule
					float3 newStartPosition = float3(0.0f, 0.0f, 0.0f);
					float3 newEndPosition = float3(0.0f, 0.0f, 0.0f);
					PushCapsuleOutOfFixedCapsule3D(m_ropeCapsules[sentCapsuleIndex].m_capsule, m_capsules[capsuleIndex].m_capsule, newStartPosition, newEndPosition);
					if (newStartPosition.x != m_ropeCapsules[sentCapsuleIndex].m_capsule.m_bone.m_start.x ||
						newStartPosition.y != m_ropeCapsules[sentCapsuleIndex].m_capsule.m_bone.m_start.y ||
						newStartPosition.z != m_ropeCapsules[sentCapsuleIndex].m_capsule.m_bone.m_start.z)
					{
						float3 displacementStart = newStartPosition - m_ropeCapsules[sentCapsuleIndex].m_capsule.m_bone.m_start;
						m_ropeCapsules[sentCapsuleIndex].m_isCollidingStart = 1;
						m_ropeCapsules[sentCapsuleIndex].m_collisionNormalStart = GetNormalized(displacementStart);
						m_ropeCapsules[sentCapsuleIndex].m_jacobiCorrectionStart += (displacementStart);
						m_ropeCapsules[sentCapsuleIndex].m_jacobiConstraintTotalStart++;
					}
					if (newEndPosition.x != m_ropeCapsules[sentCapsuleIndex].m_capsule.m_bone.m_end.x ||
						newEndPosition.y != m_ropeCapsules[sentCapsuleIndex].m_capsule.m_bone.m_end.y ||
						newEndPosition.z != m_ropeCapsules[sentCapsuleIndex].m_capsule.m_bone.m_end.z)
					{
						float3 displacementEnd = newEndPosition - m_ropeCapsules[sentCapsuleIndex].m_capsule.m_bone.m_end;
						m_ropeCapsules[sentCapsuleIndex].m_isCollidingEnd = 1;
						m_ropeCapsules[sentCapsuleIndex].m_collisionNormalEnd = GetNormalized(displacementEnd);
						m_ropeCapsules[sentCapsuleIndex].m_jacobiCorrectionEnd += (displacementEnd);
						m_ropeCapsules[sentCapsuleIndex].m_jacobiConstraintTotalEnd++;
					}
				}
			}
		}
	}

	//Spheres
	for (int sphereIndex = 0; sphereIndex < m_totalSpheres; sphereIndex++)
	{
		//Macro Check
		uint macroLowBitResult = (m_ropeCapsules[sentCapsuleIndex].m_macroBitRegions.m_lowBits & m_spheres[sphereIndex].m_macroBitRegions.m_lowBits);
		uint macroHighBitResult = (m_ropeCapsules[sentCapsuleIndex].m_macroBitRegions.m_highBits & m_spheres[sphereIndex].m_macroBitRegions.m_highBits);
		if (m_ropeCapsules[sentCapsuleIndex].m_macroBitRegions.m_highBits == 0 && m_spheres[sphereIndex].m_macroBitRegions.m_highBits == 0)
		{
			macroHighBitResult = 1;
		}
		if (m_ropeCapsules[sentCapsuleIndex].m_macroBitRegions.m_lowBits == 0 && m_spheres[sphereIndex].m_macroBitRegions.m_lowBits == 0)
		{
			macroLowBitResult = 1;
		}
		if (macroLowBitResult != 0 || macroHighBitResult != 0)
		{
			//Micro Check
			uint microLowBitResult = (m_ropeCapsules[sentCapsuleIndex].m_microBitRegions.m_lowBits & m_spheres[sphereIndex].m_microBitRegions.m_lowBits);
			uint microHighBitResult = (m_ropeCapsules[sentCapsuleIndex].m_microBitRegions.m_highBits & m_spheres[sphereIndex].m_microBitRegions.m_highBits);
			if (m_ropeCapsules[sentCapsuleIndex].m_microBitRegions.m_highBits == 0 && m_spheres[sphereIndex].m_microBitRegions.m_highBits == 0)
			{
				microHighBitResult = 1;
			}
			if (m_ropeCapsules[sentCapsuleIndex].m_microBitRegions.m_lowBits == 0 && m_spheres[sphereIndex].m_microBitRegions.m_lowBits == 0)
			{
				microLowBitResult = 1;
			}
			if (microLowBitResult != 0 || microHighBitResult != 0)
			{
				//Capsule vs Sphere
				float3 newStartPosition = float3(0.0f, 0.0f, 0.0f);
				float3 newEndPosition = float3(0.0f, 0.0f, 0.0f);
				PushCapsuleOutOfFixedSphere3D(m_ropeCapsules[sentCapsuleIndex].m_capsule, m_spheres[sphereIndex].m_sphere.m_center, m_spheres[sphereIndex].m_sphere.m_radius, newStartPosition, newEndPosition);
				if (newStartPosition.x != m_ropeCapsules[sentCapsuleIndex].m_capsule.m_bone.m_start.x ||
					newStartPosition.y != m_ropeCapsules[sentCapsuleIndex].m_capsule.m_bone.m_start.y ||
					newStartPosition.z != m_ropeCapsules[sentCapsuleIndex].m_capsule.m_bone.m_start.z)
				{
					float3 displacementStart = newStartPosition - m_ropeCapsules[sentCapsuleIndex].m_capsule.m_bone.m_start;
					m_ropeCapsules[sentCapsuleIndex].m_isCollidingStart = 1;
					m_ropeCapsules[sentCapsuleIndex].m_collisionNormalStart = GetNormalized(displacementStart);
					m_ropeCapsules[sentCapsuleIndex].m_jacobiCorrectionStart += (displacementStart);
					m_ropeCapsules[sentCapsuleIndex].m_jacobiConstraintTotalStart++;
				}
				if (newEndPosition.x != m_ropeCapsules[sentCapsuleIndex].m_capsule.m_bone.m_end.x ||
					newEndPosition.y != m_ropeCapsules[sentCapsuleIndex].m_capsule.m_bone.m_end.y ||
					newEndPosition.z != m_ropeCapsules[sentCapsuleIndex].m_capsule.m_bone.m_end.z)
				{
					float3 displacementEnd = newEndPosition - m_ropeCapsules[sentCapsuleIndex].m_capsule.m_bone.m_end;
					m_ropeCapsules[sentCapsuleIndex].m_isCollidingEnd = 1;
					m_ropeCapsules[sentCapsuleIndex].m_collisionNormalEnd = GetNormalized(displacementEnd);
					m_ropeCapsules[sentCapsuleIndex].m_jacobiCorrectionEnd += (displacementEnd);
					m_ropeCapsules[sentCapsuleIndex].m_jacobiConstraintTotalEnd++;
				}
			}
		}
	}


	//Update proposed position based on jacobi correction
	if (m_ropeCapsules[sentCapsuleIndex].m_jacobiConstraintTotalStart != 0 && m_particleIsAttached[sentCapsuleIndex] == 0)
	{
		m_particleProposedPositions[sentCapsuleIndex] += (m_ropeCapsules[sentCapsuleIndex].m_jacobiCorrectionStart / m_ropeCapsules[sentCapsuleIndex].m_jacobiConstraintTotalStart);
		m_particleJacobiCorrections[sentCapsuleIndex] = float4(0.0f, 0.0f, 0.0f, 0.0f);
		m_particleCollisionNormals[sentCapsuleIndex] = m_ropeCapsules[sentCapsuleIndex].m_collisionNormalStart;
	}
	//Update proposed position based on jacobi correction
	if (m_ropeCapsules[sentCapsuleIndex].m_jacobiConstraintTotalEnd != 0 && m_particleIsAttached[sentCapsuleIndex + 1] == 0)
	{
		m_particleProposedPositions[sentCapsuleIndex + 1] += (m_ropeCapsules[sentCapsuleIndex].m_jacobiCorrectionEnd / m_ropeCapsules[sentCapsuleIndex].m_jacobiConstraintTotalEnd);
		m_particleJacobiCorrections[sentCapsuleIndex + 1] = float4(0.0f, 0.0f, 0.0f, 0.0f);
		m_particleCollisionNormals[sentCapsuleIndex + 1] = m_ropeCapsules[sentCapsuleIndex].m_collisionNormalEnd;
	}

	//World Bounds Collisions
	ProjectWorldBoundsConstraintsJacobi(sentCapsuleIndex);
	ProjectWorldBoundsConstraintsJacobi(sentCapsuleIndex + 1);
}
