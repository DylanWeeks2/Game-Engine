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
	Sphere3		m_sphere;
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
RWStructuredBuffer<float3>					m_particlePositions : register(u0);
RWStructuredBuffer<float3>					m_particleVelocities : register(u1);
RWStructuredBuffer<float3>					m_particleProposedPositions : register(u2);
RWStructuredBuffer<float4>					m_particleJacobiCorrections : register(u3);
RWStructuredBuffer<float3>					m_particleCollisionNormals : register(u4);
RWStructuredBuffer<uint64_t>				m_particleMacroBitRegions : register(u5);
RWStructuredBuffer<uint64_t>				m_particleMicroBitRegions : register(u6);
StructuredBuffer<int>						m_particleIsAttached : register(t0);
StructuredBuffer<float>					m_particleMasses : register(t1);
StructuredBuffer<float>					m_particleInverseMasses : register(t2);
StructuredBuffer<AABBCollisionObject>		m_aabbs : register(t3);
StructuredBuffer<OBBCollisionObject>		m_obbs : register(t4);
StructuredBuffer<CapsuleCollisionObject>	m_capsules : register(t5);
StructuredBuffer<CylinderCollisionObject>	m_cylinders : register(t6);
StructuredBuffer<SphereCollisionObject>		m_spheres : register(t7);

//MATH UTILITIES
//-----------------------------------------------------------------------------------------------
float GetLength(float3 vec3)
{
	float xSquared = vec3.x * vec3.x;
	float ySquared = vec3.y * vec3.y;
	float zSquared = vec3.z * vec3.z;
	float sqrtValue = xSquared + ySquared + zSquared;
	return float(sqrt(sqrtValue));
}

//-----------------------------------------------------------------------------------------------
float GetLengthSquared(float3 vec3)
{
	float xSquared = vec3.x * vec3.x;
	float ySquared = vec3.y * vec3.y;
	float zSquared = vec3.z * vec3.z;
	return float(xSquared + ySquared + zSquared);
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
float GetLengthfloat2(float2 vec2) 
{
	float xSquared = vec2.x * vec2.x;
	float ySquared = vec2.y * vec2.y;
	float sqrtValue = xSquared + ySquared;
	return sqrt(sqrtValue);
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
bool DoSpheresOverlap(float3 centerA, float radiusA, float3 centerB, float radiusB)
{
	if (GetDistanceSquared3D(centerA, centerB) < (radiusA + radiusB) * (radiusA + radiusB))
	{
		return true;
	}

	return false;
}

//-----------------------------------------------------------------------------------------------
bool PushSphereOutOfSphere3D(int particleAIndex, float3 sphereCenterA, float sphereRadiusA, int particleBIndex, float3 sphereCenterB, float sphereRadiusB)
{
	float3 previousPos = sphereCenterA;
	float3 displacement = (sphereCenterA - sphereCenterB);
	float distanceSquared = GetLengthSquared(displacement);
	if ((sphereRadiusA + sphereRadiusB) * (sphereRadiusA + sphereRadiusB) >= distanceSquared)
	{
		float offset = sphereRadiusA + sphereRadiusB - sqrt(distanceSquared);
		displacement = SetLength(displacement, offset);
		sphereCenterA += displacement * 0.5f;
		sphereCenterB -= displacement * 0.5f;

		//Jacobi Updates
		float3 currentPos = sphereCenterA;
		displacement = currentPos - previousPos;
		m_particleCollisionNormals[particleAIndex] = GetNormalized(displacement);
		m_particleJacobiCorrections[particleAIndex].xyz += (displacement);
		m_particleJacobiCorrections[particleAIndex].w++;
		return true;
	}

	return false;
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

//COMPUTE SHADER LOGIC
//-----------------------------------------------------------------------------------------------
int GetRegionIndexForCoords(int3 coords)
{
	return coords.x | (coords.y << 3);
}

//-----------------------------------------------------------------------------------------------
void AssignBitRegionsParticle(int sentParticleIndex, int currentRegion, int regionDifferenceX, int regionDifferenceY, bool isMicro)
{
	uint64_t mask;
	mask.m_lowBits = 0;
	mask.m_highBits = 0;
	if (isMicro == false)
	{
		m_particleMacroBitRegions[sentParticleIndex].m_lowBits = 0;
		m_particleMacroBitRegions[sentParticleIndex].m_highBits = 0;
		for (int regionIndexY = 0; regionIndexY <= regionDifferenceY; regionIndexY++)
		{
			for (int regionIndexX = 0; regionIndexX <= regionDifferenceX; regionIndexX++)
			{
				int bit = currentRegion + regionIndexX;
				if (bit < 32)
				{
					mask.m_lowBits = 1u << bit;
					m_particleMacroBitRegions[sentParticleIndex].m_lowBits |= mask.m_lowBits;
				}
				else
				{
					bit -= 32;
					mask.m_highBits = 1u << bit;
					m_particleMacroBitRegions[sentParticleIndex].m_highBits |= mask.m_highBits;
				}
			}
			currentRegion += 8;
		}
	}
	else
	{
		m_particleMicroBitRegions[sentParticleIndex].m_lowBits = 0;
		m_particleMicroBitRegions[sentParticleIndex].m_highBits = 0;
		for (int regionIndexY = 0; regionIndexY <= regionDifferenceY; regionIndexY++)
		{
			for (int regionIndexX = 0; regionIndexX <= regionDifferenceX; regionIndexX++)
			{
				int bit = currentRegion + regionIndexX;
				if (bit < 32)
				{
					mask.m_lowBits = 1u << bit;
					m_particleMicroBitRegions[sentParticleIndex].m_lowBits |= mask.m_lowBits;
				}
				else
				{
					bit -= 32;
					mask.m_highBits = 1u << bit;
					m_particleMicroBitRegions[sentParticleIndex].m_highBits |= mask.m_highBits;
				}
			}
			currentRegion += 8;
		}
	}
}

//-----------------------------------------------------------------------------------------------
void BitRegionDetectionSingleParticle(int sentParticleIndex)
{
	//Initializations
	m_particleMacroBitRegions[sentParticleIndex].m_lowBits = 0;
	m_particleMacroBitRegions[sentParticleIndex].m_highBits = 0;
	m_particleMicroBitRegions[sentParticleIndex].m_lowBits = 0;
	m_particleMicroBitRegions[sentParticleIndex].m_highBits = 0;
	float3 center = m_particleProposedPositions[sentParticleIndex];
	float m_bitRegionScale = 1.0f / 8.0f;
	float m_worldScaleX = abs(m_worldBounds.m_maxs.x - m_worldBounds.m_mins.x) * m_bitRegionScale;
	float m_worldScaleY = abs(m_worldBounds.m_maxs.y - m_worldBounds.m_mins.y) * m_bitRegionScale;
	float m_macroScaleX = 1.0f / m_worldScaleX;
	float m_macroScaleY = 1.0f / m_worldScaleY;
	float m_microScaleX = 8.0f;
	float m_microScaleY = 8.0f;

	//Macro
	float minX = (center.x - m_ropeRadius) * m_macroScaleX;
	float minY = (center.y - m_ropeRadius) * m_macroScaleY;
	float maxX = (center.x + m_ropeRadius) * m_macroScaleX;
	float maxY = (center.y + m_ropeRadius) * m_macroScaleY;
	int3 minXMinY = int3(int(minX), int(minY), 0);
	int3 minXMaxY = int3(int(minX), int(maxY), 0);
	int3 maxXMinY = int3(int(maxX), int(minY), 0);
	int3 maxXMaxY = int3(int(maxX), int(maxY), 0);
	int region1 = GetRegionIndexForCoords(minXMinY);
	int region2 = GetRegionIndexForCoords(minXMaxY);
	int region3 = GetRegionIndexForCoords(maxXMinY);
	int region4 = GetRegionIndexForCoords(maxXMaxY);
	int regionDifference12 = region2 - region1;
	int currentRegion = region1;
	int regionDifferenceX = region3 - region1;
	int regionDifferenceY = regionDifference12 >> 3;
	AssignBitRegionsParticle(sentParticleIndex, currentRegion, regionDifferenceX, regionDifferenceY, false);

	//Micro
	int3 minXMinYMicro = int3(int((minX - minXMinY.x) * m_microScaleX), int((minY - minXMinY.y) * m_microScaleY), 0);
	int3 minXMaxYMicro = int3(int((minX - minXMaxY.x) * m_microScaleX), int((maxY - minXMaxY.y) * m_microScaleY), 0);
	int3 maxXMinYMicro = int3(int((maxX - maxXMinY.x) * m_microScaleX), int((minY - maxXMinY.y) * m_microScaleY), 0);
	int3 maxXMaxYMicro = int3(int((maxX - maxXMaxY.x) * m_microScaleX), int((maxY - maxXMaxY.y) * m_microScaleY), 0);
	int region1Micro = GetRegionIndexForCoords(minXMinYMicro);
	int region2Micro = GetRegionIndexForCoords(minXMaxYMicro);
	int region3Micro = GetRegionIndexForCoords(maxXMinYMicro);
	int region4Micro = GetRegionIndexForCoords(maxXMaxYMicro);

	//All regions are the same CORRECT
	if (region1 == region2 && region1 == region3)
	{
		currentRegion = region1Micro;
		regionDifference12 = region2Micro - region1Micro;
		regionDifferenceX = region3Micro - region1Micro;
		regionDifferenceY = regionDifference12 >> 3;
		AssignBitRegionsParticle(sentParticleIndex, currentRegion, regionDifferenceX, regionDifferenceY, true);
	}
	//Y regions are different
	else if (region1 != region2 && region1 == region3)
	{
		//Spans across multiple Y macro regions CORRECT
		if (region2 - region1 > 8)
		{
			regionDifferenceY = 7;
			regionDifferenceX = region3Micro - region1Micro;
			currentRegion = region1Micro - (region1Micro >> 3) * 8;
			AssignBitRegionsParticle(sentParticleIndex, currentRegion, regionDifferenceX, regionDifferenceY, true);
		}
		//Span across 1 Y macro region
		else
		{
			//region 1/3 to top CORRECT
			regionDifferenceX = region3Micro - region1Micro;
			regionDifferenceY = 7 - (region1Micro >> 3);
			currentRegion = region1Micro;
			AssignBitRegionsParticle(sentParticleIndex, currentRegion, regionDifferenceX, regionDifferenceY, true);
			//bottom to region 2/4 CORRECT
			regionDifferenceX = region4Micro - region2Micro;
			regionDifferenceY = (region2Micro >> 3);
			currentRegion = region2Micro - (region2Micro >> 3) * 8;
			AssignBitRegionsParticle(sentParticleIndex, currentRegion, regionDifferenceX, regionDifferenceY, true);
		}
	}
	//X regions are different
	else if (region1 == region2 && region1 != region3)
	{
		//Spans across multiple X macro regions CORRECT
		if (region3 - region1 > 1)
		{
			regionDifferenceX = 7;
			regionDifferenceY = region2Micro - region1Micro;
			currentRegion = (region1Micro >> 3) * 8;
			AssignBitRegionsParticle(sentParticleIndex, currentRegion, regionDifferenceX, regionDifferenceY, true);
		}
		//Span across 1 X macro region
		else
		{
			//region 1/2 to right CORRECT
			regionDifferenceX = ((region1Micro >> 3) * 8 + 7) - region1Micro;
			regionDifferenceY = region2Micro - region1Micro;
			currentRegion = region1Micro;
			AssignBitRegionsParticle(sentParticleIndex, currentRegion, regionDifferenceX, regionDifferenceY, true);
			//left to region 3/4 CORRECT
			currentRegion = (region3Micro >> 3) * 8;
			regionDifferenceX = region3Micro - currentRegion;
			regionDifferenceY = region2Micro - region1Micro;
			AssignBitRegionsParticle(sentParticleIndex, currentRegion, regionDifferenceX, regionDifferenceY, true);
		}
	}
	//All macro regions are different
	else if (region1 != region2 && region1 != region3)
	{
		//Spans across multiple X and Y regions CORRECT
		if (region2 - region1 > 8 && region3 - region1 > 1)
		{
			uint64_t mask;
			mask.m_lowBits = 0;
			mask.m_highBits = 0;
			mask.m_lowBits = ~mask.m_lowBits;
			mask.m_highBits = ~mask.m_highBits;
			m_particleMicroBitRegions[sentParticleIndex] = mask;
		}
		//Spans across 1 Y and multiple X regions
		else if (region2 - region1 == 8 && region3 - region1 > 1)
		{
			//region 1/3 to top CORRECT
			regionDifferenceX = 7;
			regionDifferenceY = 7 - (region1Micro >> 3);
			currentRegion = (region1Micro >> 3) * 8;
			AssignBitRegionsParticle(sentParticleIndex, currentRegion, regionDifferenceX, regionDifferenceY, true);
			//bottom to region 2/4 CORRECT
			regionDifferenceX = 7;
			regionDifferenceY = (region2Micro >> 3);
			currentRegion = 0;
			AssignBitRegionsParticle(sentParticleIndex, currentRegion, regionDifferenceX, regionDifferenceY, true);
		}
		//Spans across 1 X and multiple Y regions
		else if (region2 - region1 > 8 && region3 - region1 == 1)
		{
			//region 1/2 to right CORRECT		
			regionDifferenceX = ((region1Micro >> 3) * 8 + 7) - region1Micro;
			regionDifferenceY = 7;
			currentRegion = region1Micro - (region1Micro >> 3) * 8;
			AssignBitRegionsParticle(sentParticleIndex, currentRegion, regionDifferenceX, regionDifferenceY, true);
			//left to region 3/4 					
			currentRegion = 0;
			regionDifferenceX = region3Micro - (region3Micro >> 3) * 8;
			regionDifferenceY = 7;
			AssignBitRegionsParticle(sentParticleIndex, currentRegion, regionDifferenceX, regionDifferenceY, true);
		}
		//Spans across 1 X and 1 Y region
		else if (region2 - region1 == 8 && region3 - region1 == 1)
		{
			//region1 to right/top
			regionDifferenceX = ((region1Micro >> 3) * 8 + 7) - region1Micro;
			regionDifferenceY = 7 - (region1Micro >> 3);
			currentRegion = region1Micro;
			AssignBitRegionsParticle(sentParticleIndex, currentRegion, regionDifferenceX, regionDifferenceY, true);
			//left/top to region 3
			regionDifferenceX = region3Micro - (region3Micro >> 3) * 8;
			regionDifferenceY = 7 - (region3Micro >> 3);
			currentRegion = (region3Micro >> 3) * 8;
			AssignBitRegionsParticle(sentParticleIndex, currentRegion, regionDifferenceX, regionDifferenceY, true);
			//bottom/right to region 2
			regionDifferenceX = ((region2Micro >> 3) * 8 + 7) - region2Micro;
			regionDifferenceY = (region2Micro >> 3);
			currentRegion = region2Micro - (region2Micro >> 3) * 8;
			AssignBitRegionsParticle(sentParticleIndex, currentRegion, regionDifferenceX, regionDifferenceY, true);
			//bottom/left to region 4
			regionDifferenceX = region4Micro - (region4Micro >> 3) * 8;
			regionDifferenceY = (region4Micro >> 3);
			currentRegion = 0;
			AssignBitRegionsParticle(sentParticleIndex, currentRegion, regionDifferenceX, regionDifferenceY, true);
		}
	}
}

//-----------------------------------------------------------------------------------------------
void ProjectWorldBoundsConstraintsSpheresJacobi(int sentParticleIndex)
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
	float3 correction = (newPosition - m_particleProposedPositions[sentParticleIndex]);
	m_particleJacobiCorrections[sentParticleIndex].xyz += correction;
}

//------------------------------------------------------------------------------------------------
[numthreads(threadX, threadY, threadZ)]
void CSMain(uint3 threadGroupId : SV_GroupID, uint threadIndex : SV_GroupIndex)
{
	uint startIndex = threadGroupId.x * threadX;
	uint sentParticleIndex = startIndex + (threadIndex);
	if (sentParticleIndex >= m_totalParticles)
		return;

	//Update bit regions
	BitRegionDetectionSingleParticle(sentParticleIndex);

	//Self Collisions
	if (m_isSelfCollisionEnabled == 1)
	{
		for (int particleIndex = 0; particleIndex < m_totalParticles; particleIndex++)
		{
			if (particleIndex != sentParticleIndex && particleIndex != sentParticleIndex - 1 && particleIndex != sentParticleIndex + 1)
			{
				//Macro
				uint macroLowBitResult = (m_particleMacroBitRegions[sentParticleIndex].m_lowBits & m_particleMacroBitRegions[particleIndex].m_lowBits);
				uint macroHighBitResult = (m_particleMacroBitRegions[sentParticleIndex].m_highBits & m_particleMacroBitRegions[particleIndex].m_highBits);
				if (m_particleMacroBitRegions[sentParticleIndex].m_highBits == 0 && m_particleMacroBitRegions[particleIndex].m_highBits == 0)
				{
					macroHighBitResult = 1;
				}
				if (m_particleMacroBitRegions[sentParticleIndex].m_lowBits == 0 && m_particleMacroBitRegions[particleIndex].m_lowBits == 0)
				{
					macroLowBitResult = 1;
				}
				if (macroLowBitResult != 0 || macroHighBitResult != 0)
				{
					//Micro
					uint microLowBitResult = (m_particleMicroBitRegions[sentParticleIndex].m_lowBits & m_particleMicroBitRegions[particleIndex].m_lowBits);
					uint microHighBitResult = (m_particleMicroBitRegions[sentParticleIndex].m_highBits & m_particleMicroBitRegions[particleIndex].m_highBits);
					if (m_particleMicroBitRegions[sentParticleIndex].m_highBits == 0 && m_particleMicroBitRegions[particleIndex].m_highBits == 0)
					{
						microHighBitResult = 1;
					}
					if (m_particleMicroBitRegions[sentParticleIndex].m_lowBits == 0 && m_particleMicroBitRegions[particleIndex].m_lowBits == 0)
					{
						microLowBitResult = 1;
					}
					if (microLowBitResult != 0 || microHighBitResult != 0)
					{
						//Sphere vs Sphere
						PushSphereOutOfSphere3D(sentParticleIndex, m_particleProposedPositions[sentParticleIndex], m_ropeRadius, particleIndex, m_particleProposedPositions[particleIndex], m_ropeRadius);
					}
				}
			}
		}
	}

	//AABBs
	for (int aabbIndex = 0; aabbIndex < m_totalAABBs; aabbIndex++)
	{
		//Macro Check
		uint macroLowBitResult = (m_particleMacroBitRegions[sentParticleIndex].m_lowBits & m_aabbs[aabbIndex].m_macroBitRegions.m_lowBits);
		uint macroHighBitResult = (m_particleMacroBitRegions[sentParticleIndex].m_highBits & m_aabbs[aabbIndex].m_macroBitRegions.m_highBits);
		if (m_particleMacroBitRegions[sentParticleIndex].m_highBits == 0 && m_aabbs[aabbIndex].m_macroBitRegions.m_highBits == 0)
		{
			macroHighBitResult = 1;
		}
		if (m_particleMacroBitRegions[sentParticleIndex].m_lowBits == 0 && m_aabbs[aabbIndex].m_macroBitRegions.m_lowBits == 0)
		{
			macroLowBitResult = 1;
		}
		if (macroLowBitResult != 0 || macroHighBitResult != 0)
		{
			//Micro Check
			uint microLowBitResult = (m_particleMicroBitRegions[sentParticleIndex].m_lowBits & m_aabbs[aabbIndex].m_microBitRegions.m_lowBits);
			uint microHighBitResult = (m_particleMicroBitRegions[sentParticleIndex].m_highBits & m_aabbs[aabbIndex].m_microBitRegions.m_highBits);
			if (m_particleMicroBitRegions[sentParticleIndex].m_highBits == 0 && m_aabbs[aabbIndex].m_microBitRegions.m_highBits == 0)
			{
				microHighBitResult = 1;
			}
			if (m_particleMicroBitRegions[sentParticleIndex].m_lowBits == 0 && m_aabbs[aabbIndex].m_microBitRegions.m_lowBits == 0)
			{
				microLowBitResult = 1;
			}
			if (microLowBitResult != 0 || microHighBitResult != 0)
			{
				//Bounding Disc Check
				if (DoSpheresOverlap(m_particleProposedPositions[sentParticleIndex], m_ropeRadius, m_aabbs[aabbIndex].m_boundingDiscCenter, m_aabbs[aabbIndex].m_boundingDiscRadius))
				{
					//Sphere vs AABB
					float3 newPosition = PushSphereOutOfFixedAABB3D(m_particleProposedPositions[sentParticleIndex], m_ropeRadius, m_aabbs[aabbIndex].m_aabb);
					if (newPosition.x != m_particleProposedPositions[sentParticleIndex].x || newPosition.y != m_particleProposedPositions[sentParticleIndex].y || newPosition.z != m_particleProposedPositions[sentParticleIndex].z)
					{
						float3 displacement = newPosition - m_particleProposedPositions[sentParticleIndex];
						m_particleCollisionNormals[sentParticleIndex] = GetNormalized(displacement);
						m_particleJacobiCorrections[sentParticleIndex].xyz += (displacement);
						m_particleJacobiCorrections[sentParticleIndex].w++;
					}
				}
			}
		}
	}

	//OBBs
	for (int obbIndex = 0; obbIndex < m_totalOBBs; obbIndex++)
	{
		//Macro Check
		uint macroLowBitResult = (m_particleMacroBitRegions[sentParticleIndex].m_lowBits & m_obbs[obbIndex].m_macroBitRegions.m_lowBits);
		uint macroHighBitResult = (m_particleMacroBitRegions[sentParticleIndex].m_highBits & m_obbs[obbIndex].m_macroBitRegions.m_highBits);
		if (m_particleMacroBitRegions[sentParticleIndex].m_highBits == 0 && m_obbs[obbIndex].m_macroBitRegions.m_highBits == 0)
		{
			macroHighBitResult = 1;
		}
		if (m_particleMacroBitRegions[sentParticleIndex].m_lowBits == 0 && m_obbs[obbIndex].m_macroBitRegions.m_lowBits == 0)
		{
			macroLowBitResult = 1;
		}
		if (macroLowBitResult != 0 || macroHighBitResult != 0)
		{
			//Micro Check
			uint microLowBitResult = (m_particleMicroBitRegions[sentParticleIndex].m_lowBits & m_obbs[obbIndex].m_microBitRegions.m_lowBits);
			uint microHighBitResult = (m_particleMicroBitRegions[sentParticleIndex].m_highBits & m_obbs[obbIndex].m_microBitRegions.m_highBits);
			if (m_particleMicroBitRegions[sentParticleIndex].m_highBits == 0 && m_obbs[obbIndex].m_microBitRegions.m_highBits == 0)
			{
				microHighBitResult = 1;
			}
			if (m_particleMicroBitRegions[sentParticleIndex].m_lowBits == 0 && m_obbs[obbIndex].m_microBitRegions.m_lowBits == 0)
			{
				microLowBitResult = 1;
			}
			if (microLowBitResult != 0 || microHighBitResult != 0)
			{
				//Bounding Disc Check
				if (DoSpheresOverlap(m_particleProposedPositions[sentParticleIndex], m_ropeRadius, m_obbs[obbIndex].m_boundingDiscCenter, m_obbs[obbIndex].m_boundingDiscRadius))
				{
					//Sphere vs OBB
					float3 newPosition = PushDiscOutOfFixedOBB3D(m_particleProposedPositions[sentParticleIndex], m_ropeRadius, m_obbs[obbIndex].m_obb);
					if (newPosition.x != m_particleProposedPositions[sentParticleIndex].x || newPosition.y != m_particleProposedPositions[sentParticleIndex].y || newPosition.z != m_particleProposedPositions[sentParticleIndex].z)
					{
						float3 displacement = newPosition - m_particleProposedPositions[sentParticleIndex];
						m_particleCollisionNormals[sentParticleIndex] = GetNormalized(displacement);
						m_particleJacobiCorrections[sentParticleIndex].xyz += (displacement);
						m_particleJacobiCorrections[sentParticleIndex].w++;
					}
				}
			}
		}
	}

	//Cylinders
	for (int cylinderIndex = 0; cylinderIndex < m_totalCylinders; cylinderIndex++)
	{
		//Macro Check
		uint macroLowBitResult = (m_particleMacroBitRegions[sentParticleIndex].m_lowBits & m_cylinders[cylinderIndex].m_macroBitRegions.m_lowBits);
		uint macroHighBitResult = (m_particleMacroBitRegions[sentParticleIndex].m_highBits & m_cylinders[cylinderIndex].m_macroBitRegions.m_highBits);
		if (m_particleMacroBitRegions[sentParticleIndex].m_highBits == 0 && m_cylinders[cylinderIndex].m_macroBitRegions.m_highBits == 0)
		{
			macroHighBitResult = 1;
		}
		if (m_particleMacroBitRegions[sentParticleIndex].m_lowBits == 0 && m_cylinders[cylinderIndex].m_macroBitRegions.m_lowBits == 0)
		{
			macroLowBitResult = 1;
		}
		if (macroLowBitResult != 0 || macroHighBitResult != 0)
		{
			//Micro Check
			uint microLowBitResult = (m_particleMicroBitRegions[sentParticleIndex].m_lowBits & m_cylinders[cylinderIndex].m_microBitRegions.m_lowBits);
			uint microHighBitResult = (m_particleMicroBitRegions[sentParticleIndex].m_highBits & m_cylinders[cylinderIndex].m_microBitRegions.m_highBits);
			if (m_particleMicroBitRegions[sentParticleIndex].m_highBits == 0 && m_cylinders[cylinderIndex].m_microBitRegions.m_highBits == 0)
			{
				microHighBitResult = 1;
			}
			if (m_particleMicroBitRegions[sentParticleIndex].m_lowBits == 0 && m_cylinders[cylinderIndex].m_microBitRegions.m_lowBits == 0)
			{
				microLowBitResult = 1;
			}
			if (microLowBitResult != 0 || microHighBitResult != 0)
			{
				//Bounding Disc Check
				if (DoSpheresOverlap(m_particleProposedPositions[sentParticleIndex], m_ropeRadius, m_cylinders[cylinderIndex].m_boundingDiscCenter, m_cylinders[cylinderIndex].m_boundingDiscRadius))
				{
					//Sphere vs Cylinder
					float3 newPosition = PushSphereOutOfFixedCylinder3D(m_particleProposedPositions[sentParticleIndex], m_ropeRadius, m_cylinders[cylinderIndex].m_cylinder);
					if (newPosition.x != m_particleProposedPositions[sentParticleIndex].x || newPosition.y != m_particleProposedPositions[sentParticleIndex].y || newPosition.z != m_particleProposedPositions[sentParticleIndex].z)
					{
						float3 displacement = newPosition - m_particleProposedPositions[sentParticleIndex];
						m_particleCollisionNormals[sentParticleIndex] = GetNormalized(displacement);
						m_particleJacobiCorrections[sentParticleIndex].x += displacement.x;
						m_particleJacobiCorrections[sentParticleIndex].y += displacement.y;
						m_particleJacobiCorrections[sentParticleIndex].z += displacement.z;
						m_particleJacobiCorrections[sentParticleIndex].w++;
					}
				}
			}
		}
	}

	//Capsules
	for (int capsuleIndex = 0; capsuleIndex < m_totalCapsules; capsuleIndex++)
	{
		//Macro Check
		uint macroLowBitResult = (m_particleMacroBitRegions[sentParticleIndex].m_lowBits & m_capsules[capsuleIndex].m_macroBitRegions.m_lowBits);
		uint macroHighBitResult = (m_particleMacroBitRegions[sentParticleIndex].m_highBits & m_capsules[capsuleIndex].m_macroBitRegions.m_highBits);
		if (m_particleMacroBitRegions[sentParticleIndex].m_highBits == 0 && m_capsules[capsuleIndex].m_macroBitRegions.m_highBits == 0)
		{
			macroHighBitResult = 1;
		}
		if (m_particleMacroBitRegions[sentParticleIndex].m_lowBits == 0 && m_capsules[capsuleIndex].m_macroBitRegions.m_lowBits == 0)
		{
			macroLowBitResult = 1;
		}
		if (macroLowBitResult != 0 || macroHighBitResult != 0)
		{
			//Micro Check
			uint microLowBitResult = (m_particleMicroBitRegions[sentParticleIndex].m_lowBits & m_capsules[capsuleIndex].m_microBitRegions.m_lowBits);
			uint microHighBitResult = (m_particleMicroBitRegions[sentParticleIndex].m_highBits & m_capsules[capsuleIndex].m_microBitRegions.m_highBits);
			if (m_particleMicroBitRegions[sentParticleIndex].m_highBits == 0 && m_capsules[capsuleIndex].m_microBitRegions.m_highBits == 0)
			{
				microHighBitResult = 1;
			}
			if (m_particleMicroBitRegions[sentParticleIndex].m_lowBits == 0 && m_capsules[capsuleIndex].m_microBitRegions.m_lowBits == 0)
			{
				microLowBitResult = 1;
			}
			if (microLowBitResult != 0 || microHighBitResult != 0)
			{
				//Bounding Disc Check
				if (DoSpheresOverlap(m_particleProposedPositions[sentParticleIndex], m_ropeRadius, m_capsules[capsuleIndex].m_boundingDiscCenter, m_capsules[capsuleIndex].m_boundingDiscRadius))
				{
					//Sphere vs Cylinder
					float3 newPosition = PushSphereOutOfFixedCapsule3D(m_particleProposedPositions[sentParticleIndex], m_ropeRadius, m_capsules[capsuleIndex].m_capsule);
					if (newPosition.x != m_particleProposedPositions[sentParticleIndex].x || newPosition.y != m_particleProposedPositions[sentParticleIndex].y || newPosition.z != m_particleProposedPositions[sentParticleIndex].z)
					{
						float3 displacement = newPosition - m_particleProposedPositions[sentParticleIndex];
						m_particleCollisionNormals[sentParticleIndex] = GetNormalized(displacement);
						m_particleJacobiCorrections[sentParticleIndex].xyz += (displacement);
						m_particleJacobiCorrections[sentParticleIndex].w++;
					}
				}
			}
		}
	}

	//Spheres
	for (int sphereIndex = 0; sphereIndex < m_totalSpheres; sphereIndex++)
	{
		//Macro Check
		uint macroLowBitResult = (m_particleMacroBitRegions[sentParticleIndex].m_lowBits & m_spheres[sphereIndex].m_macroBitRegions.m_lowBits);
		uint macroHighBitResult = (m_particleMacroBitRegions[sentParticleIndex].m_highBits & m_spheres[sphereIndex].m_macroBitRegions.m_highBits);
		if (m_particleMacroBitRegions[sentParticleIndex].m_highBits == 0 && m_spheres[sphereIndex].m_macroBitRegions.m_highBits == 0)
		{
			macroHighBitResult = 1;
		}
		if (m_particleMacroBitRegions[sentParticleIndex].m_lowBits == 0 && m_spheres[sphereIndex].m_macroBitRegions.m_lowBits == 0)
		{
			macroLowBitResult = 1;
		}
		if (macroLowBitResult != 0 || macroHighBitResult != 0)
		{
			//Micro Check
			uint microLowBitResult = (m_particleMicroBitRegions[sentParticleIndex].m_lowBits & m_spheres[sphereIndex].m_microBitRegions.m_lowBits);
			uint microHighBitResult = (m_particleMicroBitRegions[sentParticleIndex].m_highBits & m_spheres[sphereIndex].m_microBitRegions.m_highBits);
			if (m_particleMicroBitRegions[sentParticleIndex].m_highBits == 0 && m_spheres[sphereIndex].m_microBitRegions.m_highBits == 0)
			{
				microHighBitResult = 1;
			}
			if (m_particleMicroBitRegions[sentParticleIndex].m_lowBits == 0 && m_spheres[sphereIndex].m_microBitRegions.m_lowBits == 0)
			{
				microLowBitResult = 1;
			}
			if (microLowBitResult != 0 || microHighBitResult != 0)
			{
				//Sphere vs Cylinder
				float3 newPosition = PushSphereOutOfFixedSphere3D(m_particleProposedPositions[sentParticleIndex], m_ropeRadius, m_spheres[sphereIndex].m_sphere.m_center, m_spheres[sphereIndex].m_sphere.m_radius);
				if (newPosition.x != m_particleProposedPositions[sentParticleIndex].x || newPosition.y != m_particleProposedPositions[sentParticleIndex].y || newPosition.z != m_particleProposedPositions[sentParticleIndex].z)
				{
					float3 displacement = newPosition - m_particleProposedPositions[sentParticleIndex];
					m_particleCollisionNormals[sentParticleIndex] = GetNormalized(displacement);
					m_particleJacobiCorrections[sentParticleIndex].xyz += (displacement);
					m_particleJacobiCorrections[sentParticleIndex].w++;
				}
			}
		}
	}


	//World Bounds Collisions
	ProjectWorldBoundsConstraintsSpheresJacobi(sentParticleIndex);
}
