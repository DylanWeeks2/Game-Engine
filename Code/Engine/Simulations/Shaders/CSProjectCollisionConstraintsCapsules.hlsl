static const uint threadX = 32;
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
	double3 m_mins;
	double3 m_maxs;
};

//------------------------------------------------------------------------------------------------
struct AABBCollisionObject
{
	double		m_vptrOffset;
	uint64_t	m_macroBitRegions;
	uint64_t	m_microBitRegions;
	double3		m_boundingDiscCenter;
	double		m_boundingDiscRadius;
	AABB3		m_aabb;
};

//------------------------------------------------------------------------------------------------
struct OBB3
{
	double3 m_center;
	double3 m_iBasisNormal;
	double3 m_jBasisNormal;
	double3 m_kBasisNormal;
	double3 m_halfDimensions;
};

//------------------------------------------------------------------------------------------------
struct OBBCollisionObject
{
	double		m_vptrOffset;
	uint64_t	m_macroBitRegions;
	uint64_t	m_microBitRegions;
	double3		m_boundingDiscCenter;
	double		m_boundingDiscRadius;
	OBB3		m_obb;
};

//------------------------------------------------------------------------------------------------
struct Cylinder3
{
	double3 m_start;
	double3 m_end;
	double3 m_iBasis;
	double3 m_jBasis;
	double3 m_kBasis;
	double  m_radius;
};

//------------------------------------------------------------------------------------------------
struct CylinderCollisionObject
{
	double		m_vptrOffset;
	uint64_t	m_macroBitRegions;
	uint64_t	m_microBitRegions;
	double3		m_boundingDiscCenter;
	double		m_boundingDiscRadius;
	Cylinder3	m_cylinder;
};

//------------------------------------------------------------------------------------------------
struct Capsule3
{
	Cylinder3	m_bone;
	double		m_radius;
};

//------------------------------------------------------------------------------------------------
struct CapsuleCollisionObject
{
	double		m_vptrOffset;
	uint64_t	m_macroBitRegions;
	uint64_t	m_microBitRegions;
	double3		m_boundingDiscCenter;
	double		m_boundingDiscRadius;
	Capsule3	m_capsule;
	double3		m_collisionNormalStart;
	double3		m_jacobiCorrectionStart;
	int			m_jacobiConstraintTotalStart;
	int			m_isCollidingStart;
	double3		m_collisionNormalEnd;
	double3		m_jacobiCorrectionEnd;
	int			m_jacobiConstraintTotalEnd;
	int			m_isCollidingEnd;
};

//------------------------------------------------------------------------------------------------
struct Sphere3
{
	double3 m_center;
	double  m_radius;
};

//------------------------------------------------------------------------------------------------
struct SphereCollisionObject
{
	double		m_vptrOffset;
	uint64_t	m_macroBitRegions;
	uint64_t	m_microBitRegions;
	double3		m_boundingDiscCenter;
	double		m_boundingDiscRadius;
	Sphere3 m_sphere;
};

//------------------------------------------------------------------------------------------------
struct LineSegment3
{
	double3 m_start;
	double3 m_end;
};

//------------------------------------------------------------------------------------------------
cbuffer RopeConstantBuffer : register(b0)
{
	uint	m_totalParticles;
	double	m_ropeRadius;
	double	m_gravityCoefficient;
	double	m_physicsTimestep;
	double	m_dampingCoefficient;
	int		m_totalSolverIterations;
	double	m_desiredDistance;
	double	m_compressionCoefficient;
	double	m_stretchingCoefficient;
	double	m_kineticFrictionCoefficient;
	double	m_bendingConstraintDistance;
	double	m_bendingCoefficient;
	AABB3	m_worldBounds;
	int		m_totalCollisionObjects;
	int		m_totalAABBs;
	int		m_totalOBBs;
	int		m_totalCylinders;
	int		m_totalCapsules;
	int		m_totalSpheres;
};

//------------------------------------------------------------------------------------------------
RWStructuredBuffer<CapsuleCollisionObject>	m_ropeCapsules:register(u7);
StructuredBuffer<AABBCollisionObject>		m_aabbs : register(t3);
StructuredBuffer<OBBCollisionObject>		m_obbs : register(t4);
StructuredBuffer<CapsuleCollisionObject>	m_capsules : register(t5);
StructuredBuffer<CylinderCollisionObject>	m_cylinders : register(t6);
StructuredBuffer<SphereCollisionObject>		m_spheres : register(t7);

//MATH UTILITIES
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
double GetLength(double3 vec3)
{
	double xSquared = vec3.x * vec3.x;
	double ySquared = vec3.y * vec3.y;
	double zSquared = vec3.z * vec3.z;
	double sqrtValue = xSquared + ySquared + zSquared;
	return double(sqrt(float(sqrtValue)));
}

//-----------------------------------------------------------------------------------------------
double GetLengthSquared(double3 vec3)
{
	double xSquared = vec3.x * vec3.x;
	double ySquared = vec3.y * vec3.y;
	double zSquared = vec3.z * vec3.z;
	return double(xSquared + ySquared + zSquared);
}

//-----------------------------------------------------------------------------------------------
double GetDistanceSquared3D(double3 positionA, double3 positionB)
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
bool DoSpheresOverlap(double3 centerA, double radiusA, double3 centerB, double radiusB)
{
	if (GetDistanceSquared3D(centerA, centerB) < (radiusA + radiusB) * (radiusA + radiusB))
	{
		return true;
	}

	return false;
}

//-----------------------------------------------------------------------------------------------
double3 GetNormalized(double3 vec3)
{
	if (GetLength(vec3) == 0.0f)
	{
		return double3(0.0f, 0.0f, 0.0f);
	}

	double scale = 1.0f / GetLength(vec3);
	return double3(vec3.x * scale, vec3.y * scale, vec3.z * scale);
}

//-----------------------------------------------------------------------------------------------
double DotProduct3D(double3 a, double3 b)
{
	return (a.x * b.x) + (a.y * b.y) + (a.z * b.z);
}

//-----------------------------------------------------------------------------------------------
double GetProjectedLength3D(double3 vectorToProject, double3 vectorToProjectOnto)
{
	return DotProduct3D(vectorToProject, GetNormalized(vectorToProjectOnto));
}

//-----------------------------------------------------------------------------------------------
double3 GetProjectedOnto3D(double3 vectorToProject, double3 vectorToProjectOnto)
{
	double projectedLength = GetProjectedLength3D(vectorToProject, vectorToProjectOnto);
	return projectedLength * GetNormalized(vectorToProjectOnto);
}

//-----------------------------------------------------------------------------------------------
double3 SetLength(double3 vecToAlter, double maxLength)
{
	vecToAlter = GetNormalized(vecToAlter) * maxLength;
	return vecToAlter;
}

//-----------------------------------------------------------------------------------------------
double3 GetNearestPointAABB3(AABB3 aabb, double3 referencePosition)
{
	double3 nearestPoint;

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
double3 GetDimensionsAABB3(AABB3 aabb)
{
	double3 dimensions = double3(aabb.m_maxs.x - aabb.m_mins.x, aabb.m_maxs.y - aabb.m_mins.y, aabb.m_maxs.z - aabb.m_mins.z);
	return dimensions;
}

//-----------------------------------------------------------------------------------------------
double3 GetCenterAABB3(AABB3 aabb)
{
	double3 center = (aabb.m_mins + aabb.m_maxs) * 0.5f;
	return center;
}

//-----------------------------------------------------------------------------------------------
double3 GetNearestEdgePositionAABB3(AABB3 aabb, double3 referencePosition)
{
	double3 edgePos = referencePosition;
	double3 center = GetCenterAABB3(aabb);
	double3 halfDimensions = GetDimensionsAABB3(aabb) * 0.5f;
	double distancesToSides[6];
	distancesToSides[0] = ((center.x + halfDimensions.x) - referencePosition.x);
	distancesToSides[1] = (referencePosition.x - (center.x - halfDimensions.x));
	distancesToSides[2] = ((center.y + halfDimensions.y) - referencePosition.y);
	distancesToSides[3] = (referencePosition.y - (center.y - halfDimensions.y));
	distancesToSides[4] = ((center.z + halfDimensions.z) - referencePosition.z);
	distancesToSides[5] = (referencePosition.z - (center.z - halfDimensions.z));

	double bestDistance = 10000000000000000000.0;
	int bestDistanceIndex = -1;
	double secondBestDistance = 10000000000000000000.0;
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
AABB3 SetDimensionsAABB3(AABB3 aabb, double3 newDimensions)
{
	double3 center = GetCenterAABB3(aabb);
	double originalCenterX = center.x;
	double originalCenterY = center.y;
	double originalCenterZ = center.z;

	double halfOfX = newDimensions.x * 0.5f;
	double halfOfY = newDimensions.y * 0.5f;
	double halfOfZ = newDimensions.z * 0.5f;

	aabb.m_mins.x = originalCenterX - halfOfX;
	aabb.m_maxs.x = originalCenterX + halfOfX;
	aabb.m_mins.y = originalCenterY - halfOfY;
	aabb.m_maxs.y = originalCenterY + halfOfY;
	aabb.m_mins.z = originalCenterZ - halfOfZ;
	aabb.m_maxs.z = originalCenterZ + halfOfZ;
	return aabb;
}

//-----------------------------------------------------------------------------------------------
bool IsPointInsideAABB3D(double3 refPoint, AABB3 aabb)
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
double GetLengthdouble2(double2 vec2) 
{
	double xSquared = vec2.x * vec2.x;
	double ySquared = vec2.y * vec2.y;
	double sqrtValue = xSquared + ySquared;
	return sqrt(float(sqrtValue));
}

//-----------------------------------------------------------------------------------------------
double2 GetNormalizeddouble2(double2 vec2)
{
	if (vec2.x == 0.0f && vec2.y == 0.0f)
	{
		return double2(0.0f, 0.0f);
	}
	double scale = 1.0f / GetLengthdouble2(vec2);
	return double2(vec2.x * scale, vec2.y * scale);
}

//-----------------------------------------------------------------------------------------------
double2 SetLengthdouble2(double2 vecToAlter, double newLength)
{
	vecToAlter = GetNormalizeddouble2(vecToAlter) * newLength;
	return vecToAlter;
}

//-----------------------------------------------------------------------------------------------
double2 ClampLengthdouble2(double2 vectorToClamp, double maxLength)
{
	double currentLength = GetLengthdouble2(vectorToClamp);
	if (currentLength > maxLength)
	{
		double scale = 1 / (currentLength / maxLength);
		vectorToClamp.x *= scale;
		vectorToClamp.y *= scale;
	}
	return vectorToClamp;
}

//-----------------------------------------------------------------------------------------------
double3 PushSphereOutOfFixedAABB3D(double3 mobileSphereCenter, double sphereRadius, AABB3 fixedBox)
{
	double3 previousPos = mobileSphereCenter;
	double3 nearestPoint = GetNearestPointAABB3(fixedBox, mobileSphereCenter);
	if (mobileSphereCenter.x == nearestPoint.x && mobileSphereCenter.y == nearestPoint.y && mobileSphereCenter.z == nearestPoint.z)
	{
		double3 halfDimensions = GetDimensionsAABB3(fixedBox) * 0.5f;
		double3 aabbCenter = GetCenterAABB3(fixedBox);
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
		return mobileSphereCenter;
		//return true;
	}
	else
	{
		double3 displacementFromPointToCenter = mobileSphereCenter - nearestPoint;
		double distSquaredPointToCenter = GetLengthSquared(displacementFromPointToCenter);
		if (distSquaredPointToCenter < sphereRadius * sphereRadius)
		{
			double offset = sphereRadius - sqrt(float(distSquaredPointToCenter));
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
double3 PushDiscOutOfFixedOBB3D(double3 mobileSphereCenter, double sphereRadius, OBB3 fixedOBB)
{
	//Transform to local space
	double4x4 modelMatrix;
	modelMatrix[0] = double4(fixedOBB.m_iBasisNormal, 0.0f);
	modelMatrix[1] = double4(fixedOBB.m_jBasisNormal, 0.0f);
	modelMatrix[2] = double4(fixedOBB.m_kBasisNormal, 0.0f);
	modelMatrix[3] = double4(fixedOBB.m_center, 0.0f);

	AABB3 aabb;
	aabb.m_mins = double3(0.0f, 0.0f, 0.0f);
	aabb.m_maxs = double3(0.0f, 0.0f, 0.0f);
	double3 dimensions;
	dimensions.x = fixedOBB.m_halfDimensions.x + fixedOBB.m_halfDimensions.x;
	dimensions.y = fixedOBB.m_halfDimensions.y + fixedOBB.m_halfDimensions.y;
	dimensions.z = fixedOBB.m_halfDimensions.z + fixedOBB.m_halfDimensions.z;
	aabb = SetDimensionsAABB3(aabb, dimensions);

	double3 center = double3(0.0f, 0.0f, 0.0f);
	double3 displacementToSphere = mobileSphereCenter - fixedOBB.m_center;
	double projectionIBasis = DotProduct3D(displacementToSphere, fixedOBB.m_iBasisNormal);
	double projectionJBasis = DotProduct3D(displacementToSphere, fixedOBB.m_jBasisNormal);
	double projectionKBasis = DotProduct3D(displacementToSphere, fixedOBB.m_kBasisNormal);
	double3 localSphereCenter = center + double3(1.0f, 0.0f, 0.0f) * projectionIBasis + double3(0.0f, 1.0f, 0.0f) * projectionJBasis + double3(0.0f, 0.0f, 1.0f) * projectionKBasis;

	double3 newPosition = PushSphereOutOfFixedAABB3D(localSphereCenter, sphereRadius, aabb);
	if (newPosition.x != localSphereCenter.x || newPosition.y != localSphereCenter.y || newPosition.z != localSphereCenter.z)
	{
		//Tranform back to world space
		double3 transformedPosition = double3(0.0f, 0.0f, 0.0f);
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
double2 GetNearestPointOnDisc2D(double2 referencePosition, double2 discCenter, double discRadius)
{
	double2 distance = referencePosition - discCenter;
	distance = ClampLengthdouble2(distance, discRadius);
	return discCenter + distance;
}

//-----------------------------------------------------------------------------------------------
double3 GetNearestPointOnCylinderZ3D(double3 referencePosition, Cylinder3 cylinder)
{
	double3 nearestPoint;
	double2 referencePositionXY = double2(referencePosition.x, referencePosition.y);
	double2 cylinderCenterXY = double2(cylinder.m_start.x, cylinder.m_start.y);
	if (referencePosition.z < cylinder.m_start.z)
	{
		double2 nearestPointXY = GetNearestPointOnDisc2D(referencePositionXY, cylinderCenterXY, cylinder.m_radius);
		nearestPoint = double3(nearestPointXY.x, nearestPointXY.y, cylinder.m_start.z);
	}
	else if (referencePosition.z > cylinder.m_end.z)
	{
		double2 nearestPointXY = GetNearestPointOnDisc2D(referencePositionXY, cylinderCenterXY, cylinder.m_radius);
		nearestPoint = double3(nearestPointXY.x, nearestPointXY.y, cylinder.m_end.z);
	}
	else
	{
		double2 nearestPointXY = GetNearestPointOnDisc2D(referencePositionXY, cylinderCenterXY, cylinder.m_radius);
		nearestPoint = double3(nearestPointXY.x, nearestPointXY.y, referencePosition.z);
	}
	return nearestPoint;
}

//-----------------------------------------------------------------------------------------------
double3 PushSphereOutOfFixedCylinderZ3D(double3 mobileSphereCenter, double mobileSphereRadius, Cylinder3 cylinder)
{
	double3 nearestPoint = GetNearestPointOnCylinderZ3D(mobileSphereCenter, cylinder);
	if (mobileSphereCenter.x == nearestPoint.x && mobileSphereCenter.y == nearestPoint.y && mobileSphereCenter.z == nearestPoint.z)
	{
		double2 displacementFromCylinderCenterXY = double2(mobileSphereCenter.x, mobileSphereCenter.y) - double2(cylinder.m_start.x, cylinder.m_start.y);
		double distanceToOutside = cylinder.m_radius - GetLengthdouble2(displacementFromCylinderCenterXY);
		double distanceToTop = abs(mobileSphereCenter.z - cylinder.m_end.z);
		double distanceToBottom = abs(mobileSphereCenter.z - cylinder.m_start.z);

		if (distanceToOutside < distanceToTop)
		{
			if (distanceToOutside < distanceToBottom)
			{
				double2 cylinderCenterXY = double2(cylinder.m_start.x, cylinder.m_start.y);
				double2 newCenter = cylinderCenterXY + GetNormalizeddouble2(displacementFromCylinderCenterXY) * (cylinder.m_radius + mobileSphereRadius);
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
		double3 differenceFromPointToCenter = mobileSphereCenter - nearestPoint;
		double distance = GetLengthSquared(differenceFromPointToCenter);
		if (distance < mobileSphereRadius * mobileSphereRadius)
		{
			double offset = mobileSphereRadius - sqrt(float(distance));
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
double3 PushSphereOutOfFixedCylinder3D(double3 mobileSphereCenter, double mobileSphereRadius, Cylinder3 cylinder)
{
	//Transform to local space
	double4x4 modelMatrix;
	modelMatrix[0] = double4(cylinder.m_iBasis, 0.0f);
	modelMatrix[1] = double4(cylinder.m_jBasis, 0.0f);
	modelMatrix[2] = double4(cylinder.m_kBasis, 0.0f);
	modelMatrix[3] = double4(cylinder.m_start, 0.0f);

	double length = GetLength(cylinder.m_end - cylinder.m_start);
	Cylinder3 localCylinder;
	localCylinder.m_radius = cylinder.m_radius;
	localCylinder.m_start = double3(0.0f, 0.0f, 0.0f);
	localCylinder.m_end = localCylinder.m_start + double3(0.0f, 0.0f, length);
	localCylinder.m_iBasis = double3(1.0f, 0.0f, 0.0f);
	localCylinder.m_jBasis = double3(0.0f, 1.0f, 0.0f);
	localCylinder.m_kBasis = double3(0.0f, 0.0f, 1.0f);

	double3 displacementToSphere = mobileSphereCenter - cylinder.m_start;
	double projectionIBasis = DotProduct3D(displacementToSphere, cylinder.m_iBasis);
	double projectionJBasis = DotProduct3D(displacementToSphere, cylinder.m_jBasis);
	double projectionKBasis = DotProduct3D(displacementToSphere, cylinder.m_kBasis);
	double3 localSphereCenter = localCylinder.m_start + localCylinder.m_iBasis * projectionIBasis + localCylinder.m_jBasis * projectionJBasis + localCylinder.m_kBasis * projectionKBasis;

	double3 newPosition = PushSphereOutOfFixedCylinderZ3D(localSphereCenter, mobileSphereRadius, localCylinder);
	if (newPosition.x != localSphereCenter.x || newPosition.y != localSphereCenter.y || newPosition.z != localSphereCenter.z)
	{
		double3 transformedPosition = double3(0.0f, 0.0f, 0.0f);
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
double3 GetNearestPointOnLineSegment3D(double3 referencePosition, LineSegment3 lineSegment)
{
	double3 startDisplacement = referencePosition - lineSegment.m_start;
	double3 endDisplacement = referencePosition - lineSegment.m_end;
	double3 lineSegmentDisplacement = lineSegment.m_end - lineSegment.m_start;

	if (DotProduct3D(lineSegmentDisplacement, startDisplacement) < 0.0f)
	{
		return lineSegment.m_start;
	}

	if (DotProduct3D(lineSegmentDisplacement, endDisplacement) > 0.0f)
	{
		return lineSegment.m_end;
	}

	double3 nearestPoint = GetProjectedOnto3D(startDisplacement, lineSegmentDisplacement);
	return lineSegment.m_start + nearestPoint;
}

//-----------------------------------------------------------------------------------------------
double3 PushSphereOutOfFixedCapsule3D(double3 mobileSphereCenter, double mobileSphereRadius, Capsule3 capsule)
{
	LineSegment3 bone;
	bone.m_start = capsule.m_bone.m_start;
	bone.m_end = capsule.m_bone.m_end;
	double3 nearestPointBone = GetNearestPointOnLineSegment3D(mobileSphereCenter, bone);
	double3 differenceFromPointToCenter = mobileSphereCenter - nearestPointBone;
	double distance = GetLengthSquared(differenceFromPointToCenter);
	if (distance < ((mobileSphereRadius + capsule.m_radius) * (mobileSphereRadius + capsule.m_radius)))
	{
		double offset = mobileSphereRadius + capsule.m_radius - sqrt(float(distance));
		differenceFromPointToCenter = SetLength(differenceFromPointToCenter, offset);
		mobileSphereCenter += differenceFromPointToCenter;
		return mobileSphereCenter;
		//return true;
	}

	return mobileSphereCenter;
	//return false;
}

//-----------------------------------------------------------------------------------------------
double3 PushSphereOutOfFixedSphere3D(double3 mobileSphereCenter, double mobileSphereRadius, double3 fixedSphereCenter, double fixedSphereRadius)
{
	double3 difference = (mobileSphereCenter - fixedSphereCenter);
	double distanceSquared = GetLengthSquared(difference);
	if ((fixedSphereRadius + mobileSphereRadius) * (fixedSphereRadius + mobileSphereRadius) >= distanceSquared)
	{
		double offset = fixedSphereRadius + mobileSphereRadius - sqrt(float(distanceSquared));
		difference = SetLength(difference, offset);
		mobileSphereCenter += difference;
		return mobileSphereCenter;
		//return true;
	}

	return mobileSphereCenter;
	//return false;
}

//-----------------------------------------------------------------------------------------------
void PushCapsuleOutOfFixedSphere3D(Capsule3 capsule, double3 fixedSphereCenter, double fixedSphereRadius, out double3 returnPosition1, out double3 returnPosition2)
{
	LineSegment3 bone;
	bone.m_start = capsule.m_bone.m_start;
	bone.m_end = capsule.m_bone.m_end;
	double3 nearestPointOnBone = GetNearestPointOnLineSegment3D(fixedSphereCenter, bone);
	double3 originalNearestPointOnBone = nearestPointOnBone;
	double3 diplacementFromCenterToBone = nearestPointOnBone - fixedSphereCenter;
	double distanceFromCenterToBone = GetLengthSquared(diplacementFromCenterToBone);
	double overallRadii = (fixedSphereRadius + capsule.m_radius);
	if (distanceFromCenterToBone < overallRadii * overallRadii)
	{
		//Do Sphere Push
		double offset = overallRadii - sqrt(float(distanceFromCenterToBone));
		diplacementFromCenterToBone = SetLength(diplacementFromCenterToBone, offset);
		nearestPointOnBone += diplacementFromCenterToBone;

		//Get Push Difference and spread accurately to capsule start and end
		double3 nearestPointDisplacement = nearestPointOnBone - originalNearestPointOnBone;
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
			double3 startToEndOfCapsuleDisplacement = capsule.m_bone.m_end - capsule.m_bone.m_start;
			double3 capsuleStartToLinePointDisplacement = (originalNearestPointOnBone - capsule.m_bone.m_end);
			double startToEndOfCapsuleDistance = GetLength(startToEndOfCapsuleDisplacement);
			double capsuleStartToLinePointDistance = GetLength(capsuleStartToLinePointDisplacement);
			double percentToPushStart = capsuleStartToLinePointDistance / startToEndOfCapsuleDistance;
			double percentToPushEnd = (startToEndOfCapsuleDistance - capsuleStartToLinePointDistance) / startToEndOfCapsuleDistance;
			capsule.m_bone.m_start += nearestPointDisplacement * percentToPushStart;
			capsule.m_bone.m_end += nearestPointDisplacement * percentToPushEnd;
		}
	}
	returnPosition1 = capsule.m_bone.m_start;
	returnPosition2 = capsule.m_bone.m_end;
}

//-----------------------------------------------------------------------------------------------
void PushCapsuleOutOfFixedAABB3D(Capsule3 capsule, AABB3 aabb, out double3 returnPosition1, out double3 returnPosition2)
{
	//Initial Sphere Pushes
	bool returnValue = false;
	capsule.m_bone.m_start = PushSphereOutOfFixedAABB3D(capsule.m_bone.m_start, capsule.m_radius, aabb);
	capsule.m_bone.m_end = PushSphereOutOfFixedAABB3D(capsule.m_bone.m_end, capsule.m_radius, aabb);

	//Calculate intersects and get best intersect
	double tValues[6];
	double bestTValue = 1000000000000.0f;
	double bestDistanceSquared = 1000000000000.0f;
	double3 bestIntersectPoint = double3(-1000.0f, -1000.0f, -1000.0f);
	double3 bestNearestPointAABB = double3(-1000.0f, -1000.0f, -1000.0f);
	double3 capsuleRayStartToEnd = (capsule.m_bone.m_end - capsule.m_bone.m_start);
	double rayZScale = 1.0 / capsuleRayStartToEnd.z;
	double rayYScale = 1.0 / capsuleRayStartToEnd.y;
	double rayXScale = 1.0 / capsuleRayStartToEnd.x;
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
			double3 intersectPoint = capsule.m_bone.m_start + (tValues[tIndex] * capsuleRayStartToEnd);
			double3 nearestPointAABB = GetNearestEdgePositionAABB3(aabb, intersectPoint);
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
		returnPosition1 = capsule.m_bone.m_start;
		returnPosition2 = capsule.m_bone.m_end;
		return;
	}

	//Calculate nearest points and get displacements
	double3 center = GetCenterAABB3(aabb);
	LineSegment3 bone;
	bone.m_start = capsule.m_bone.m_start;
	bone.m_end = capsule.m_bone.m_end;
	double3 nearestPointBone = GetNearestPointOnLineSegment3D(bestNearestPointAABB, bone);
	bestNearestPointAABB = GetNearestEdgePositionAABB3(aabb, nearestPointBone);
	nearestPointBone = GetNearestPointOnLineSegment3D(bestNearestPointAABB, bone);
	if (IsPointInsideAABB3D(nearestPointBone, aabb))
	{
		double3 displacementCenterToBone = nearestPointBone - center;
		double3 displacementCenterToCorner = bestNearestPointAABB - center;
		if (GetLengthSquared(displacementCenterToBone) < GetLengthSquared(displacementCenterToCorner))
		{
			double3 displacement = bestNearestPointAABB - nearestPointBone;
			displacement = SetLength(displacement, GetLength(displacement) + capsule.m_radius);
			capsule.m_bone.m_start += displacement;
			capsule.m_bone.m_end += displacement;
		}
	}
	else
	{
		double3 displacement = nearestPointBone - bestNearestPointAABB;
		double distanceSquared = GetLengthSquared(displacement);
		if (distanceSquared < capsule.m_radius * capsule.m_radius)
		{
			double offset = capsule.m_radius - sqrt(float(distanceSquared));
			displacement = SetLength(displacement, offset);
			capsule.m_bone.m_start += displacement;
			capsule.m_bone.m_end += displacement;
		}
	}
	
	returnPosition1 = capsule.m_bone.m_start;
	returnPosition2 = capsule.m_bone.m_end;
}

//-----------------------------------------------------------------------------------------------
void PushCapsuleOutOfFixedOBB3D(Capsule3 capsule, OBB3 obb, out double3 returnPosition1, out double3 returnPosition2)
{
	//Transform to local space
	double4x4 modelMatrix;
	modelMatrix[0] = double4(obb.m_iBasisNormal, 0.0f);
	modelMatrix[1] = double4(obb.m_jBasisNormal, 0.0f);
	modelMatrix[2] = double4(obb.m_kBasisNormal, 0.0f);
	modelMatrix[3] = double4(obb.m_center, 0.0f);

	AABB3 aabb;
	aabb.m_mins = double3(0.0f, 0.0f, 0.0f);
	aabb.m_maxs = double3(0.0f, 0.0f, 0.0f);
	double3 dimensions;
	dimensions.x = obb.m_halfDimensions.x + obb.m_halfDimensions.x;
	dimensions.y = obb.m_halfDimensions.y + obb.m_halfDimensions.y;
	dimensions.z = obb.m_halfDimensions.z + obb.m_halfDimensions.z;
	aabb = SetDimensionsAABB3(aabb, dimensions);

	double3 center = double3(0.0f, 0.0f, 0.0f);
	double3 displacementToCapsuleStart = capsule.m_bone.m_start - obb.m_center;
	double projectionIBasis = DotProduct3D(displacementToCapsuleStart, obb.m_iBasisNormal);
	double projectionJBasis = DotProduct3D(displacementToCapsuleStart, obb.m_jBasisNormal);
	double projectionKBasis = DotProduct3D(displacementToCapsuleStart, obb.m_kBasisNormal);
	double3 localCapsuleStart = center + double3(1.0f, 0.0f, 0.0f) * projectionIBasis + double3(0.0f, 1.0f, 0.0f) * projectionJBasis + double3(0.0f, 0.0f, 1.0f) * projectionKBasis;
	double3 displacementToCapsuleEnd = capsule.m_bone.m_end - obb.m_center;
	projectionIBasis = DotProduct3D(displacementToCapsuleEnd, obb.m_iBasisNormal);
	projectionJBasis = DotProduct3D(displacementToCapsuleEnd, obb.m_jBasisNormal);
	projectionKBasis = DotProduct3D(displacementToCapsuleEnd, obb.m_kBasisNormal);
	double3 localCapsuleEnd = center + double3(1.0f, 0.0f, 0.0f) * projectionIBasis + double3(0.0f, 1.0f, 0.0f) * projectionJBasis + double3(0.0f, 0.0f, 1.0f) * projectionKBasis;

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
		double3 transformedPosition = double3(0.0f, 0.0f, 0.0f);
		transformedPosition.x = (modelMatrix[0].x * returnPosition1.x) + (modelMatrix[1].x * returnPosition1.y) + (modelMatrix[2].x * returnPosition1.z) + modelMatrix[3].x;
		transformedPosition.y = (modelMatrix[0].y * returnPosition1.x) + (modelMatrix[1].y * returnPosition1.y) + (modelMatrix[2].y * returnPosition1.z) + modelMatrix[3].y;
		transformedPosition.z = (modelMatrix[0].z * returnPosition1.x) + (modelMatrix[1].z * returnPosition1.y) + (modelMatrix[2].z * returnPosition1.z) + modelMatrix[3].z;
		returnPosition1 = transformedPosition;
		transformedPosition = double3(0.0f, 0.0f, 0.0f); 
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
void GetNearestPointsBetweenLines3D(LineSegment3 lineA, LineSegment3 lineB, out double3 nearestPointLineA, out double3 nearestPointLineB)
{
	double3 directionA = lineA.m_end - lineA.m_start;
	double3 directionB = lineB.m_end - lineB.m_start;

	double b = DotProduct3D(directionA, directionB);
	if (b == 1 || b == -1)
	{
		nearestPointLineA = lineA.m_start;
		nearestPointLineB = lineB.m_start;
		return;
	}

	double3 r = lineA.m_start - lineB.m_start;
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
}

//-----------------------------------------------------------------------------------------------
void PushCapsuleOutOfFixedCylinderZ3D(Capsule3 mobileCapsule, Cylinder3 fixedCylinder, out double3 returnPosition1, out double3 returnPosition2)
{
	//Evaluate top and bottom intersections
	LineSegment3 mobileLine;
	mobileLine.m_start = mobileCapsule.m_bone.m_start;
	mobileLine.m_end = mobileCapsule.m_bone.m_end;
	LineSegment3 fixedLine;
	fixedLine.m_start = fixedCylinder.m_start;
	fixedLine.m_end = fixedCylinder.m_end;
	double3 topOfCylinder = double3(0.0f, 0.0f, 0.0f);
	double3 bottomOfCylinder = double3(0.0f, 0.0f, 0.0f);
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
		double3 nearestPoint1 = double3(0.0f, 0.0f, 0.0f);
		double3 nearestPoint2 = double3(0.0f, 0.0f, 0.0f);
		GetNearestPointsBetweenLines3D(mobileLine, fixedLine, nearestPoint1, nearestPoint2);
		double3 displacementNearestPoints = nearestPoint1 - nearestPoint2;
		double distanceNearestPointsSquared = GetLengthSquared(displacementNearestPoints);
		if (distanceNearestPointsSquared < (mobileCapsule.m_radius + fixedCylinder.m_radius) * (mobileCapsule.m_radius + fixedCylinder.m_radius))
		{
			double offset = (mobileCapsule.m_radius + fixedCylinder.m_radius - sqrt(float(distanceNearestPointsSquared))) * 0.5f;
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
				double3 startToEndOfCapsuleDisplacement = mobileCapsule.m_bone.m_end - mobileCapsule.m_bone.m_start;
				double3 capsuleStartToLinePointDisplacement = (nearestPoint1 - mobileCapsule.m_bone.m_end);
				double startToEndOfCapsuleDistance = GetLength(startToEndOfCapsuleDisplacement);
				double capsuleStartToLinePointDistance = GetLength(capsuleStartToLinePointDisplacement);
				double percentToPushStart = capsuleStartToLinePointDistance / startToEndOfCapsuleDistance;
				double percentToPushEnd = (startToEndOfCapsuleDistance - capsuleStartToLinePointDistance) / startToEndOfCapsuleDistance;
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
		double3 nearestPointOnBone = mobileCapsule.m_bone.m_end;
		double3 nearestPointCylinder = GetNearestPointOnCylinderZ3D(nearestPointOnBone, fixedCylinder);
		double3 displacement = nearestPointOnBone - nearestPointCylinder;
		double distanceSquared = GetLengthSquared(displacement);
		if (nearestPointOnBone.x == nearestPointCylinder.x && nearestPointOnBone.y == nearestPointCylinder.y && nearestPointOnBone.z == nearestPointCylinder.z)
		{
			double distanceToTop = fixedCylinder.m_end.z - nearestPointCylinder.z;
			double3 deltaPos = double3(0.0, 0.0, 1.0) * (distanceToTop + mobileCapsule.m_radius);
			mobileCapsule.m_bone.m_end += deltaPos;
			returnValue = true;
		}
		else if (distanceSquared < (mobileCapsule.m_radius * mobileCapsule.m_radius))
		{
			double offset = mobileCapsule.m_radius - sqrt(float(distanceSquared));
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
			double distanceToTop = fixedCylinder.m_end.z - nearestPointCylinder.z;
			double3 deltaPos = double3(0.0, 0.0, 1.0) * (distanceToTop + mobileCapsule.m_radius);
			mobileCapsule.m_bone.m_start += deltaPos;
			returnValue = true;
		}
		else if (distanceSquared < (mobileCapsule.m_radius * mobileCapsule.m_radius))
		{
			double offset = mobileCapsule.m_radius - sqrt(float(distanceSquared));
			displacement = SetLength(displacement, offset);
			mobileCapsule.m_bone.m_start += displacement;
			returnValue = true;
		}

		double3 mobileCapsuleDireciton = mobileCapsule.m_bone.m_end - mobileCapsule.m_bone.m_start;
		double tTop = (topOfCylinder.z - mobileCapsule.m_bone.m_start.z) / mobileCapsuleDireciton.z;
		if (mobileCapsuleDireciton.z != 0.0)
		{
			mobileLine.m_start = mobileCapsule.m_bone.m_start;
			mobileLine.m_end = mobileCapsule.m_bone.m_end;
			double3 intersectPoint = mobileCapsule.m_bone.m_start + tTop * mobileCapsuleDireciton;
			double3 nearestPointCylinder = GetNearestPointOnCylinderZ3D(intersectPoint, fixedCylinder);
			double3 nearestPointOnBone = GetNearestPointOnLineSegment3D(nearestPointCylinder, mobileLine);
			nearestPointCylinder = GetNearestPointOnCylinderZ3D(nearestPointOnBone, fixedCylinder);
			nearestPointOnBone = GetNearestPointOnLineSegment3D(nearestPointCylinder, mobileLine);
			double3 displacement = nearestPointOnBone - nearestPointCylinder;
			double distanceSquared = GetLengthSquared(displacement);
			if (nearestPointOnBone.x == nearestPointCylinder.x && nearestPointOnBone.y == nearestPointCylinder.y && nearestPointOnBone.z == nearestPointCylinder.z)
			{
				double3 cylinderDisplacement = nearestPointCylinder - fixedCylinder.m_end;
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
					double3 startToEndOfCapsuleDisplacement = mobileCapsule.m_bone.m_end - mobileCapsule.m_bone.m_start;
					double3 capsuleStartToLinePointDisplacement = (nearestPointOnBone - mobileCapsule.m_bone.m_end);
					double startToEndOfCapsuleDistance = GetLength(startToEndOfCapsuleDisplacement);
					double capsuleStartToLinePointDistance = GetLength(capsuleStartToLinePointDisplacement);
					double percentToPushStart = capsuleStartToLinePointDistance / startToEndOfCapsuleDistance;
					double percentToPushEnd = (startToEndOfCapsuleDistance - capsuleStartToLinePointDistance) / startToEndOfCapsuleDistance;
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
				double offset = mobileCapsule.m_radius - sqrt(float(distanceSquared));
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
					double3 startToEndOfCapsuleDisplacement = mobileCapsule.m_bone.m_end - mobileCapsule.m_bone.m_start;
					double3 capsuleStartToLinePointDisplacement = (nearestPointOnBone - mobileCapsule.m_bone.m_end);
					double startToEndOfCapsuleDistance = GetLength(startToEndOfCapsuleDisplacement);
					double capsuleStartToLinePointDistance = GetLength(capsuleStartToLinePointDisplacement);
					double percentToPushStart = capsuleStartToLinePointDistance / startToEndOfCapsuleDistance;
					double percentToPushEnd = (startToEndOfCapsuleDistance - capsuleStartToLinePointDistance) / startToEndOfCapsuleDistance;
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
		double3 nearestPointOnBone = mobileCapsule.m_bone.m_end;
		double3 nearestPointCylinder = GetNearestPointOnCylinderZ3D(nearestPointOnBone, fixedCylinder);
		double3 displacement = nearestPointOnBone - nearestPointCylinder;
		double distanceSquared = GetLengthSquared(displacement);
		if (nearestPointOnBone.x == nearestPointCylinder.x && nearestPointOnBone.y == nearestPointCylinder.y && nearestPointOnBone.z == nearestPointCylinder.z)
		{
			double distanceToBottom = nearestPointCylinder.z - fixedCylinder.m_start.z;
			double3 deltaPos = double3(0.0, 0.0, -1.0) * (distanceToBottom + mobileCapsule.m_radius);
			mobileCapsule.m_bone.m_end += deltaPos;
			returnValue = true;
		}
		else if (distanceSquared < (mobileCapsule.m_radius * mobileCapsule.m_radius))
		{
			double offset = mobileCapsule.m_radius - sqrt(float(distanceSquared));
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
			double distanceToBottom = nearestPointCylinder.z - fixedCylinder.m_start.z;
			double3 deltaPos = double3(0.0, 0.0, -1.0) * (distanceToBottom + mobileCapsule.m_radius);
			mobileCapsule.m_bone.m_start += deltaPos;
			returnValue = true;
		}
		else if (distanceSquared < (mobileCapsule.m_radius * mobileCapsule.m_radius))
		{
			double offset = mobileCapsule.m_radius - sqrt(float(distanceSquared));
			displacement = SetLength(displacement, offset);
			mobileCapsule.m_bone.m_start += displacement;
			returnValue = true;
		}

		double3 mobileCapsuleDireciton = mobileCapsule.m_bone.m_end - mobileCapsule.m_bone.m_start;
		double tBottom = (bottomOfCylinder.z - mobileCapsule.m_bone.m_start.z) / mobileCapsuleDireciton.z;
		if (mobileCapsuleDireciton.z != 0.0)
		{
			mobileLine.m_start = mobileCapsule.m_bone.m_start;
			mobileLine.m_end = mobileCapsule.m_bone.m_end;
			double3 intersectPoint = mobileCapsule.m_bone.m_start + tBottom * mobileCapsuleDireciton;
			double3 nearestPointCylinder = GetNearestPointOnCylinderZ3D(intersectPoint, fixedCylinder);
			double3 nearestPointOnBone = GetNearestPointOnLineSegment3D(nearestPointCylinder, mobileLine);
			nearestPointCylinder = GetNearestPointOnCylinderZ3D(nearestPointOnBone, fixedCylinder);
			nearestPointOnBone = GetNearestPointOnLineSegment3D(nearestPointCylinder, mobileLine);
			double3 displacement = nearestPointOnBone - nearestPointCylinder;
			double distanceSquared = GetLengthSquared(displacement);
			if (nearestPointOnBone.x == nearestPointCylinder.x && nearestPointOnBone.y == nearestPointCylinder.y && nearestPointOnBone.z == nearestPointCylinder.z)
			{
				double3 cylinderDisplacement = nearestPointCylinder - fixedCylinder.m_end;
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
					double3 startToEndOfCapsuleDisplacement = mobileCapsule.m_bone.m_end - mobileCapsule.m_bone.m_start;
					double3 capsuleStartToLinePointDisplacement = (nearestPointOnBone - mobileCapsule.m_bone.m_end);
					double startToEndOfCapsuleDistance = GetLength(startToEndOfCapsuleDisplacement);
					double capsuleStartToLinePointDistance = GetLength(capsuleStartToLinePointDisplacement);
					double percentToPushStart = capsuleStartToLinePointDistance / startToEndOfCapsuleDistance;
					double percentToPushEnd = (startToEndOfCapsuleDistance - capsuleStartToLinePointDistance) / startToEndOfCapsuleDistance;
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
				double offset = mobileCapsule.m_radius - sqrt(float(distanceSquared));
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
					double3 startToEndOfCapsuleDisplacement = mobileCapsule.m_bone.m_end - mobileCapsule.m_bone.m_start;
					double3 capsuleStartToLinePointDisplacement = (nearestPointOnBone - mobileCapsule.m_bone.m_end);
					double startToEndOfCapsuleDistance = GetLength(startToEndOfCapsuleDisplacement);
					double capsuleStartToLinePointDistance = GetLength(capsuleStartToLinePointDisplacement);
					double percentToPushStart = capsuleStartToLinePointDistance / startToEndOfCapsuleDistance;
					double percentToPushEnd = (startToEndOfCapsuleDistance - capsuleStartToLinePointDistance) / startToEndOfCapsuleDistance;
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
void PushCapsuleOutOfFixedCylinder3D(Capsule3 mobileCapsule, Cylinder3 fixedCylinder, out double3 returnPosition1, out double3 returnPosition2)
{
	//Transform to local space
	double4x4 modelMatrix;
	modelMatrix[0] = double4(fixedCylinder.m_iBasis, 0.0f);
	modelMatrix[1] = double4(fixedCylinder.m_jBasis, 0.0f);
	modelMatrix[2] = double4(fixedCylinder.m_kBasis, 0.0f);
	modelMatrix[3] = double4(fixedCylinder.m_start, 0.0f);

	double length = GetLength(fixedCylinder.m_end - fixedCylinder.m_start);
	Cylinder3 localCylinder;
	localCylinder.m_radius = fixedCylinder.m_radius;
	localCylinder.m_start = double3(0.0, 0.0, 0.0);
	localCylinder.m_end = localCylinder.m_start + double3(0.0f, 0.0f, length);
	localCylinder.m_iBasis = double3(1.0f, 0.0f, 0.0f);
	localCylinder.m_jBasis = double3(0.0f, 1.0f, 0.0f);
	localCylinder.m_kBasis = double3(0.0f, 0.0f, 1.0f);

	double3 displacementToCapsuleStart = mobileCapsule.m_bone.m_start - fixedCylinder.m_start;
	double projectionIBasis = DotProduct3D(displacementToCapsuleStart, fixedCylinder.m_iBasis);
	double projectionJBasis = DotProduct3D(displacementToCapsuleStart, fixedCylinder.m_jBasis);
	double projectionKBasis = DotProduct3D(displacementToCapsuleStart, fixedCylinder.m_kBasis);
	double3 localCapsuleStart = localCylinder.m_start + localCylinder.m_iBasis * projectionIBasis + localCylinder.m_jBasis * projectionJBasis + localCylinder.m_kBasis * projectionKBasis;
	double3 displacementToCapsuleEnd = mobileCapsule.m_bone.m_end - fixedCylinder.m_start;
	projectionIBasis = DotProduct3D(displacementToCapsuleEnd, fixedCylinder.m_iBasis);
	projectionJBasis = DotProduct3D(displacementToCapsuleEnd, fixedCylinder.m_jBasis);
	projectionKBasis = DotProduct3D(displacementToCapsuleEnd, fixedCylinder.m_kBasis);
	double3 localCapsuleEnd = localCylinder.m_start + localCylinder.m_iBasis * projectionIBasis + localCylinder.m_jBasis * projectionJBasis + localCylinder.m_kBasis * projectionKBasis;

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
		double3 transformedPosition = double3(0.0f, 0.0f, 0.0f);
		transformedPosition.x = (modelMatrix[0].x * returnPosition1.x) + (modelMatrix[1].x * returnPosition1.y) + (modelMatrix[2].x * returnPosition1.z) + modelMatrix[3].x;
		transformedPosition.y = (modelMatrix[0].y * returnPosition1.x) + (modelMatrix[1].y * returnPosition1.y) + (modelMatrix[2].y * returnPosition1.z) + modelMatrix[3].y;
		transformedPosition.z = (modelMatrix[0].z * returnPosition1.x) + (modelMatrix[1].z * returnPosition1.y) + (modelMatrix[2].z * returnPosition1.z) + modelMatrix[3].z;
		returnPosition1 = transformedPosition;
		transformedPosition = double3(0.0f, 0.0f, 0.0f);
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
void PushCapsuleOutOfFixedCapsule3D(Capsule3 mobileCapsule, Capsule3 fixedCapsule, out double3 returnPosition1, out double3 returnPosition2)
{
	LineSegment3 mobileLine;
	mobileLine.m_start = mobileCapsule.m_bone.m_start;
	mobileLine.m_end = mobileCapsule.m_bone.m_end;
	LineSegment3 fixedLine;
	fixedLine.m_start = fixedCapsule.m_bone.m_start;
	fixedLine.m_end = fixedCapsule.m_bone.m_end;
	double3 nearestPoint1 = double3(0.0, 0.0, 0.0);
	double3 nearestPoint2 = double3(0.0, 0.0, 0.0);
	GetNearestPointsBetweenLines3D(mobileLine, fixedLine, nearestPoint1, nearestPoint2);

	double3 displacementNearestPoints = nearestPoint1 - nearestPoint2;
	double distanceNearestPoints = GetLengthSquared(displacementNearestPoints);
	double overallRadii = mobileCapsule.m_radius + fixedCapsule.m_radius;
	if (distanceNearestPoints < overallRadii * overallRadii)
	{
		double offset = overallRadii - sqrt(float(distanceNearestPoints));
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
			double3 startToEndOfCapsuleDisplacement = mobileCapsule.m_bone.m_end - mobileCapsule.m_bone.m_start;
			double3 capsuleStartToLinePointDisplacement = (nearestPoint1 - mobileCapsule.m_bone.m_end);
			double startToEndOfCapsuleDistance = GetLength(startToEndOfCapsuleDisplacement);
			double capsuleStartToLinePointDistance = GetLength(capsuleStartToLinePointDisplacement);
			double percentToPushStart = capsuleStartToLinePointDistance / startToEndOfCapsuleDistance;
			double percentToPushEnd = (startToEndOfCapsuleDistance - capsuleStartToLinePointDistance) / startToEndOfCapsuleDistance;
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
void PushCapsuleOutOfCapsule3D(Capsule3 capsuleA, Capsule3 capsuleB, out double3 returnPosition1, out double3 returnPosition2)
{
	LineSegment3 lineA;
	lineA.m_start = capsuleA.m_bone.m_start;
	lineA.m_end = capsuleA.m_bone.m_end;
	LineSegment3 lineB;
	lineB.m_start = capsuleB.m_bone.m_start;
	lineB.m_end = capsuleB.m_bone.m_end;
	double3 nearestPoint1 = double3(0.0, 0.0, 0.0);
	double3 nearestPoint2 = double3(0.0, 0.0, 0.0);
	GetNearestPointsBetweenLines3D(lineA, lineB, nearestPoint1, nearestPoint2);

	double3 displacementNearestPoints = nearestPoint1 - nearestPoint2;
	double distanceNearestPoints = GetLengthSquared(displacementNearestPoints);
	double overallRadii = capsuleA.m_radius + capsuleB.m_radius;
	if (distanceNearestPoints < overallRadii * overallRadii)
	{
		double offset = (overallRadii - sqrt(float(distanceNearestPoints))) * 0.5f;
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
			double3 startToEndOfCapsuleDisplacement = capsuleA.m_bone.m_end - capsuleA.m_bone.m_start;
			double3 capsuleStartToLinePointDisplacement = (nearestPoint1 - capsuleA.m_bone.m_end);
			double startToEndOfCapsuleDistance = GetLength(startToEndOfCapsuleDisplacement);
			double capsuleStartToLinePointDistance = GetLength(capsuleStartToLinePointDisplacement);
			double percentToPushStart = capsuleStartToLinePointDistance / startToEndOfCapsuleDistance;
			double percentToPushEnd = (startToEndOfCapsuleDistance - capsuleStartToLinePointDistance) / startToEndOfCapsuleDistance;
			capsuleA.m_bone.m_start += displacementNearestPoints * percentToPushStart;
			capsuleA.m_bone.m_end += displacementNearestPoints * percentToPushEnd;
		}
		/*capsuleA.m_bone.m_start += displacementNearestPoints;
		capsuleA.m_bone.m_end += displacementNearestPoints;*/
	}

	returnPosition1 = capsuleA.m_bone.m_start;
	returnPosition2 = capsuleA.m_bone.m_end;
}

//COMPUTE SHADER LOGIC
//------------------------------------------------------------------------------------------------
[numthreads(threadX, threadY, threadZ)]
void CSMain(uint3 threadGroupId : SV_GroupID, uint threadIndex : SV_GroupIndex)
{
	//Initializations
	uint startIndex = threadGroupId.x * threadX;
	uint sentCapsuleIndex = startIndex + (threadIndex);
	if (sentCapsuleIndex >= m_totalParticles - 1)
		return;

	//Self Collision
	int totalCapsules = m_totalParticles - 1;
	for (int capsuleIndex = 0; capsuleIndex < totalCapsules; capsuleIndex++)
	{
		//Early Out Check
		if (capsuleIndex == sentCapsuleIndex || capsuleIndex == sentCapsuleIndex + 1 || capsuleIndex == sentCapsuleIndex - 1 || capsuleIndex == sentCapsuleIndex + 2 || capsuleIndex == sentCapsuleIndex - 2)
			continue;

		//Macro Check
		uint macroLowBitResult = (m_ropeCapsules[sentCapsuleIndex].m_macroBitRegions.m_lowBits & m_ropeCapsules[capsuleIndex].m_macroBitRegions.m_lowBits);
		uint macroHighBitResult = (m_ropeCapsules[sentCapsuleIndex].m_macroBitRegions.m_highBits & m_ropeCapsules[capsuleIndex].m_macroBitRegions.m_highBits);
		if (m_ropeCapsules[sentCapsuleIndex].m_macroBitRegions.m_highBits == 0 && m_ropeCapsules[capsuleIndex].m_macroBitRegions.m_highBits == 0)
		{
			macroHighBitResult = 1;
		}
		if (m_ropeCapsules[sentCapsuleIndex].m_macroBitRegions.m_lowBits == 0 && m_ropeCapsules[capsuleIndex].m_macroBitRegions.m_lowBits == 0)
		{
			macroLowBitResult = 1;
		}
		if (macroLowBitResult != 0 || macroHighBitResult != 0)
		{
			//Micro Check
			uint microLowBitResult = (m_ropeCapsules[sentCapsuleIndex].m_microBitRegions.m_lowBits & m_ropeCapsules[capsuleIndex].m_microBitRegions.m_lowBits);
			uint microHighBitResult = (m_ropeCapsules[sentCapsuleIndex].m_microBitRegions.m_highBits & m_ropeCapsules[capsuleIndex].m_microBitRegions.m_highBits);
			if (m_ropeCapsules[sentCapsuleIndex].m_microBitRegions.m_highBits == 0 && m_ropeCapsules[capsuleIndex].m_microBitRegions.m_highBits == 0)
			{
				microHighBitResult = 1;
			}
			if (m_ropeCapsules[sentCapsuleIndex].m_microBitRegions.m_lowBits == 0 && m_ropeCapsules[capsuleIndex].m_microBitRegions.m_lowBits == 0)
			{
				microLowBitResult = 1;
			}
			if (microLowBitResult != 0 || microHighBitResult != 0)
			{
				//Bounding Disc Check
				if (DoSpheresOverlap(m_ropeCapsules[sentCapsuleIndex].m_boundingDiscCenter, m_ropeCapsules[sentCapsuleIndex].m_boundingDiscRadius, m_ropeCapsules[capsuleIndex].m_boundingDiscCenter, m_ropeCapsules[capsuleIndex].m_boundingDiscRadius))
				{
					//Capsule vs Capsule
					double3 newStartPosition = double3(0.0, 0.0, 0.0);
					double3 newEndPosition = double3(0.0, 0.0, 0.0);
					PushCapsuleOutOfCapsule3D(m_ropeCapsules[sentCapsuleIndex].m_capsule, m_ropeCapsules[capsuleIndex].m_capsule, newStartPosition, newEndPosition);
					if (newStartPosition.x != m_ropeCapsules[sentCapsuleIndex].m_capsule.m_bone.m_start.x ||
						newStartPosition.y != m_ropeCapsules[sentCapsuleIndex].m_capsule.m_bone.m_start.y ||
						newStartPosition.z != m_ropeCapsules[sentCapsuleIndex].m_capsule.m_bone.m_start.z)
					{
						double3 displacementStart = newStartPosition - m_ropeCapsules[sentCapsuleIndex].m_capsule.m_bone.m_start;
						m_ropeCapsules[sentCapsuleIndex].m_isCollidingStart = 1;
						m_ropeCapsules[sentCapsuleIndex].m_collisionNormalStart = GetNormalized(displacementStart);
						m_ropeCapsules[sentCapsuleIndex].m_jacobiCorrectionStart += (displacementStart);
						m_ropeCapsules[sentCapsuleIndex].m_jacobiConstraintTotalStart++;
					}
					if (newEndPosition.x != m_ropeCapsules[sentCapsuleIndex].m_capsule.m_bone.m_end.x ||
						newEndPosition.y != m_ropeCapsules[sentCapsuleIndex].m_capsule.m_bone.m_end.y ||
						newEndPosition.z != m_ropeCapsules[sentCapsuleIndex].m_capsule.m_bone.m_end.z)
					{
						double3 displacementEnd = newEndPosition - m_ropeCapsules[sentCapsuleIndex].m_capsule.m_bone.m_end;
						m_ropeCapsules[sentCapsuleIndex].m_isCollidingEnd = 1;
						m_ropeCapsules[sentCapsuleIndex].m_collisionNormalEnd = GetNormalized(displacementEnd);
						m_ropeCapsules[sentCapsuleIndex].m_jacobiCorrectionEnd += (displacementEnd);
						m_ropeCapsules[sentCapsuleIndex].m_jacobiConstraintTotalEnd++;
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
					double3 newStartPosition = double3(0.0, 0.0, 0.0);
					double3 newEndPosition = double3(0.0, 0.0, 0.0);
					PushCapsuleOutOfFixedAABB3D(m_ropeCapsules[sentCapsuleIndex].m_capsule, m_aabbs[aabbIndex].m_aabb, newStartPosition, newEndPosition);
					if (newStartPosition.x != m_ropeCapsules[sentCapsuleIndex].m_capsule.m_bone.m_start.x ||
						newStartPosition.y != m_ropeCapsules[sentCapsuleIndex].m_capsule.m_bone.m_start.y ||
						newStartPosition.z != m_ropeCapsules[sentCapsuleIndex].m_capsule.m_bone.m_start.z)
					{
						double3 displacementStart = newStartPosition - m_ropeCapsules[sentCapsuleIndex].m_capsule.m_bone.m_start;
						m_ropeCapsules[sentCapsuleIndex].m_isCollidingStart = 1;
						m_ropeCapsules[sentCapsuleIndex].m_collisionNormalStart = GetNormalized(displacementStart);
						m_ropeCapsules[sentCapsuleIndex].m_jacobiCorrectionStart += (displacementStart);
						m_ropeCapsules[sentCapsuleIndex].m_jacobiConstraintTotalStart++;
					}
					if (newEndPosition.x != m_ropeCapsules[sentCapsuleIndex].m_capsule.m_bone.m_end.x ||
						newEndPosition.y != m_ropeCapsules[sentCapsuleIndex].m_capsule.m_bone.m_end.y ||
						newEndPosition.z != m_ropeCapsules[sentCapsuleIndex].m_capsule.m_bone.m_end.z)
					{
						double3 displacementEnd = newEndPosition - m_ropeCapsules[sentCapsuleIndex].m_capsule.m_bone.m_end;
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
					double3 newStartPosition = double3(0.0, 0.0, 0.0);
					double3 newEndPosition = double3(0.0, 0.0, 0.0);
					PushCapsuleOutOfFixedOBB3D(m_ropeCapsules[sentCapsuleIndex].m_capsule, m_obbs[obbIndex].m_obb, newStartPosition, newEndPosition);
					if (newStartPosition.x != m_ropeCapsules[sentCapsuleIndex].m_capsule.m_bone.m_start.x ||
						newStartPosition.y != m_ropeCapsules[sentCapsuleIndex].m_capsule.m_bone.m_start.y ||
						newStartPosition.z != m_ropeCapsules[sentCapsuleIndex].m_capsule.m_bone.m_start.z)
					{
						double3 displacementStart = newStartPosition - m_ropeCapsules[sentCapsuleIndex].m_capsule.m_bone.m_start;
						m_ropeCapsules[sentCapsuleIndex].m_isCollidingStart = 1;
						m_ropeCapsules[sentCapsuleIndex].m_collisionNormalStart = GetNormalized(displacementStart);
						m_ropeCapsules[sentCapsuleIndex].m_jacobiCorrectionStart += (displacementStart);
						m_ropeCapsules[sentCapsuleIndex].m_jacobiConstraintTotalStart++;
					}
					if (newEndPosition.x != m_ropeCapsules[sentCapsuleIndex].m_capsule.m_bone.m_end.x ||
						newEndPosition.y != m_ropeCapsules[sentCapsuleIndex].m_capsule.m_bone.m_end.y ||
						newEndPosition.z != m_ropeCapsules[sentCapsuleIndex].m_capsule.m_bone.m_end.z)
					{
						double3 displacementEnd = newEndPosition - m_ropeCapsules[sentCapsuleIndex].m_capsule.m_bone.m_end;
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
					double3 newStartPosition = double3(0.0, 0.0, 0.0);
					double3 newEndPosition = double3(0.0, 0.0, 0.0);
					PushCapsuleOutOfFixedCylinder3D(m_ropeCapsules[sentCapsuleIndex].m_capsule, m_cylinders[cylinderIndex].m_cylinder, newStartPosition, newEndPosition);
					if (newStartPosition.x != m_ropeCapsules[sentCapsuleIndex].m_capsule.m_bone.m_start.x ||
						newStartPosition.y != m_ropeCapsules[sentCapsuleIndex].m_capsule.m_bone.m_start.y ||
						newStartPosition.z != m_ropeCapsules[sentCapsuleIndex].m_capsule.m_bone.m_start.z)
					{
						double3 displacementStart = newStartPosition - m_ropeCapsules[sentCapsuleIndex].m_capsule.m_bone.m_start;
						m_ropeCapsules[sentCapsuleIndex].m_isCollidingStart = 1;
						m_ropeCapsules[sentCapsuleIndex].m_collisionNormalStart = GetNormalized(displacementStart);
						m_ropeCapsules[sentCapsuleIndex].m_jacobiCorrectionStart += (displacementStart);
						m_ropeCapsules[sentCapsuleIndex].m_jacobiConstraintTotalStart++;
					}
					if (newEndPosition.x != m_ropeCapsules[sentCapsuleIndex].m_capsule.m_bone.m_end.x ||
						newEndPosition.y != m_ropeCapsules[sentCapsuleIndex].m_capsule.m_bone.m_end.y ||
						newEndPosition.z != m_ropeCapsules[sentCapsuleIndex].m_capsule.m_bone.m_end.z)
					{
						double3 displacementEnd = newEndPosition - m_ropeCapsules[sentCapsuleIndex].m_capsule.m_bone.m_end;
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
					double3 newStartPosition = double3(0.0, 0.0, 0.0);
					double3 newEndPosition = double3(0.0, 0.0, 0.0);
					PushCapsuleOutOfFixedCapsule3D(m_ropeCapsules[sentCapsuleIndex].m_capsule, m_capsules[capsuleIndex].m_capsule, newStartPosition, newEndPosition);
					if (newStartPosition.x != m_ropeCapsules[sentCapsuleIndex].m_capsule.m_bone.m_start.x ||
						newStartPosition.y != m_ropeCapsules[sentCapsuleIndex].m_capsule.m_bone.m_start.y ||
						newStartPosition.z != m_ropeCapsules[sentCapsuleIndex].m_capsule.m_bone.m_start.z)
					{
						double3 displacementStart = newStartPosition - m_ropeCapsules[sentCapsuleIndex].m_capsule.m_bone.m_start;
						m_ropeCapsules[sentCapsuleIndex].m_isCollidingStart = 1;
						m_ropeCapsules[sentCapsuleIndex].m_collisionNormalStart = GetNormalized(displacementStart);
						m_ropeCapsules[sentCapsuleIndex].m_jacobiCorrectionStart += (displacementStart);
						m_ropeCapsules[sentCapsuleIndex].m_jacobiConstraintTotalStart++;
					}
					if (newEndPosition.x != m_ropeCapsules[sentCapsuleIndex].m_capsule.m_bone.m_end.x ||
						newEndPosition.y != m_ropeCapsules[sentCapsuleIndex].m_capsule.m_bone.m_end.y ||
						newEndPosition.z != m_ropeCapsules[sentCapsuleIndex].m_capsule.m_bone.m_end.z)
					{
						double3 displacementEnd = newEndPosition - m_ropeCapsules[sentCapsuleIndex].m_capsule.m_bone.m_end;
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
				double3 newStartPosition = double3(0.0, 0.0, 0.0);
				double3 newEndPosition = double3(0.0, 0.0, 0.0);
				PushCapsuleOutOfFixedSphere3D(m_ropeCapsules[sentCapsuleIndex].m_capsule, m_spheres[sphereIndex].m_sphere.m_center, m_spheres[sphereIndex].m_sphere.m_radius, newStartPosition, newEndPosition);
				if (newStartPosition.x != m_ropeCapsules[sentCapsuleIndex].m_capsule.m_bone.m_start.x ||
					newStartPosition.y != m_ropeCapsules[sentCapsuleIndex].m_capsule.m_bone.m_start.y ||
					newStartPosition.z != m_ropeCapsules[sentCapsuleIndex].m_capsule.m_bone.m_start.z)
				{
					double3 displacementStart = newStartPosition - m_ropeCapsules[sentCapsuleIndex].m_capsule.m_bone.m_start;
					m_ropeCapsules[sentCapsuleIndex].m_isCollidingStart = 1;
					m_ropeCapsules[sentCapsuleIndex].m_collisionNormalStart = GetNormalized(displacementStart);
					m_ropeCapsules[sentCapsuleIndex].m_jacobiCorrectionStart += (displacementStart);
					m_ropeCapsules[sentCapsuleIndex].m_jacobiConstraintTotalStart++;
				}
				if (newEndPosition.x != m_ropeCapsules[sentCapsuleIndex].m_capsule.m_bone.m_end.x ||
					newEndPosition.y != m_ropeCapsules[sentCapsuleIndex].m_capsule.m_bone.m_end.y ||
					newEndPosition.z != m_ropeCapsules[sentCapsuleIndex].m_capsule.m_bone.m_end.z)
				{
					double3 displacementEnd = newEndPosition - m_ropeCapsules[sentCapsuleIndex].m_capsule.m_bone.m_end;
					m_ropeCapsules[sentCapsuleIndex].m_isCollidingEnd = 1;
					m_ropeCapsules[sentCapsuleIndex].m_collisionNormalEnd = GetNormalized(displacementEnd);
					m_ropeCapsules[sentCapsuleIndex].m_jacobiCorrectionEnd += (displacementEnd);
					m_ropeCapsules[sentCapsuleIndex].m_jacobiConstraintTotalEnd++;
				}
			}
		}
	}
}
