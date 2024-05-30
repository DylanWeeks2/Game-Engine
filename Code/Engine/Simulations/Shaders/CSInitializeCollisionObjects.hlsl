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
RWStructuredBuffer<AABBCollisionObject>			m_aabbs:register(u1);
RWStructuredBuffer<OBBCollisionObject>			m_obbs:register(u2);
RWStructuredBuffer<CylinderCollisionObject>		m_cylinders:register(u3);
RWStructuredBuffer<CapsuleCollisionObject>		m_capsules:register(u4);
RWStructuredBuffer<SphereCollisionObject>		m_spheres:register(u5);

//MATH FUNCITONS
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

//COMPUTE SHADER LOGIC
//-----------------------------------------------------------------------------------------------
int GetRegionIndexForCoords(int3 coords)
{
	return coords.x | (coords.y << 3);
}

//-----------------------------------------------------------------------------------------------
void AssignBitRegionsCollisionObject(int objectIndex, int currentRegion, int regionDifferenceX, int regionDifferenceY, bool isMicro)
{
	uint64_t mask;
	mask.m_lowBits = 0;
	mask.m_highBits = 0;
	if (isMicro == false)
	{
		for (int regionIndexY = 0; regionIndexY <= regionDifferenceY; regionIndexY++)
		{
			for (int regionIndexX = 0; regionIndexX <= regionDifferenceX; regionIndexX++)
			{
				int bit = currentRegion + regionIndexX;
				if (bit < 32)
				{
					mask.m_lowBits = 1u << bit;
					//AABB
					if (objectIndex < m_totalAABBs)
					{
						m_aabbs[objectIndex].m_macroBitRegions.m_lowBits |= mask.m_lowBits;
					}
					//OBB
					else if (objectIndex < m_totalAABBs + m_totalOBBs)
					{
						m_obbs[objectIndex - m_totalAABBs].m_macroBitRegions.m_lowBits |= mask.m_lowBits;
					}
					//Cylinders
					else if (objectIndex < m_totalAABBs + m_totalOBBs + m_totalCylinders)
					{
						m_cylinders[objectIndex - (m_totalAABBs + m_totalOBBs)].m_macroBitRegions.m_lowBits |= mask.m_lowBits;
					}
					//Capsules
					else if (objectIndex < m_totalAABBs + m_totalOBBs + m_totalCylinders + m_totalCapsules)
					{
						m_capsules[objectIndex - (m_totalAABBs + m_totalOBBs + m_totalCylinders)].m_macroBitRegions.m_lowBits |= mask.m_lowBits;
					}
					//Spheres
					else if (objectIndex < m_totalAABBs + m_totalOBBs + m_totalCylinders + m_totalCapsules + m_totalSpheres)
					{
						m_spheres[objectIndex - (m_totalAABBs + m_totalOBBs + m_totalCylinders + m_totalCapsules)].m_macroBitRegions.m_lowBits |= mask.m_lowBits;
					}
				}
				else
				{
					bit -= 32;
					mask.m_highBits = 1u << bit;
					//AABB
					if (objectIndex < m_totalAABBs)
					{
						m_aabbs[objectIndex].m_macroBitRegions.m_highBits |= mask.m_highBits;
					}
					//OBB
					else if (objectIndex < m_totalAABBs + m_totalOBBs)
					{
						m_obbs[objectIndex - m_totalAABBs].m_macroBitRegions.m_highBits |= mask.m_highBits;
					}
					//Cylinders
					else if (objectIndex < m_totalAABBs + m_totalOBBs + m_totalCylinders)
					{
						m_cylinders[objectIndex - (m_totalAABBs + m_totalOBBs)].m_macroBitRegions.m_highBits |= mask.m_highBits;
					}
					//Capsules
					else if (objectIndex < m_totalAABBs + m_totalOBBs + m_totalCylinders + m_totalCapsules)
					{
						m_capsules[objectIndex - (m_totalAABBs + m_totalOBBs + m_totalCylinders)].m_macroBitRegions.m_highBits |= mask.m_highBits;
					}
					//Spheres
					else if (objectIndex < m_totalAABBs + m_totalOBBs + m_totalCylinders + m_totalCapsules + m_totalSpheres)
					{
						m_spheres[objectIndex - (m_totalAABBs + m_totalOBBs + m_totalCylinders + m_totalCapsules)].m_macroBitRegions.m_highBits |= mask.m_highBits;
					}
				}
			}
			currentRegion += 8;
		}
	}
	else
	{
		for (int regionIndexY = 0; regionIndexY <= regionDifferenceY; regionIndexY++)
		{
			for (int regionIndexX = 0; regionIndexX <= regionDifferenceX; regionIndexX++)
			{
				int bit = currentRegion + regionIndexX;
				if (bit < 32)
				{
					mask.m_lowBits = 1u << bit;
					//AABB
					if (objectIndex < m_totalAABBs)
					{
						m_aabbs[objectIndex].m_microBitRegions.m_lowBits |= mask.m_lowBits;
					}
					//OBB
					else if (objectIndex < m_totalAABBs + m_totalOBBs)
					{
						m_obbs[objectIndex - m_totalAABBs].m_microBitRegions.m_lowBits |= mask.m_lowBits;
					}
					//Cylinders
					else if (objectIndex < m_totalAABBs + m_totalOBBs + m_totalCylinders)
					{
						m_cylinders[objectIndex - (m_totalAABBs + m_totalOBBs)].m_microBitRegions.m_lowBits |= mask.m_lowBits;
					}
					//Capsules
					else if (objectIndex < m_totalAABBs + m_totalOBBs + m_totalCylinders + m_totalCapsules)
					{
						m_capsules[objectIndex - (m_totalAABBs + m_totalOBBs + m_totalCylinders)].m_microBitRegions.m_lowBits |= mask.m_lowBits;
					}
					//Spheres
					else if (objectIndex < m_totalAABBs + m_totalOBBs + m_totalCylinders + m_totalCapsules + m_totalSpheres)
					{
						m_spheres[objectIndex - (m_totalAABBs + m_totalOBBs + m_totalCylinders + m_totalCapsules)].m_microBitRegions.m_lowBits |= mask.m_lowBits;
					}
				}
				else
				{
					bit -= 32;
					mask.m_highBits = 1u << bit;
					//AABB
					if (objectIndex < m_totalAABBs)
					{
						m_aabbs[objectIndex].m_microBitRegions.m_highBits |= mask.m_highBits;
					}
					//OBB
					else if (objectIndex < m_totalAABBs + m_totalOBBs)
					{
						m_obbs[objectIndex - m_totalAABBs].m_microBitRegions.m_highBits |= mask.m_highBits;
					}
					//Cylinders
					else if (objectIndex < m_totalAABBs + m_totalOBBs + m_totalCylinders)
					{
						m_cylinders[objectIndex - (m_totalAABBs + m_totalOBBs)].m_microBitRegions.m_highBits |= mask.m_highBits;
					}
					//Capsules
					else if (objectIndex < m_totalAABBs + m_totalOBBs + m_totalCylinders + m_totalCapsules)
					{
						m_capsules[objectIndex - (m_totalAABBs + m_totalOBBs + m_totalCylinders)].m_microBitRegions.m_highBits |= mask.m_highBits;
					}
					//Spheres
					else if (objectIndex < m_totalAABBs + m_totalOBBs + m_totalCylinders + m_totalCapsules + m_totalSpheres)
					{
						m_spheres[objectIndex - (m_totalAABBs + m_totalOBBs + m_totalCylinders + m_totalCapsules)].m_microBitRegions.m_highBits |= mask.m_highBits;
					}
				}
			}
			currentRegion += 8;
		}
	}
}

//------------------------------------------------------------------------------------------------
[numthreads(1, 1, 1)]
void CSMain(uint3 dispatchThreadId : SV_DispatchThreadID)
{
	//Initializations
	uint objectIndex = dispatchThreadId.x;
	float m_worldScaleX = abs(m_worldBounds.m_maxs.x - m_worldBounds.m_mins.x) * (1.0f / 8.0f);
	float m_worldScaleY = abs(m_worldBounds.m_maxs.y - m_worldBounds.m_mins.y) * (1.0f / 8.0f);
	float m_macroScaleX = 1.0f / m_worldScaleX;
	float m_macroScaleY = 1.0f / m_worldScaleY;
	float m_microScaleX = 8.0f;
	float m_microScaleY = 8.0f;
	float minX = 0.0f;
	float minY = 0.0f;
	float maxX = 0.0f;
	float maxY = 0.0f;

	//AABB
	if (objectIndex < m_totalAABBs)
	{
		if (m_aabbs[objectIndex].m_aabb.m_mins.x < m_aabbs[objectIndex].m_aabb.m_maxs.x)
			minX = m_aabbs[objectIndex].m_aabb.m_mins.x * m_macroScaleX;
		else
			minX = m_aabbs[objectIndex].m_aabb.m_maxs.x * m_macroScaleX;
		if (m_aabbs[objectIndex].m_aabb.m_mins.y < m_aabbs[objectIndex].m_aabb.m_maxs.y)
			minY = m_aabbs[objectIndex].m_aabb.m_mins.y * m_macroScaleY;
		else
			minY = m_aabbs[objectIndex].m_aabb.m_maxs.y * m_macroScaleY;
		if (m_aabbs[objectIndex].m_aabb.m_mins.x > m_aabbs[objectIndex].m_aabb.m_maxs.x)
			maxX = m_aabbs[objectIndex].m_aabb.m_mins.x * m_macroScaleX;
		else
			maxX = m_aabbs[objectIndex].m_aabb.m_maxs.x * m_macroScaleX;
		if (m_aabbs[objectIndex].m_aabb.m_mins.y > m_aabbs[objectIndex].m_aabb.m_maxs.y)
			maxY = m_aabbs[objectIndex].m_aabb.m_mins.y * m_macroScaleY;
		else
			maxY = m_aabbs[objectIndex].m_aabb.m_maxs.y * m_macroScaleY;
	}
	//OBBs
	else if (objectIndex < m_totalAABBs + m_totalOBBs)
	{
		float3 center = m_obbs[m_totalAABBs - objectIndex].m_obb.m_center;
		float3 ibasis = m_obbs[m_totalAABBs - objectIndex].m_obb.m_iBasisNormal;
		float3 jbasis = m_obbs[m_totalAABBs - objectIndex].m_obb.m_jBasisNormal;
		float3 kbasis = m_obbs[m_totalAABBs - objectIndex].m_obb.m_kBasisNormal;
		float x = m_obbs[m_totalAABBs - objectIndex].m_obb.m_halfDimensions.x;
		float y = m_obbs[m_totalAABBs - objectIndex].m_obb.m_halfDimensions.y;
		float z = m_obbs[m_totalAABBs - objectIndex].m_obb.m_halfDimensions.z;
		float3 corners[8];
		corners[0] = ((ibasis * x + jbasis * y + kbasis * z) + center);
		corners[1] = ((ibasis * x + jbasis * -y + kbasis * z) + center);
		corners[2] = ((ibasis * x + jbasis * y + kbasis * -z) + center);
		corners[3] = ((ibasis * x + jbasis * -y + kbasis * -z) + center);
		corners[4] = ((ibasis * -x + jbasis * y + kbasis * z) + center);
		corners[5] = ((ibasis * -x + jbasis * -y + kbasis * z) + center);
		corners[6] = ((ibasis * -x + jbasis * y + kbasis * -z) + center);
		corners[7] = ((ibasis * -x + jbasis * -y + kbasis * -z) + center);

		minX = 10000000.0f; minY = 10000000.0f; maxX = 0.0f; maxY = 0.0f;
		for (int index = 0; index < 8; index++)
		{
			if (corners[index].x < minX)
				minX = corners[index].x;
			if (corners[index].x > maxX)
				maxX = corners[index].x;
			if (corners[index].y < minY)
				minY = corners[index].y;
			if (corners[index].y > maxY)
				maxY = corners[index].y;
		}
		minX *= m_macroScaleX;
		maxX *= m_macroScaleX;
		minY *= m_macroScaleY;
		maxY *= m_macroScaleY;
	}
	//Cylinders
	else if (objectIndex < m_totalAABBs + m_totalOBBs + m_totalCylinders)
	{
		float radius = m_cylinders[(m_totalAABBs + m_totalOBBs) - objectIndex].m_cylinder.m_radius;
		float3 start = m_cylinders[(m_totalAABBs + m_totalOBBs) - objectIndex].m_cylinder.m_start;
		float3 end = m_cylinders[(m_totalAABBs + m_totalOBBs) - objectIndex].m_cylinder.m_end;
		float minX1 = (start.x - radius) * m_macroScaleX;
		float minY1 = (start.y - radius) * m_macroScaleY;
		float maxX1 = (start.x + radius) * m_macroScaleX;
		float maxY1 = (start.y + radius) * m_macroScaleY;
		float minX2 = (end.x - radius) * m_macroScaleX;
		float minY2 = (end.y - radius) * m_macroScaleY;
		float maxX2 = (end.x + radius) * m_macroScaleX;
		float maxY2 = (end.y + radius) * m_macroScaleY;

		if (minX1 < minX2)
			minX = minX1;
		else
			minX = minX2;
		if (minY1 < minY2)
			minY = minY1;
		else
			minY = minY2;
		if (maxX1 > maxX2)
			maxX = maxX1;
		else
			maxX = maxX2;
		if (maxY1 > maxY2)
			maxY = maxY1;
		else
			maxY = maxY2;
	}
	//Capsules
	else if (objectIndex < m_totalAABBs + m_totalOBBs + m_totalCylinders + m_totalCapsules)
	{
		float radius = m_capsules[(m_totalAABBs + m_totalOBBs + m_totalCylinders) - objectIndex].m_capsule.m_radius;
		float3 start = m_capsules[(m_totalAABBs + m_totalOBBs + m_totalCylinders) - objectIndex].m_capsule.m_bone.m_start;
		float3 end = m_capsules[(m_totalAABBs + m_totalOBBs + m_totalCylinders) - objectIndex].m_capsule.m_bone.m_end;
		float minX1 = (start.x - radius) * m_macroScaleX;
		float minY1 = (start.y - radius) * m_macroScaleY;
		float maxX1 = (start.x + radius) * m_macroScaleX;
		float maxY1 = (start.y + radius) * m_macroScaleY;
		float minX2 = (end.x - radius) * m_macroScaleX;
		float minY2 = (end.y - radius) * m_macroScaleY;
		float maxX2 = (end.x + radius) * m_macroScaleX;
		float maxY2 = (end.y + radius) * m_macroScaleY;

		if (minX1 < minX2)
			minX = minX1;
		else
			minX = minX2;
		if (minY1 < minY2)
			minY = minY1;
		else
			minY = minY2;
		if (maxX1 > maxX2)
			maxX = maxX1;
		else
			maxX = maxX2;
		if (maxY1 > maxY2)
			maxY = maxY1;
		else
			maxY = maxY2;
	}
	else if (objectIndex < m_totalAABBs + m_totalOBBs + m_totalCylinders + m_totalCapsules + m_totalSpheres)
	{
		SphereCollisionObject currentSphere = m_spheres[(m_totalAABBs + m_totalOBBs + m_totalCylinders + m_totalCapsules) - objectIndex];
		minX = (currentSphere.m_sphere.m_center.x - currentSphere.m_sphere.m_radius) * m_macroScaleX;
		minY = (currentSphere.m_sphere.m_center.y - currentSphere.m_sphere.m_radius) * m_macroScaleY;
		maxX = (currentSphere.m_sphere.m_center.x + currentSphere.m_sphere.m_radius) * m_macroScaleX;
		maxY = (currentSphere.m_sphere.m_center.y + currentSphere.m_sphere.m_radius) * m_macroScaleY;
	}

	minX = GetClamped(minX, 0.0f, 7.0f);
	maxX = GetClamped(maxX, 0.0f, 7.0f);
	minY = GetClamped(minY, 0.0f, 8.0f);
	maxY = GetClamped(maxY, 0.0f, 8.0f);
	uint64_t macroBitRegions;
	macroBitRegions.m_lowBits = 0;
	macroBitRegions.m_highBits = 0;
	uint64_t microBitRegions;
	microBitRegions.m_lowBits = 0;
	microBitRegions.m_highBits = 0;

	//Macro
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
	AssignBitRegionsCollisionObject(objectIndex, currentRegion, regionDifferenceX, regionDifferenceY, false);

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
		AssignBitRegionsCollisionObject(objectIndex, currentRegion, regionDifferenceX, regionDifferenceY, true);
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
			AssignBitRegionsCollisionObject(objectIndex, currentRegion, regionDifferenceX, regionDifferenceY, true);
		}
		//Span across 1 Y macro region
		else
		{
			//region 1/3 to top CORRECT
			regionDifferenceX = region3Micro - region1Micro;
			regionDifferenceY = 7 - (region1Micro >> 3);
			currentRegion = region1Micro;
			AssignBitRegionsCollisionObject(objectIndex, currentRegion, regionDifferenceX, regionDifferenceY, true);
			//bottom to region 2/4 CORRECT
			regionDifferenceX = region4Micro - region2Micro;
			regionDifferenceY = (region2Micro >> 3);
			currentRegion = region2Micro - (region2Micro >> 3) * 8;
			AssignBitRegionsCollisionObject(objectIndex, currentRegion, regionDifferenceX, regionDifferenceY, true);
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
			AssignBitRegionsCollisionObject(objectIndex, currentRegion, regionDifferenceX, regionDifferenceY, true);
		}
		//Span across 1 X macro region
		else
		{
			//region 1/2 to right CORRECT
			regionDifferenceX = ((region1Micro >> 3) * 8 + 7) - region1Micro;
			regionDifferenceY = region2Micro - region1Micro;
			currentRegion = region1Micro;
			AssignBitRegionsCollisionObject(objectIndex, currentRegion, regionDifferenceX, regionDifferenceY, true);
			//left to region 3/4 CORRECT
			currentRegion = (region3Micro >> 3) * 8;
			regionDifferenceX = region3Micro - currentRegion;
			regionDifferenceY = region2Micro - region1Micro;
			AssignBitRegionsCollisionObject(objectIndex, currentRegion, regionDifferenceX, regionDifferenceY, true);
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
			m_aabbs[objectIndex].m_microBitRegions = mask;
		}
		//Spans across 1 Y and multiple X regions
		else if (region2 - region1 == 8 && region3 - region1 > 1)
		{
			//region 1/3 to top CORRECT
			regionDifferenceX = 7;
			regionDifferenceY = 7 - (region1Micro >> 3);
			currentRegion = (region1Micro >> 3) * 8;
			AssignBitRegionsCollisionObject(objectIndex, currentRegion, regionDifferenceX, regionDifferenceY, true);
			//bottom to region 2/4 CORRECT
			regionDifferenceX = 7;
			regionDifferenceY = (region2Micro >> 3);
			currentRegion = 0;
			AssignBitRegionsCollisionObject(objectIndex, currentRegion, regionDifferenceX, regionDifferenceY, true);
		}
		//Spans across 1 X and multiple Y regions
		else if (region2 - region1 > 8 && region3 - region1 == 1)
		{
			//region 1/2 to right CORRECT		
			regionDifferenceX = ((region1Micro >> 3) * 8 + 7) - region1Micro;
			regionDifferenceY = 7;
			currentRegion = region1Micro - (region1Micro >> 3) * 8;
			AssignBitRegionsCollisionObject(objectIndex, currentRegion, regionDifferenceX, regionDifferenceY, true);
			//left to region 3/4 					
			currentRegion = 0;
			regionDifferenceX = region3Micro - (region3Micro >> 3) * 8;
			regionDifferenceY = 7;
			AssignBitRegionsCollisionObject(objectIndex, currentRegion, regionDifferenceX, regionDifferenceY, true);
		}
		//Spans across 1 X and 1 Y region
		else if (region2 - region1 == 8 && region3 - region1 == 1)
		{
			//region1 to right/top
			regionDifferenceX = ((region1Micro >> 3) * 8 + 7) - region1Micro;
			regionDifferenceY = 7 - (region1Micro >> 3);
			currentRegion = region1Micro;
			AssignBitRegionsCollisionObject(objectIndex, currentRegion, regionDifferenceX, regionDifferenceY, true);
			//left/top to region 3
			regionDifferenceX = region3Micro - (region3Micro >> 3) * 8;
			regionDifferenceY = 7 - (region3Micro >> 3);
			currentRegion = (region3Micro >> 3) * 8;
			AssignBitRegionsCollisionObject(objectIndex, currentRegion, regionDifferenceX, regionDifferenceY, true);
			//bottom/right to region 2
			regionDifferenceX = ((region2Micro >> 3) * 8 + 7) - region2Micro;
			regionDifferenceY = (region2Micro >> 3);
			currentRegion = region2Micro - (region2Micro >> 3) * 8;
			AssignBitRegionsCollisionObject(objectIndex, currentRegion, regionDifferenceX, regionDifferenceY, true);
			//bottom/left to region 4
			regionDifferenceX = region4Micro - (region4Micro >> 3) * 8;
			regionDifferenceY = (region4Micro >> 3);
			currentRegion = 0;
			AssignBitRegionsCollisionObject(objectIndex, currentRegion, regionDifferenceX, regionDifferenceY, true);
		}
	}
}
