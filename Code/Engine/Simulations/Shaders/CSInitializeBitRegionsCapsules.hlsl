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
	float3 m_mins;
	float3 m_maxs;
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
RWStructuredBuffer<CapsuleCollisionObject>	m_ropeCapsules:register(u7);

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

//COMPUTE SHADER LOGIC
//-----------------------------------------------------------------------------------------------
int GetRegionIndexForCoords(int3 coords)
{
	return coords.x | (coords.y << 3);
}

//-----------------------------------------------------------------------------------------------
void AssignBitRegionsCapsule(int sentCapsuleIndex, int currentRegion, int regionDifferenceX, int regionDifferenceY, bool isMicro)
{
	uint64_t mask;
	mask.m_lowBits = 0;
	mask.m_highBits = 0;
	if (isMicro == false)
	{
		m_ropeCapsules[sentCapsuleIndex].m_macroBitRegions.m_lowBits = 0;
		m_ropeCapsules[sentCapsuleIndex].m_macroBitRegions.m_highBits = 0;
		for (int regionIndexY = 0; regionIndexY <= regionDifferenceY; regionIndexY++)
		{
			for (int regionIndexX = 0; regionIndexX <= regionDifferenceX; regionIndexX++)
			{
				int bit = currentRegion + regionIndexX;
				if (bit < 32)
				{
					mask.m_lowBits = 1u << bit;
					m_ropeCapsules[sentCapsuleIndex].m_macroBitRegions.m_lowBits |= mask.m_lowBits;
				}
				else
				{
					bit -= 32;
					mask.m_highBits = 1u << bit;
					m_ropeCapsules[sentCapsuleIndex].m_macroBitRegions.m_highBits |= mask.m_highBits;
				}
			}
			currentRegion += 8;
		}
	}
	else
	{
		m_ropeCapsules[sentCapsuleIndex].m_microBitRegions.m_lowBits = 0;
		m_ropeCapsules[sentCapsuleIndex].m_microBitRegions.m_highBits = 0;
		for (int regionIndexY = 0; regionIndexY <= regionDifferenceY; regionIndexY++)
		{
			for (int regionIndexX = 0; regionIndexX <= regionDifferenceX; regionIndexX++)
			{
				int bit = currentRegion + regionIndexX;
				if (bit < 32)
				{
					mask.m_lowBits = 1u << bit;
					m_ropeCapsules[sentCapsuleIndex].m_microBitRegions.m_lowBits |= mask.m_lowBits;
				}
				else
				{
					bit -= 32;
					mask.m_highBits = 1u << bit;
					m_ropeCapsules[sentCapsuleIndex].m_microBitRegions.m_highBits |= mask.m_highBits;
				}
			}
			currentRegion += 8;
		}
	}
}

//-----------------------------------------------------------------------------------------------
void BitRegionDetectionSingleCapsule(int sentCapsuleIndex)
{
	//Initializations
	m_ropeCapsules[sentCapsuleIndex].m_macroBitRegions.m_lowBits = 0;
	m_ropeCapsules[sentCapsuleIndex].m_macroBitRegions.m_highBits = 0;
	m_ropeCapsules[sentCapsuleIndex].m_microBitRegions.m_lowBits = 0;
	m_ropeCapsules[sentCapsuleIndex].m_microBitRegions.m_highBits = 0;
	float m_bitRegionScale = 1.0f / 8.0f;
	float m_worldScaleX = abs(m_worldBounds.m_maxs.x - m_worldBounds.m_mins.x) * m_bitRegionScale;
	float m_worldScaleY = abs(m_worldBounds.m_maxs.y - m_worldBounds.m_mins.y) * m_bitRegionScale;
	float m_macroScaleX = 1.0f / m_worldScaleX;
	float m_macroScaleY = 1.0f / m_worldScaleY;
	float m_microScaleX = 8.0f;
	float m_microScaleY = 8.0f;

	//Macro
	float minX = 1000000000.0; float minY = 1000000000.0; float maxX = 0.0f; float maxY = 0.0f;
	float radius = m_ropeCapsules[sentCapsuleIndex].m_capsule.m_radius;
	float3 start = m_ropeCapsules[sentCapsuleIndex].m_capsule.m_bone.m_start;
	float3 end = m_ropeCapsules[sentCapsuleIndex].m_capsule.m_bone.m_end;
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
	AssignBitRegionsCapsule(sentCapsuleIndex, currentRegion, regionDifferenceX, regionDifferenceY, false);

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
		AssignBitRegionsCapsule(sentCapsuleIndex, currentRegion, regionDifferenceX, regionDifferenceY, true);
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
			AssignBitRegionsCapsule(sentCapsuleIndex, currentRegion, regionDifferenceX, regionDifferenceY, true);
		}
		//Span across 1 Y macro region
		else
		{
			//region 1/3 to top CORRECT
			regionDifferenceX = region3Micro - region1Micro;
			regionDifferenceY = 7 - (region1Micro >> 3);
			currentRegion = region1Micro;
			AssignBitRegionsCapsule(sentCapsuleIndex, currentRegion, regionDifferenceX, regionDifferenceY, true);
			//bottom to region 2/4 CORRECT
			regionDifferenceX = region4Micro - region2Micro;
			regionDifferenceY = (region2Micro >> 3);
			currentRegion = region2Micro - (region2Micro >> 3) * 8;
			AssignBitRegionsCapsule(sentCapsuleIndex, currentRegion, regionDifferenceX, regionDifferenceY, true);
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
			AssignBitRegionsCapsule(sentCapsuleIndex, currentRegion, regionDifferenceX, regionDifferenceY, true);
		}
		//Span across 1 X macro region
		else
		{
			//region 1/2 to right CORRECT
			regionDifferenceX = ((region1Micro >> 3) * 8 + 7) - region1Micro;
			regionDifferenceY = region2Micro - region1Micro;
			currentRegion = region1Micro;
			AssignBitRegionsCapsule(sentCapsuleIndex, currentRegion, regionDifferenceX, regionDifferenceY, true);
			//left to region 3/4 CORRECT
			currentRegion = (region3Micro >> 3) * 8;
			regionDifferenceX = region3Micro - currentRegion;
			regionDifferenceY = region2Micro - region1Micro;
			AssignBitRegionsCapsule(sentCapsuleIndex, currentRegion, regionDifferenceX, regionDifferenceY, true);
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
			m_ropeCapsules[sentCapsuleIndex].m_microBitRegions = mask;
		}
		//Spans across 1 Y and multiple X regions
		else if (region2 - region1 == 8 && region3 - region1 > 1)
		{
			//region 1/3 to top CORRECT
			regionDifferenceX = 7;
			regionDifferenceY = 7 - (region1Micro >> 3);
			currentRegion = (region1Micro >> 3) * 8;
			AssignBitRegionsCapsule(sentCapsuleIndex, currentRegion, regionDifferenceX, regionDifferenceY, true);
			//bottom to region 2/4 CORRECT
			regionDifferenceX = 7;
			regionDifferenceY = (region2Micro >> 3);
			currentRegion = 0;
			AssignBitRegionsCapsule(sentCapsuleIndex, currentRegion, regionDifferenceX, regionDifferenceY, true);
		}
		//Spans across 1 X and multiple Y regions
		else if (region2 - region1 > 8 && region3 - region1 == 1)
		{
			//region 1/2 to right CORRECT		
			regionDifferenceX = ((region1Micro >> 3) * 8 + 7) - region1Micro;
			regionDifferenceY = 7;
			currentRegion = region1Micro - (region1Micro >> 3) * 8;
			AssignBitRegionsCapsule(sentCapsuleIndex, currentRegion, regionDifferenceX, regionDifferenceY, true);
			//left to region 3/4 					
			currentRegion = 0;
			regionDifferenceX = region3Micro - (region3Micro >> 3) * 8;
			regionDifferenceY = 7;
			AssignBitRegionsCapsule(sentCapsuleIndex, currentRegion, regionDifferenceX, regionDifferenceY, true);
		}
		//Spans across 1 X and 1 Y region
		else if (region2 - region1 == 8 && region3 - region1 == 1)
		{
			//region1 to right/top
			regionDifferenceX = ((region1Micro >> 3) * 8 + 7) - region1Micro;
			regionDifferenceY = 7 - (region1Micro >> 3);
			currentRegion = region1Micro;
			AssignBitRegionsCapsule(sentCapsuleIndex, currentRegion, regionDifferenceX, regionDifferenceY, true);
			//left/top to region 3
			regionDifferenceX = region3Micro - (region3Micro >> 3) * 8;
			regionDifferenceY = 7 - (region3Micro >> 3);
			currentRegion = (region3Micro >> 3) * 8;
			AssignBitRegionsCapsule(sentCapsuleIndex, currentRegion, regionDifferenceX, regionDifferenceY, true);
			//bottom/right to region 2
			regionDifferenceX = ((region2Micro >> 3) * 8 + 7) - region2Micro;
			regionDifferenceY = (region2Micro >> 3);
			currentRegion = region2Micro - (region2Micro >> 3) * 8;
			AssignBitRegionsCapsule(sentCapsuleIndex, currentRegion, regionDifferenceX, regionDifferenceY, true);
			//bottom/left to region 4
			regionDifferenceX = region4Micro - (region4Micro >> 3) * 8;
			regionDifferenceY = (region4Micro >> 3);
			currentRegion = 0;
			AssignBitRegionsCapsule(sentCapsuleIndex, currentRegion, regionDifferenceX, regionDifferenceY, true);
		}
	}
}

//------------------------------------------------------------------------------------------------
[numthreads(threadX, threadY, threadZ)]
void CSMain(uint3 threadGroupId : SV_GroupID, uint threadIndex : SV_GroupIndex)
{
	//Initializations
	uint startIndex = threadGroupId.x * threadX;
	uint sentParticleIndex = startIndex + (threadIndex);
	if (sentParticleIndex >= m_totalParticles)
		return;
	bool checkPreviousCapsule = false;
	bool checkNextCapsule = false;

	//Update bit regions
	if (sentParticleIndex != 0)
	{
		m_ropeCapsules[sentParticleIndex - 1].m_capsule.m_bone.m_start = m_particleProposedPositions[sentParticleIndex - 1];
		m_ropeCapsules[sentParticleIndex - 1].m_capsule.m_bone.m_end = m_particleProposedPositions[sentParticleIndex];
		m_ropeCapsules[sentParticleIndex - 1].m_boundingDiscCenter = (m_ropeCapsules[sentParticleIndex - 1].m_capsule.m_bone.m_start + m_ropeCapsules[sentParticleIndex - 1].m_capsule.m_bone.m_end) * 0.5f;
		m_ropeCapsules[sentParticleIndex - 1].m_boundingDiscRadius = GetLength(m_ropeCapsules[sentParticleIndex - 1].m_capsule.m_bone.m_start - m_ropeCapsules[sentParticleIndex - 1].m_boundingDiscCenter) + m_ropeRadius;
		m_ropeCapsules[sentParticleIndex - 1].m_capsule.m_radius = m_ropeRadius;
		m_ropeCapsules[sentParticleIndex - 1].m_capsule.m_bone.m_radius = m_ropeRadius;
		m_ropeCapsules[sentParticleIndex - 1].m_isCollidingStart = 0;
		m_ropeCapsules[sentParticleIndex - 1].m_collisionNormalStart = float3(0.0f, 0.0f, 0.0f);
		m_ropeCapsules[sentParticleIndex - 1].m_jacobiCorrectionStart = float3(0.0f, 0.0f, 0.0f);
		m_ropeCapsules[sentParticleIndex - 1].m_jacobiConstraintTotalStart = 0;
		m_ropeCapsules[sentParticleIndex - 1].m_isCollidingEnd = 0;
		m_ropeCapsules[sentParticleIndex - 1].m_collisionNormalEnd = float3(0.0f, 0.0f, 0.0f);
		m_ropeCapsules[sentParticleIndex - 1].m_jacobiCorrectionEnd = float3(0.0f, 0.0f, 0.0f);
		m_ropeCapsules[sentParticleIndex - 1].m_jacobiConstraintTotalEnd = 0;
		BitRegionDetectionSingleCapsule(sentParticleIndex - 1);
	}
	if (sentParticleIndex != m_totalParticles - 1)
	{
		m_ropeCapsules[sentParticleIndex].m_capsule.m_bone.m_start = m_particleProposedPositions[sentParticleIndex];
		m_ropeCapsules[sentParticleIndex].m_capsule.m_bone.m_end = m_particleProposedPositions[sentParticleIndex + 1];
		m_ropeCapsules[sentParticleIndex].m_boundingDiscCenter = (m_ropeCapsules[sentParticleIndex].m_capsule.m_bone.m_start + m_ropeCapsules[sentParticleIndex].m_capsule.m_bone.m_end) * 0.5f;
		m_ropeCapsules[sentParticleIndex].m_boundingDiscRadius = GetLength(m_ropeCapsules[sentParticleIndex].m_capsule.m_bone.m_start - m_ropeCapsules[sentParticleIndex].m_boundingDiscCenter) + m_ropeRadius;
		m_ropeCapsules[sentParticleIndex].m_capsule.m_radius = m_ropeRadius;
		m_ropeCapsules[sentParticleIndex].m_capsule.m_bone.m_radius = m_ropeRadius;
		m_ropeCapsules[sentParticleIndex].m_isCollidingStart = 0;
		m_ropeCapsules[sentParticleIndex].m_collisionNormalStart = float3(0.0f, 0.0f, 0.0f);
		m_ropeCapsules[sentParticleIndex].m_jacobiCorrectionStart = float3(0.0f, 0.0f, 0.0f);
		m_ropeCapsules[sentParticleIndex].m_jacobiConstraintTotalStart = 0;
		m_ropeCapsules[sentParticleIndex].m_isCollidingEnd = 0;
		m_ropeCapsules[sentParticleIndex].m_collisionNormalEnd = float3(0.0f, 0.0f, 0.0f);
		m_ropeCapsules[sentParticleIndex].m_jacobiCorrectionEnd = float3(0.0f, 0.0f, 0.0f);
		m_ropeCapsules[sentParticleIndex].m_jacobiConstraintTotalEnd = 0;
		BitRegionDetectionSingleCapsule(sentParticleIndex);
	}
}
