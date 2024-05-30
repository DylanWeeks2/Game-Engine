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
RWStructuredBuffer<CapsuleCollisionObject>	m_ropeCapsules:register(u7);
StructuredBuffer<int>						m_particleIsAttached : register(t0);
StructuredBuffer<float>					m_particleMasses : register(t1);
StructuredBuffer<float>					m_particleInverseMasses : register(t2);

//COMPUTE SHADER LOGIC
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

//------------------------------------------------------------------------------------------------
[numthreads(threadX, threadY, threadZ)]
void CSMain(uint3 threadGroupId : SV_GroupID, uint threadIndex : SV_GroupIndex)
{
	//Initializations
	uint startIndex = threadGroupId.x * threadX;
	uint sentParticleIndex = startIndex + (threadIndex);
	if (sentParticleIndex >= m_totalParticles)
		return;

	if (m_particleIsAttached[sentParticleIndex] == 1)
		return;

	//Update Particles
	if (m_ropeCapsules[sentParticleIndex - 1].m_isCollidingEnd == 1)
	{
		m_particleJacobiCorrections[sentParticleIndex].xyz += m_ropeCapsules[sentParticleIndex - 1].m_jacobiCorrectionEnd;
		m_particleJacobiCorrections[sentParticleIndex].w += m_ropeCapsules[sentParticleIndex - 1].m_jacobiConstraintTotalEnd;
		m_particleCollisionNormals[sentParticleIndex] = m_ropeCapsules[sentParticleIndex - 1].m_collisionNormalEnd;
	}
	if (m_ropeCapsules[sentParticleIndex].m_isCollidingStart == 1)
	{
		m_particleJacobiCorrections[sentParticleIndex].xyz += m_ropeCapsules[sentParticleIndex].m_jacobiCorrectionStart;
		m_particleJacobiCorrections[sentParticleIndex].w += m_ropeCapsules[sentParticleIndex].m_jacobiConstraintTotalStart;
		m_particleCollisionNormals[sentParticleIndex] = m_ropeCapsules[sentParticleIndex].m_collisionNormalStart;
	}

	//World Bounds Collisions
	ProjectWorldBoundsConstraintsJacobi(sentParticleIndex);
}
