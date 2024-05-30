static const uint threadX = 32;
static const uint threadY = 1;
static const uint threadZ = 1;

//BUFFER AND STRUCT LOGIC
//------------------------------------------------------------------------------------------------
struct AABB3
{
	float3 m_mins;
	float3 m_maxs;
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
};

//------------------------------------------------------------------------------------------------
RWStructuredBuffer<float3>		m_particleProposedPositions : register(u2);
RWStructuredBuffer<float4>		m_particleJacobiCorrections : register(u3);
StructuredBuffer<int>			m_particleIsAttached : register(t0);

//COMPUTE SHADER LOGIC
//------------------------------------------------------------------------------------------------
[numthreads(threadX, threadY, threadZ)]
void CSMain(uint3 threadGroupId : SV_GroupID, uint threadIndex : SV_GroupIndex)
{
	uint startIndex = threadGroupId.x * threadX;
	uint particleIndex = startIndex + (threadIndex);
	if (particleIndex >= m_totalParticles)
		return;

	if (m_particleIsAttached[particleIndex] == 0)
	{
		if (m_particleJacobiCorrections[particleIndex].w != 0.0f)
		{
			m_particleProposedPositions[particleIndex] += m_particleJacobiCorrections[particleIndex].xyz / m_particleJacobiCorrections[particleIndex].w;
			m_particleJacobiCorrections[particleIndex] = float4(0.0f, 0.0f, 0.0f, 0.0f);
		}
	}
}
