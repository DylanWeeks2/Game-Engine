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
	int		m_isSelfCollisionEnabled;
};

//------------------------------------------------------------------------------------------------
RWStructuredBuffer<float3>		m_particlePositions : register(u0);
RWStructuredBuffer<float3>		m_particleVelocities : register(u1);
RWStructuredBuffer<float3>		m_particleProposedPositions : register(u2);
RWStructuredBuffer<float4>		m_particleJacobiCorrections : register(u3);
RWStructuredBuffer<float3>		m_particleCollisionNormals : register(u4);
StructuredBuffer<int>			m_particleIsAttached : register(t0);

//MATH UTILITIES
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
float3 GetNormalized(float3 vec3)
{
	float length = GetLength(vec3);
	if (length == 0.0f)
	{
		vec3 = float3(0.0f, 0.0f, 0.0f);
		return vec3;
	}
	else
	{
		float scale = 1.0f / length;
		vec3 = float3(vec3.x * scale, vec3.y * scale, vec3.z * scale);
		return vec3;
	}
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

//COMPUTE SHADER LOGIC
//------------------------------------------------------------------------------------------------
[numthreads(threadX, threadY, threadZ)]
void CSMain(uint3 threadGroupId : SV_GroupID, uint threadIndex : SV_GroupIndex)
{
	uint startIndex = threadGroupId.x * threadX;
	uint particleIndex = startIndex + (threadIndex);
	if (particleIndex >= m_totalParticles)
		return;

	if (m_particleIsAttached[particleIndex] == 1)
	{
		m_particlePositions[particleIndex] = m_particleProposedPositions[particleIndex];
		return;
	}

	//Update proposed position based on jacobi correction
	if (m_particleJacobiCorrections[particleIndex].w != 0.0f)
	{
		m_particleProposedPositions[particleIndex] += (m_particleJacobiCorrections[particleIndex].xyz / m_particleJacobiCorrections[particleIndex].w);
		m_particleJacobiCorrections[particleIndex] = float4(0.0f, 0.0f, 0.0f, 0.0f);
	}

	//Calculate new velocity
	m_particleVelocities[particleIndex] = (m_particleProposedPositions[particleIndex] - m_particlePositions[particleIndex]) / m_physicsTimestep;

	//Friction Logic
	if (m_particleCollisionNormals[particleIndex].x != 0.0f || m_particleCollisionNormals[particleIndex].y != 0.0f || m_particleCollisionNormals[particleIndex].z != 0.0f)
	{
		//Static Friction
		if (GetLength(m_particleVelocities[particleIndex]) < 0.15f)
		{
			m_particleVelocities[particleIndex] = float3(0.0f, 0.0f, 0.0f);
			m_particleCollisionNormals[particleIndex] = float3(0.0f, 0.0f, 0.0f);
			return;
		}

		//Kinetic Friction
		float3 tangentalFriction = GetProjectedOnto3D(m_particleVelocities[particleIndex], m_particleCollisionNormals[particleIndex]) - m_particleVelocities[particleIndex];
		m_particleVelocities[particleIndex] += tangentalFriction * m_kineticFrictionCoefficient;
		m_particleCollisionNormals[particleIndex] = float3(0.0f, 0.0f, 0.0f);
	}

	m_particlePositions[particleIndex] = m_particleProposedPositions[particleIndex];
}