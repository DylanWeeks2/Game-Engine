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
RWStructuredBuffer<uint64_t>	m_particleMacroBitRegions : register(u5);
RWStructuredBuffer<uint64_t>	m_particleMicroBitRegions : register(u6);
StructuredBuffer<int>			m_particleIsAttached : register(t0);
StructuredBuffer<float>		m_particleMasses : register(t1);
StructuredBuffer<float>		m_particleInverseMasses : register(t2);

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
//-----------------------------------------------------------------------------------------------
void ProjectDistanceConstraintJacobi(int particleAIndex, int particleBIndex, bool isParticleA)
{
	//Calculates direction and overflow along with weight Coefficients
	float3 displacement = m_particleProposedPositions[particleBIndex] - m_particleProposedPositions[particleAIndex];
	float desiredDistance = (m_desiredDistance);
	float distanceConstraint = (GetLength(displacement) - desiredDistance);

	//Early out check
	if (distanceConstraint == 0.0f)
	{
		return;
	}

	float3 gradient = GetNormalized(displacement);
	float coefficientValue = 0.0f;
	if (distanceConstraint < 0.0f)
	{
		coefficientValue = m_compressionCoefficient;
	}
	else
	{
		coefficientValue = m_stretchingCoefficient;
	}

	if (m_particleIsAttached[particleAIndex] == 0 && m_particleIsAttached[particleBIndex] == 0)
	{
		float pointAWeightCoefficient = (m_particleInverseMasses[particleAIndex] / (m_particleInverseMasses[particleAIndex] + m_particleInverseMasses[particleBIndex]));
		float pointBWeightCoefficient = (m_particleInverseMasses[particleBIndex] / (m_particleInverseMasses[particleAIndex] + m_particleInverseMasses[particleBIndex]));

		float3 deltaParticleA = distanceConstraint * pointAWeightCoefficient * gradient * coefficientValue;
		float3 deltaParticleB = distanceConstraint * pointBWeightCoefficient * gradient * coefficientValue;

		if (isParticleA)
		{
			m_particleJacobiCorrections[particleAIndex].x += deltaParticleA.x;
			m_particleJacobiCorrections[particleAIndex].y += deltaParticleA.y;
			m_particleJacobiCorrections[particleAIndex].z += deltaParticleA.z;
			m_particleJacobiCorrections[particleAIndex].w++;
		}
		else
		{
			m_particleJacobiCorrections[particleBIndex].x -= deltaParticleB.x;
			m_particleJacobiCorrections[particleBIndex].y -= deltaParticleB.y;
			m_particleJacobiCorrections[particleBIndex].z -= deltaParticleB.z;
			m_particleJacobiCorrections[particleBIndex].w++;
		}
	}
	else if (m_particleIsAttached[particleAIndex] == 1 && m_particleIsAttached[particleBIndex] == 0)
	{
		float3 deltaParticleB = distanceConstraint * gradient * coefficientValue;
		if (isParticleA == false)
		{
			m_particleJacobiCorrections[particleBIndex].x -= deltaParticleB.x;
			m_particleJacobiCorrections[particleBIndex].y -= deltaParticleB.y;
			m_particleJacobiCorrections[particleBIndex].z -= deltaParticleB.z;
			m_particleJacobiCorrections[particleBIndex].w++;
		}
	}
	else //if (m_particles[particleAIndex].m_isAttached == 0 && m_particles[particleBIndex].m_isAttached == 1)
	{
		float3 deltaParticleA = distanceConstraint * gradient * coefficientValue;
		if (isParticleA)
		{

			m_particleJacobiCorrections[particleAIndex].x += deltaParticleA.x;
			m_particleJacobiCorrections[particleAIndex].y += deltaParticleA.y;
			m_particleJacobiCorrections[particleAIndex].z += deltaParticleA.z;
			m_particleJacobiCorrections[particleAIndex].w++;
		}
	}
}

//-----------------------------------------------------------------------------------------------
void ProjectBendingConstraintJacobi(int particleAIndex, int particleBIndex, int particleCIndex, bool isParticleA)
{
	//Calculates direction and overflow along with weight Coefficients
	float3 displacement = m_particleProposedPositions[particleCIndex] - m_particleProposedPositions[particleAIndex];
	float distanceConstraint = GetLength(displacement) - m_bendingConstraintDistance;

	//Check if less than the minimum distance, if so no need to constrain
	if (distanceConstraint > 0.0f)
	{
		return;
	}

	float3 gradient = GetNormalized(displacement);
	float3 directionCheck = GetNormalized(m_particleProposedPositions[particleBIndex] - m_particleProposedPositions[particleAIndex]);

	//Because this is a cheap method to check for bending need to ensure the direction inst the same
	if (gradient.x == directionCheck.x && gradient.y == directionCheck.y && gradient.z == directionCheck.z)
	{
		return;
	}

	if (m_particleIsAttached[particleAIndex] == 0 && m_particleIsAttached[particleCIndex] == 0)
	{
		float pointAWeightCoefficient = (m_particleInverseMasses[particleAIndex] / (m_particleInverseMasses[particleAIndex] + m_particleInverseMasses[particleCIndex]));
		float pointCWeightCoefficient = (m_particleInverseMasses[particleCIndex] / (m_particleInverseMasses[particleAIndex] + m_particleInverseMasses[particleCIndex]));

		float3 deltaParticleA = pointAWeightCoefficient * distanceConstraint * gradient * m_bendingCoefficient;
		float3 deltaParticleC = pointCWeightCoefficient * distanceConstraint * gradient * m_bendingCoefficient;

		if (isParticleA)
		{
			m_particleJacobiCorrections[particleAIndex].x += deltaParticleA.x;
			m_particleJacobiCorrections[particleAIndex].y += deltaParticleA.y;
			m_particleJacobiCorrections[particleAIndex].z += deltaParticleA.z;
			m_particleJacobiCorrections[particleAIndex].w++;
		}
		else
		{
			m_particleJacobiCorrections[particleCIndex].x -= deltaParticleC.x;
			m_particleJacobiCorrections[particleCIndex].y -= deltaParticleC.y;
			m_particleJacobiCorrections[particleCIndex].z -= deltaParticleC.z;
			m_particleJacobiCorrections[particleCIndex].w++;
		}
	}
	else if (m_particleIsAttached[particleAIndex] == 1 && m_particleIsAttached[particleCIndex] == 0)
	{
		float3 deltaParticleC = distanceConstraint * gradient * m_bendingCoefficient;
		if (isParticleA == false)
		{
			m_particleJacobiCorrections[particleCIndex].x -= deltaParticleC.x;
			m_particleJacobiCorrections[particleCIndex].y -= deltaParticleC.y;
			m_particleJacobiCorrections[particleCIndex].z -= deltaParticleC.z;
			m_particleJacobiCorrections[particleCIndex].w++;
		}
	}
	else //if (m_particles[particleAIndex].m_isAttached == 0 && m_particles[particleCIndex].m_isAttached == 1)
	{
		float3 deltaParticleA = distanceConstraint * gradient * m_bendingCoefficient;
		if (isParticleA)
		{
			m_particleJacobiCorrections[particleAIndex].x += deltaParticleA.x;
			m_particleJacobiCorrections[particleAIndex].y += deltaParticleA.y;
			m_particleJacobiCorrections[particleAIndex].z += deltaParticleA.z;
			m_particleJacobiCorrections[particleAIndex].w++;
		}
	}
}

//------------------------------------------------------------------------------------------------
[numthreads(threadX, threadY, threadZ)]
void CSMain(uint3 threadGroupId : SV_GroupID, uint threadIndex : SV_GroupIndex)
{
	uint startIndex = threadGroupId.x * threadX;
	uint particleIndex = startIndex + (threadIndex);
	if (particleIndex >= m_totalParticles)
		return;

	//Distance Constraints
	int test = particleIndex - 1;
	if (test >= 0)
	{
		ProjectDistanceConstraintJacobi(particleIndex - 1, particleIndex, false);
	}
	if (particleIndex + 1 < m_totalParticles)
	{
		ProjectDistanceConstraintJacobi(particleIndex, particleIndex + 1, true);
	}
	//Bending Constraints
	if (particleIndex - 2 >= 0)
	{
		ProjectBendingConstraintJacobi(particleIndex - 2, particleIndex - 1, particleIndex, false);
	}
	if (particleIndex + 2 < m_totalParticles)
	{
		ProjectBendingConstraintJacobi(particleIndex, particleIndex + 1, particleIndex + 2, true);
	}
}
