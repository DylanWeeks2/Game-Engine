//BUFFER AND STRUCT LOGIC
//-----------------------------------------------------------------------------------------------
struct GameStructuredBufferVariables
{
	float	m_originalGrabLengthFromCamera;
	int		m_isRopeBeingGrabbed;
	int		m_grabbedRopeParticleIndex;
	int		m_wasPointAlreadyLocked;
};

//-----------------------------------------------------------------------------------------------
struct RaycastResult3D
{
	bool	m_didImpact;
	float	m_impactDist;
	float3	m_impactPos;
	float3	m_impactNormal;
	float3	m_rayFwdNormal;
	float3	m_rayStartPos;
	float	m_rayMaxLength;
};

//------------------------------------------------------------------------------------------------
struct AABB3
{
	float3 m_mins;
	float3 m_maxs;
};

//------------------------------------------------------------------------------------------------
cbuffer GameConstantBufferVariables : register(b0)
{
	float3	m_grabPosition;
	int		m_isGrabbing;
	float3	m_grabDirection;
	int		m_shouldLock;
	int		m_unlockAllParticles;
	int		m_isMouseBeingScrolled;
	float	m_mouseScrollDelta;
	int		m_shouldLockParticle;
}

//------------------------------------------------------------------------------------------------
cbuffer RopeConstantBuffer : register(b1)
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
RWStructuredBuffer<float3>							m_particlePositions : register(u0);
RWStructuredBuffer<float3>							m_particleProposedPositions : register(u1);
RWStructuredBuffer<int>								m_particleIsAttached : register(u2);
RWStructuredBuffer<GameStructuredBufferVariables>	m_gameStructuredBufferVariables : register(u3);

//MATH FUNCITONS
//-----------------------------------------------------------------------------------------------
float GetLength(float3 vec3)
{
	float xSquared = vec3.x * vec3.x;
	float ySquared = vec3.y * vec3.y;
	float zSquared = vec3.z * vec3.z;
	float sqrtValue = xSquared + ySquared + zSquared;
	return float(sqrt(float(sqrtValue)));
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
float3 GetNormalized(float3 vec3)
{
	float3 zero = float3(0.0f, 0.0f, 0.0f);
	if (GetLength(vec3) == 0.0f)
	{
		vec3 = float3(0.f, 0.f, 0.f);
		return zero;
	}

	zero = vec3;
	float scale = 1.0f / GetLength(vec3);
	return float3(vec3.x * scale, vec3.y * scale, vec3.z * scale);
}

//-----------------------------------------------------------------------------------------------
float3 CrossProduct3D(float3 a, float3 b)
{
	return float3((a.y * b.z - a.z * b.y), (a.z * b.x - a.x * b.z), (a.x * b.y - a.y * b.x));
}

//-----------------------------------------------------------------------------------------------
float DotProduct3D(float3 a, float3 b)
{
	return (a.x * b.x) + (a.y * b.y) + (a.z * b.z);
}

//-----------------------------------------------------------------------------------------------
bool IsPointInsideSphere3D(float3 sentPoint, float3 sphereCenter, float sphereRadius)
{
	bool returnValue = false;
	float3 displacement = (sphereCenter - sentPoint);
	float distanceSquared = GetLengthSquared(displacement);
	if (sphereRadius * sphereRadius > distanceSquared)
	{
		returnValue = true;
		return returnValue;
	}
	return returnValue;
}

//-----------------------------------------------------------------------------------------------
RaycastResult3D RaycastVsSphere3D(float3 startPos, float3 direction, float maxDist, float3 sphereCenter, float sphereRadius)
{
	RaycastResult3D result;
	result.m_rayStartPos = startPos;
	result.m_rayFwdNormal = GetNormalized(direction);
	result.m_rayMaxLength = maxDist;
	result.m_didImpact = false;
	result.m_didImpact;
	result.m_impactDist = 0.0;
	result.m_impactPos = float3(0.0, 0.0, 0.0);
	result.m_impactNormal = float3(0.0, 0.0, 0.0);

	float3 displacement = sphereCenter - startPos;
	float3 iBasis = direction;
	float3 jBasis = CrossProduct3D(float3(0.0f, 0.0f, 1.0f), iBasis);
	if (jBasis.x == 0.0f && jBasis.y == 0.0f && jBasis.z == 0.0f)
	{
		jBasis = float3(0.0f, 1.0f, 0.0f);
	}
	else
	{
		jBasis = GetNormalized(jBasis);
	}
	float3 kBasis = GetNormalized(CrossProduct3D(iBasis, jBasis));
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

	float distanceToNearestEdgePoint = sqrt(float(discRadiusSquared - displacementJBasisSquared));
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
	result.m_impactNormal = GetNormalized(result.m_impactPos - sphereCenter);
	return result;
}

//COMPUTE SHADER LOGIC
//------------------------------------------------------------------------------------------------
void AttachRopeParticle(int particleIndex)
{
	m_particleIsAttached[particleIndex] = 1;
}

//-----------------------------------------------------------------------------------------------
void UnattachRopeParticle(int particleIndex)
{
	m_particleIsAttached[particleIndex] = 0;
}

//-----------------------------------------------------------------------------------------------
void UpdateGrabbedRopeParticle(int sentParticleIndex)
{
	float distance = m_gameStructuredBufferVariables[0].m_originalGrabLengthFromCamera;
	float3 newPosition = m_grabPosition + (m_grabDirection * distance);
	m_particlePositions[sentParticleIndex] = newPosition;
	m_particleProposedPositions[sentParticleIndex] = newPosition;
}

//------------------------------------------------------------------------------------------------
[numthreads(1, 1, 1)]
void CSMain(uint3 dispatchThreadId : SV_DispatchThreadID)
{
	//Unloacking Rope Logic (All Particles)
	if (m_unlockAllParticles == 1)
	{
		for (uint particleIndex = 0; particleIndex < m_totalParticles; particleIndex++)
		{
			UnattachRopeParticle(particleIndex);
			m_gameStructuredBufferVariables[0].m_isRopeBeingGrabbed = 0;
			m_gameStructuredBufferVariables[0].m_grabbedRopeParticleIndex = -1;
		}
		return;
	}	

	//Rope Grab Logic
	if (m_isGrabbing == 1 && m_gameStructuredBufferVariables[0].m_isRopeBeingGrabbed == 0)
	{
		for (uint particleIndex = 0; particleIndex < m_totalParticles; particleIndex++)
		{
			RaycastResult3D rayResult = RaycastVsSphere3D(m_grabPosition, m_grabDirection, 10.0f, m_particlePositions[particleIndex], m_ropeRadius);
			if (rayResult.m_didImpact == true)
			{
				if (m_particleIsAttached[particleIndex] == 1)
				{
					m_gameStructuredBufferVariables[0].m_wasPointAlreadyLocked = 1;
				}

				AttachRopeParticle(particleIndex);
				m_gameStructuredBufferVariables[0].m_grabbedRopeParticleIndex = particleIndex;
				m_gameStructuredBufferVariables[0].m_originalGrabLengthFromCamera = GetLength(m_grabPosition - m_particlePositions[particleIndex]);
				m_gameStructuredBufferVariables[0].m_isRopeBeingGrabbed = 1;
				break;
			}
		}

		if (m_gameStructuredBufferVariables[0].m_isRopeBeingGrabbed == 0)
		{
			m_gameStructuredBufferVariables[0].m_grabbedRopeParticleIndex = -1;
		}
	}
	else if (m_isGrabbing == 0)
	{
		if (m_shouldLockParticle == 0)
		{
			if (m_gameStructuredBufferVariables[0].m_wasPointAlreadyLocked == 1)
			{
				m_gameStructuredBufferVariables[0].m_wasPointAlreadyLocked = 0;
			}
			else if(m_gameStructuredBufferVariables[0].m_grabbedRopeParticleIndex != -1)
			{
				UnattachRopeParticle(m_gameStructuredBufferVariables[0].m_grabbedRopeParticleIndex);
			}
		}
		m_gameStructuredBufferVariables[0].m_isRopeBeingGrabbed = 0;
		m_gameStructuredBufferVariables[0].m_grabbedRopeParticleIndex = -1;
	}

	//Update Rope Logic
	if (m_gameStructuredBufferVariables[0].m_isRopeBeingGrabbed == 1 && 
		m_gameStructuredBufferVariables[0].m_grabbedRopeParticleIndex != -1)
	{
		if (m_isMouseBeingScrolled == 1)
		{
			m_gameStructuredBufferVariables[0].m_originalGrabLengthFromCamera += m_mouseScrollDelta * m_gameStructuredBufferVariables[0].m_originalGrabLengthFromCamera;
		}

		UpdateGrabbedRopeParticle(m_gameStructuredBufferVariables[0].m_grabbedRopeParticleIndex);
	}
}
