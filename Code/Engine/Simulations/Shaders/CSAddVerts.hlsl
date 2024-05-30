#define PI 3.1415926538
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
struct Vertex_PCUTBN
{
	float3 m_position;
	float4 m_color;
	float2 m_uvTexCoords;
	float3 m_tangent;
	float3 m_binormal;
	float3 m_normal;
};

//------------------------------------------------------------------------------------------------
struct AABB2
{
	float2 m_mins;
	float2 m_maxs;
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
StructuredBuffer<float3>						m_particles:register(t0);
RWStructuredBuffer<Vertex_PCUTBN>				m_verts:register(u1);

//MATH UTILITIES
//-----------------------------------------------------------------------------------------------
float ConvertDegreesToRadians(float degrees)
{
	return degrees * float((PI / 180.0f));
}

//-----------------------------------------------------------------------------------------------
float CosDegrees(float degrees)
{
	return cos(ConvertDegreesToRadians(degrees));
}

//-----------------------------------------------------------------------------------------------
float SinDegrees(float degrees)
{
	return sin(ConvertDegreesToRadians(degrees));
}

//-----------------------------------------------------------------------------------------------
float3 MakeFromPolarDegrees3D(float longitudeDegrees, float latitudeDegrees, float length)
{
	return float3(length * CosDegrees(longitudeDegrees) * CosDegrees(latitudeDegrees), length * SinDegrees(longitudeDegrees) * CosDegrees(latitudeDegrees), length * -SinDegrees(latitudeDegrees));
}

//------------------------------------------------------------------------------------------------
float2 MakeFromPolarDegrees2D(float orientationDegrees, float length)
{
	return float2(length * CosDegrees(orientationDegrees), length * SinDegrees(orientationDegrees));
}

//-----------------------------------------------------------------------------------------------
float RangeMap(float inValue, float inStart, float inEnd, float outStart, float outEnd)
{
	float inRange = inEnd - inStart;
	float outRange = outEnd - outStart;
	float scale = outRange / inRange;

	float outValue = scale * (inValue - inStart) + outStart;
	return outValue;
}

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
float3 CrossProduct3D(float3 a, float3 b)
{
	return float3((a.y * b.z - a.z * b.y), (a.z * b.x - a.x * b.z), (a.x * b.y - a.y * b.x));
}

//COMPUTE SHADER LOGIC
//-----------------------------------------------------------------------------------------------
void SetVertsForQuad3D(int startVertIndex, float3 bottomLeft, float3 bottomRight, float3 topLeft, float3 topRight, float4 color, AABB2 UVs)
{
	float3 normal = float3(0.0f, 0.0f, 0.0f);
	Vertex_PCUTBN bottomLeftVert;
	bottomLeftVert.m_position = bottomLeft;
	bottomLeftVert.m_color = color;
	bottomLeftVert.m_uvTexCoords = UVs.m_mins;
	bottomLeftVert.m_tangent = float3(0.0f, 0.0f, 0.0f);
	bottomLeftVert.m_binormal = float3(0.0f, 0.0f, 0.0f);
	bottomLeftVert.m_normal = normal;
	Vertex_PCUTBN bottomRightVert;
	bottomRightVert.m_position = bottomRight;
	bottomRightVert.m_color = color;
	bottomRightVert.m_uvTexCoords = float2(UVs.m_maxs.x, UVs.m_mins.y);
	bottomRightVert.m_tangent = float3(0.0f, 0.0f, 0.0f);
	bottomRightVert.m_binormal = float3(0.0f, 0.0f, 0.0f);
	bottomRightVert.m_normal = normal;
	Vertex_PCUTBN topRightVert;
	topRightVert.m_position = topRight;
	topRightVert.m_color = color;
	topRightVert.m_uvTexCoords = UVs.m_maxs;
	topRightVert.m_tangent = float3(0.0f, 0.0f, 0.0f);
	topRightVert.m_binormal = float3(0.0f, 0.0f, 0.0f);
	topRightVert.m_normal = normal;
	Vertex_PCUTBN topLeftVert;
	topLeftVert.m_position = topLeft;
	topLeftVert.m_color = color;
	topLeftVert.m_uvTexCoords = float2(UVs.m_mins.x, UVs.m_maxs.y);
	topLeftVert.m_tangent = float3(0.0f, 0.0f, 0.0f);
	topLeftVert.m_binormal = float3(0.0f, 0.0f, 0.0f);
	topLeftVert.m_normal = normal;


	m_verts[startVertIndex + 0] = bottomLeftVert;
	m_verts[startVertIndex + 1] = bottomRightVert;
	m_verts[startVertIndex + 2] = topRightVert;
	m_verts[startVertIndex + 3] = bottomLeftVert;
	m_verts[startVertIndex + 4] = topRightVert;
	m_verts[startVertIndex + 5] = topLeftVert;
}

//-----------------------------------------------------------------------------------------------
void SetVertsForHemisphereZUp3D(int startVertIndex, float3 center, float radius, float4 color, AABB2 UVs, int numLatitudeSlices)
{
	float yawIncrementAmount = 360.0f / (numLatitudeSlices * 2.0f);
	float pitchIncrementAmount = 180.0f / numLatitudeSlices;
	for (int yawIndex = 0; yawIndex < numLatitudeSlices * 2.0f; yawIndex++)
	{
		float minYaw = yawIndex * yawIncrementAmount;
		float maxYaw = minYaw + yawIncrementAmount;
		for (int pitchIndex = 0; pitchIndex < numLatitudeSlices / 2; pitchIndex++)
		{
			float minPitch = -90.0f + pitchIndex * pitchIncrementAmount;
			float maxPitch = minPitch + pitchIncrementAmount;
			float3 position = center;
			float3 topLeft = MakeFromPolarDegrees3D(minYaw, maxPitch, radius) + position;
			float3 topRight = MakeFromPolarDegrees3D(maxYaw, maxPitch, radius) + position;
			float3 bottomLeft = MakeFromPolarDegrees3D(minYaw, minPitch, radius) + position;
			float3 bottomRight = MakeFromPolarDegrees3D(maxYaw, minPitch, radius) + position;

			float minU = RangeMap(minYaw, 0.0f, 360.0f, UVs.m_mins.x, UVs.m_maxs.x);
			float maxU = RangeMap(maxYaw, 0.0f, 360.0f, UVs.m_mins.x, UVs.m_maxs.x);
			float minV = RangeMap(minPitch, -90.0f, 90.0f, UVs.m_maxs.y, UVs.m_mins.y);
			float maxV = RangeMap(maxPitch, -90.0f, 90.0f, UVs.m_maxs.y, UVs.m_mins.y);
			AABB2 quadUVs;
			quadUVs.m_mins = float2(minU, maxV);
			quadUVs.m_maxs = float2(maxU, minV);
			SetVertsForQuad3D(startVertIndex, topLeft, topRight, bottomLeft, bottomRight, color, quadUVs);
			startVertIndex += 6;
		}
	}
}

//-----------------------------------------------------------------------------------------------
void SetVertsForCylinderSidesOnlyZ3D(int startVertIndex, float2 centerXY, float2 minMaxZ, float radius, float numSlices, float4 tint, AABB2 UVs)
{
	float yawIncrementAmount = 360.0f / (float(numSlices) * 2.0f);
	for (float yaw = 0.0f; yaw < 360.0f; yaw += yawIncrementAmount)
	{
		float minYaw = yaw;
		float maxYaw = yaw + yawIncrementAmount;
		if (360.f - yaw < yawIncrementAmount)
		{
			minYaw = yaw;
			maxYaw = yaw + (360.0f - yaw);
		}
		float2 position = centerXY;
		float2 leftMadeFromPolar = MakeFromPolarDegrees2D(minYaw, radius);
		float2 rightMadeFromPolar = MakeFromPolarDegrees2D(maxYaw, radius);
		float3 topLeft = float3(leftMadeFromPolar.x + position.x, leftMadeFromPolar.y + position.y, minMaxZ.y);
		float3 topRight = float3(rightMadeFromPolar.x + position.x, rightMadeFromPolar.y + position.y, minMaxZ.y);
		float3 bottomLeft = float3(leftMadeFromPolar.x + position.x, leftMadeFromPolar.y + position.y, minMaxZ.x);
		float3 bottomRight = float3(rightMadeFromPolar.x + position.x, rightMadeFromPolar.y + position.y, minMaxZ.x);

		float minU = RangeMap(minYaw, 0.0f, 360.0f, UVs.m_mins.x, UVs.m_maxs.x);
		float maxU = RangeMap(maxYaw, 0.0f, 360.0f, UVs.m_mins.x, UVs.m_maxs.x);
		float minV = RangeMap(minMaxZ.x, minMaxZ.x, minMaxZ.y, UVs.m_mins.y, UVs.m_maxs.y);
		float maxV = RangeMap(minMaxZ.y, minMaxZ.x, minMaxZ.y, UVs.m_mins.y, UVs.m_maxs.y);
		AABB2 quadUVs;
		quadUVs.m_mins = float2(minU, minV);
		quadUVs.m_maxs = float2(maxU, maxV);
		SetVertsForQuad3D(startVertIndex, bottomLeft, bottomRight, topLeft, topRight, tint, quadUVs);
		startVertIndex += 6;
	}
}

//-----------------------------------------------------------------------------------------------
void SetVertsForHemisphereZDown3D(int startVertIndex, float3 center, float radius, const float4 color, AABB2 UVs, int numLatitudeSlices)
{
	float yawIncrementAmount = 360.0f / (numLatitudeSlices * 2.0f);
	float pitchIncrementAmount = 180.0f / numLatitudeSlices;
	for (int yawIndex = 0; yawIndex < numLatitudeSlices * 2.0f; yawIndex++)
	{
		float minYaw = yawIndex * yawIncrementAmount;
		float maxYaw = minYaw + yawIncrementAmount;
		for (int pitchIndex = numLatitudeSlices / 2; pitchIndex < numLatitudeSlices; pitchIndex++)
		{
			float minPitch = -90.0f + pitchIndex * pitchIncrementAmount;
			float maxPitch = minPitch + pitchIncrementAmount;
			float3 position = center;
			float3 topLeft = MakeFromPolarDegrees3D(minYaw, maxPitch, radius) + position;
			float3 topRight = MakeFromPolarDegrees3D(maxYaw, maxPitch, radius) + position;
			float3 bottomLeft = MakeFromPolarDegrees3D(minYaw, minPitch, radius) + position;
			float3 bottomRight = MakeFromPolarDegrees3D(maxYaw, minPitch, radius) + position;

			float minU = RangeMap(minYaw, 0.0f, 360.0f, UVs.m_mins.x, UVs.m_maxs.x);
			float maxU = RangeMap(maxYaw, 0.0f, 360.0f, UVs.m_mins.x, UVs.m_maxs.x);
			float minV = RangeMap(minPitch, -90.0f, 90.0f, UVs.m_maxs.y, UVs.m_mins.y);
			float maxV = RangeMap(maxPitch, -90.0f, 90.0f, UVs.m_maxs.y, UVs.m_mins.y);
			AABB2 quadUVs;
			quadUVs.m_mins = float2(minU, maxV);
			quadUVs.m_maxs = float2(maxU, minV);
			SetVertsForQuad3D(startVertIndex, topLeft, topRight, bottomLeft, bottomRight, color, quadUVs);
			startVertIndex += 6;
		}
	}
}

//-----------------------------------------------------------------------------------------------
void SetVertsForCapsule3D(int startVertIndex, Capsule3 capsule, float4 tint, AABB2 UVs, int numSlices)
{
	int originalVertIndex = startVertIndex;
	float3 displacement = float3(capsule.m_bone.m_end.x, capsule.m_bone.m_end.y, capsule.m_bone.m_end.z) - float3(capsule.m_bone.m_start.x, capsule.m_bone.m_start.y, capsule.m_bone.m_start.z);
	float distance = GetLength(displacement);
	float percentToStart = (capsule.m_radius + capsule.m_radius) / (distance + capsule.m_radius + capsule.m_radius);
	float percentToEnd = (distance + capsule.m_radius) / (distance + capsule.m_radius + capsule.m_radius);
	AABB2 newUVs = UVs;
	newUVs.m_maxs.y *= percentToStart;
	SetVertsForHemisphereZDown3D(startVertIndex, float3(0.0f, 0.0f, 0.0f), float(capsule.m_radius), tint, newUVs, numSlices);
	startVertIndex += 384;
	newUVs.m_mins.y = (capsule.m_radius) / (distance + capsule.m_radius + capsule.m_radius);
	newUVs.m_maxs.y = percentToEnd;
	SetVertsForCylinderSidesOnlyZ3D(startVertIndex, float2(0.0f, 0.0f), float2(0.0f, distance), float(capsule.m_radius), float(numSlices), tint, newUVs);
	startVertIndex += 96;
	newUVs.m_mins.y = (distance) / (distance + capsule.m_radius + capsule.m_radius);
	newUVs.m_maxs.y = 1.0f;
	SetVertsForHemisphereZUp3D(startVertIndex, float3(0.0f, 0.0f, distance), float(capsule.m_radius), tint, newUVs, numSlices);
	startVertIndex += 384;

	//Tranform back to world space
	float3 kBasis = GetNormalized(displacement);
	float3 iBasis = CrossProduct3D(float3(0.0f, 1.0f, 0.0f), kBasis);
	if (iBasis.x == 0.0f && iBasis.y == 0.0f && iBasis.z == 0.0f)
	{
		iBasis = float3(1.0f, 0.0f, 0.0f);
	}
	else
	{
		iBasis = GetNormalized(iBasis);
	}
	float3 jBasis = GetNormalized(CrossProduct3D(kBasis, iBasis));
	float4x4 transform;
	transform[0] = float4(iBasis, 0.0f);
	transform[1] = float4(jBasis, 0.0f);
	transform[2] = float4(kBasis, 0.0f);
	transform[3] = float4(float3(capsule.m_bone.m_start.x, capsule.m_bone.m_start.y, capsule.m_bone.m_start.z), 0.0f);
	int endVertIndex = 864;
	for (int vertIndex = 0; vertIndex < endVertIndex; vertIndex++)
	{
		int currentIndex = originalVertIndex + vertIndex;
		float3 currentPosition = m_verts[currentIndex].m_position;
		float3 transformedPosition = float3(0.0f, 0.0f, 0.0f);
		transformedPosition.x = (transform[0].x * currentPosition.x) + (transform[1].x * currentPosition.y) + (transform[2].x * currentPosition.z) + transform[3].x;
		transformedPosition.y = (transform[0].y * currentPosition.x) + (transform[1].y * currentPosition.y) + (transform[2].y * currentPosition.z) + transform[3].y;
		transformedPosition.z = (transform[0].z * currentPosition.x) + (transform[1].z * currentPosition.y) + (transform[2].z * currentPosition.z) + transform[3].z;
		m_verts[originalVertIndex + vertIndex].m_position = transformedPosition;
	}
}

//------------------------------------------------------------------------------------------------
[numthreads(threadX, threadY, threadZ)]
void CSMain(uint3 threadGroupId : SV_GroupID, uint threadIndex : SV_GroupIndex)
{
	uint startIndex = threadGroupId.x * threadX;
	uint sentParticleIndex = startIndex + (threadIndex);
	if (sentParticleIndex >= m_totalParticles - 1)
		return;

	/*m_particles[sentParticleIndex] = float3(sentParticleIndex, threadGroupIndex, threadIndex);
	return;*/

	int totalVertsPerCapsule = 864;
	int startVertIndex = totalVertsPerCapsule * (sentParticleIndex);
	Capsule3 capsule;
	Cylinder3 bone;
	bone.m_start = m_particles[sentParticleIndex];
	bone.m_end = m_particles[sentParticleIndex + 1];
	bone.m_iBasis = float3(0.0f, 0.0f, 1.0f);
	bone.m_jBasis = float3(0.0f, 0.0f, 1.0f);
	bone.m_kBasis = float3(0.0f, 0.0f, 1.0f);
	bone.m_radius = m_ropeRadius;
	capsule.m_bone = bone;
	capsule.m_radius = m_ropeRadius;
	AABB2 UVs;
	UVs.m_mins = float2(0.0f, 0.0f);
	UVs.m_maxs = float2(1.0f, 1.0f);
	SetVertsForCapsule3D(startVertIndex, capsule, float4(1.0f, 1.0f, 1.0f, 1.0f), UVs, 8);
}
