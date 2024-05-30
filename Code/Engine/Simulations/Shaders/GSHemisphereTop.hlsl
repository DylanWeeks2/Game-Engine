#define PI 3.1415926538
#define maxVertCount 96

//------------------------------------------------------------------------------------------------
cbuffer ParticleBufferSize : register(b0) 
{
	uint ParticleBufferSize;
};

//------------------------------------------------------------------------------------------------
cbuffer CameraConstants : register(b2)
{
	float4x4 ProjectionMatrix;
	float4x4 ViewMatrix;
};

//------------------------------------------------------------------------------------------------
cbuffer ModelConstants : register(b3)
{
	float4x4 ModelMatrix;
	float4 ModelColor;
};

//------------------------------------------------------------------------------------------------
struct uint64_t
{
	uint m_lowBits;
	uint m_highBits;
};

//------------------------------------------------------------------------------------------------
struct VS_OUTPUT
{
	float4 position : SV_Position;
	float4 color : COLOR;
	float2 uv : TEXCOORD;
	float4 tangent : TANGENT;
	float4 binormal : BINORMAL;
	float4 normal : NORMAL;
	uint index : INDEX;
};

//------------------------------------------------------------------------------------------------
struct GS_OUTPUT
{
	float4 position : SV_Position;
	uint index : INDEX;
};

//------------------------------------------------------------------------------------------------
struct AABB2
{
	float2 m_mins;
	float2 m_maxs;
};

//------------------------------------------------------------------------------------------------
Texture2D diffuseTexture : register(t0);
SamplerState diffuseSampler : register(s0);
StructuredBuffer<float3> m_particleBuffer : register(t1);

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

//------------------------------------------------------------------------------------------------
float2 MakeFromPolarDegrees2D(float orientationDegrees, float length)
{
	return float2(length * CosDegrees(orientationDegrees), length * SinDegrees(orientationDegrees));
}

//-----------------------------------------------------------------------------------------------
float3 MakeFromPolarDegrees3D(float longitudeDegrees, float latitudeDegrees, float length)
{
	return float3(length * CosDegrees(longitudeDegrees) * CosDegrees(latitudeDegrees), length * SinDegrees(longitudeDegrees) * CosDegrees(latitudeDegrees), length * -SinDegrees(latitudeDegrees));
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
float RangeMap(float inValue, float inStart, float inEnd, float outStart, float outEnd)
{
	float inRange = inEnd - inStart;
	float outRange = outEnd - outStart;
	float scale = outRange / inRange;

	float outValue = scale * (inValue - inStart) + outStart;
	return outValue;
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
float3 CrossProduct3D(float3 a, float3 b)
{
	return float3((a.y * b.z - a.z * b.y), (a.z * b.x - a.x * b.z), (a.x * b.y - a.y * b.x));
}

//------------------------------------------------------------------------------------------------
//VERTEX SHADER
VS_OUTPUT VertexMain(uint vertexID : SV_VertexID)
{
	VS_OUTPUT output;
	output.position = float4(m_particleBuffer[vertexID], 1);
	output.color = float4(255.0f, 255.0f, 255.0f, 255.0f);
	output.uv = float2(0.0f, 1.0f);
	output.tangent = float4(0, 0, 0, 1);
	output.binormal = float4(0, 0, 0, 1);
	output.normal = float4(0, 0, 0, 1);
	output.index = vertexID;
	return output;
}

//------------------------------------------------------------------------------------------------
//GEOMETRY SHADER
[maxvertexcount(maxVertCount)]
void GSMain(point VS_OUTPUT input[1], inout LineStream< GS_OUTPUT > output)
{
	//Quit if at final index
	if (input[0].index >= ParticleBufferSize)
	{
		return;
	}

	//Initializations
	GS_OUTPUT verts[maxVertCount];
	for (int i = 0; i < maxVertCount; ++i)
	{
		verts[i].position = input[0].position;
		verts[i].index = input[0].index;
	}
	float radius = 0.015f;
	int vertCounter = 0;
	float numSlices = 4.0f;
	float3 positionA = input[0].position.xyz;
	float3 positionB = m_particleBuffer[input[0].index + 1].xyz;
	float3 displacementAB = positionB - positionA;
	float distanceAB = GetLength(displacementAB);
	AABB2 indexs;
	indexs.m_mins = float2(0.0f, 0.0f);
	indexs.m_maxs = float2(1.0f, 1.0f);

	//Verts for hemisphere top Z3D
	vertCounter = 0;
	float yawIncrementAmount = 360.0f / (numSlices * 2.0f);
	float pitchIncrementAmount = 180.0f / numSlices;
	for (int yawIndex = 0; yawIndex < (numSlices * 2.0f); yawIndex++)
	{
		float minYaw = yawIndex * yawIncrementAmount;
		float maxYaw = minYaw + yawIncrementAmount;
		for (int pitchIndex = 0; pitchIndex < numSlices / 2; pitchIndex++)
		{
			float minPitch		= -90.0f + pitchIndex * pitchIncrementAmount;
			float maxPitch		= minPitch + pitchIncrementAmount;
			float3 position		= float3(0.0f, 0.0f, 0.0f);
			float3 bottomLeft	= MakeFromPolarDegrees3D(minYaw, maxPitch, radius) + position;
			float3 bottomRight	= MakeFromPolarDegrees3D(maxYaw, maxPitch, radius) + position;
			float3 topLeft		= MakeFromPolarDegrees3D(minYaw, minPitch, radius) + position;
			float3 topRight		= MakeFromPolarDegrees3D(maxYaw, minPitch, radius) + position;

			float minU = RangeMap(minYaw, 0.0f, 360.0f, indexs.m_mins.x, indexs.m_maxs.x);
			float maxU = RangeMap(maxYaw, 0.0f, 360.0f, indexs.m_mins.x, indexs.m_maxs.x);
			float minV = RangeMap(minPitch, -90.0f, 90.0f, indexs.m_maxs.y, indexs.m_mins.y);
			float maxV = RangeMap(maxPitch, -90.0f, 90.0f, indexs.m_maxs.y, indexs.m_mins.y);
			AABB2 quadindexs;
			quadindexs.m_mins = float2(minU, maxV);
			quadindexs.m_maxs = float2(maxU, minV);

			verts[vertCounter].position = float4(bottomLeft, 1.0f);
			verts[vertCounter].index = input[0].index;
			vertCounter++;
			verts[vertCounter].position = float4(bottomRight, 1.0f);
			verts[vertCounter].index = input[0].index;
			vertCounter++;
			verts[vertCounter].position = float4(topRight, 1.0f);
			verts[vertCounter].index = input[0].index;
			vertCounter++;
			verts[vertCounter].position = float4(bottomLeft, 1.0f);
			verts[vertCounter].index = input[0].index;
			vertCounter++;
			verts[vertCounter].position = float4(topLeft, 1.0f);
			verts[vertCounter].index = input[0].index;
			vertCounter++;
			verts[vertCounter].position = float4(topRight, 1.0f);
			verts[vertCounter].index = input[0].index;
			vertCounter++;
		}
	}

	float3 kBasis = GetNormalized(displacementAB);
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
	transform[3] = float4(input[0].position.xyz, 0.0f);

	for (int index = 0; index < maxVertCount; index++)
	{
		//Transforms all verts back
		float3 transformedPosition = float3(0.0f, 0.0f, 0.0f);
		transformedPosition.x = (transform[0].x * verts[index].position.x) + (transform[1].x * verts[index].position.y) + (transform[2].x * verts[index].position.z) + transform[3].x;
		transformedPosition.y = (transform[0].y * verts[index].position.x) + (transform[1].y * verts[index].position.y) + (transform[2].y * verts[index].position.z) + transform[3].y;
		transformedPosition.z = (transform[0].z * verts[index].position.x) + (transform[1].z * verts[index].position.y) + (transform[2].z * verts[index].position.z) + transform[3].z;

		float4 localPosition = float4(transformedPosition.xyz, 1.0f);
		float4 worldPosition = mul(ModelMatrix, localPosition);
		float4 viewPosition = mul(ViewMatrix, worldPosition);
		float4 clipPosition = mul(ProjectionMatrix, viewPosition);
		verts[index].position = clipPosition;
		output.Append(verts[index]);
	}
}

//------------------------------------------------------------------------------------------------
//PIXEL SHADER
float4 PixelMain(GS_OUTPUT input) : SV_Target0
{
	float4 textureColor = diffuseTexture.Sample(diffuseSampler, input.index);
	float4 vertexColor;
	if (input.index == 0)
	{
		vertexColor = float4(255.0f, 0.0f, 0.0f, 255.0f);
	}
	else if (input.index == ParticleBufferSize - 1)
	{
		vertexColor = float4(0.0f, 0.0f, 255.0f, 255.0f);
	}
	else
	{
		vertexColor = float4(255.0f, 255.0f, 255.0f, 255.0f);
	}
	float4 modelColor = ModelColor;
	float4 color = textureColor * vertexColor * modelColor;
	clip(color.a - 0.01f);
	return color;
}
