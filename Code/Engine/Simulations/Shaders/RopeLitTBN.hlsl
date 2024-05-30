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
struct v2p_t
{
	float4 clipPosition : SV_Position;
	float4 worldPosition : POSITION;
	float4 color : COLOR;
	float2 uv : TEXCOORD;
	float4 worldTangent : TANGENT;
	float4 worldBinormal : BINORMAL;
	float4 worldNormal : NORMAL;
};

//------------------------------------------------------------------------------------------------
cbuffer LightConstants : register(b1)
{
	float3 SunDirection;
	float SunIntensity;
	float AmbientIntensity;
	float3 WorldEyePosition;
	float4 SunColor;
	float4 AmbientColor;
	float4 PointLightColor;
	float4 PointLightRanges[100];
	float4 PointLightIntensities[100];
	float4 PointLightPositions[100];
	int NormalMode;
	int SpecularMode;
	float SpecularIntensity;
	float SpecularPower;
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
Texture2D diffuseTexture : register(t0);
Texture2D normalTexture : register(t1);
Texture2D specGlossEmitTexture : register(t2);
StructuredBuffer<Vertex_PCUTBN> m_verts : register(t3);

//------------------------------------------------------------------------------------------------
SamplerState samplerState : register(s0);

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

//VERTEX SHADER
//------------------------------------------------------------------------------------------------
v2p_t VertexMain(uint vertexID : SV_VertexID)
{
	bool calculateTangentSpace = false;
	int currentIndex = vertexID;
	Vertex_PCUTBN vert0;
	Vertex_PCUTBN vert1;
	Vertex_PCUTBN vert2;
	if (currentIndex % 3 == 0)
	{
		vert0 = m_verts[currentIndex];
		vert1 = m_verts[currentIndex + 1];
		vert2 = m_verts[currentIndex + 2];
	}
	else if (currentIndex % 3 == 1)
	{
		vert0 = m_verts[currentIndex - 1];
		vert1 = m_verts[currentIndex];
		vert2 = m_verts[currentIndex + 1];
	}
	else if(currentIndex % 3 == 2)
	{
		vert0 = m_verts[currentIndex - 1];
		vert1 = m_verts[currentIndex - 2];
		vert2 = m_verts[currentIndex];
	}

	float3 edge1 = vert1.m_position - vert0.m_position;
	float3 edge2 = vert2.m_position - vert0.m_position;

	float x1 = vert1.m_uvTexCoords.x - vert0.m_uvTexCoords.x;
	float x2 = vert2.m_uvTexCoords.x - vert0.m_uvTexCoords.x;
	float y1 = vert1.m_uvTexCoords.y - vert0.m_uvTexCoords.y;
	float y2 = vert2.m_uvTexCoords.y - vert0.m_uvTexCoords.y;

	float r = 0.0f;
	float3 t;
	float3 b;
	if ((x1 * y2 - x2 * y1) != 0.0f)
	{
		r = 1.0f / (x1 * y2 - x2 * y1);
		t = (edge1 * y2 - edge2 * y1) * r;
		b = (edge2 * x1 - edge1 * x2) * r;
		vert0.m_tangent += t;
		vert1.m_tangent += t;
		vert2.m_tangent += t;
		vert0.m_binormal += b;
		vert1.m_binormal += b;
		vert2.m_binormal += b;
		float3 u = GetNormalized(vert2.m_position - vert1.m_position);
		float3 v = GetNormalized(vert0.m_position - vert1.m_position);
		float3 normal = GetNormalized(CrossProduct3D(u, v));
		vert0.m_normal = normal;
		vert1.m_normal = normal;
		vert2.m_normal = normal;
		calculateTangentSpace = true;
	}

	Vertex_PCUTBN currentVertex;
	if (currentIndex % 3 == 0)
	{
		currentVertex = vert0;
	}
	else if (currentIndex % 3 == 1)
	{
		currentVertex = vert1;
	}
	else if (currentIndex % 3 == 2)
	{
		currentVertex = vert2;
		currentVertex.m_normal *= -1.0f;
	}

	if (calculateTangentSpace)
	{
		float3 IBasis = currentVertex.m_tangent;
		float3 JBasis = currentVertex.m_binormal;
		float3 KBasis = currentVertex.m_normal;

		//I
		IBasis = GetNormalized(IBasis);

		//K
		float KDotI = DotProduct3D(KBasis, IBasis);
		float3 KIDisplacement = KDotI * IBasis;
		KBasis -= KIDisplacement;
		KBasis = GetNormalized(KBasis);

		//J
		float JDotI = DotProduct3D(JBasis, IBasis);
		float3 JIDisplacement = JDotI * IBasis;
		JBasis -= JIDisplacement;
		float JDotK = DotProduct3D(JBasis, KBasis);
		float3 JKDisplacement = JDotK * KBasis;
		JBasis -= JKDisplacement;
		JBasis = GetNormalized(JBasis);

		currentVertex.m_tangent = IBasis;
		currentVertex.m_binormal = JBasis;
	}


	

	float4 worldPosition = mul(ModelMatrix, float4(currentVertex.m_position, 1));
	float4 viewPosition = mul(ViewMatrix, worldPosition);
	float4 clipPosition = mul(ProjectionMatrix, viewPosition);

	v2p_t v2p;
	v2p.clipPosition = clipPosition;
	v2p.worldPosition = worldPosition;
	v2p.color = currentVertex.m_color;
	v2p.uv = currentVertex.m_uvTexCoords;
	v2p.worldTangent = mul(ModelMatrix, float4(currentVertex.m_tangent, 0));
	v2p.worldBinormal = mul(ModelMatrix, float4(currentVertex.m_binormal, 0));
	v2p.worldNormal = mul(ModelMatrix, float4(currentVertex.m_normal, 0));
	return v2p;
}

//------------------------------------------------------------------------------------------------
float4 PixelMain(v2p_t input) : SV_Target0
{
	float4 textureColor = diffuseTexture.Sample(samplerState, input.uv);

	float3 worldNormal;
	if (NormalMode == 0)
	{
		float3x3 TBNMatrix = float3x3(normalize(input.worldTangent.xyz), normalize(input.worldBinormal.xyz), normalize(input.worldNormal.xyz));
		float3 tangentNormal = 2.0f * normalTexture.Sample(samplerState, input.uv).rgb - 1.0f;
		worldNormal = mul(tangentNormal, TBNMatrix);
	}
	else
	{
		worldNormal = normalize(input.worldNormal.xyz);
	}

	float specularIntensity = 0.0f;
	float specularPower = 0.0f;
	if (SpecularMode == 0)
	{
		float3 specGlossEmit = specGlossEmitTexture.Sample(samplerState, input.uv).rgb;
		specularIntensity = specGlossEmit.r;
		specularPower = 31.0f * specGlossEmit.g + 1.0f;
	}
	else
	{
		specularIntensity = SpecularIntensity;
		specularPower = SpecularPower;
	}

	float3 worldViewDirection = normalize(WorldEyePosition - input.worldPosition.xyz);
	float3 worldHalfVector = normalize(-SunDirection + worldViewDirection);
	float ndotH = saturate(dot(worldNormal, worldHalfVector));
	float specular = pow(ndotH, specularPower) * specularIntensity;

	float ambient = AmbientIntensity;
	float directional = SunIntensity * saturate(dot(normalize(worldNormal), -SunDirection));
	float4 lightColor = saturate(float4((ambient + directional + specular).xxx, 1));
	float4 vertexColor = input.color;
	float4 modelColor = ModelColor;
	float4 color = lightColor * textureColor * vertexColor * modelColor;
	clip(color.a - 0.01f);
	return color;
}
