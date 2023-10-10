#include "VertexUtils.hpp"
#include "Vertex_PCU.hpp"
#include "Vertex_PNCU.hpp"
#include "Engine/Math/Vec2.hpp"
#include "Engine/Math/Vec3.hpp"
#include "Engine/Math/MathUtils.hpp"
#include "Engine/Core/Rgba8.hpp"
#include "Engine/Math/AABB2.hpp"
#include "Engine/Math/LineSegment2.hpp"
#include "Engine/Math/Capsule2.hpp"
#include "Engine/Math/OBB2.hpp"
#include "Engine/Math/Mat44.hpp"
#include "Engine/Math/AABB3.hpp"
#include "Engine/Math/FloatRange.hpp"
#include "Engine/Math/EulerAngles.hpp"

//-----------------------------------------------------------------------------------------------
void TransformVertexArrayXY3D(int numVerts, Vertex_PCU* verts, float uniformScaleXY, float rotationDegreesAboutZ, Vec2 const& translationXY)
{
	for (int vertexCounter = 0; vertexCounter < numVerts; vertexCounter++) //loops through each vertex and transforms them
	{
		TransformPositionXY3D(verts[vertexCounter].m_position, uniformScaleXY, rotationDegreesAboutZ, translationXY);
	}
}

//-----------------------------------------------------------------------------------------------
void TransformVertexArrayXY3D(int numVerts, Vertex_PCU* verts, float uniformScaleXY, Vec2 iBasis, Vec2 jBasis, Vec2 const& translationXY)
{
	iBasis *= uniformScaleXY;
	jBasis *= uniformScaleXY;

	for (int vertexCounter = 0; vertexCounter < numVerts; vertexCounter++) //loops through each vertex and transforms them
	{
		TransformPositionXY3D(verts[vertexCounter].m_position, iBasis, jBasis, translationXY);
	}
}

//-----------------------------------------------------------------------------------------------
void TransformVertexArray3D(std::vector<Vertex_PCU>& verts, const Mat44& transform)
{
	for (int vertexCounter = 0; vertexCounter < verts.size(); vertexCounter++) //loops through each vertex and transforms them
	{
		TransformPosition3D(verts[vertexCounter].m_position, transform);
	}
}

//-----------------------------------------------------------------------------------------------
void AddVertsForAABB2D(std::vector<Vertex_PCU>& verts, AABB2 const& bounds, Rgba8 const& color)
{
	Vec3 bottomLeft = Vec3(bounds.m_mins.x, bounds.m_mins.y, 0.0f);
	Vec3 bottomRight = Vec3(bounds.m_maxs.x, bounds.m_mins.y, 0.0f);
	Vec3 topLeft = Vec3(bounds.m_mins.x, bounds.m_maxs.y, 0.0f);
	Vec3 topRight = Vec3(bounds.m_maxs.x, bounds.m_maxs.y, 0.0f);

	verts.push_back(Vertex_PCU(bottomLeft, color, Vec2(0.0f, 0.0f)));
	verts.push_back(Vertex_PCU(bottomRight, color, Vec2(1.0f, 0.0f)));
	verts.push_back(Vertex_PCU(topRight, color, Vec2(1.0f, 1.0f)));

	verts.push_back(Vertex_PCU(bottomLeft, color, Vec2(0.0f, 0.0f)));
	verts.push_back(Vertex_PCU(topRight, color, Vec2(1.0f, 1.0f)));
	verts.push_back(Vertex_PCU(topLeft, color, Vec2(0.0f, 1.0f)));

}

//-----------------------------------------------------------------------------------------------
void AddVertsForAABB2D(std::vector<Vertex_PCU>& verts, AABB2 const& bounds, Rgba8 const& color, AABB2 UVs)
{
	Vec3 bottomLeft = Vec3(bounds.m_mins.x, bounds.m_mins.y, 0.0f);
	Vec3 bottomRight = Vec3(bounds.m_maxs.x, bounds.m_mins.y, 0.0f);
	Vec3 topLeft = Vec3(bounds.m_mins.x, bounds.m_maxs.y, 0.0f);
	Vec3 topRight = Vec3(bounds.m_maxs.x, bounds.m_maxs.y, 0.0f);

	verts.push_back(Vertex_PCU(bottomLeft, color, UVs.m_mins));
	verts.push_back(Vertex_PCU(bottomRight, color, Vec2(UVs.m_maxs.x, UVs.m_mins.y)));
	verts.push_back(Vertex_PCU(topRight, color, Vec2(UVs.m_maxs.x, UVs.m_maxs.y)));

	verts.push_back(Vertex_PCU(bottomLeft, color, Vec2(UVs.m_mins.x, UVs.m_mins.y)));
	verts.push_back(Vertex_PCU(topRight, color, Vec2(UVs.m_maxs.x, UVs.m_maxs.y)));
	verts.push_back(Vertex_PCU(topLeft, color, Vec2(UVs.m_mins.x, UVs.m_maxs.y)));
}

//-----------------------------------------------------------------------------------------------
void AddVertsForAABB2D(std::vector<Vertex_PCU>& verts, AABB2 const& bounds, Rgba8 const& color, Vec2& uvAtMins, Vec2& uvAtMaxs)
{
	Vec3 bottomLeft = Vec3(bounds.m_mins.x, bounds.m_mins.y, 0.0f);
	Vec3 bottomRight = Vec3(bounds.m_maxs.x, bounds.m_mins.y, 0.0f);
	Vec3 topLeft = Vec3(bounds.m_mins.x, bounds.m_maxs.y, 0.0f);
	Vec3 topRight = Vec3(bounds.m_maxs.x, bounds.m_maxs.y, 0.0f);

	verts.push_back(Vertex_PCU(bottomLeft, color, Vec2(uvAtMins.x, uvAtMins.y)));
	verts.push_back(Vertex_PCU(bottomRight, color, Vec2(uvAtMaxs.x, uvAtMins.y)));
	verts.push_back(Vertex_PCU(topRight, color, Vec2(uvAtMaxs.x, uvAtMaxs.y)));

	verts.push_back(Vertex_PCU(bottomLeft, color, Vec2(uvAtMins.x, uvAtMins.y)));
	verts.push_back(Vertex_PCU(topRight, color, Vec2(uvAtMaxs.x, uvAtMaxs.y)));
	verts.push_back(Vertex_PCU(topLeft, color, Vec2(uvAtMins.x, uvAtMaxs.y)));
}

//-----------------------------------------------------------------------------------------------
void AddVertsForOBB2D(std::vector<Vertex_PCU>& verts, OBB2 const& box, Rgba8 const& color)
{
	Vec2 jBasisNormal = box.m_iBasisNormal.GetRotated90Degrees();
	Vec2 right = box.m_iBasisNormal * box.m_halfDimensions.x;
	Vec2 up = jBasisNormal * box.m_halfDimensions.y;

	Vec2 topRight = box.m_center + right + up;
	Vec2 topLeft = box.m_center - right + up;
	Vec2 bottomLeft = box.m_center - right - up;
	Vec2 bottomRight = box.m_center + right - up;

	verts.push_back(Vertex_PCU(Vec3(bottomLeft.x, bottomLeft.y, 0.0f), color, Vec2(0.0f, 0.0f)));
	verts.push_back(Vertex_PCU(Vec3(bottomRight.x, bottomRight.y, 0.0f), color, Vec2(0.0f, 0.0f)));
	verts.push_back(Vertex_PCU(Vec3(topRight.x, topRight.y, 0.0f), color, Vec2(0.0f, 0.0f)));

	verts.push_back(Vertex_PCU(Vec3(bottomLeft.x, bottomLeft.y, 0.0f), color, Vec2(0.0f, 0.0f)));
	verts.push_back(Vertex_PCU(Vec3(topRight.x, topRight.y, 0.0f), color, Vec2(0.0f, 0.0f)));
	verts.push_back(Vertex_PCU(Vec3(topLeft.x, topLeft.y, 0.0f), color, Vec2(0.0f, 0.0f)));
}

//-----------------------------------------------------------------------------------------------
void AddVertsForDisc2D(std::vector<Vertex_PCU>& verts, Vec2 const& center, float radius, Rgba8 const& color)
{
	constexpr int NUM_SIDES = 64;
	constexpr int NUM_TRIS = NUM_SIDES;
	constexpr int NUM_VERTS = 3 * NUM_TRIS;

	float degreesPerDebrisSide = 360.0f / static_cast<float>(NUM_SIDES);
	Vec2 localVertPositions[NUM_SIDES];

	for (int sideIndex = 0; sideIndex < NUM_SIDES; sideIndex++)
	{
		float degrees = degreesPerDebrisSide * static_cast<float>(sideIndex);

		localVertPositions[sideIndex].x = center.x + radius * CosDegrees(degrees);
		localVertPositions[sideIndex].y = center.y + radius * SinDegrees(degrees);
	}

	for (int triIndex = 0; triIndex < NUM_TRIS; triIndex++) //this loop now goes in using the radii information and starts forming the randomly generated triangles
	{
		int startRadiusIndex = triIndex;
		int endRadiusIndex = (triIndex + 1) % NUM_SIDES;

		Vec2 secondVertOffset = localVertPositions[startRadiusIndex];
		Vec2 thirdVertOffset = localVertPositions[endRadiusIndex];

		//sets local verts based on vertex offsets for each triangle
		verts.push_back(Vertex_PCU(Vec3(center.x, center.y, 0.0f), color, Vec2(0.0f, 0.0f)));
		verts.push_back(Vertex_PCU(Vec3(secondVertOffset.x, secondVertOffset.y, 0.0f), color, Vec2(0.0f, 0.0f)));
		verts.push_back(Vertex_PCU(Vec3(thirdVertOffset.x, thirdVertOffset.y, 0.0f), color, Vec2(0.0f, 0.0f)));
	}
}

//-----------------------------------------------------------------------------------------------
void AddVertsForDisc3D(std::vector<Vertex_PNCU>& verts, Vec3 const& center, float radius, int numSides, Rgba8 const& color)
{
	int NUM_SIDES = numSides;
	int NUM_TRIS = NUM_SIDES;

	float degreesPerDebrisSide = 360.0f / static_cast<float>(NUM_SIDES);
	std::vector<Vec3> localVertPositions;
	localVertPositions.resize(NUM_SIDES);

	for (int sideIndex = 0; sideIndex < NUM_SIDES; sideIndex++)
	{
		float degrees = degreesPerDebrisSide * static_cast<float>(sideIndex);

		localVertPositions[sideIndex].x = center.x + radius * CosDegrees(degrees);
		localVertPositions[sideIndex].y = center.y + radius * SinDegrees(degrees);
	}

	for (int triIndex = 0; triIndex < NUM_TRIS; triIndex++) //this loop now goes in using the radii information and starts forming the randomly generated triangles
	{
		int startRadiusIndex = triIndex;
		int endRadiusIndex = (triIndex + 1) % NUM_SIDES;

		Vec3 secondVertOffset = localVertPositions[startRadiusIndex];
		Vec3 thirdVertOffset = localVertPositions[endRadiusIndex];

		//sets local verts based on vertex offsets for each triangle
		Vec3 normal = CrossProduct3D(secondVertOffset - center, thirdVertOffset - secondVertOffset).GetNormalized();
		verts.push_back(Vertex_PNCU(center, normal, color, Vec2(0.0f, 0.0f)));
		verts.push_back(Vertex_PNCU(secondVertOffset, normal, color, Vec2(0.0f, 0.0f)));
		verts.push_back(Vertex_PNCU(thirdVertOffset, normal, color, Vec2(0.0f, 0.0f)));
	}
}

//-----------------------------------------------------------------------------------------------
void AddVertsForRing3D(std::vector<Vertex_PNCU>& verts, Vec3 const& center, float radius, float thickness, int numSides, Rgba8 const& color)
{
	int NUM_SIDES = numSides;
	float DEGREES_PER_SIDE = 360.f / static_cast<float>(NUM_SIDES);

	float halfThickness = .5f * thickness;
	float innerRadius = radius - halfThickness;
	float outerRadius = radius + halfThickness;

	for (int sideCounter = 0; sideCounter < NUM_SIDES; sideCounter++)
	{
		//computer angle related terms
		float startDegrees = DEGREES_PER_SIDE * static_cast<float>(sideCounter);
		float endDegrees = DEGREES_PER_SIDE * static_cast<float>(sideCounter + 1);
		float cosStart = CosDegrees(startDegrees);
		float sinStart = SinDegrees(startDegrees);
		float cosEnd = CosDegrees(endDegrees);
		float sinEnd = SinDegrees(endDegrees);

		//Compute inner and out positions
		Vec3 innerStartPosition = Vec3(center.x + innerRadius * cosStart, center.y + innerRadius * sinStart, center.z);
		Vec3 outerStartPosition = Vec3(center.x + outerRadius * cosStart, center.y + outerRadius * sinStart, center.z);
		Vec3 outerEndPosition = Vec3(center.x + outerRadius * cosEnd, center.y + outerRadius * sinEnd, center.z);
		Vec3 innerEndPosition = Vec3(center.x + innerRadius * cosEnd, center.y + innerRadius * sinEnd, center.z);

		//initializes the vertexes (inner first, then outer second)
		Vec3 normal = CrossProduct3D(innerStartPosition - innerEndPosition, outerStartPosition - innerStartPosition).GetNormalized();
		verts.push_back(Vertex_PNCU(innerEndPosition, normal, color, Vec2(0.f, 0.f)));
		verts.push_back(Vertex_PNCU(innerStartPosition, normal, color, Vec2(0.f, 0.f)));
		verts.push_back(Vertex_PNCU(outerStartPosition, normal, color, Vec2(0.f, 0.f)));
		verts.push_back(Vertex_PNCU(innerEndPosition, normal, color, Vec2(0.f, 0.f)));
		verts.push_back(Vertex_PNCU(outerStartPosition, normal, color, Vec2(0.f, 0.f)));
		verts.push_back(Vertex_PNCU(outerEndPosition, normal, color, Vec2(0.f, 0.f)));
	}
}

//-----------------------------------------------------------------------------------------------
void AddVertsForRing3D(std::vector<Vertex_PNCU>& verts, std::vector<int>& indices, Vec3 const& center, float radius, float thickness, int numSides, Rgba8 const& color)
{
	int NUM_SIDES = numSides;
	float DEGREES_PER_SIDE = 360.f / static_cast<float>(NUM_SIDES);

	float halfThickness = .5f * thickness;
	float innerRadius = radius - halfThickness;
	float outerRadius = radius + halfThickness;

	int previousInnerVertIndex = 0;
	int previousOuterVertIndex = 0;
	int originalInnerVertIndex = 0;
	int originalOuterVertIndex = 0;
	for (int sideCounter = 0; sideCounter < NUM_SIDES; sideCounter++)
	{
		//computer angle related terms
		float startDegrees = DEGREES_PER_SIDE * static_cast<float>(sideCounter);
		float endDegrees = DEGREES_PER_SIDE * static_cast<float>(sideCounter + 1);
		float cosStart = CosDegrees(startDegrees);
		float sinStart = SinDegrees(startDegrees);
		float cosEnd = CosDegrees(endDegrees);
		float sinEnd = SinDegrees(endDegrees);

		//Compute inner and out positions
		Vec3 innerStartPosition = Vec3(center.x + innerRadius * cosStart, center.y + innerRadius * sinStart, center.z);
		Vec3 outerStartPosition = Vec3(center.x + outerRadius * cosStart, center.y + outerRadius * sinStart, center.z);
		Vec3 outerEndPosition = Vec3(center.x + outerRadius * cosEnd, center.y + outerRadius * sinEnd, center.z);
		Vec3 innerEndPosition = Vec3(center.x + innerRadius * cosEnd, center.y + innerRadius * sinEnd, center.z);

		/*verts.push_back(Vertex_PNCU(innerEndPosition, normal, color, Vec2(0.f, 0.f)));
		verts.push_back(Vertex_PNCU(innerStartPosition, normal, color, Vec2(0.f, 0.f)));
		verts.push_back(Vertex_PNCU(outerStartPosition, normal, color, Vec2(0.f, 0.f)));
		verts.push_back(Vertex_PNCU(innerEndPosition, normal, color, Vec2(0.f, 0.f)));
		verts.push_back(Vertex_PNCU(outerStartPosition, normal, color, Vec2(0.f, 0.f)));
		verts.push_back(Vertex_PNCU(outerEndPosition, normal, color, Vec2(0.f, 0.f)));*/

		//initializes the vertexes (inner first, then outer second)
		Vec3 normal = CrossProduct3D(innerStartPosition - innerEndPosition, outerStartPosition - innerStartPosition).GetNormalized();
		
		if(sideCounter == 0)
		{
			verts.push_back(Vertex_PNCU(innerEndPosition, normal, color, Vec2(0.f, 0.f)));
			verts.push_back(Vertex_PNCU(innerStartPosition, normal, color, Vec2(0.f, 0.f)));
			verts.push_back(Vertex_PNCU(outerEndPosition, normal, color, Vec2(0.f, 0.f)));
			verts.push_back(Vertex_PNCU(outerStartPosition, normal, color, Vec2(0.f, 0.f)));
			
			indices.push_back(int(verts.size()) - 4);
			indices.push_back(int(verts.size()) - 3);
			indices.push_back(int(verts.size()) - 1);
			indices.push_back(int(verts.size()) - 4);
			indices.push_back(int(verts.size()) - 1);
			indices.push_back(int(verts.size()) - 2);

			previousInnerVertIndex = int(verts.size()) - 4;
			previousOuterVertIndex = int(verts.size()) - 2;
			originalInnerVertIndex = int(verts.size()) - 3;
			originalOuterVertIndex = int(verts.size()) - 1;
		}
		else if(sideCounter != NUM_SIDES - 1)
		{
			verts.push_back(Vertex_PNCU(innerEndPosition, normal, color, Vec2(0.f, 0.f)));
			verts.push_back(Vertex_PNCU(outerEndPosition, normal, color, Vec2(0.f, 0.f)));
			
			indices.push_back(int(verts.size()) - 2);
			indices.push_back(previousInnerVertIndex);
			indices.push_back(previousOuterVertIndex);
			indices.push_back(int(verts.size()) - 2);
			indices.push_back(previousOuterVertIndex);
			indices.push_back(int(verts.size()) - 1);

			previousInnerVertIndex = int(verts.size()) - 2;
			previousOuterVertIndex = int(verts.size()) - 1;
		}
		else
		{
			indices.push_back(originalInnerVertIndex);
			indices.push_back(previousInnerVertIndex);
			indices.push_back(previousOuterVertIndex);
			indices.push_back(originalInnerVertIndex);
			indices.push_back(previousOuterVertIndex);
			indices.push_back(originalOuterVertIndex);
		}
	}
}

//-----------------------------------------------------------------------------------------------
void AddVertsForLineSegment2D(std::vector<Vertex_PCU>& verts, LineSegment2 const& lineSegment, float thickness, Rgba8 const& color)
{
	constexpr int NUM_VERTS = 6;
	float radius = thickness * .5f;
	Vec2 displacement = lineSegment.m_start - lineSegment.m_end;
	Vec2 forwardDirection = displacement.GetNormalized();
	Vec2 forwardStart = forwardDirection * radius;
	Vec2 leftStart = forwardStart.GetRotated90Degrees();
	Vec2 endLeft = lineSegment.m_end - forwardStart + leftStart;
	Vec2 endRight = lineSegment.m_end - forwardStart - leftStart;
	Vec2 startLeft = lineSegment.m_start + forwardStart + leftStart;
	Vec2 startRight = lineSegment.m_start + forwardStart - leftStart;

	//forms the vertices's based off calculations
	verts.push_back(Vertex_PCU(Vec3(startLeft.x, startLeft.y, 0.f), color, Vec2(0.f, 0.f)));
	verts.push_back(Vertex_PCU(Vec3(endRight.x, endRight.y, 0.f), color, Vec2(0.f, 0.f)));
	verts.push_back(Vertex_PCU(Vec3(startRight.x, startRight.y, 0.f), color, Vec2(0.f, 0.f)));
	verts.push_back(Vertex_PCU(Vec3(endRight.x, endRight.y, 0.f), color, Vec2(0.f, 0.f)));
	verts.push_back(Vertex_PCU(Vec3(startLeft.x, startLeft.y, 0.f), color, Vec2(0.f, 0.f)));
	verts.push_back(Vertex_PCU(Vec3(endLeft.x, endLeft.y, 0.f), color, Vec2(0.f, 0.f)));
}

//-----------------------------------------------------------------------------------------------
void AddVertsForLineSegment3D(std::vector<Vertex_PCU>& verts, Vec3 const& start, Vec3 const& end, float thickness, Rgba8 const& color)
{
	Vec3 iBasis = (end - start).GetNormalized();
	Vec3 jBasis = CrossProduct3D(Vec3(0.0f, 0.0f, 1.0f), iBasis);
	if (jBasis == Vec3(0.0f, 0.0f, 0.0f))
	{
		jBasis = Vec3(0.0f, 1.0f, 0.0f);
	}
	else
	{
		jBasis.Normalize();
	}
	Vec3 kBasis = CrossProduct3D(iBasis, jBasis).GetNormalized();
	float distance = (end - start).GetLength();
	AddVertsForCylinderZ3D(verts, Vec2(0.0f, 0.0f), FloatRange(distance, 0.0f), thickness, 10, color);

	Mat44 transform;
	transform.SetIJK3D(kBasis, jBasis, iBasis);
	transform.SetTranslation3D(start);
	TransformVertexArray3D(verts, transform);
}

//-----------------------------------------------------------------------------------------------
void AddVertsForCapsule2D(std::vector<Vertex_PCU>& verts, Capsule2 const& capsule, Rgba8 const& color)
{
	Vec2 start = capsule.m_bone.m_start;
	Vec2 end = capsule.m_bone.m_end;
	float radius = capsule.m_radius;

	Vec2 forwardNormal = (end - start).GetNormalized();
	Vec2 leftNormal = forwardNormal.GetRotated90Degrees();
	Vec2 startLeft = start + (leftNormal * radius);
	Vec2 startRight = start - (leftNormal * radius);
	Vec2 endLeft = end + (leftNormal * radius);
	Vec2 endRight = end - (leftNormal * radius);

	verts.push_back(Vertex_PCU(Vec3(startRight.x, startRight.y, 0.0f), color, Vec2(0.0f, 0.0f)));
	verts.push_back(Vertex_PCU(Vec3(endRight.x, endRight.y, 0.0f), color, Vec2(0.0f, 0.0f)));
	verts.push_back(Vertex_PCU(Vec3(endLeft.x, endLeft.y, 0.0f), color, Vec2(0.0f, 0.0f)));
	verts.push_back(Vertex_PCU(Vec3(endLeft.x, endLeft.y, 0.0f), color, Vec2(0.0f, 0.0f)));
	verts.push_back(Vertex_PCU(Vec3(startLeft.x, startLeft.y, 0.0f), color, Vec2(0.0f, 0.0f)));
	verts.push_back(Vertex_PCU(Vec3(startRight.x, startRight.y, 0.0f), color, Vec2(0.0f, 0.0f)));
	AddVertsForSector2D(verts, start, forwardNormal * -1.0f, 180.0f, radius, color);
	AddVertsForSector2D(verts, end, forwardNormal, 180.0f, radius, color);
}

//-----------------------------------------------------------------------------------------------
void AddVertsForSector2D(std::vector<Vertex_PCU>& verts, Vec2 const& sectorTip, Vec2 const& sectorForwardNormal, float sectorApertureDegrees, float sectorRadius, Rgba8 const& color)
{
	constexpr int NUM_SIDES = 32;
	constexpr int NUM_TRIS = NUM_SIDES;
	constexpr int NUM_VERTS = 3 * NUM_TRIS;

	float degreesPerSide = sectorApertureDegrees / static_cast<float>(NUM_SIDES);
	float startDegrees = sectorForwardNormal.GetOrientationDegrees() - 90.0f;
	Vec3 center = Vec3(sectorTip.x, sectorTip.y, 0.0f);
	
	for (int sideIndex = 0; sideIndex < NUM_SIDES; sideIndex++)
	{
		Vec3 startVertex = Vec3(center.x + sectorRadius * CosDegrees(startDegrees), center.y + sectorRadius * SinDegrees(startDegrees), 0.0f);
		Vec3 endVertex = Vec3(center.x + sectorRadius * CosDegrees(startDegrees + degreesPerSide), center.y + sectorRadius * SinDegrees(startDegrees + degreesPerSide), 0.0f);

		verts.push_back(Vertex_PCU(startVertex, color, Vec2(0.0f, 0.0f)));
		verts.push_back(Vertex_PCU(endVertex, color, Vec2(0.0f, 0.0f)));
		verts.push_back(Vertex_PCU(center, color, Vec2(0.0f, 0.0f)));
		startDegrees += degreesPerSide;
	}
}

//-----------------------------------------------------------------------------------------------
void AddVertsForArrow2D(std::vector<Vertex_PCU>& verts, Vec2 tailPos, Vec2 tipPos, float arrowSize, float lineThickness, Rgba8 const& color)
{
	//Dont Draw 0 length ray
	if ((tailPos - tipPos).GetLength() == 0.0f)
	{
		return;
	}

	LineSegment2 rayLine;
	rayLine.m_start = tailPos;
	rayLine.m_end = tipPos;

	AddVertsForLineSegment2D(verts, rayLine, lineThickness, color);

	Vec2 direction = tipPos - tailPos;
	float orientation = direction.GetOrientationDegrees() - 90.0f;

	std::vector<Vertex_PCU> arrowTipVerts;
	arrowTipVerts.reserve(3);
	arrowTipVerts.push_back(Vertex_PCU(Vec3(0.0f, 0.0f, 0.0f), color, Vec2(0.0f, 0.0f)));
	arrowTipVerts.push_back(Vertex_PCU(Vec3(-1.0f, -1.0f, 0.0f), color, Vec2(0.0f, 0.0f)));
	arrowTipVerts.push_back(Vertex_PCU(Vec3(1.0f, -1.0f, 0.0f), color, Vec2(0.0f, 0.0f)));

	TransformVertexArrayXY3D(3, arrowTipVerts.data(), arrowSize, orientation, tipPos);

	verts.push_back(arrowTipVerts[0]);
	verts.push_back(arrowTipVerts[1]);
	verts.push_back(arrowTipVerts[2]);
}

//-----------------------------------------------------------------------------------------------
void AddVertsForQuad3D(std::vector<Vertex_PCU>& verts, const Vec3& bottomLeft, const Vec3& bottomRight, const Vec3& topLeft, const Vec3& topRight, const Rgba8& color, const AABB2& UVs)
{
	verts.push_back(Vertex_PCU(bottomLeft, color, UVs.m_mins));
	verts.push_back(Vertex_PCU(bottomRight, color, Vec2(UVs.m_maxs.x, UVs.m_mins.y)));
	verts.push_back(Vertex_PCU(topRight, color, UVs.m_maxs));

	verts.push_back(Vertex_PCU(bottomLeft, color, UVs.m_mins));
	verts.push_back(Vertex_PCU(topRight, color, UVs.m_maxs));
	verts.push_back(Vertex_PCU(topLeft, color, Vec2(UVs.m_mins.x, UVs.m_maxs.y)));
}

//-----------------------------------------------------------------------------------------------
void AddVertsForQuad3D(std::vector<Vertex_PCU>& verts, std::vector<unsigned int>& indexes, const Vec3& bottomLeft, const Vec3& bottomRight, const Vec3& topLeft, const Vec3& topRight, const Rgba8& color, const AABB2& UVs)
{
	//verts
	verts.push_back(Vertex_PCU(bottomLeft, color, UVs.m_mins));
	verts.push_back(Vertex_PCU(bottomRight, color, Vec2(UVs.m_maxs.x, UVs.m_mins.y)));
	verts.push_back(Vertex_PCU(topRight, color, UVs.m_maxs));
	verts.push_back(Vertex_PCU(topLeft, color, Vec2(UVs.m_mins.x, UVs.m_maxs.y)));

	//indexes
	indexes.push_back(static_cast<int>(verts.size()) - 4);
	indexes.push_back(static_cast<int>(verts.size()) - 3);
	indexes.push_back(static_cast<int>(verts.size()) - 2);
	indexes.push_back(static_cast<int>(verts.size()) - 4);
	indexes.push_back(static_cast<int>(verts.size()) - 2);
	indexes.push_back(static_cast<int>(verts.size()) - 1);
}

//-----------------------------------------------------------------------------------------------
void AddVertsForAABB3D(std::vector<Vertex_PCU>& verts, const AABB3& bounds, const Rgba8& color, const AABB2& UVs)
{
	float minX = bounds.m_mins.x;
	float minY = bounds.m_mins.y;
	float minZ = bounds.m_mins.z;
	float maxX = bounds.m_maxs.x;
	float maxY = bounds.m_maxs.y;
	float maxZ = bounds.m_maxs.z;

	AddVertsForQuad3D(verts, Vec3(maxX, minY, minZ), Vec3(maxX, maxY, minZ), Vec3(maxX, minY, maxZ), Vec3(maxX, maxY, maxZ), color, UVs);
	AddVertsForQuad3D(verts, Vec3(minX, maxY, minZ), Vec3(minX, minY, minZ), Vec3(minX, maxY, maxZ), Vec3(minX, minY, maxZ), color, UVs);
	AddVertsForQuad3D(verts, Vec3(maxX, maxY, minZ), Vec3(minX, maxY, minZ), Vec3(maxX, maxY, maxZ), Vec3(minX, maxY, maxZ), color, UVs);
	AddVertsForQuad3D(verts, Vec3(minX, minY, minZ), Vec3(maxX, minY, minZ), Vec3(minX, minY, maxZ), Vec3(maxX, minY, maxZ), color, UVs);
	AddVertsForQuad3D(verts, Vec3(minX, minY, maxZ), Vec3(maxX, minY, maxZ), Vec3(minX, maxY, maxZ), Vec3(maxX, maxY, maxZ), color, UVs);
	AddVertsForQuad3D(verts,  Vec3(minX, maxY, minZ), Vec3(maxX, maxY, minZ), Vec3(minX, minY, minZ), Vec3(maxX, minY, minZ), color, UVs);
}

//-----------------------------------------------------------------------------------------------
void AddVertsForSphere3D(std::vector<Vertex_PCU>& verts, const Vec3& center, float radius, const Rgba8& color, const AABB2& UVs, int numLatitudeSlices)
{
	float yawIncrementAmount = 360.0f / (numLatitudeSlices * 2.0f);
	float pitchIncrementAmount = 180.0f / numLatitudeSlices;
	for (int yawIndex = 0; yawIndex < numLatitudeSlices * 2.0f; yawIndex ++)
	{
		float minYaw = yawIndex * yawIncrementAmount;
		float maxYaw = minYaw + yawIncrementAmount;
		for (int pitchIndex = 0; pitchIndex < numLatitudeSlices; pitchIndex ++)
		{
			float minPitch = -90.0f + pitchIndex * pitchIncrementAmount;
			float maxPitch = minPitch + pitchIncrementAmount;
			Vec3 position = center;
			Vec3 topLeft = position.MakeFromPolarDegrees(minYaw, maxPitch, radius) + position;
			Vec3 topRight = position.MakeFromPolarDegrees(maxYaw, maxPitch, radius) + position;
			Vec3 bottomLeft = position.MakeFromPolarDegrees(minYaw, minPitch, radius) + position;
			Vec3 bottomRight = position.MakeFromPolarDegrees(maxYaw, minPitch, radius) + position;

			float minU = RangeMap(minYaw, 0.0f, 360.0f, UVs.m_mins.x, UVs.m_maxs.x);
			float maxU = RangeMap(maxYaw, 0.0f, 360.0f, UVs.m_mins.x, UVs.m_maxs.x);
			float minV = RangeMap(minPitch, -90.0f, 90.0f, UVs.m_maxs.y, UVs.m_mins.y);
			float maxV = RangeMap(maxPitch, -90.0f, 90.0f, UVs.m_maxs.y, UVs.m_mins.y);
			AABB2 quadUVs;
			quadUVs.m_mins = Vec2(minU, maxV);
			quadUVs.m_maxs = Vec2(maxU, minV);
			AddVertsForQuad3D(verts, topLeft, topRight, bottomLeft, bottomRight, color, quadUVs);
		}
	}
}

//-----------------------------------------------------------------------------------------------
void AddVertsForSphere3DReverseUVs(std::vector<Vertex_PCU>& verts, const Vec3& center, float radius, const Rgba8& color, const AABB2& UVs, int numLatitudeSlices)
{
	float yawIncrementAmount = 360.0f / (numLatitudeSlices * 2.0f);
	float pitchIncrementAmount = 180.0f / numLatitudeSlices;
	for (int yawIndex = 0; yawIndex < numLatitudeSlices * 2.0f; yawIndex++)
	{
		float minYaw = yawIndex * yawIncrementAmount;
		float maxYaw = minYaw + yawIncrementAmount;
		for (int pitchIndex = 0; pitchIndex < numLatitudeSlices; pitchIndex++)
		{
			float minPitch = -90.0f + pitchIndex * pitchIncrementAmount;
			float maxPitch = minPitch + pitchIncrementAmount;
			Vec3 position = center;
			Vec3 topLeft = position.MakeFromPolarDegrees(minYaw, maxPitch, radius) + position;
			Vec3 topRight = position.MakeFromPolarDegrees(maxYaw, maxPitch, radius) + position;
			Vec3 bottomLeft = position.MakeFromPolarDegrees(minYaw, minPitch, radius) + position;
			Vec3 bottomRight = position.MakeFromPolarDegrees(maxYaw, minPitch, radius) + position;

			float minU = RangeMap(minYaw, 0.0f, 360.0f, UVs.m_mins.x, UVs.m_maxs.x);
			float maxU = RangeMap(maxYaw, 0.0f, 360.0f, UVs.m_mins.x, UVs.m_maxs.x);
			float minV = RangeMap(minPitch, -90.0f, 90.0f, UVs.m_maxs.y, UVs.m_mins.y);
			float maxV = RangeMap(maxPitch, -90.0f, 90.0f, UVs.m_maxs.y, UVs.m_mins.y);
			AABB2 quadUVs;
			quadUVs.m_mins = Vec2(minU, maxV);
			quadUVs.m_maxs = Vec2(maxU, minV);
			AddVertsForQuad3D(verts, topRight, topLeft, bottomRight, bottomLeft, color, quadUVs);
		}
	}
}

//-----------------------------------------------------------------------------------------------
void AddVertsForRing2D(std::vector<Vertex_PCU>& verts, Vec2 const& center, float radius, float thickness, Rgba8 const& color)
{
	constexpr int NUM_SIDES = 32;
	constexpr int NUM_TRIS = 2 * NUM_SIDES;
	constexpr int NUM_VERTS = 3 * NUM_TRIS;
	constexpr float DEGREES_PER_SIDE = 360.f / static_cast<float>(NUM_SIDES);

	float halfThickness = .5f * thickness;
	float innerRadius = radius - halfThickness;
	float outerRadius = radius + halfThickness;

	for (int sideCounter = 0; sideCounter < NUM_SIDES; sideCounter++)
	{
		//computer angle related terms
		float startDegrees = DEGREES_PER_SIDE * static_cast<float>(sideCounter);
		float endDegrees = DEGREES_PER_SIDE * static_cast<float>(sideCounter + 1);
		float cosStart = CosDegrees(startDegrees);
		float sinStart = SinDegrees(startDegrees);
		float cosEnd = CosDegrees(endDegrees);
		float sinEnd = SinDegrees(endDegrees);

		//Compute inner and out positions
		Vec3 innerStartPosition = Vec3(center.x + innerRadius * cosStart, center.y + innerRadius * sinStart, 0.f);
		Vec3 outerStartPosition = Vec3(center.x + outerRadius * cosStart, center.y + outerRadius * sinStart, 0.f);
		Vec3 outerEndPosition = Vec3(center.x + outerRadius * cosEnd, center.y + outerRadius * sinEnd, 0.f);
		Vec3 innerEndPosition = Vec3(center.x + innerRadius * cosEnd, center.y + innerRadius * sinEnd, 0.f);

		//initializes the vertexes (inner first, then outer second)
		verts.push_back(Vertex_PCU(innerEndPosition, color, Vec2(0.f, 0.f)));
		verts.push_back(Vertex_PCU(innerStartPosition, color, Vec2(0.f, 0.f)));
		verts.push_back(Vertex_PCU(outerStartPosition, color, Vec2(0.f, 0.f)));
		verts.push_back(Vertex_PCU(innerEndPosition, color, Vec2(0.f, 0.f)));
		verts.push_back(Vertex_PCU(outerStartPosition, color, Vec2(0.f, 0.f)));
		verts.push_back(Vertex_PCU(outerEndPosition, color, Vec2(0.f, 0.f)));
	}
}

//-----------------------------------------------------------------------------------------------
void AddVertsForCylinderZ3D(std::vector<Vertex_PCU>& verts, Vec2 const& centerXY, FloatRange const& minMaxZ, float radius, float numSlices, Rgba8 const& tint, AABB2 const& UVs)
{
	float yawIncrementAmount = 360.0f / (static_cast<float>(numSlices) * 2.0f);
  	for (float yaw = 0.0f; yaw < 360.0f; yaw += yawIncrementAmount)
	{
		float minYaw = yaw;
		float maxYaw = yaw + yawIncrementAmount;
		if (360.f - yaw < yawIncrementAmount)
		{
			minYaw = yaw;
			maxYaw = yaw + (360.0f - yaw);
		}
		Vec2 position = centerXY;
		Vec3 topLeft = Vec3(position.MakeFromPolarDegrees(minYaw, radius).x + position.x, position.MakeFromPolarDegrees(minYaw, radius).y + position.y, minMaxZ.m_max);
		Vec3 topRight = Vec3(position.MakeFromPolarDegrees(maxYaw, radius).x + position.x, position.MakeFromPolarDegrees(maxYaw, radius).y + position.y, minMaxZ.m_max);
		Vec3 bottomLeft = Vec3(position.MakeFromPolarDegrees(minYaw, radius).x + position.x, position.MakeFromPolarDegrees(minYaw, radius).y + position.y, minMaxZ.m_min);
		Vec3 bottomRight = Vec3(position.MakeFromPolarDegrees(maxYaw, radius).x + position.x, position.MakeFromPolarDegrees(maxYaw, radius).y + position.y, minMaxZ.m_min);

		float minU = RangeMap(minYaw, 0.0f, 360.0f, UVs.m_mins.x, UVs.m_maxs.x);
		float maxU = RangeMap(maxYaw, 0.0f, 360.0f, UVs.m_mins.x, UVs.m_maxs.x);
		float minV = RangeMap(minMaxZ.m_min, minMaxZ.m_min, minMaxZ.m_max, UVs.m_mins.y, UVs.m_maxs.y);
		float maxV = RangeMap(minMaxZ.m_max, minMaxZ.m_min, minMaxZ.m_max, UVs.m_mins.y, UVs.m_maxs.y);
		AABB2 quadUVs;
		quadUVs.m_mins = Vec2(minU, minV);
		quadUVs.m_maxs = Vec2(maxU, maxV);
		AddVertsForQuad3D(verts, bottomLeft, bottomRight, topLeft, topRight, tint, quadUVs);

		//Top and Bottom of Cylinder
		Vec3 topCenter = Vec3(position.x, position.y, minMaxZ.m_max);
		Vec3 bottomCenter = Vec3(position.x, position.y, minMaxZ.m_min);
		Vec2 topLeftUV = Vec2(RangeMap(topLeft.x, -radius + position.x, radius + position.x, 0.0f, 1.0f), RangeMap(topLeft.y, -radius + position.y, radius + position.y, 0.0f, 1.0f));
		Vec2 topRightUV = Vec2(RangeMap(topRight.x, -radius + position.x, radius + position.x, 0.0f, 1.0f), RangeMap(topRight.y, -radius + position.y, radius + position.y, 0.0f, 1.0f));
		Vec2 bottomLeftUV = Vec2(RangeMap(bottomLeft.x, -radius + position.x, radius + position.x, 0.0f, 1.0f), RangeMap(bottomLeft.y, -radius + position.y, radius + position.y, 1.0f, 0.0f));
		Vec2 bottomRightUV = Vec2(RangeMap(bottomRight.x, -radius + position.x, radius + position.x, 0.0f, 1.0f), RangeMap(bottomRight.y, -radius + position.y, radius + position.y, 1.0f, 0.0f));

		//Top
		verts.push_back(Vertex_PCU(topCenter, tint, Vec2(0.5f, 0.5f)));
		verts.push_back(Vertex_PCU(topLeft, tint, topLeftUV));
		verts.push_back(Vertex_PCU(topRight, tint, topRightUV));
		//Bottom
		verts.push_back(Vertex_PCU(bottomCenter, tint, Vec2(0.5f, 0.5f)));
		verts.push_back(Vertex_PCU(bottomRight, tint, bottomRightUV));
		verts.push_back(Vertex_PCU(bottomLeft, tint, bottomLeftUV));
	}
}

//-----------------------------------------------------------------------------------------------
void AddVertsForCylinderZ3DReverseUVsSidesOnly(std::vector<Vertex_PCU>& verts, Vec2 const& centerXY, FloatRange const& minMaxZ, float radius, float numSlices, Rgba8 const& tint, AABB2 const& UVs)
{
	float yawIncrementAmount = 360.0f / (static_cast<float>(numSlices) * 2.0f);
	for (float yaw = 0.0f; yaw < 360.0f; yaw += yawIncrementAmount)
	{
		float minYaw = yaw;
		float maxYaw = yaw + yawIncrementAmount;
		if (360.f - yaw < yawIncrementAmount)
		{
			minYaw = yaw;
			maxYaw = yaw + (360.0f - yaw);
		}
		Vec2 position = centerXY;
		Vec3 topLeft = Vec3(position.MakeFromPolarDegrees(minYaw, radius).x + position.x, position.MakeFromPolarDegrees(minYaw, radius).y + position.y, minMaxZ.m_max);
		Vec3 topRight = Vec3(position.MakeFromPolarDegrees(maxYaw, radius).x + position.x, position.MakeFromPolarDegrees(maxYaw, radius).y + position.y, minMaxZ.m_max);
		Vec3 bottomLeft = Vec3(position.MakeFromPolarDegrees(minYaw, radius).x + position.x, position.MakeFromPolarDegrees(minYaw, radius).y + position.y, minMaxZ.m_min);
		Vec3 bottomRight = Vec3(position.MakeFromPolarDegrees(maxYaw, radius).x + position.x, position.MakeFromPolarDegrees(maxYaw, radius).y + position.y, minMaxZ.m_min);

		float minU = RangeMap(minYaw, 0.0f, 360.0f, UVs.m_mins.x, UVs.m_maxs.x);
		float maxU = RangeMap(maxYaw, 0.0f, 360.0f, UVs.m_mins.x, UVs.m_maxs.x);
		float minV = RangeMap(minMaxZ.m_min, minMaxZ.m_min, minMaxZ.m_max, UVs.m_mins.y, UVs.m_maxs.y);
		float maxV = RangeMap(minMaxZ.m_max, minMaxZ.m_min, minMaxZ.m_max, UVs.m_mins.y, UVs.m_maxs.y);
		AABB2 quadUVs;
		quadUVs.m_mins = Vec2(minU, minV);
		quadUVs.m_maxs = Vec2(maxU, maxV);
		AddVertsForQuad3D(verts, bottomRight, bottomLeft, topRight, topLeft, tint, quadUVs);
	}
}

//-----------------------------------------------------------------------------------------------
void AddVertsForConeZ3D(std::vector<Vertex_PCU>& verts, Vec2 const& centerXY, FloatRange const& minMaxZ, float radius, float numSlices, Rgba8 const& tint, AABB2 const& UVs)
{
	UVs;
	float yawIncrementAmount = 360.0f / (static_cast<float>(numSlices) * 2.0f);
	for (float yaw = 0.0f; yaw < 360.0f; yaw += yawIncrementAmount)
	{
		float minYaw = yaw;
		float maxYaw = yaw + yawIncrementAmount;
		Vec2 position = centerXY;
		Vec3 topLeft = Vec3(position.MakeFromPolarDegrees(minYaw, radius).x + position.x, position.MakeFromPolarDegrees(minYaw, radius).y + position.y, minMaxZ.m_max);
		Vec3 topRight = Vec3(position.MakeFromPolarDegrees(maxYaw, radius).x + position.x, position.MakeFromPolarDegrees(maxYaw, radius).y + position.y, minMaxZ.m_max);

		/*float minU = RangeMap(minYaw, 0.0f, 360.0f, UVs.m_mins.x, UVs.m_maxs.x);
		float maxU = RangeMap(maxYaw, 0.0f, 360.0f, UVs.m_mins.x, UVs.m_maxs.x);
		float minV = RangeMap(minMaxZ.m_min, minMaxZ.m_min, minMaxZ.m_max, UVs.m_mins.y, UVs.m_maxs.y);
		float maxV = RangeMap(minMaxZ.m_max, minMaxZ.m_min, minMaxZ.m_max, UVs.m_mins.y, UVs.m_maxs.y);*/

		//Top and Bottom of Cylinder
		Vec3 topCenter = Vec3(position.x, position.y, minMaxZ.m_max);
		Vec3 bottomCenter = Vec3(position.x, position.y, minMaxZ.m_min);
		Vec2 bottomLeftUV = Vec2(RangeMap(topLeft.x, -radius, radius, 0.0f, 1.0f), RangeMap(topLeft.y, -radius, radius, 0.0f, 1.0f));
		Vec2 bottomRightUV = Vec2(RangeMap(topRight.x, -radius, radius, 0.0f, 1.0f), RangeMap(topRight.y, -radius, radius, 0.0f, 1.0f));

		//Sides
		verts.push_back(Vertex_PCU(topCenter, tint, Vec2(0.0f, 0.0f)));
		verts.push_back(Vertex_PCU(topLeft, tint, Vec2(0.0f, 0.0f)));
		verts.push_back(Vertex_PCU(topRight, tint, Vec2(0.0f, 0.0f)));
		//Bottom
		verts.push_back(Vertex_PCU(bottomCenter, tint, Vec2(0.5f, 0.5f)));
		verts.push_back(Vertex_PCU(topRight, tint, bottomRightUV));
		verts.push_back(Vertex_PCU(topLeft, tint, bottomLeftUV));
	}
}

//-----------------------------------------------------------------------------------------------
void AddVertsForArrow3D(std::vector<Vertex_PCU>& verts, Vec3 tailPos, Vec3 tipPos, float lineThickness, Rgba8 const& tint)
{
	Vec3 iBasis = (tipPos - tailPos).GetNormalized();
	Vec3 jBasis = CrossProduct3D(Vec3(0.0f, 0.0f, 1.0f), iBasis);
	if (jBasis == Vec3(0.0f, 0.0f, 0.0f))
	{
		jBasis = Vec3(0.0f, 1.0f, 0.0f);
	}
	else
	{
		jBasis.Normalize();
	}
	Vec3 kBasis = CrossProduct3D(iBasis, jBasis).GetNormalized();
	float distance = (tipPos - tailPos).GetLength();
	FloatRange cylinderRange = FloatRange(RangeMap(0.75f, 0.0f, 1.0f, 0.0f, distance), 0.0f);
	FloatRange coneRange = FloatRange(distance, RangeMap(0.75f, 0.0f, 1.0f, 0.0f, distance));
	int startVertPos = static_cast<int>(verts.size());
	AddVertsForCylinderZ3D(verts, Vec2(0.0f, 0.0f), cylinderRange, lineThickness * 0.5f, 8, tint);
	AddVertsForConeZ3D(verts, Vec2(0.0f, 0.0f), coneRange, lineThickness * 0.8f, 8, tint);
	int endVertPos = static_cast<int>(verts.size()) - 1;
	std::vector<Vertex_PCU> tempVerts(verts.begin() + startVertPos, verts.begin() + endVertPos + 1);

	Mat44 transform;
	transform.SetIJK3D(kBasis, jBasis, iBasis);
	transform.SetTranslation3D(tailPos);
	TransformVertexArray3D(tempVerts, transform);
	for (int vertIndex = startVertPos; vertIndex < verts.size(); vertIndex++)
	{
		verts[vertIndex] = tempVerts[vertIndex - startVertPos];
	}
}

//-----------------------------------------------------------------------------------------------
void AddVertsForCylinder3D(std::vector<Vertex_PCU>& verts, const Vec3& start, const Vec3& end, float radius, Rgba8 const& tint, AABB2 const& UVs, int numSlices)
{
	Vec3 iBasis = (end - start).GetNormalized();
	Vec3 jBasis = CrossProduct3D(Vec3(0.0f, 0.0f, 1.0f), iBasis);
	if (jBasis == Vec3(0.0f, 0.0f, 0.0f))
	{
		jBasis = Vec3(0.0f, 1.0f, 0.0f);
	}
	else
	{
		jBasis.Normalize();
	}
	Vec3 kBasis = CrossProduct3D(iBasis, jBasis).GetNormalized();
	float distance = (end - start).GetLength();
	int startVertPos = static_cast<int>(verts.size());
	AddVertsForCylinderZ3D(verts, Vec2(0.0f, 0.0f), FloatRange(distance, 0.0f), radius, static_cast<float>(numSlices), tint, UVs);
	int endVertPos = static_cast<int>(verts.size()) - 1;
	std::vector<Vertex_PCU> tempVerts(verts.begin() + startVertPos, verts.begin() + endVertPos + 1);

	Mat44 transform;
	transform.SetIJK3D(kBasis, jBasis, iBasis);
	transform.SetTranslation3D(start);
	TransformVertexArray3D(tempVerts, transform);
	for (int vertIndex = startVertPos; vertIndex < verts.size(); vertIndex++)
	{
		verts[vertIndex] = tempVerts[vertIndex - startVertPos];
	}
}

//-----------------------------------------------------------------------------------------------
void AddVertsForCone3D(std::vector<Vertex_PCU>& verts, const Vec3& start, const Vec3& end, float radius, Rgba8 const& tint, AABB2 const& UVs, int numSlices)
{
	Vec3 iBasis = (end - start).GetNormalized();
	Vec3 jBasis = CrossProduct3D(Vec3(0.0f, 0.0f, 1.0f), iBasis);
	if (jBasis == Vec3(0.0f, 0.0f, 0.0f))
	{
		jBasis = Vec3(0.0f, 1.0f, 0.0f);
	}
	else
	{
		jBasis.Normalize();
	}
	Vec3 kBasis = CrossProduct3D(iBasis, jBasis).GetNormalized();
	float distance = (end - start).GetLength();
	int startVertPos = static_cast<int>(verts.size());
	AddVertsForConeZ3D(verts, Vec2(0.0f, 0.0f), FloatRange(distance, 0.0f), radius, static_cast<float>(numSlices), tint, UVs);
	int endVertPos = static_cast<int>(verts.size()) - 1;
	std::vector<Vertex_PCU> tempVerts(verts.begin() + startVertPos, verts.begin() + endVertPos + 1);

	Mat44 transform;
	transform.SetIJK3D(kBasis, jBasis, iBasis);
	transform.SetTranslation3D(start);
	TransformVertexArray3D(tempVerts, transform);
	for (int vertIndex = startVertPos; vertIndex < verts.size(); vertIndex++)
	{
		verts[vertIndex] = tempVerts[vertIndex - startVertPos];
	}
}

//-----------------------------------------------------------------------------------------------
AABB2 GetVertexBounds2D(const std::vector<Vertex_PCU>& verts)
{
	float minX = 100000000.0f;
	float minY = 100000000.0f;
	float maxX = 0.0f;
	float maxY = 0.0f;

	for (int vertIndex = 0; vertIndex < verts.size(); vertIndex++)
	{
		if (verts[vertIndex].m_position.x < minX)
		{
			minX = verts[vertIndex].m_position.x;
		}
		if (verts[vertIndex].m_position.y < minY)
		{
			minY = verts[vertIndex].m_position.y;
		}
		if (verts[vertIndex].m_position.x > maxX)
		{
			maxX = verts[vertIndex].m_position.x;
		}
		if (verts[vertIndex].m_position.y > maxY)
		{
			maxY = verts[vertIndex].m_position.y;
		}
	}

	AABB2 bounds;
	bounds.m_mins = Vec2(minX, minY);
	bounds.m_mins = Vec2(maxX, maxY);
	return bounds;
}

//-----------------------------------------------------------------------------------------------
void AddVertsForRoundedQuad3D(std::vector<Vertex_PNCU>& verts, const Vec3& bottomLeft, const Vec3& bottomRight, const Vec3& topLeft, const Vec3& topRight, const Rgba8& color, AABB2 const& UVs)
{
	Vec3 topMiddle = (topRight + topLeft) * 0.5f;
	Vec3 bottomMiddle = (bottomRight + bottomLeft) * 0.5f;
	Vec3 u = (bottomRight - bottomLeft);
	Vec3 v = (topRight - bottomLeft);
	Vec3 middleNormal = CrossProduct3D(u, v).GetNormalized();
	Vec3 bottomLeftNormal = (bottomLeft - bottomMiddle).GetNormalized();
	Vec3 bottomRightNormal = (bottomRight - bottomMiddle).GetNormalized();
	Vec3 topLeftNormal = (topLeft - topMiddle).GetNormalized();
	Vec3 topRightNormal = (topRight - topMiddle).GetNormalized();
	
	float rangeX = UVs.m_maxs.x - UVs.m_mins.x;

	verts.push_back(Vertex_PNCU(bottomLeft, bottomLeftNormal, color, UVs.m_mins));
	verts.push_back(Vertex_PNCU(bottomMiddle, middleNormal, color, Vec2(UVs.m_mins.x + rangeX * 0.5f, UVs.m_mins.y)));
	verts.push_back(Vertex_PNCU(topMiddle, middleNormal, color, Vec2(UVs.m_mins.x + rangeX * 0.5f, UVs.m_maxs.y)));

	verts.push_back(Vertex_PNCU(bottomLeft, bottomLeftNormal, color, UVs.m_mins));
	verts.push_back(Vertex_PNCU(topMiddle, middleNormal, color, Vec2(UVs.m_mins.x + rangeX * 0.5f, UVs.m_maxs.y)));
	verts.push_back(Vertex_PNCU(topLeft, topLeftNormal, color, Vec2(UVs.m_mins.x, UVs.m_maxs.y)));

	verts.push_back(Vertex_PNCU(bottomMiddle, middleNormal, color, Vec2(UVs.m_mins.x + rangeX * 0.5f, UVs.m_mins.y)));
	verts.push_back(Vertex_PNCU(bottomRight, bottomRightNormal, color, Vec2(UVs.m_maxs.x, UVs.m_mins.y)));
	verts.push_back(Vertex_PNCU(topRight, topRightNormal, color, UVs.m_maxs));

	verts.push_back(Vertex_PNCU(bottomMiddle, middleNormal, color, Vec2(UVs.m_mins.x + rangeX * 0.5f, UVs.m_mins.y)));
	verts.push_back(Vertex_PNCU(topRight, topRightNormal, color, UVs.m_maxs));
	verts.push_back(Vertex_PNCU(topMiddle, middleNormal, color, Vec2(UVs.m_mins.x + rangeX * 0.5f, UVs.m_maxs.y)));
}

//-----------------------------------------------------------------------------------------------
void AddVertsForQuad3D(std::vector<Vertex_PNCU>& verts, std::vector<unsigned int>& indexes, const Vec3& bottomLeft, const Vec3& bottomRight, const Vec3& topLeft, const Vec3& topRight, const Rgba8& color, AABB2 const& UVs)
{
	Vec3 u = (bottomRight - bottomLeft).GetNormalized();
	Vec3 v = (topRight - bottomLeft).GetNormalized();
	Vec3 normal = CrossProduct3D(u, v).GetNormalized();

	//verts
	verts.push_back(Vertex_PNCU(bottomLeft, normal, color, UVs.m_mins));
	verts.push_back(Vertex_PNCU(bottomRight, normal, color, Vec2(UVs.m_maxs.x, UVs.m_mins.y)));
	verts.push_back(Vertex_PNCU(topRight, normal, color, UVs.m_maxs));
	verts.push_back(Vertex_PNCU(topLeft, normal, color, Vec2(UVs.m_mins.x, UVs.m_maxs.y)));

	//indexes
	indexes.push_back(static_cast<int>(verts.size()) - 4);
	indexes.push_back(static_cast<int>(verts.size()) - 3);
	indexes.push_back(static_cast<int>(verts.size()) - 2);
	indexes.push_back(static_cast<int>(verts.size()) - 4);
	indexes.push_back(static_cast<int>(verts.size()) - 2);
	indexes.push_back(static_cast<int>(verts.size()) - 1);
}

//-----------------------------------------------------------------------------------------------
void AddVertsForQuad3D(std::vector<Vertex_PNCU>& verts, const Vec3& bottomLeft, const Vec3& bottomRight, const Vec3& topLeft, const Vec3& topRight, const Rgba8& color, const AABB2& UVs)
{
	Vec3 u = (bottomRight - bottomLeft).GetNormalized();
	Vec3 v = (topRight - bottomLeft).GetNormalized();
	Vec3 normal = CrossProduct3D(u, v).GetNormalized();

	verts.push_back(Vertex_PNCU(bottomLeft, normal, color, UVs.m_mins));
	verts.push_back(Vertex_PNCU(bottomRight, normal, color, Vec2(UVs.m_maxs.x, UVs.m_mins.y)));
	verts.push_back(Vertex_PNCU(topRight, normal, color, UVs.m_maxs));
	
	verts.push_back(Vertex_PNCU(bottomLeft, normal, color, UVs.m_mins));
	verts.push_back(Vertex_PNCU(topRight, normal, color, UVs.m_maxs));
	verts.push_back(Vertex_PNCU(topLeft, normal, color, Vec2(UVs.m_mins.x, UVs.m_maxs.y)));
}

//-----------------------------------------------------------------------------------------------
void AddVertsForCube3D(std::vector<Vertex_PCU>& verts, const Vec3& center, const float radius, const Rgba8& color, const AABB2& UVs)
{
	AABB3 bounds(center - Vec3(radius, radius, radius), center + Vec3(radius, radius, radius));
	AddVertsForAABB3D(verts, bounds, color, UVs);
}

//-----------------------------------------------------------------------------------------------
void AddVertsForOBB3D(std::vector<Vertex_PCU>& verts, const OBB3& m_box, Rgba8 const& tint, AABB2 const& UVs)
{
	verts;
	m_box;
	tint;
	UVs;
}
