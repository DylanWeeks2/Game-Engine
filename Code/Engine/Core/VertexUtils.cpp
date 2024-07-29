#include "VertexUtils.hpp"
#include "Vertex_PCU.hpp"
#include "Vertex_PCUTBN.hpp"
#include "Engine/Math/Vec2.hpp"
#include "Engine/Math/Vec3.hpp"
#include "Engine/Math/DPVec2.hpp"
#include "Engine/Math/DPVec3.hpp"
#include "Engine/Math/MathUtils.hpp"
#include "Engine/Core/Rgba8.hpp"
#include "Engine/Math/AABB2.hpp"
#include "Engine/Math/DPAABB3.hpp"
#include "Engine/Math/LineSegment2.hpp"
#include "Engine/Math/Capsule2.hpp"
#include "Engine/Math/OBB2.hpp"
#include "Engine/Math/Mat44.hpp"
#include "Engine/Math/DPMat44.hpp"
#include "Engine/Math/AABB3.hpp"
#include "Engine/Math/FloatRange.hpp"
#include "Engine/Math/EulerAngles.hpp"
#include "Engine/Math/OBB3.hpp"
#include "Engine/Math/DPOBB3.hpp"
#include "Engine/Math/Cylinder3.hpp"
#include "Engine/Math/Capsule3.hpp"
#include "Engine/Math/DPCapsule3.hpp"
#include "Engine/Math/ConvexPoly2D.hpp"
#include "Engine/Math/ConvexPoly3D.hpp"
#include "Engine/Math/DoubleRange.hpp"
#include "Engine/Math/ConvexHull3D.hpp"

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
void TransformVertexArray3D(std::vector<Vertex_PCUTBN>& verts, const Mat44& transform)
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
	NUM_VERTS;

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
void AddVertsForDisc3D(std::vector<Vertex_PCUTBN>& verts, Vec3 const& center, float radius, int numSides, Rgba8 const& color)
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
		verts.push_back(Vertex_PCUTBN(center, color, Vec2(0.0f, 0.0f), Vec3(), Vec3(), normal));
		verts.push_back(Vertex_PCUTBN(secondVertOffset, color, Vec2(0.0f, 0.0f), Vec3(), Vec3(), normal));
		verts.push_back(Vertex_PCUTBN(thirdVertOffset, color, Vec2(0.0f, 0.0f), Vec3(), Vec3(), normal));
	}
}

//-----------------------------------------------------------------------------------------------
void AddVertsForDisc3D(std::vector<Vertex_PCU>& verts, Vec3 const& center, float radius, int numSides, Rgba8 const& color)
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
		verts.push_back(Vertex_PCU(center, color, Vec2(0.0f, 0.0f)));
		verts.push_back(Vertex_PCU(secondVertOffset, color, Vec2(0.0f, 0.0f)));
		verts.push_back(Vertex_PCU(thirdVertOffset, color, Vec2(0.0f, 0.0f)));
	}
}

//-----------------------------------------------------------------------------------------------
void AddVertsForRing3D(std::vector<Vertex_PCUTBN>& verts, Vec3 const& center, float radius, float thickness, int numSides, Rgba8 const& color)
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
		verts.push_back(Vertex_PCUTBN(innerEndPosition, color, Vec2(0.f, 0.f), Vec3(), Vec3(), normal));
		verts.push_back(Vertex_PCUTBN(innerStartPosition, color, Vec2(0.f, 0.f), Vec3(), Vec3(), normal));
		verts.push_back(Vertex_PCUTBN(outerStartPosition, color, Vec2(0.f, 0.f), Vec3(), Vec3(), normal));
		verts.push_back(Vertex_PCUTBN(innerEndPosition, color, Vec2(0.f, 0.f), Vec3(), Vec3(), normal));
		verts.push_back(Vertex_PCUTBN(outerStartPosition, color, Vec2(0.f, 0.f), Vec3(), Vec3(), normal));
		verts.push_back(Vertex_PCUTBN(outerEndPosition, color, Vec2(0.f, 0.f), Vec3(), Vec3(), normal));
	}
}

//-----------------------------------------------------------------------------------------------
void AddVertsForRing3D(std::vector<Vertex_PCUTBN>& verts, std::vector<unsigned int>& indices, Vec3 const& center, float radius, float thickness, int numSides, Rgba8 const& color)
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

		/*verts.push_back(Vertex_PCUTBN(innerEndPosition, normal, color, Vec2(0.f, 0.f)));
		verts.push_back(Vertex_PCUTBN(innerStartPosition, normal, color, Vec2(0.f, 0.f)));
		verts.push_back(Vertex_PCUTBN(outerStartPosition, normal, color, Vec2(0.f, 0.f)));
		verts.push_back(Vertex_PCUTBN(innerEndPosition, normal, color, Vec2(0.f, 0.f)));
		verts.push_back(Vertex_PCUTBN(outerStartPosition, normal, color, Vec2(0.f, 0.f)));
		verts.push_back(Vertex_PCUTBN(outerEndPosition, normal, color, Vec2(0.f, 0.f)));*/

		//initializes the vertexes (inner first, then outer second)
		Vec3 normal = CrossProduct3D(innerStartPosition - innerEndPosition, outerStartPosition - innerStartPosition).GetNormalized();
		
		if(sideCounter == 0)
		{
			verts.push_back(Vertex_PCUTBN(innerEndPosition, color, Vec2(0.f, 0.f), Vec3(), Vec3(), normal));
			verts.push_back(Vertex_PCUTBN(innerStartPosition, color, Vec2(0.f, 0.f), Vec3(), Vec3(), normal));
			verts.push_back(Vertex_PCUTBN(outerEndPosition, color, Vec2(0.f, 0.f), Vec3(), Vec3(), normal));
			verts.push_back(Vertex_PCUTBN(outerStartPosition, color, Vec2(0.f, 0.f), Vec3(), Vec3(), normal));
			
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
			verts.push_back(Vertex_PCUTBN(innerEndPosition, color, Vec2(0.f, 0.f), Vec3(), Vec3(), normal));
			verts.push_back(Vertex_PCUTBN(outerEndPosition, color, Vec2(0.f, 0.f), Vec3(), Vec3(), normal));
			
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
	NUM_VERTS;
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
void AddVertsForCapsule3D(std::vector<Vertex_PCU>& verts, Capsule3 const& capsule, Rgba8 const& tint, AABB2 const& UVs, int numSlices)
{
	float distance = (capsule.m_bone.m_end - capsule.m_bone.m_start).GetLength();
	int startVertPos = static_cast<int>(verts.size());
	AddVertsForCylinderSidesOnlyZ3D(verts, Vec2(0.0f, 0.0f), FloatRange(0.0f, distance), capsule.m_radius, static_cast<float>(numSlices), tint, UVs);
	AddVertsForHemisphereZUp3D(verts, Vec3(0.0f, 0.0f, distance), capsule.m_radius, tint, UVs, numSlices);
	AddVertsForHemisphereZDown3D(verts, Vec3(), capsule.m_radius, tint, UVs, numSlices);
	int endVertPos = static_cast<int>(verts.size()) - 1;
	std::vector<Vertex_PCU> tempVerts(verts.begin() + startVertPos, verts.begin() + endVertPos + 1);

	Mat44 transform;
	transform.SetIJK3D(capsule.m_bone.m_iBasis, capsule.m_bone.m_jBasis, capsule.m_bone.m_kBasis);
	transform.SetTranslation3D(capsule.m_bone.m_start);
	TransformVertexArray3D(tempVerts, transform);
	for (int vertIndex = startVertPos; vertIndex < verts.size(); vertIndex++)
	{
		verts[vertIndex] = tempVerts[vertIndex - startVertPos];
	}
}

//-----------------------------------------------------------------------------------------------
void AddVertsForCapsule3D(std::vector<Vertex_PCU>& verts, DPCapsule3 const& capsule, Rgba8 const& tint, AABB2 const& UVs, int numSlices)
{
	float distance = float((capsule.m_bone.m_end - capsule.m_bone.m_start).GetLength());
	int startVertPos = static_cast<int>(verts.size());
	double percentToStart = (capsule.m_radius + capsule.m_radius) / (distance + capsule.m_radius + capsule.m_radius);
	double percentToEnd = (distance + capsule.m_radius) / (distance + capsule.m_radius + capsule.m_radius);
	AABB2 newUVs = UVs;
	newUVs.m_maxs.y *= float(percentToStart);
	AddVertsForHemisphereZDown3D(verts, Vec3(), float(capsule.m_radius), tint, newUVs, numSlices);
	newUVs.m_mins.y = float((capsule.m_radius) / (distance + capsule.m_radius + capsule.m_radius));
	newUVs.m_maxs.y = float(percentToEnd);
	AddVertsForCylinderSidesOnlyZ3D(verts, Vec2(0.0f, 0.0f), FloatRange(0.0f, distance), float(capsule.m_radius), static_cast<float>(numSlices), tint, newUVs);
	newUVs.m_mins.y = float((distance) / (distance + capsule.m_radius + capsule.m_radius));
	newUVs.m_maxs.y = 1.0f;
	AddVertsForHemisphereZUp3D(verts, Vec3(0.0f, 0.0f, distance), float(capsule.m_radius), tint, newUVs, numSlices);
	int endVertPos = static_cast<int>(verts.size()) - 1;
	std::vector<Vertex_PCU> tempVerts(verts.begin() + startVertPos, verts.begin() + endVertPos + 1);

	Mat44 transform;
	transform.SetIJK3D(capsule.m_bone.m_iBasis, capsule.m_bone.m_jBasis, capsule.m_bone.m_kBasis);
	transform.SetTranslation3D(capsule.m_bone.m_start);
	TransformVertexArray3D(tempVerts, transform);
	for (int vertIndex = startVertPos; vertIndex < verts.size(); vertIndex++)
	{
		verts[vertIndex] = tempVerts[vertIndex - startVertPos];
	}
}

//-----------------------------------------------------------------------------------------------
void AddVertsForCapsule3D(std::vector<Vertex_PCUTBN>& verts, Capsule3 const& capsule, Rgba8 const& tint, AABB2 const& UVs, int numSlices)
{
	float distance = float((capsule.m_bone.m_end - capsule.m_bone.m_start).GetLength());
	int startVertPos = static_cast<int>(verts.size());
	float percentToStart = (capsule.m_radius + capsule.m_radius) / (distance + capsule.m_radius + capsule.m_radius);
	float percentToEnd = (distance + capsule.m_radius) / (distance + capsule.m_radius + capsule.m_radius);
	AABB2 newUVs = UVs;
	newUVs.m_maxs.y *= percentToStart;
	AddVertsForHemisphereZDown3D(verts, Vec3(), float(capsule.m_radius), tint, newUVs, numSlices);
	newUVs.m_mins.y = (capsule.m_radius) / (distance + capsule.m_radius + capsule.m_radius);
	newUVs.m_maxs.y = percentToEnd;
	AddVertsForCylinderSidesOnlyZ3D(verts, Vec2(0.0f, 0.0f), FloatRange(0.0f, distance), float(capsule.m_radius), static_cast<float>(numSlices), tint, newUVs);
	newUVs.m_mins.y = (distance) / (distance + capsule.m_radius + capsule.m_radius);
	newUVs.m_maxs.y = 1.0f;
	AddVertsForHemisphereZUp3D(verts, Vec3(0.0f, 0.0f, distance), float(capsule.m_radius), tint, newUVs, numSlices);
	int endVertPos = static_cast<int>(verts.size()) - 1;
	std::vector<Vertex_PCUTBN> tempVerts(verts.begin() + startVertPos, verts.begin() + endVertPos + 1);

	Mat44 transform;
	transform.SetIJK3D(capsule.m_bone.m_iBasis, capsule.m_bone.m_jBasis, capsule.m_bone.m_kBasis);
	transform.SetTranslation3D(capsule.m_bone.m_start);
	TransformVertexArray3D(tempVerts, transform);
	for (int vertIndex = startVertPos; vertIndex < verts.size(); vertIndex++)
	{
		verts[vertIndex] = tempVerts[vertIndex - startVertPos];
	}

	CalculateTangentSpaceVectors(verts);
}

//-----------------------------------------------------------------------------------------------
void AddVertsForCapsule3D(std::vector<Vertex_PCUTBN>& verts, DPCapsule3 const& capsule, Rgba8 const& tint, AABB2 const& UVs, int numSlices)
{
	float distance = float((capsule.m_bone.m_end - capsule.m_bone.m_start).GetLength());
	int startVertPos = static_cast<int>(verts.size());
	double percentToStart = (capsule.m_radius + capsule.m_radius) / (distance + capsule.m_radius + capsule.m_radius);
	double percentToEnd = (distance + capsule.m_radius) / (distance + capsule.m_radius + capsule.m_radius);
	AABB2 newUVs = UVs;
	newUVs.m_maxs.y *= float(percentToStart);
	AddVertsForHemisphereZDown3D(verts, Vec3(), float(capsule.m_radius), tint, newUVs, numSlices);
	newUVs.m_mins.y = float((capsule.m_radius) / (distance + capsule.m_radius + capsule.m_radius));
	newUVs.m_maxs.y = float(percentToEnd);
	AddVertsForCylinderSidesOnlyZ3D(verts, Vec2(0.0f, 0.0f), FloatRange(0.0f, distance), float(capsule.m_radius), static_cast<float>(numSlices), tint, newUVs);
	newUVs.m_mins.y = float((distance) / (distance + capsule.m_radius + capsule.m_radius));
	newUVs.m_maxs.y = 1.0f;
	AddVertsForHemisphereZUp3D(verts, Vec3(0.0f, 0.0f, distance), float(capsule.m_radius), tint, newUVs, numSlices);
	int endVertPos = static_cast<int>(verts.size()) - 1;
	std::vector<Vertex_PCUTBN> tempVerts(verts.begin() + startVertPos, verts.begin() + endVertPos + 1);

	Mat44 transform;
	transform.SetIJK3D(capsule.m_bone.m_iBasis, capsule.m_bone.m_jBasis, capsule.m_bone.m_kBasis);
	transform.SetTranslation3D(capsule.m_bone.m_start);
	TransformVertexArray3D(tempVerts, transform);
	for (int vertIndex = startVertPos; vertIndex < verts.size(); vertIndex++)
	{
		verts[vertIndex] = tempVerts[vertIndex - startVertPos];
	}

	CalculateTangentSpaceVectors(verts);
}

//-----------------------------------------------------------------------------------------------
void AddVertsForSector2D(std::vector<Vertex_PCU>& verts, Vec2 const& sectorTip, Vec2 const& sectorForwardNormal, float sectorApertureDegrees, float sectorRadius, Rgba8 const& color)
{
	constexpr int NUM_SIDES = 32;
	constexpr int NUM_TRIS = NUM_SIDES;
	constexpr int NUM_VERTS = 3 * NUM_TRIS;
	NUM_VERTS;
	
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
void AddVertsForAABB3D(std::vector<Vertex_PCU>& verts, const DPAABB3& bounds, const Rgba8& color, const AABB2& UVs)
{
	float minX = float(bounds.m_mins.x);
	float minY = float(bounds.m_mins.y);
	float minZ = float(bounds.m_mins.z);
	float maxX = float(bounds.m_maxs.x);
	float maxY = float(bounds.m_maxs.y);
	float maxZ = float(bounds.m_maxs.z);

	AddVertsForQuad3D(verts, Vec3(maxX, minY, minZ), Vec3(maxX, maxY, minZ), Vec3(maxX, minY, maxZ), Vec3(maxX, maxY, maxZ), color, UVs);
	AddVertsForQuad3D(verts, Vec3(minX, maxY, minZ), Vec3(minX, minY, minZ), Vec3(minX, maxY, maxZ), Vec3(minX, minY, maxZ), color, UVs);
	AddVertsForQuad3D(verts, Vec3(maxX, maxY, minZ), Vec3(minX, maxY, minZ), Vec3(maxX, maxY, maxZ), Vec3(minX, maxY, maxZ), color, UVs);
	AddVertsForQuad3D(verts, Vec3(minX, minY, minZ), Vec3(maxX, minY, minZ), Vec3(minX, minY, maxZ), Vec3(maxX, minY, maxZ), color, UVs);
	AddVertsForQuad3D(verts, Vec3(minX, minY, maxZ), Vec3(maxX, minY, maxZ), Vec3(minX, maxY, maxZ), Vec3(maxX, maxY, maxZ), color, UVs);
	AddVertsForQuad3D(verts, Vec3(minX, maxY, minZ), Vec3(maxX, maxY, minZ), Vec3(minX, minY, minZ), Vec3(maxX, minY, minZ), color, UVs);
}

//-----------------------------------------------------------------------------------------------
void AddVertsForAABB3D(std::vector<Vertex_PCUTBN>& verts, const AABB3& bounds, const Rgba8& color, const AABB2& UVs)
{
	float minX = float(bounds.m_mins.x);
	float minY = float(bounds.m_mins.y);
	float minZ = float(bounds.m_mins.z);
	float maxX = float(bounds.m_maxs.x);
	float maxY = float(bounds.m_maxs.y);
	float maxZ = float(bounds.m_maxs.z);

	AddVertsForQuad3D(verts, Vec3(maxX, minY, minZ), Vec3(maxX, maxY, minZ), Vec3(maxX, minY, maxZ), Vec3(maxX, maxY, maxZ), color, UVs);
	AddVertsForQuad3D(verts, Vec3(minX, maxY, minZ), Vec3(minX, minY, minZ), Vec3(minX, maxY, maxZ), Vec3(minX, minY, maxZ), color, UVs);
	AddVertsForQuad3D(verts, Vec3(maxX, maxY, minZ), Vec3(minX, maxY, minZ), Vec3(maxX, maxY, maxZ), Vec3(minX, maxY, maxZ), color, UVs);
	AddVertsForQuad3D(verts, Vec3(minX, minY, minZ), Vec3(maxX, minY, minZ), Vec3(minX, minY, maxZ), Vec3(maxX, minY, maxZ), color, UVs);
	AddVertsForQuad3D(verts, Vec3(minX, minY, maxZ), Vec3(maxX, minY, maxZ), Vec3(minX, maxY, maxZ), Vec3(maxX, maxY, maxZ), color, UVs);
	AddVertsForQuad3D(verts, Vec3(minX, maxY, minZ), Vec3(maxX, maxY, minZ), Vec3(minX, minY, minZ), Vec3(maxX, minY, minZ), color, UVs);

	CalculateTangentSpaceVectors(verts);
}

//-----------------------------------------------------------------------------------------------
void AddVertsForAABB3D(std::vector<Vertex_PCUTBN>& verts, const DPAABB3& bounds, const Rgba8& color, const AABB2& UVs)
{
	float minX = float(bounds.m_mins.x);
	float minY = float(bounds.m_mins.y);
	float minZ = float(bounds.m_mins.z);
	float maxX = float(bounds.m_maxs.x);
	float maxY = float(bounds.m_maxs.y);
	float maxZ = float(bounds.m_maxs.z);

	AddVertsForQuad3D(verts, Vec3(maxX, minY, minZ), Vec3(maxX, maxY, minZ), Vec3(maxX, minY, maxZ), Vec3(maxX, maxY, maxZ), color, UVs);
	AddVertsForQuad3D(verts, Vec3(minX, maxY, minZ), Vec3(minX, minY, minZ), Vec3(minX, maxY, maxZ), Vec3(minX, minY, maxZ), color, UVs);
	AddVertsForQuad3D(verts, Vec3(maxX, maxY, minZ), Vec3(minX, maxY, minZ), Vec3(maxX, maxY, maxZ), Vec3(minX, maxY, maxZ), color, UVs);
	AddVertsForQuad3D(verts, Vec3(minX, minY, minZ), Vec3(maxX, minY, minZ), Vec3(minX, minY, maxZ), Vec3(maxX, minY, maxZ), color, UVs);
	AddVertsForQuad3D(verts, Vec3(minX, minY, maxZ), Vec3(maxX, minY, maxZ), Vec3(minX, maxY, maxZ), Vec3(maxX, maxY, maxZ), color, UVs);
	AddVertsForQuad3D(verts, Vec3(minX, maxY, minZ), Vec3(maxX, maxY, minZ), Vec3(minX, minY, minZ), Vec3(maxX, minY, minZ), color, UVs);

	CalculateTangentSpaceVectors(verts);
}


//-----------------------------------------------------------------------------------------------
void AddVertsForAABB3D(std::vector<Vertex_PCUTBN>& verts, std::vector<unsigned int>& indexes, const AABB3& bounds, const Rgba8& color, const AABB2& UVs)
{
	float minX = bounds.m_mins.x;
	float minY = bounds.m_mins.y;
	float minZ = bounds.m_mins.z;
	float maxX = bounds.m_maxs.x;
	float maxY = bounds.m_maxs.y;
	float maxZ = bounds.m_maxs.z;

	AddVertsForQuad3D(verts, indexes, Vec3(maxX, minY, minZ), Vec3(maxX, maxY, minZ), Vec3(maxX, minY, maxZ), Vec3(maxX, maxY, maxZ), color, UVs);
	AddVertsForQuad3D(verts, indexes, Vec3(minX, maxY, minZ), Vec3(minX, minY, minZ), Vec3(minX, maxY, maxZ), Vec3(minX, minY, maxZ), color, UVs);
	AddVertsForQuad3D(verts, indexes, Vec3(maxX, maxY, minZ), Vec3(minX, maxY, minZ), Vec3(maxX, maxY, maxZ), Vec3(minX, maxY, maxZ), color, UVs);
	AddVertsForQuad3D(verts, indexes, Vec3(minX, minY, minZ), Vec3(maxX, minY, minZ), Vec3(minX, minY, maxZ), Vec3(maxX, minY, maxZ), color, UVs);
	AddVertsForQuad3D(verts, indexes, Vec3(minX, minY, maxZ), Vec3(maxX, minY, maxZ), Vec3(minX, maxY, maxZ), Vec3(maxX, maxY, maxZ), color, UVs);
	AddVertsForQuad3D(verts, indexes, Vec3(minX, maxY, minZ), Vec3(maxX, maxY, minZ), Vec3(minX, minY, minZ), Vec3(maxX, minY, minZ), color, UVs);

	CalculateTangentSpaceVectors(verts, indexes);
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
void AddVertsForSphere3D(std::vector<Vertex_PCUTBN>& verts, const Vec3& center, float radius, const Rgba8& color, const AABB2& UVs, int numLatitudeSlices)
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
			AddVertsForQuad3D(verts, topLeft, topRight, bottomLeft, bottomRight, color, quadUVs);
		}
	}

	CalculateTangentSpaceVectors(verts);
}

//-----------------------------------------------------------------------------------------------
void AddVertsForReverseSphere3D(std::vector<Vertex_PCUTBN>& verts, const Vec3& center, float radius, const Rgba8& color, const AABB2& UVs, int numLatitudeSlices)
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
			quadUVs.m_mins = Vec2(maxU, maxV);
			quadUVs.m_maxs = Vec2(minU, minV);
			AddVertsForQuad3D(verts, topRight, topLeft, bottomRight, bottomLeft, color, quadUVs);
		}
	}

	CalculateTangentSpaceVectors(verts);
}

//-----------------------------------------------------------------------------------------------
void AddVertsForSphere3D(std::vector<Vertex_PCUTBN>& verts, std::vector<unsigned int>& indexes, const Vec3& center, float radius, const Rgba8& color, const AABB2& UVs, int numLatitudeSlices)
{
	float yawIncrementAmount = 360.0f / (numLatitudeSlices * 2.0f);
	float pitchIncrementAmount = 180.0f / numLatitudeSlices;
	int previousTopIndex = -1;
	int previousBottomIndex = -1;
	int startTopLeftIndex = -1;
	int startBottomLeftIndex = -1;
	int startBottomRightIndex = -1;
	
	for (int pitchIndex = 0; pitchIndex < numLatitudeSlices; pitchIndex++)
	{
		float minPitch = -90.0f + pitchIndex * pitchIncrementAmount;
		float maxPitch = minPitch + pitchIncrementAmount;
		for (int yawIndex = 0; yawIndex < numLatitudeSlices * 2.0f; yawIndex++)
		{
			float minYaw = yawIndex * yawIncrementAmount;
			float maxYaw = minYaw + yawIncrementAmount;
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
			Vec3 u = (topRight - topLeft).GetNormalized();
			Vec3 v = (bottomRight - topLeft).GetNormalized();
			Vec3 normal = CrossProduct3D(u, v).GetNormalized();
			
			//Start Layer
			if (pitchIndex == 0)
			{
				//AddVertsForQuad3D(verts, indexes, topLeft, topRight, bottomLeft, bottomRight, color, quadUVs);
				if (yawIndex == 0)
				{
					verts.push_back(Vertex_PCUTBN(topLeft, color, quadUVs.m_mins, Vec3(), Vec3(), normal));
					verts.push_back(Vertex_PCUTBN(topRight, color, Vec2(quadUVs.m_maxs.x, quadUVs.m_mins.y), Vec3(), Vec3(), normal));
					verts.push_back(Vertex_PCUTBN(bottomLeft, color, quadUVs.m_maxs, Vec3(), Vec3(), Vec3(0.0f, 0.0f, 1.0f)));

					previousTopIndex = static_cast<int>(verts.size()) - 1;
					previousBottomIndex = static_cast<int>(verts.size()) - 2;
					startTopLeftIndex = static_cast<int>(verts.size()) - 1;
					startBottomLeftIndex = static_cast<int>(verts.size()) - 3;
					startBottomRightIndex = static_cast<int>(verts.size()) - 2;
					indexes.push_back(static_cast<int>(verts.size()) - 3);
					indexes.push_back(static_cast<int>(verts.size()) - 2);
					indexes.push_back(static_cast<int>(verts.size()) - 1);
				}
				else
				{
					if (yawIndex != numLatitudeSlices * 2.0f - 1)
					{
						verts.push_back(Vertex_PCUTBN(topRight, color, Vec2(quadUVs.m_maxs.x, quadUVs.m_mins.y), Vec3(), Vec3(), normal));

						indexes.push_back(previousBottomIndex);
						indexes.push_back(static_cast<int>(verts.size()) - 1);
						indexes.push_back(previousTopIndex);
						previousBottomIndex = static_cast<int>(verts.size()) - 1;
					}
					else
					{
						indexes.push_back(previousBottomIndex);
						indexes.push_back(startBottomLeftIndex);
						indexes.push_back(startTopLeftIndex);
					}
				}
			}
			else if(pitchIndex == 1) //Second Layer From Top
			{
				if (yawIndex == 0)
				{
					verts.push_back(Vertex_PCUTBN(topLeft, color, quadUVs.m_mins, Vec3(), Vec3(), normal));
					verts.push_back(Vertex_PCUTBN(topRight, color, Vec2(quadUVs.m_maxs.x, quadUVs.m_mins.y), Vec3(), Vec3(), normal));
					
					indexes.push_back(static_cast<int>(verts.size()) - 2);
					indexes.push_back(static_cast<int>(verts.size()) - 1);
					indexes.push_back(startBottomRightIndex);
					indexes.push_back(static_cast<int>(verts.size()) - 2);
					indexes.push_back(startBottomRightIndex);
					indexes.push_back(startBottomLeftIndex);
					previousTopIndex = startBottomRightIndex;
					previousBottomIndex = static_cast<int>(verts.size()) - 1;
					startTopLeftIndex = startBottomLeftIndex;
					startBottomLeftIndex = static_cast<int>(verts.size()) - 2;
					startBottomRightIndex = static_cast<int>(verts.size()) - 1;
				}
				else
				{
					if (yawIndex == 1)
					{
						verts.push_back(Vertex_PCUTBN(topRight, color, Vec2(quadUVs.m_maxs.x, quadUVs.m_mins.y), Vec3(), Vec3(), normal));

						indexes.push_back(previousBottomIndex);
						indexes.push_back(static_cast<int>(verts.size()) - 1);
						indexes.push_back(previousTopIndex + 2);
						indexes.push_back(previousBottomIndex);
						indexes.push_back(previousTopIndex + 2);
						indexes.push_back(previousTopIndex);
						previousBottomIndex = static_cast<int>(verts.size()) - 1;
						previousTopIndex = previousTopIndex + 2;
					}
					else if (yawIndex != numLatitudeSlices * 2.0f - 1)
					{
						verts.push_back(Vertex_PCUTBN(topRight, color, Vec2(quadUVs.m_maxs.x, quadUVs.m_mins.y), Vec3(), Vec3(), normal));

						indexes.push_back(previousBottomIndex);
						indexes.push_back(static_cast<int>(verts.size()) - 1);
						indexes.push_back(previousTopIndex + 1);
						indexes.push_back(previousBottomIndex);
						indexes.push_back(previousTopIndex + 1);
						indexes.push_back(previousTopIndex);
						previousBottomIndex = static_cast<int>(verts.size()) - 1;
						previousTopIndex = previousTopIndex + 1;
					}
					else
					{
						indexes.push_back(previousBottomIndex);
						indexes.push_back(startBottomLeftIndex);
						indexes.push_back(startTopLeftIndex);
						indexes.push_back(previousBottomIndex);
						indexes.push_back(startTopLeftIndex);
						indexes.push_back(previousTopIndex);
					}
				}
			}
			else if (pitchIndex != numLatitudeSlices - 1)
			{
				if (yawIndex == 0)
				{
					verts.push_back(Vertex_PCUTBN(topLeft, color, quadUVs.m_mins, Vec3(), Vec3(), normal));
					verts.push_back(Vertex_PCUTBN(topRight, color, Vec2(quadUVs.m_maxs.x, quadUVs.m_mins.y), Vec3(), Vec3(), normal));

					indexes.push_back(static_cast<int>(verts.size()) - 2);
					indexes.push_back(static_cast<int>(verts.size()) - 1);
					indexes.push_back(startBottomRightIndex);
					indexes.push_back(static_cast<int>(verts.size()) - 2);
					indexes.push_back(startBottomRightIndex);
					indexes.push_back(startBottomLeftIndex);

					previousTopIndex = startBottomRightIndex;
					previousBottomIndex = static_cast<int>(verts.size()) - 1;
					startTopLeftIndex = startBottomLeftIndex;
					startBottomLeftIndex = static_cast<int>(verts.size()) - 2;
					startBottomRightIndex = static_cast<int>(verts.size()) - 1;
				}
				else
				{
					if (yawIndex != numLatitudeSlices * 2.0f - 1)
					{
						verts.push_back(Vertex_PCUTBN(topRight, color, Vec2(quadUVs.m_maxs.x, quadUVs.m_mins.y), Vec3(), Vec3(), normal));

						indexes.push_back(previousBottomIndex);
						indexes.push_back(static_cast<int>(verts.size()) - 1);
						indexes.push_back(previousTopIndex + 1);
						indexes.push_back(previousBottomIndex);
						indexes.push_back(previousTopIndex + 1);
						indexes.push_back(previousTopIndex);

						previousBottomIndex = static_cast<int>(verts.size()) - 1;
						previousTopIndex = previousTopIndex + 1;
					}
					else
					{
						indexes.push_back(previousBottomIndex);
						indexes.push_back(startBottomLeftIndex);
						indexes.push_back(startTopLeftIndex);
						indexes.push_back(previousBottomIndex);
						indexes.push_back(startTopLeftIndex);
						indexes.push_back(previousTopIndex);
					}
				}
			}
			else
			{
				if (yawIndex == 0)
				{
					verts.push_back(Vertex_PCUTBN(topLeft, color, quadUVs.m_mins, Vec3(), Vec3(), Vec3(0.0f, 0.0f, -1.0f)));
					
					indexes.push_back(static_cast<int>(verts.size()) - 1);
					indexes.push_back(startBottomRightIndex);
					indexes.push_back(startBottomLeftIndex);

					previousTopIndex = startBottomRightIndex;
					startTopLeftIndex = startBottomLeftIndex;
				}
				else if(yawIndex != numLatitudeSlices * 2.0f - 1)
				{
					indexes.push_back(static_cast<int>(verts.size()) - 1);
					indexes.push_back(previousTopIndex + 1);
					indexes.push_back(previousTopIndex);

					previousTopIndex += 1;
				}
				else
				{
					indexes.push_back(static_cast<int>(verts.size()) - 1);
					indexes.push_back(startTopLeftIndex);
					indexes.push_back(previousTopIndex);
				}
			}
			//AddVertsForQuad3D(verts, indexes, topLeft, topRight, bottomLeft, bottomRight, color, quadUVs);
		}
	}

	CalculateTangentSpaceVectors(verts, indexes);
}

//-----------------------------------------------------------------------------------------------
void AddVertsForHemisphereZUp3D(std::vector<Vertex_PCU>& verts, const Vec3& center, float radius, const Rgba8& color, const AABB2& UVs, int numLatitudeSlices)
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
void AddVertsForHemisphereZUp3D(std::vector<Vertex_PCUTBN>& verts, const Vec3& center, float radius, const Rgba8& color, const AABB2& UVs, int numLatitudeSlices)
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
void AddVertsForHemisphereZDown3D(std::vector<Vertex_PCU>& verts, const Vec3& center, float radius, const Rgba8& color, const AABB2& UVs, int numLatitudeSlices)
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
void AddVertsForHemisphereZDown3D(std::vector<Vertex_PCUTBN>& verts, const Vec3& center, float radius, const Rgba8& color, const AABB2& UVs, int numLatitudeSlices)
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
	NUM_VERTS;
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
void AddVertsForCylinderZ3D(std::vector<Vertex_PCUTBN>& verts, Vec2 const& centerXY, FloatRange const& minMaxZ, float radius, float numSlices, Rgba8 const& tint, AABB2 const& UVs)
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
		verts.push_back(Vertex_PCUTBN(topCenter, tint, Vec2(0.5f, 0.5f), Vec3(), Vec3(), Vec3()));
		verts.push_back(Vertex_PCUTBN(topLeft, tint, topLeftUV, Vec3(), Vec3(), Vec3()));
		verts.push_back(Vertex_PCUTBN(topRight, tint, topRightUV, Vec3(), Vec3(), Vec3()));
		//Bottom
		verts.push_back(Vertex_PCUTBN(bottomCenter, tint, Vec2(0.5f, 0.5f), Vec3(), Vec3(), Vec3()));
		verts.push_back(Vertex_PCUTBN(bottomRight, tint, bottomRightUV, Vec3(), Vec3(), Vec3()));
		verts.push_back(Vertex_PCUTBN(bottomLeft, tint, bottomLeftUV, Vec3(), Vec3(), Vec3()));
	}
}

//-----------------------------------------------------------------------------------------------
void AddVertsForCylinderSidesOnlyZ3D(std::vector<Vertex_PCU>& verts, Vec2 const& centerXY, FloatRange const& minMaxZ, float radius, float numSlices, Rgba8 const& tint, AABB2 const& UVs)
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
	}
}

//-----------------------------------------------------------------------------------------------
void AddVertsForCylinderSidesOnlyZ3D(std::vector<Vertex_PCUTBN>& verts, Vec2 const& centerXY, FloatRange const& minMaxZ, float radius, float numSlices, Rgba8 const& tint, AABB2 const& UVs)
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
	FloatRange cylinderRange = FloatRange(RangeMap(0.85f, 0.0f, 1.0f, 0.0f, distance), 0.0f);
	FloatRange coneRange = FloatRange(distance, RangeMap(0.85f, 0.0f, 1.0f, 0.0f, distance));
	int startVertPos = static_cast<int>(verts.size());
	AddVertsForCylinderZ3D(verts, Vec2(0.0f, 0.0f), cylinderRange, lineThickness * 0.5f, 8, tint);
	AddVertsForConeZ3D(verts, Vec2(0.0f, 0.0f), coneRange, lineThickness, 8, tint);
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
void AddVertsForCylinder3D(std::vector<Vertex_PCU>& verts, Cylinder3 const& cylinder, Rgba8 const& tint, AABB2 const& UVs, int numSlices)
{
	float distance = (cylinder.m_end - cylinder.m_start).GetLength();
	int startVertPos = static_cast<int>(verts.size());
	AddVertsForCylinderZ3D(verts, Vec2(0.0f, 0.0f), FloatRange(0.0f, distance), cylinder.m_radius, static_cast<float>(numSlices), tint, UVs);
	int endVertPos = static_cast<int>(verts.size()) - 1;
	std::vector<Vertex_PCU> tempVerts(verts.begin() + startVertPos, verts.begin() + endVertPos + 1);

	Mat44 transform;
	transform.SetIJK3D(cylinder.m_iBasis, cylinder.m_jBasis, cylinder.m_kBasis);
	transform.SetTranslation3D(cylinder.m_start);
	TransformVertexArray3D(tempVerts, transform);
	for (int vertIndex = startVertPos; vertIndex < verts.size(); vertIndex++)
	{
		verts[vertIndex] = tempVerts[vertIndex - startVertPos];
	}
}

//-----------------------------------------------------------------------------------------------
void AddVertsForCylinder3D(std::vector<Vertex_PCU>& verts, DPCylinder3 const& cylinder, Rgba8 const& tint, AABB2 const& UVs, int numSlices)
{
	float distance = float((cylinder.m_end - cylinder.m_start).GetLength());
	int startVertPos = static_cast<int>(verts.size());
	AddVertsForCylinderZ3D(verts, Vec2(0.0f, 0.0f), FloatRange(0.0f, distance), float(cylinder.m_radius), static_cast<float>(numSlices), tint, UVs);
	int endVertPos = static_cast<int>(verts.size()) - 1;
	std::vector<Vertex_PCU> tempVerts(verts.begin() + startVertPos, verts.begin() + endVertPos + 1);

	Mat44 transform;
	transform.SetIJK3D(cylinder.m_iBasis, cylinder.m_jBasis, cylinder.m_kBasis);
	transform.SetTranslation3D(cylinder.m_start);
	TransformVertexArray3D(tempVerts, transform);
	for (int vertIndex = startVertPos; vertIndex < verts.size(); vertIndex++)
	{
		verts[vertIndex] = tempVerts[vertIndex - startVertPos];
	}
}

//-----------------------------------------------------------------------------------------------
void AddVertsForCylinder3D(std::vector<Vertex_PCUTBN>& verts, Cylinder3 const& cylinder, Rgba8 const& tint, AABB2 const& UVs, int numSlices)
{
	float distance = float((cylinder.m_end - cylinder.m_start).GetLength());
	int startVertPos = static_cast<int>(verts.size());
	AddVertsForCylinderZ3D(verts, Vec2(0.0f, 0.0f), FloatRange(0.0f, distance), float(cylinder.m_radius), static_cast<float>(numSlices), tint, UVs);
	int endVertPos = static_cast<int>(verts.size()) - 1;
	std::vector<Vertex_PCUTBN> tempVerts(verts.begin() + startVertPos, verts.begin() + endVertPos + 1);

	Mat44 transform;
	transform.SetIJK3D(cylinder.m_iBasis, cylinder.m_jBasis, cylinder.m_kBasis);
	transform.SetTranslation3D(cylinder.m_start);
	TransformVertexArray3D(tempVerts, transform);
	for (int vertIndex = startVertPos; vertIndex < verts.size(); vertIndex++)
	{
		verts[vertIndex] = tempVerts[vertIndex - startVertPos];
	}

	CalculateTangentSpaceVectors(verts);
}


//-----------------------------------------------------------------------------------------------
void AddVertsForCylinder3D(std::vector<Vertex_PCUTBN>& verts, DPCylinder3 const& cylinder, Rgba8 const& tint, AABB2 const& UVs, int numSlices)
{
	float distance = float((cylinder.m_end - cylinder.m_start).GetLength());
	int startVertPos = static_cast<int>(verts.size());
	AddVertsForCylinderZ3D(verts, Vec2(0.0f, 0.0f), FloatRange(0.0f, distance), float(cylinder.m_radius), static_cast<float>(numSlices), tint, UVs);
	int endVertPos = static_cast<int>(verts.size()) - 1;
	std::vector<Vertex_PCUTBN> tempVerts(verts.begin() + startVertPos, verts.begin() + endVertPos + 1);

	Mat44 transform;
	transform.SetIJK3D(cylinder.m_iBasis, cylinder.m_jBasis, cylinder.m_kBasis);
	transform.SetTranslation3D(cylinder.m_start);
	TransformVertexArray3D(tempVerts, transform);
	for (int vertIndex = startVertPos; vertIndex < verts.size(); vertIndex++)
	{
		verts[vertIndex] = tempVerts[vertIndex - startVertPos];
	}

	CalculateTangentSpaceVectors(verts);
}

//-----------------------------------------------------------------------------------------------
void AddVertsForCylinderSidesOnly3D(std::vector<Vertex_PCU>& verts, Cylinder3 const& cylinder, Rgba8 const& tint, AABB2 const& UVs, int numSlices)
{
	float distance = (cylinder.m_end - cylinder.m_start).GetLength();
	int startVertPos = static_cast<int>(verts.size());
	AddVertsForCylinderSidesOnlyZ3D(verts, Vec2(0.0f, 0.0f), FloatRange(0.0f, distance), cylinder.m_radius, static_cast<float>(numSlices), tint, UVs);
	int endVertPos = static_cast<int>(verts.size()) - 1;
	std::vector<Vertex_PCU> tempVerts(verts.begin() + startVertPos, verts.begin() + endVertPos + 1);

	Mat44 transform;
	transform.SetIJK3D(cylinder.m_iBasis, cylinder.m_jBasis, cylinder.m_kBasis);
	transform.SetTranslation3D(cylinder.m_start);
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
void AddVertsForRoundedQuad3D(std::vector<Vertex_PCUTBN>& verts, const Vec3& bottomLeft, const Vec3& bottomRight, const Vec3& topLeft, const Vec3& topRight, const Rgba8& color, AABB2 const& UVs)
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

	verts.push_back(Vertex_PCUTBN(bottomLeft, color, UVs.m_mins, Vec3(), Vec3(), bottomLeftNormal));
	verts.push_back(Vertex_PCUTBN(bottomMiddle, color, Vec2(UVs.m_mins.x + rangeX * 0.5f, UVs.m_mins.y), Vec3(), Vec3(), middleNormal));
	verts.push_back(Vertex_PCUTBN(topMiddle, color, Vec2(UVs.m_mins.x + rangeX * 0.5f, UVs.m_maxs.y), Vec3(), Vec3(), middleNormal));
					
	verts.push_back(Vertex_PCUTBN(bottomLeft, color, UVs.m_mins, Vec3(), Vec3(), bottomLeftNormal));
	verts.push_back(Vertex_PCUTBN(topMiddle, color, Vec2(UVs.m_mins.x + rangeX * 0.5f, UVs.m_maxs.y), Vec3(), Vec3(), middleNormal));
	verts.push_back(Vertex_PCUTBN(topLeft, color, Vec2(UVs.m_mins.x, UVs.m_maxs.y), Vec3(), Vec3(), topLeftNormal));
					
	verts.push_back(Vertex_PCUTBN(bottomMiddle, color, Vec2(UVs.m_mins.x + rangeX * 0.5f, UVs.m_mins.y), Vec3(), Vec3(), middleNormal));
	verts.push_back(Vertex_PCUTBN(bottomRight, color, Vec2(UVs.m_maxs.x, UVs.m_mins.y), Vec3(), Vec3(), bottomRightNormal));
	verts.push_back(Vertex_PCUTBN(topRight, color, UVs.m_maxs, Vec3(), Vec3(), topRightNormal));
					
	verts.push_back(Vertex_PCUTBN(bottomMiddle, color, Vec2(UVs.m_mins.x + rangeX * 0.5f, UVs.m_mins.y), Vec3(), Vec3(), middleNormal));
	verts.push_back(Vertex_PCUTBN(topRight, color, UVs.m_maxs, Vec3(), Vec3(), topRightNormal));
	verts.push_back(Vertex_PCUTBN(topMiddle, color, Vec2(UVs.m_mins.x + rangeX * 0.5f, UVs.m_maxs.y), Vec3(), Vec3(), middleNormal));
}

//-----------------------------------------------------------------------------------------------
void AddVertsForQuad3D(std::vector<Vertex_PCUTBN>& verts, std::vector<unsigned int>& indexes, const Vec3& bottomLeft, const Vec3& bottomRight, const Vec3& topLeft, const Vec3& topRight, const Rgba8& color, AABB2 const& UVs)
{
	Vec3 u = (bottomRight - bottomLeft).GetNormalized();
	Vec3 v = (topRight - bottomLeft).GetNormalized();
	Vec3 normal = CrossProduct3D(u, v).GetNormalized();

	//verts
	verts.push_back(Vertex_PCUTBN(bottomLeft, color, UVs.m_mins, Vec3(), Vec3(), normal));
	verts.push_back(Vertex_PCUTBN(bottomRight, color, Vec2(UVs.m_maxs.x, UVs.m_mins.y), Vec3(), Vec3(), normal));
	verts.push_back(Vertex_PCUTBN(topRight, color, UVs.m_maxs, Vec3(), Vec3(), normal));
	verts.push_back(Vertex_PCUTBN(topLeft, color, Vec2(UVs.m_mins.x, UVs.m_maxs.y), Vec3(), Vec3(), normal));

	//indexes
	indexes.push_back(static_cast<int>(verts.size()) - 4);
	indexes.push_back(static_cast<int>(verts.size()) - 3);
	indexes.push_back(static_cast<int>(verts.size()) - 2);
	indexes.push_back(static_cast<int>(verts.size()) - 4);
	indexes.push_back(static_cast<int>(verts.size()) - 2);
	indexes.push_back(static_cast<int>(verts.size()) - 1);

	CalculateTangentSpaceVectors(verts, indexes);
}

//-----------------------------------------------------------------------------------------------
void AddVertsForQuad3D(std::vector<Vertex_PCUTBN>& verts, const Vec3& bottomLeft, const Vec3& bottomRight, const Vec3& topLeft, const Vec3& topRight, const Rgba8& color, const AABB2& UVs)
{
	Vec3 u = (bottomRight - bottomLeft).GetNormalized();
	Vec3 v = (topRight - bottomLeft).GetNormalized();
	Vec3 normal = CrossProduct3D(u, v).GetNormalized();

	verts.push_back(Vertex_PCUTBN(bottomLeft, color, UVs.m_mins, Vec3(), Vec3(), normal));
	verts.push_back(Vertex_PCUTBN(bottomRight, color, Vec2(UVs.m_maxs.x, UVs.m_mins.y), Vec3(), Vec3(), normal));
	verts.push_back(Vertex_PCUTBN(topRight, color, UVs.m_maxs, Vec3(), Vec3(), normal));
	
	verts.push_back(Vertex_PCUTBN(bottomLeft, color, UVs.m_mins, Vec3(), Vec3(), normal));
	verts.push_back(Vertex_PCUTBN(topRight, color, UVs.m_maxs, Vec3(), Vec3(), normal));
	verts.push_back(Vertex_PCUTBN(topLeft, color, Vec2(UVs.m_mins.x, UVs.m_maxs.y), Vec3(), Vec3(), normal));
}

//-----------------------------------------------------------------------------------------------
void AddVertsForCube3D(std::vector<Vertex_PCU>& verts, const Vec3& center, const float radius, const Rgba8& color, const AABB2& UVs)
{
	AABB3 bounds(center - Vec3(radius, radius, radius), center + Vec3(radius, radius, radius));
	AddVertsForAABB3D(verts, bounds, color, UVs);
}

//-----------------------------------------------------------------------------------------------
void AddVertsForOBB3D(std::vector<Vertex_PCU>& verts, const OBB3& obb, Rgba8 const& tint, AABB2 const& UVs)
{
	AABB3 aabb;
	aabb.m_mins = Vec3(-obb.m_halfDimensions.x, -obb.m_halfDimensions.y, -obb.m_halfDimensions.z);
	aabb.m_maxs = Vec3(obb.m_halfDimensions.x, obb.m_halfDimensions.y, obb.m_halfDimensions.z);
	
	Vec3 iBasis = obb.m_iBasisNormal;
	Vec3 jBasis = obb.m_jBasisNormal;
	Vec3 kBasis = obb.m_kBasisNormal;
	int startVertPos = static_cast<int>(verts.size());
	AddVertsForAABB3D(verts, aabb, tint, UVs);
	int endVertPos = static_cast<int>(verts.size()) - 1;
	std::vector<Vertex_PCU> tempVerts(verts.begin() + startVertPos, verts.begin() + endVertPos + 1);

	Mat44 transform;
	transform.SetIJK3D(iBasis, jBasis, kBasis);
	transform.SetTranslation3D(obb.m_center);
	TransformVertexArray3D(tempVerts, transform);
	for (int vertIndex = startVertPos; vertIndex < verts.size(); vertIndex++)
	{
		verts[vertIndex] = tempVerts[vertIndex - startVertPos];
	}
}

//-----------------------------------------------------------------------------------------------
void AddVertsForOBB3D(std::vector<Vertex_PCU>& verts, const DPOBB3& obb, Rgba8 const& tint, AABB2 const& UVs)
{
	DPAABB3 aabb;
	aabb.m_mins = DPVec3(-obb.m_halfDimensions.x, -obb.m_halfDimensions.y, -obb.m_halfDimensions.z);
	aabb.m_maxs = DPVec3(obb.m_halfDimensions.x, obb.m_halfDimensions.y, obb.m_halfDimensions.z);

	Vec3 iBasis = obb.m_iBasisNormal;
	Vec3 jBasis = obb.m_jBasisNormal;
	Vec3 kBasis = obb.m_kBasisNormal;
	int startVertPos = static_cast<int>(verts.size());
	AddVertsForAABB3D(verts, aabb, tint, UVs);
	int endVertPos = static_cast<int>(verts.size()) - 1;
	std::vector<Vertex_PCU> tempVerts(verts.begin() + startVertPos, verts.begin() + endVertPos + 1);

	Mat44 transform;
	transform.SetIJK3D(iBasis, jBasis, kBasis);
	transform.SetTranslation3D(obb.m_center);
	TransformVertexArray3D(tempVerts, transform);
	for (int vertIndex = startVertPos; vertIndex < verts.size(); vertIndex++)
	{
		verts[vertIndex] = tempVerts[vertIndex - startVertPos];
	}
}

//-----------------------------------------------------------------------------------------------
void AddVertsForOBB3D(std::vector<Vertex_PCUTBN>& verts, const OBB3& obb, Rgba8 const& tint, AABB2 const& UVs)
{
	AABB3 aabb;
	aabb.m_mins = Vec3(-obb.m_halfDimensions.x, -obb.m_halfDimensions.y, -obb.m_halfDimensions.z);
	aabb.m_maxs = Vec3(obb.m_halfDimensions.x, obb.m_halfDimensions.y, obb.m_halfDimensions.z);

	Vec3 iBasis = obb.m_iBasisNormal;
	Vec3 jBasis = obb.m_jBasisNormal;
	Vec3 kBasis = obb.m_kBasisNormal;
	int startVertPos = static_cast<int>(verts.size());
	AddVertsForAABB3D(verts, aabb, tint, UVs);
	int endVertPos = static_cast<int>(verts.size()) - 1;
	std::vector<Vertex_PCUTBN> tempVerts(verts.begin() + startVertPos, verts.begin() + endVertPos + 1);

	Mat44 transform;
	transform.SetIJK3D(iBasis, jBasis, kBasis);
	transform.SetTranslation3D(obb.m_center);
	TransformVertexArray3D(tempVerts, transform);
	for (int vertIndex = startVertPos; vertIndex < verts.size(); vertIndex++)
	{
		verts[vertIndex] = tempVerts[vertIndex - startVertPos];
	}
	CalculateTangentSpaceVectors(verts);
}

//-----------------------------------------------------------------------------------------------
void AddVertsForOBB3D(std::vector<Vertex_PCUTBN>& verts, const DPOBB3& obb, Rgba8 const& tint, AABB2 const& UVs)
{
	DPAABB3 aabb;
	aabb.m_mins = DPVec3(-obb.m_halfDimensions.x, -obb.m_halfDimensions.y, -obb.m_halfDimensions.z);
	aabb.m_maxs = DPVec3(obb.m_halfDimensions.x, obb.m_halfDimensions.y, obb.m_halfDimensions.z);

	Vec3 iBasis = obb.m_iBasisNormal;
	Vec3 jBasis = obb.m_jBasisNormal;
	Vec3 kBasis = obb.m_kBasisNormal;
	int startVertPos = static_cast<int>(verts.size());
	AddVertsForAABB3D(verts, aabb, tint, UVs);
	int endVertPos = static_cast<int>(verts.size()) - 1;
	std::vector<Vertex_PCUTBN> tempVerts(verts.begin() + startVertPos, verts.begin() + endVertPos + 1);

	Mat44 transform;
	transform.SetIJK3D(iBasis, jBasis, kBasis);
	transform.SetTranslation3D(obb.m_center);
	TransformVertexArray3D(tempVerts, transform);
	for (int vertIndex = startVertPos; vertIndex < verts.size(); vertIndex++)
	{
		verts[vertIndex] = tempVerts[vertIndex - startVertPos];
	}
	CalculateTangentSpaceVectors(verts);
}

//-----------------------------------------------------------------------------------------------
void AddVertsForLineList(std::vector<Vertex_PCUTBN>& verts, Vec3 const& p1, Vec3 const& p2, Rgba8 const& color)
{
	verts.push_back(Vertex_PCUTBN(p1, color, Vec2(), Vec3(), Vec3(), Vec3()));
	verts.push_back(Vertex_PCUTBN(p2, color, Vec2(), Vec3(), Vec3(), Vec3()));
}

//-----------------------------------------------------------------------------------------------
void AddVertsForLineList(std::vector<Vertex_PCU>& verts, Vec3 const& p1, Vec3 const& p2, Rgba8 const& color)
{
	verts.push_back(Vertex_PCU(p1, color, Vec2()));
	verts.push_back(Vertex_PCU(p2, color, Vec2()));
}

//-----------------------------------------------------------------------------------------------
void AddVertsForLineList2D(std::vector<Vertex_PCU>& verts, Vec2 const& p1, Vec2 const& p2, Rgba8 const& color)
{
	verts.push_back(Vertex_PCU(Vec3(p1.x, p1.y, 0.0f), color, Vec2()));
	verts.push_back(Vertex_PCU(Vec3(p2.x, p2.y, 0.0f), color, Vec2()));
}

//-----------------------------------------------------------------------------------------------
void AddVertsForConvexPoly2D(std::vector<Vertex_PCU>& verts, ConvexPoly2D const& convexPoly, Rgba8 const& color)
{
	for (int pointIndex = 1; pointIndex < convexPoly.m_ccwOrderedPoints.size() - 1; pointIndex ++)
	{
		verts.push_back(Vertex_PCU(Vec3(convexPoly.m_ccwOrderedPoints[0].x, convexPoly.m_ccwOrderedPoints[0].y, 0.0f), color, Vec2()));
		verts.push_back(Vertex_PCU(Vec3(convexPoly.m_ccwOrderedPoints[pointIndex].x, convexPoly.m_ccwOrderedPoints[pointIndex].y, 0.0f), color, Vec2()));
		verts.push_back(Vertex_PCU(Vec3(convexPoly.m_ccwOrderedPoints[pointIndex + 1].x, convexPoly.m_ccwOrderedPoints[pointIndex + 1].y, 0.0f), color, Vec2()));
	}
}

//-----------------------------------------------------------------------------------------------
void AddVertsForConvexPoly3D(std::vector<Vertex_PCU>& verts, ConvexPoly3D const& convexPoly, Rgba8 const& color)
{
	for (int pointIndex = 1; pointIndex < convexPoly.m_ccwOrderedPoints.size() - 1; pointIndex++)
	{
		verts.push_back(Vertex_PCU(convexPoly.m_ccwOrderedPoints[0], color, Vec2()));
		verts.push_back(Vertex_PCU(convexPoly.m_ccwOrderedPoints[pointIndex], color, Vec2()));
		verts.push_back(Vertex_PCU(convexPoly.m_ccwOrderedPoints[pointIndex + 1], color, Vec2()));
	}
}

//-----------------------------------------------------------------------------------------------
void AddVertsForWireConvexPoly3D(std::vector<Vertex_PCU>& verts, ConvexPoly3D const& convexPoly, Rgba8 const& color)
{
	for (int pointIndex = 0; pointIndex < convexPoly.m_ccwOrderedPoints.size(); pointIndex++)
	{
		if (pointIndex != convexPoly.m_ccwOrderedPoints.size() - 1)
			AddVertsForLineList(verts, convexPoly.m_ccwOrderedPoints[pointIndex], convexPoly.m_ccwOrderedPoints[pointIndex + 1], color);
		else
			AddVertsForLineList(verts, convexPoly.m_ccwOrderedPoints[pointIndex], convexPoly.m_ccwOrderedPoints[0], color);
	}
}

//-----------------------------------------------------------------------------------------------
void AddVertsForWireConvexHull3D(std::vector<Vertex_PCU>& verts, ConvexHull3D const& convexHull, Rgba8 const& color)
{
	for (int index = 0; index < convexHull.m_boundingPolys.size(); index++)
	{
		AddVertsForWireConvexPoly3D(verts, convexHull.m_boundingPolys[index], color);
	}
}

//-----------------------------------------------------------------------------------------------
void CalculateTangentSpaceVectors(std::vector<Vertex_PCUTBN>& verts, std::vector<unsigned int>& indexes)
{
	bool calculateTangentSpace = false;
	std::vector<Vec3> tempTangents(verts.size(), Vec3());
	std::vector<Vec3> tempBitangents(verts.size(), Vec3());
	for (int triIndex = 0; triIndex < indexes.size(); triIndex += 3)
	{
		int i0 = indexes[triIndex];
		int i1 = indexes[triIndex + 1];
		int i2 = indexes[triIndex + 2];
		Vec3 edge1 = verts[i1].m_position - verts[i0].m_position;
		Vec3 edge2 = verts[i2].m_position - verts[i0].m_position;

		float x1 = verts[i1].m_uvTexCoords.x - verts[i0].m_uvTexCoords.x;
		float x2 = verts[i2].m_uvTexCoords.x - verts[i0].m_uvTexCoords.x;
		float y1 = verts[i1].m_uvTexCoords.y - verts[i0].m_uvTexCoords.y;
		float y2 = verts[i2].m_uvTexCoords.y - verts[i0].m_uvTexCoords.y;

		float r = 0.0f;
		Vec3 t;
		Vec3 b;
		if ((x1 * y2 - x2 * y1) != 0.0f)
		{
			r = 1.0f / (x1 * y2 - x2 * y1);
			t = (edge1 * y2 - edge2 * y1) * r;
			b = (edge2 * x1 - edge1 * x2) * r; 
			tempTangents[i0] += t;
			tempTangents[i1] += t;
			tempTangents[i2] += t;
			tempBitangents[i0] += b;
			tempBitangents[i1] += b;
			tempBitangents[i2] += b;
			calculateTangentSpace = true;
		}
	}

	if (calculateTangentSpace)
	{
		for (int vertIndex = 0; vertIndex < verts.size(); vertIndex++)
		{
			Mat44 tangentSpaceMatrix;
			tangentSpaceMatrix.SetIJK3D(tempTangents[vertIndex], tempBitangents[vertIndex], verts[vertIndex].m_normal);
			tangentSpaceMatrix.Orthonormalize_XFwd_YLeft_ZUp();

			verts[vertIndex].m_tangent = tangentSpaceMatrix.GetIBasis3D();
			verts[vertIndex].m_binormal = tangentSpaceMatrix.GetJBasis3D();
			verts[vertIndex].m_normal = tangentSpaceMatrix.GetKBasis3D();
		}
	}
}

//-----------------------------------------------------------------------------------------------
void CalculateTangentSpaceVectors(std::vector<Vertex_PCUTBN>& verts)
{
	bool calculateTangentSpace = false;
	std::vector<Vec3> tempTangents(verts.size(), Vec3());
	std::vector<Vec3> tempBitangents(verts.size(), Vec3());
	for (int triIndex = 0; triIndex < verts.size(); triIndex += 3)
	{
		Vec3 edge1 = verts[triIndex + 1].m_position - verts[triIndex].m_position;
		Vec3 edge2 = verts[triIndex + 2].m_position - verts[triIndex].m_position;

		float x1 = verts[triIndex + 1].m_uvTexCoords.x - verts[triIndex].m_uvTexCoords.x;
		float x2 = verts[triIndex + 2].m_uvTexCoords.x - verts[triIndex].m_uvTexCoords.x;
		float y1 = verts[triIndex + 1].m_uvTexCoords.y - verts[triIndex].m_uvTexCoords.y;
		float y2 = verts[triIndex + 2].m_uvTexCoords.y - verts[triIndex].m_uvTexCoords.y;

		float r = 0.0f;
		Vec3 t;
		Vec3 b;
		if ((x1 * y2 - x2 * y1) != 0.0f)
		{
			r = 1.0f / (x1 * y2 - x2 * y1);
			t = (edge1 * y2 - edge2 * y1) * r;
			b = (edge2 * x1 - edge1 * x2) * r;
			tempTangents[triIndex] += t;
			tempTangents[triIndex + 1] += t;
			tempTangents[triIndex + 2] += t;
			tempBitangents[triIndex] += b;
			tempBitangents[triIndex + 1] += b;
			tempBitangents[triIndex + 2] += b;
			Vec3 u = (verts[triIndex + 2].m_position - verts[triIndex + 1].m_position).GetNormalized();
			Vec3 v = (verts[triIndex].m_position - verts[triIndex + 1].m_position).GetNormalized();
			Vec3 normal = CrossProduct3D(u, v).GetNormalized();
			verts[triIndex].m_normal = normal;
			verts[triIndex + 1].m_normal = normal;
			verts[triIndex + 2].m_normal = normal;
			calculateTangentSpace = true;
		}
	}

	if (calculateTangentSpace)
	{
		for (int vertIndex = 0; vertIndex < verts.size(); vertIndex++)
		{
			Mat44 tangentSpaceMatrix;
			tangentSpaceMatrix.SetIJK3D(tempTangents[vertIndex], tempBitangents[vertIndex], verts[vertIndex].m_normal);
			tangentSpaceMatrix.Orthonormalize_XFwd_YLeft_ZUp();

			verts[vertIndex].m_tangent = tangentSpaceMatrix.GetIBasis3D();
			verts[vertIndex].m_binormal = tangentSpaceMatrix.GetJBasis3D();
			//verts[vertIndex].m_normal = Vec3(-1.0f, -1.0f, 0.0f);
			//verts[vertIndex].m_normal = tangentSpaceMatrix.GetKBasis3D();
		}
	}
}
