#pragma once
#include <vector>
#include "Engine/Core/Vertex_PCU.hpp"
#include "Engine/Math/AABB2.hpp"

struct Vertex_PNCU;
struct AABB2;
struct AABB3;
struct LineSegment2;
struct Capsule2;
struct OBB2;
struct OBB3;
struct Mat44;
struct FloatRange;

void TransformVertexArrayXY3D(int numVerts, Vertex_PCU* verts, float uniformScaleXY, float rotationDegreesAboutZ, Vec2 const& translationXY);
void TransformVertexArrayXY3D(int numVerts, Vertex_PCU* verts, float uniformScaleXY, Vec2 iBasis, Vec2 jBasis, Vec2 const& translationXY);
void TransformVertexArray3D(std::vector<Vertex_PCU>& verts, const Mat44& transform);
void AddVertsForAABB2D(std::vector<Vertex_PCU>& verts, AABB2 const& bounds, Rgba8 const& color);
void AddVertsForAABB2D(std::vector<Vertex_PCU>& verts, AABB2 const& bounds, Rgba8 const& color, AABB2 UVs);
void AddVertsForAABB2D(std::vector<Vertex_PCU>& verts, AABB2 const& bounds, Rgba8 const& color, Vec2& uvAtMins, Vec2& uvAtMaxs);
void AddVertsForOBB2D(std::vector<Vertex_PCU>& verts, OBB2 const& box, Rgba8 const& color);
void AddVertsForDisc2D(std::vector<Vertex_PCU>& verts, Vec2 const& center, float radius, Rgba8 const& color);
void AddVertsForDisc3D(std::vector<Vertex_PNCU>& verts, Vec3 const& center, float radius, int numSides, Rgba8 const& color);
void AddVertsForRing3D(std::vector<Vertex_PNCU>& verts, Vec3 const& center, float radius, float thickness, int numSides, Rgba8 const& color);
void AddVertsForRing3D(std::vector<Vertex_PNCU>& verts, std::vector<int>& indices, Vec3 const& center, float radius, float thickness, int numSides, Rgba8 const& color);
void AddVertsForLineSegment2D(std::vector<Vertex_PCU>& verts, LineSegment2 const& lineSegment, float thickness, Rgba8 const& color);
void AddVertsForLineSegment3D(std::vector<Vertex_PCU>& verts, Vec3 const& start, Vec3 const& end, float thickness, Rgba8 const& color);
void AddVertsForCapsule2D(std::vector<Vertex_PCU>& verts, Capsule2 const& capsule, Rgba8 const& color);
void AddVertsForSector2D(std::vector<Vertex_PCU>& verts, Vec2 const& sectorTip, Vec2 const& sectorForwardNormal, float sectorApertureDegrees, float sectorRadius, Rgba8 const& color);
void AddVertsForArrow2D(std::vector<Vertex_PCU>& verts, Vec2 tailPos, Vec2 tipPos, float arrowSize, float lineThickness, Rgba8 const& color);
void AddVertsForQuad3D(std::vector<Vertex_PCU>& verts, const Vec3& bottomLeft, const Vec3& bottomRight, const Vec3& topLeft, const Vec3& topRight, const Rgba8& color, const AABB2& UVs);
void AddVertsForQuad3D(std::vector<Vertex_PCU>& verts, std::vector<unsigned int>& indexes, const Vec3& bottomLeft, const Vec3& bottomRight, const Vec3& topLeft, const Vec3& topRight, const Rgba8& color, const AABB2& UVs);
void AddVertsForAABB3D(std::vector<Vertex_PCU>& verts, const AABB3& bounds, const Rgba8& color = Rgba8::WHITE , const AABB2& UVs = AABB2::ZERO_TO_ONE);
void AddVertsForSphere3D(std::vector<Vertex_PCU>& verts, const Vec3& center, float radius, const Rgba8& color = Rgba8::WHITE, const AABB2& UVs = AABB2::ZERO_TO_ONE, int numLatitudeSlices = 8);
void AddVertsForSphere3DReverseUVs(std::vector<Vertex_PCU>& verts, const Vec3& center, float radius, const Rgba8& color = Rgba8::WHITE, const AABB2& UVs = AABB2::ZERO_TO_ONE, int numLatitudeSlices = 8);
void AddVertsForRing2D(std::vector<Vertex_PCU>& verts, Vec2 const& center, float radius, float thickness, Rgba8 const& color);
void AddVertsForCylinderZ3D(std::vector<Vertex_PCU>& verts, Vec2 const& centerXY, FloatRange const& minMaxZ, float radius, float numSlices, Rgba8 const& tint = Rgba8::WHITE, AABB2 const& UVs = AABB2::ZERO_TO_ONE);
void AddVertsForCylinderZ3DReverseUVsSidesOnly(std::vector<Vertex_PCU>& verts, Vec2 const& centerXY, FloatRange const& minMaxZ, float radius, float numSlices, Rgba8 const& tint = Rgba8::WHITE, AABB2 const& UVs = AABB2::ZERO_TO_ONE);
void AddVertsForConeZ3D(std::vector<Vertex_PCU>& verts, Vec2 const& centerXY, FloatRange const& minMaxZ, float radius, float numSlices, Rgba8 const& tint = Rgba8::WHITE, AABB2 const& UVs = AABB2::ZERO_TO_ONE);
void AddVertsForArrow3D(std::vector<Vertex_PCU>& verts, Vec3 tailPos, Vec3 tipPos, float lineThickness, Rgba8 const& tint = Rgba8::WHITE);
void AddVertsForCylinder3D(std::vector<Vertex_PCU>& verts, const Vec3& start, const Vec3& end, float radius, Rgba8 const& tint = Rgba8::WHITE, AABB2 const& UVs = AABB2::ZERO_TO_ONE, int numSlices = 8);
void AddVertsForCone3D(std::vector<Vertex_PCU>& verts, const Vec3& start, const Vec3& end, float radius, Rgba8 const& tint = Rgba8::WHITE, AABB2 const& UVs = AABB2::ZERO_TO_ONE, int numSlices = 8);
AABB2 GetVertexBounds2D(const std::vector<Vertex_PCU>& verts);
void AddVertsForRoundedQuad3D(std::vector<Vertex_PNCU>& verts, const Vec3& bottomLeft, const Vec3& bottomRight, const Vec3& topLeft, const Vec3& topRight, const Rgba8& color = Rgba8::WHITE, AABB2 const& UVs = AABB2::ZERO_TO_ONE);
void AddVertsForQuad3D(std::vector<Vertex_PNCU>& verts, std::vector<unsigned int>& indexes, const Vec3& bottomLeft, const Vec3& bottomRight, const Vec3& topLeft, const Vec3& topRight, const Rgba8& color = Rgba8::WHITE, AABB2 const& UVs = AABB2::ZERO_TO_ONE);
void AddVertsForQuad3D(std::vector<Vertex_PNCU>& verts, const Vec3& bottomLeft, const Vec3& bottomRight, const Vec3& topLeft, const Vec3& topRight, const Rgba8& color, const AABB2& UVs);
void AddVertsForCube3D(std::vector<Vertex_PCU>& verts, const Vec3& center, const float radius, const Rgba8& color = Rgba8::WHITE, const AABB2& UVs = AABB2::ZERO_TO_ONE);
void AddVertsForOBB3D(std::vector<Vertex_PCU>& verts, const OBB3& m_box, Rgba8 const& tint = Rgba8::WHITE, AABB2 const& UVs = AABB2::ZERO_TO_ONE);
