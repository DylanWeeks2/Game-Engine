#pragma once
#include "Engine/Math/Plane3D.hpp"
#include "RigidBody3D.hpp"
#include "Engine/Math/LineSegment3.hpp"
#include <vector>

//-----------------------------------------------------------------------------------------------
struct	Vec3;
struct	ConvexHull3D;

//-----------------------------------------------------------------------------------------------
struct Collider3D
{
public:
	Collider3D(RigidBody3D* rigidBody);
	~Collider3D(){}

	Vec3 GetFurthestPointInDireciton(Vec3 const& direciton);
	std::vector<Vec3> GetFurthestPointsInDireciton(Vec3 const& direciton);
	void GenerateBoundingSphereRadius();

public:
	RigidBody3D*				m_rigidBody = nullptr;
	ConvexHull3D*				m_hull = nullptr;
	float						m_boundingSphereRadius;
};