#pragma once
#include "Engine/Math/AABB3.hpp"
#include <vector>

//-----------------------------------------------------------------------------------------------
constexpr float VELOCITY_THRESHOLD = 0.001f;

//-----------------------------------------------------------------------------------------------
class  RigidBody3D;
struct ConvexHull3D;

//-----------------------------------------------------------------------------------------------
struct ContactManifold3D
{
	RigidBody3D* m_a;
	RigidBody3D* m_b;
	Vec3							m_contactNormal;
	std::vector<Vec3>				m_contactPoints;
	float							m_penetrationDepth;
};

//-----------------------------------------------------------------------------------------------
class PhysicsScene3D
{
public:
	PhysicsScene3D() {}
	~PhysicsScene3D() {}
	explicit PhysicsScene3D(AABB3 const& m_worldBounds);

	void							Update(float deltaSeconds);
	void							AddRigidBody(RigidBody3D* rigidBody);
	bool							RemoveRigidBody(RigidBody3D* rigidBody);

private:
	void							UpdateRigidBodies();
	void							DetectCollisions();
	bool							DetectCollisionsWorldBounds();
	void							DetectCollisionsRigidBodies();
	bool							DoesExistingManifoldExist(RigidBody3D* a, RigidBody3D* b);
	bool							RigidBodyVSGroundPlane(RigidBody3D* a);
	bool							BroadPhaseCheck(RigidBody3D* a, RigidBody3D* b);
	bool							NarrowPhaseCheck(RigidBody3D* a, RigidBody3D* b);
	bool							SAT(RigidBody3D* a, RigidBody3D* b);
	bool							GJK_EPA(RigidBody3D* a, RigidBody3D* b);
	void							GenerateContactData(RigidBody3D* a, RigidBody3D* b);
	void							ComputeContactPoints(Vec3 const& contactNormal, RigidBody3D* referenceBody, Vec3& referenceVert, int referenceIndex,
									RigidBody3D* incidentBody, Vec3& incidentVert, int incidentIndex, ContactManifold3D* contact);

	void							ResolveCollisions();

public:
	bool							m_debugDraw = true;

private:
	std::vector<RigidBody3D*>		m_rigidBodies;
	std::vector<ContactManifold3D*>	m_contactManifolds;
	AABB3							m_worldBounds;
	float							m_physicsTimestep = 0.0005f;
	float							m_physicsDebt = 0.0f;
	bool							m_isSAT = false;
};