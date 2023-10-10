#pragma once
#include "Engine/Math/Vec2.hpp"
#include "Engine/Math/OBB2.hpp"
#include "Engine/Math/AABB2.hpp"
#include "Engine/Math/Capsule2.hpp"
#include "Engine/Math/Disc2.hpp"
#include <vector>

//-----------------------------------------------------------------------------------------------
struct Vertex_PCU;
struct Spring2D;
struct Point2D;

//-----------------------------------------------------------------------------------------------
struct Shapes2D
{
public:
	Shapes2D();
	~Shapes2D();

public:
	std::vector<Disc2> m_discs;
	std::vector<Capsule2> m_capsules;
	std::vector<AABB2> m_aabbs;
	std::vector<OBB2> m_obbs;
};

//-----------------------------------------------------------------------------------------------
class PBDRope2D
{
public:
	PBDRope2D(int totalPoints, float totalMassOfRope, float dampingCoefficient, float stretchCoefficient, 
		float bendingCoefficient, float staticFrictionCoefficient, float kineticFrictionCoefficient,  
		int totalSolverIterations, Vec2 start, Vec2 end, float physicsTimestep, bool isDebugMode = false);
	PBDRope2D();
	~PBDRope2D();
	void Startup();
	void Shutdown();
	void Update(float deltaSeconds);
	void Render(std::vector<Vertex_PCU>& verts) const;
	float GetCurrentLengthOfTheRope();
	void ClearShapeReferences();

private:
	void ProjectConstraints();
	void ProjectDistanceConstraint(Point2D* pointA, Point2D* pointB);
	void ProjectBendingConstraint(Point2D* pointA, Point2D* pointC);
	void ProjectCollisionConstraints(Point2D* point);

public:
	float m_physicsTimestep = 0.0005f;
	float m_physicsDebt = 0.0f;
	float m_desiredDistance = 0.0f;
	float m_bendingConstraintDistance = 0.0f;
	float m_bendingCoefficient = 0.0f; // Lower the bend value, the greater angle of bending
	float m_stretchingCoefficient = 0.0f; // Lower the stretch value the greater the elasticity
	float m_dampingCoefficient = 0.0f;
	float m_gravityCoefficient = 9.81f;
	int m_totalSolverIterations = 10;
	int m_numberOfPointsInRope = 0;
	bool m_isGravityEnabled = true;
	bool m_isDebugMode = false;
	float m_ropeRadius = 0.025f;
	float m_staticFrictionCoefficient = 0.0f;
	float m_kineticFrictionCoefficient = 0.0f;
	float m_totalLengthOfRope = 0.0f;

	Vec2 m_ropeStartPosition;
	Vec2 m_ropeEndPosition;
	std::vector<Point2D*> m_points;
	Shapes2D* m_shapes = nullptr;
	std::vector<Capsule2> m_selfCollisionCapsules;
};