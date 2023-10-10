#pragma once
#include "Engine/Math/Vec3.hpp"
#include "Engine/Math/OBB2.hpp"
#include "Engine/Math/AABB3.hpp"
#include "Engine/Math/Capsule2.hpp"
#include "Engine/Math/Sphere3.hpp"
#include "Engine/Math/EulerAngles.hpp"
#include <vector>

//-----------------------------------------------------------------------------------------------
struct Vertex_PCU;
struct Spring2D;
struct Point3D;
struct Mat44;

//-----------------------------------------------------------------------------------------------
struct Shapes3D
{
public:
	Shapes3D();
	~Shapes3D();

public:
	std::vector<Sphere3> m_spheres;
	std::vector<Capsule2> m_capsules;
	std::vector<AABB3> m_aabbs;
	std::vector<OBB2> m_obbs;
};

//-----------------------------------------------------------------------------------------------
class PBDRope3D
{
public:
	PBDRope3D(int totalPoints, float totalMassOfRope, float dampingCoefficient, float stretchCoefficient,
		float compressionCoefficient, float bendingCoefficient, float staticFrictionCoefficient, float kineticFrictionCoefficient,
		int totalSolverIterations, Vec3 start, Vec3 end, float physicsTimestep, bool isDebugMode = false);
	PBDRope3D();
	~PBDRope3D();
	void Startup();
	void Shutdown();
	void Update(float deltaSeconds);
	void Render(std::vector<Vertex_PCU>& verts) const;
	float GetCurrentLengthOfTheRope();
	void ClearShapeReferences();
	Mat44 GetModelMatrix() const;

private:
	void ProjectConstraints();
	void ProjectDistanceConstraint(Point3D* pointA, Point3D* pointB);
	void ProjectBendingConstraint(Point3D* pointA, Point3D* pointB, Point3D* pointC);
	void ProjectCollisionConstraints(Point3D* point);

public:
	float m_physicsTimestep = 0.0005f;
	float m_physicsDebt = 0.0f;
	float m_desiredDistance = 0.0f;
	float m_bendingConstraintDistance = 0.0f;
	float m_bendingCoefficient = 0.0f; // Lower the bend value, the greater angle of bending
	float m_stretchingCoefficient = 0.0f; // Lower the stretch value the greater the elasticity
	float m_compressionCoefficient = 0.0f;
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

	Vec3 m_ropeStartPosition;
	Vec3 m_ropeEndPosition;
	EulerAngles m_orientation;
	std::vector<Point3D*> m_points;
	Shapes3D* m_shapes = nullptr;
	std::vector<Capsule2> m_selfCollisionCapsules;
};