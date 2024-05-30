#pragma once
#include "Engine/Math/Vec2.hpp"
#include <vector>

//-----------------------------------------------------------------------------------------------
struct Vertex_PCU;
struct Spring2D;
struct Point2D;

//-----------------------------------------------------------------------------------------------
enum IntegrationMethod
{
	EXPLICIT_EULER,
	SEMI_IMPLICIT_EULER,
	MIDPOINT,
	RUNGE_KUTTA,
	VERLET,
	COUNT
};

//-----------------------------------------------------------------------------------------------
class MassSpringRope2D
{
public:
	MassSpringRope2D(IntegrationMethod integrationMethod, int totalPoints, float stiffnessConstant, 
		float dampingConstant, float massOfEachPoint, Vec2 start, Vec2 end);
	MassSpringRope2D();
	~MassSpringRope2D();
	void Startup();
	void Shutdown();
	void Update(float deltaSeconds);
	void Render(std::vector<Vertex_PCU>& verts) const;

private:
	void UpdateExplicitEuler();
	void UpdateSemiImplicitEuler();
	void UpdateMidpoint();
	void UpdateRungeKutta();
	void UpdateVerlet();

public:
	std::vector<Spring2D*>	m_springs;
	std::vector<Point2D*>	m_points;
	float					m_physicsTimestep = 0.0005f;
	float					m_physicsDebt = 0.0f;
	int						m_numberOfPointsInRope = 0;
	int						m_verletSolverIterations = 0;
	float					m_stiffnessConstant = 0.0f;
	float					m_dampingConstant = 0.0f;
	float					m_ropeRadius = 0.025f;
	Vec2					m_ropeStartPosition;
	Vec2					m_ropeEndPosition;
	float					m_gravityConstant = 9.81f;
	IntegrationMethod		m_integrationMethod = IntegrationMethod::SEMI_IMPLICIT_EULER;
	bool					m_isGravityEnabled = true;
	bool					m_isDebugMode = false;
};