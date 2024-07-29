#pragma once
#include "Particles3D.hpp"
#include "Constraint3D.hpp"
#include "Engine/Math/Vec2.hpp"
#include "Engine/Math/Vec3.hpp"
#include "Engine/Math/AABB3.hpp"

//TODO: WORK IN PROGRESS

//-----------------------------------------------------------------------------------------------
class	Renderer;
class   Shader;

//-----------------------------------------------------------------------------------------------
class ClothSimulation3D
{
public:
	ClothSimulation3D() {}
	~ClothSimulation3D() {}
	explicit					ClothSimulation3D(Renderer* renderer, AABB3 worldBounds, Vec2 dimensions, int totalNumberOfParticles,
								int totalSolverIterations, Vec3 attachPointTopLeft, Vec3 attachPointTopRight);

	//Update/Render
	void						Update(float deltaSeconds);
	void						Render() const;

	//Misc Public Methods
	void						UpdateGrabbedClothParticle(int sentParticleIndex, Vec3 const& newPosition);

protected:
	void						UpdateCPU();
	void						UpdateGaussSeidel();
	void						ProjectConstraintsGaussSeidel();
	void						ProjectDistanceConstraintGaussSeidel(int constraintIndex);
	void						ProjectWorldBoundsConstraintsSpheresGaussSeidel(int sentParticleIndex);

	void						RenderDebugVerts() const;
	void						InitializeShaders();

public:
	Particles3D					m_particles;

public:
	Renderer*					m_renderer = nullptr;
	Shader*						m_renderShader = nullptr;
	std::vector<Constraint3D>	m_distanceConstraints;
	std::vector<float>			m_distanceConstraintsOriginalDists;
	AABB3						m_worldBounds;
	Vec2						m_dimensions;
	Vec3						m_grabbedParticlePosition;
	int							m_grabbedParticleIndex = -1;
	int							m_totalNumberOfParticles;
	int							m_totalSolverIterations;
	float						m_physicsTimestep = 0.005f;
	float						m_physicsDebt = 0.0f;
	float						m_gravityCoefficient = 9.81f;
	float						m_dampingCoefficient = 0.99925f;
	float						m_originalHorizontalDistance = 0.0f;
	float						m_originalVerticalDistance = 0.0f;
	float						m_originalDiagonalDistance = 0.0f;
	float						m_clothThickness = 0.01f;
	float						m_kineticFrictionCoefficient = 0.05f;
	float						m_stretchingCoefficient = 0.0f;
	float						m_compressionCoefficient = 0.0f;
	float						m_bendingCoefficient = 0.0f;
	float						m_simulationStartTime = 0.0f;
	float						m_simulationEndTime = 0.0f;
	int							m_numberOfRows;
	int							m_numberOfParticlesPerRow;
	bool						m_isDebugCloth = false;
	bool						m_isSelfCollisionEnabled = false;
};