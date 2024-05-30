#include "RopeSimulation3D.hpp"
#include "Engine/Core/Vertex_PCU.hpp"
#include "Engine/Core/Vertex_PCUTBN.hpp"
#include "Engine/Core/VertexUtils.hpp"
#include "Engine/Core/ErrorWarningAssert.hpp"
#include "Engine/Simulations/Particles3D.hpp"
#include "Engine/Core/EngineCommon.hpp"
#include "Engine/Core/Time.hpp"
#include "Engine/Math/MathUtils.hpp"
#include "Engine/Math/LineSegment3.hpp"
#include "Engine/Math/Mat44.hpp"
#include "Engine/Renderer/Renderer.hpp"
#include "Engine/Core/DebugRender.hpp"
#include "Engine/Math/Plane3D.hpp"
#include "Engine/Simulations/Constraint3D.hpp"
#include "Engine/Renderer/Renderer.hpp"
#include "Engine/Renderer/ConstantBuffer.hpp"
#include "Engine/Renderer/StructuredBuffer.hpp"
#include "Engine/Renderer/VertexBuffer.hpp"
#include "Engine/Math/Vec4.hpp"
#include "Engine/Renderer/Query.hpp"
#include <cmath>

//-----------------------------------------------------------------------------------------------
RopeSimulation3D::RopeSimulation3D(Renderer* renderer, AABB3 worldBounds, int totalParticles, float totalMassOfRope, float dampingCoefficient, float stretchCoefficient,
	float compressionCoefficient, float bendingCoefficient, float staticFrictionCoefficient, float kineticFrictionCoefficient, int totalSolverIterations,
	Vec3 start, Vec3 end, float physicsTimestep, CollisionType collisionType, bool isSelfCollisionEnabled, int totalCollisionObjects, int totalAABBs, int totalOBBs, 
	int totalCylinders, int totalCapsules, int totalSpheres)
	: m_renderer(renderer)
	, m_worldBounds(worldBounds)
	, m_numberOfParticlesInRope(totalParticles)
	, m_dampingCoefficient(dampingCoefficient)
	, m_stretchingCoefficient(stretchCoefficient)
	, m_compressionCoefficient(compressionCoefficient)
	, m_bendingCoefficient(bendingCoefficient)
	, m_staticFrictionCoefficient(staticFrictionCoefficient)
	, m_kineticFrictionCoefficient(kineticFrictionCoefficient)
	, m_totalSolverIterations(totalSolverIterations)
	, m_ropeStartPosition(start)
	, m_ropeEndPosition(end)
	, m_totalLengthOfRope((start - end).GetLength())
	, m_physicsTimestep(physicsTimestep)
	, m_collisionType(collisionType)
	, m_isSelfCollisionEnabled(isSelfCollisionEnabled)
	, m_totalCollisionObjects(totalCollisionObjects)
	, m_totalAABBs(totalAABBs)
	, m_totalOBBs(totalOBBs)
	, m_totalCylinders(totalCylinders)
	, m_totalCapsules(totalCapsules)
	, m_totalSpheres(totalSpheres)
{
	//Rope Creation/Initialization
	Vec3 displacement = end - start;
	Vec3 direction = displacement.GetNormalized();
	float magnitude = displacement.GetLength();
	m_desiredDistance = magnitude / (totalParticles - 1);
	m_bendingConstraintDistance = (m_desiredDistance * (2.0f));
	float massPerParticle = (totalMassOfRope / totalParticles);
	float inverseMassPerParticle = 1.0f / massPerParticle;
	Vec3 zero = Vec3(0.0, 0.0, 0.0);


	//Particle Declarations
	for (int particleIndex = 1; particleIndex < totalParticles; particleIndex++)
	{
		if (particleIndex == 1)
		{
			m_particles.m_positions.push_back(start + (direction * ((particleIndex - 1) * m_desiredDistance)));
			m_particles.m_velocities.push_back(zero);
			m_particles.m_proposedPositions.push_back(zero);
			m_particles.m_jacobiCorrections.push_back(Vec4(0.0, 0.0, 0.0, 0.0));
			m_particles.m_collisionNormals.push_back(zero);
			m_particles.m_macroBitRegions.push_back(0);
			m_particles.m_microBitRegions.push_back(0);
			m_particles.m_masses.push_back(massPerParticle);
			m_particles.m_inverseMasses.push_back(inverseMassPerParticle);
			m_particles.m_isAttached.push_back(1);
			m_particles.m_isSelfCollision.push_back(0);
		}

		m_particles.m_positions.push_back(start + (direction * (particleIndex * m_desiredDistance)));
		m_particles.m_velocities.push_back(zero);
		m_particles.m_proposedPositions.push_back(zero);
		m_particles.m_jacobiCorrections.push_back(Vec4(0.0, 0.0, 0.0, 0.0));
		m_particles.m_collisionNormals.push_back(zero);
		m_particles.m_macroBitRegions.push_back(0);
		m_particles.m_microBitRegions.push_back(0);
		m_particles.m_masses.push_back(massPerParticle);
		m_particles.m_inverseMasses.push_back(inverseMassPerParticle);
		m_particles.m_isAttached.push_back(0);
		m_particles.m_isSelfCollision.push_back(0);

		//Collision Capsules
		Capsule3 capsule;
		capsule.m_bone.m_start = m_particles.m_positions[m_particles.m_positions.size() - 2];
		capsule.m_bone.m_end = m_particles.m_positions[m_particles.m_positions.size() - 1];
		capsule.m_radius = m_ropeRadius;
		capsule.m_bone.m_radius = m_ropeRadius;
		CapsuleCollisionObject capsuleObject = CapsuleCollisionObject(Vec3(), 0.0f, capsule);
		m_collisionCapsules.push_back(capsuleObject);
	}

	//Initialization of Bit Regions
	InitializeBitRegions();
	m_worldScaleX = abs(m_worldBounds.m_maxs.x - m_worldBounds.m_mins.x) * m_bitRegionScale;
	m_worldScaleY = abs(m_worldBounds.m_maxs.y - m_worldBounds.m_mins.y) * m_bitRegionScale;
	m_macroScaleX = 1.0f / m_worldScaleX;
	m_macroScaleY = 1.0f / m_worldScaleY;
	m_microScaleX = 8.0f;
	m_microScaleY = 8.0f;

	//Constraint Generation
	GenerateConstraints();

	//Shader and buffer Initializations
	InitializeShaders();
}

//-----------------------------------------------------------------------------------------------
RopeSimulation3D::~RopeSimulation3D()
{
	delete m_cbRopeData;
	delete m_cbGameInteraciton;
	delete m_sbGameInteraction;
	delete m_sbParticlePositions;
	delete m_sbParticleVelocities;
	delete m_sbParticleProposedPositions;
	delete m_sbParticleJacobiCorrections;
	delete m_sbParticleCollisionNormals;
	delete m_sbParticleMacroBitRegions;
	delete m_sbParticleMicroBitRegions;
	delete m_sbParticleMasses;
	delete m_sbParticleInverseMasses;
	delete m_sbParticleIsAttached;
	delete m_sbAABBs;
	delete m_sbOBBs;
	delete m_sbSpheres;
	delete m_sbCapsules;
	delete m_sbCylinders;
	delete m_sbRopeCapsules;
	delete m_structuredBufferVerts;
	delete m_startGPUQuery;
	delete m_endGPUQuery;
	delete m_bitRegionVertexBuffer;
}

//-----------------------------------------------------------------------------------------------
void RopeSimulation3D::Startup()
{
}

//-----------------------------------------------------------------------------------------------
void RopeSimulation3D::Shutdown()
{
}

//-----------------------------------------------------------------------------------------------
void RopeSimulation3D::Update(float deltaSeconds)
{
	UpdateGPUBuffers();
	m_simulationStartTime = float(GetCurrentTimeSeconds());

	//Game Updates
	if (m_isGPUSimulated)
	{
		//Game Updates
		if (m_shouldRunGameUpdateComputeShader == true)
		{
			//Constant Buffers
			if (m_cbGameInteraciton)
			{
				delete m_cbGameInteraciton;
			}
			m_cbGameInteraciton = m_renderer->CreateConstantBuffer(sizeof(GameConstantBufferVariables));
			m_renderer->CopyCPUToGPU(&m_gameInteractionVariables, sizeof(GameConstantBufferVariables), m_cbGameInteraciton);
			m_renderer->BindConstantBuffer(0, m_cbGameInteraciton); 
			m_renderer->BindConstantBuffer(1, m_cbRopeData);
			
			//Binding Structured Buffers
			m_renderer->BindStructuredBufferUAVCS(0, m_sbParticlePositions);
			m_renderer->BindStructuredBufferUAVCS(1, m_sbParticleProposedPositions);
			m_renderer->BindStructuredBufferUAVCS(2, m_sbParticleIsAttached);
			m_renderer->BindStructuredBufferUAVCS(3, m_sbGameInteraction);
			//Dispatch
			m_renderer->DispatchComputeShader(m_csGameInteraction, 1, 1, 1);
			//Unbinding Structured Buffers
			m_renderer->UnbindStructuredBufferUAVCS(0);
			m_renderer->UnbindStructuredBufferUAVCS(1);
			m_renderer->UnbindStructuredBufferUAVCS(2);
			m_renderer->UnbindStructuredBufferUAVCS(3);
		}

		//Bindings
		m_renderer->UnbindStructuredBufferUAVCS(0);
		m_renderer->BindConstantBuffer(0, m_cbRopeData);
		m_renderer->BindStructuredBufferUAVCS(0, m_sbParticlePositions);
		m_renderer->BindStructuredBufferUAVCS(1, m_sbParticleVelocities);
		m_renderer->BindStructuredBufferUAVCS(2, m_sbParticleProposedPositions);
		m_renderer->BindStructuredBufferUAVCS(3, m_sbParticleJacobiCorrections);
		m_renderer->BindStructuredBufferUAVCS(4, m_sbParticleCollisionNormals);
		m_renderer->BindStructuredBufferUAVCS(5, m_sbParticleMacroBitRegions);
		m_renderer->BindStructuredBufferUAVCS(6, m_sbParticleMicroBitRegions);
		m_renderer->BindStructuredBufferUAVCS(7, m_sbRopeCapsules);
		m_renderer->BindStructuredBufferSRVCS(0, m_sbParticleIsAttached);
		m_renderer->BindStructuredBufferSRVCS(1, m_sbParticleMasses);
		m_renderer->BindStructuredBufferSRVCS(2, m_sbParticleInverseMasses);
		if (m_totalAABBs != 0)
			m_renderer->BindStructuredBufferSRVCS(3, m_sbAABBs);
		if (m_totalOBBs != 0)
			m_renderer->BindStructuredBufferSRVCS(4, m_sbOBBs);
		if (m_totalCapsules != 0)
			m_renderer->BindStructuredBufferSRVCS(5, m_sbCapsules);
		if (m_totalCylinders)
			m_renderer->BindStructuredBufferSRVCS(6, m_sbCylinders);
		if (m_totalSpheres)
			m_renderer->BindStructuredBufferSRVCS(7, m_sbSpheres);
	}

	

	//Main Update Loop
	m_physicsDebt += deltaSeconds;
	m_renderer->EndQuery(m_startGPUQuery);
	while (m_physicsDebt > m_physicsTimestep)
	{
		if (m_isGPUSimulated)
		{
			UpdateGPU();
		}
		else
		{
			UpdateCPU();
		}
		m_physicsDebt -= m_physicsTimestep;
	}
	m_renderer->EndQuery(m_endGPUQuery);

	if (m_isGPUSimulated)
	{
		//Unbinding
		m_renderer->UnbindStructuredBufferUAVCS(0);
		m_renderer->UnbindStructuredBufferUAVCS(1);
		m_renderer->UnbindStructuredBufferUAVCS(2);
		m_renderer->UnbindStructuredBufferUAVCS(3);
		m_renderer->UnbindStructuredBufferUAVCS(4);
		m_renderer->UnbindStructuredBufferUAVCS(5);
		m_renderer->UnbindStructuredBufferUAVCS(6);
		m_renderer->UnbindStructuredBufferUAVCS(7);
		m_renderer->UnbindStructuredBufferSRVCS(0);
		m_renderer->UnbindStructuredBufferSRVCS(1);
		m_renderer->UnbindStructuredBufferSRVCS(2);
		m_renderer->CopyGPUToCPU(m_particles.m_positions.data(), m_particles.m_positions.size() * sizeof(Vec3), m_sbParticlePositions);
	}


	if (m_isGPUSimulated == false)
	{
		//CPU simulation structured buffer
		delete m_sbParticlePositions;
		m_sbParticlePositions = m_renderer->CreateStructuredBuffer(m_particles.m_positions.size() * sizeof(Vec3), sizeof(Vec3), m_particles.m_positions.data());
	}
	
	//Vertex Updating
	if (m_isDebugRope == false)
	{
		m_renderer->BindConstantBuffer(0, m_cbRopeData);
		m_renderer->BindStructuredBufferSRVCS(0, m_sbParticlePositions);
		m_renderer->BindStructuredBufferUAVCS(1, m_structuredBufferVerts);
		m_renderer->DispatchComputeShader(m_csAddVerts, m_numberOfParticlesInRope, m_dispatchThreadY, m_dispatchThreadZ);
		m_renderer->UnbindStructuredBufferSRVCS(0);
		m_renderer->UnbindStructuredBufferUAVCS(1);
		m_renderer->UnbindConstantBuffer(0);
	}


	m_readyToQuery = true;
	m_simulationEndTime = float(GetCurrentTimeSeconds());
}

//-----------------------------------------------------------------------------------------------
void RopeSimulation3D::UpdateCPU()
{
	if (m_isJacobiSolver)
	{
		UpdateJacobi();
	}
	else
	{
		UpdateGaussSeidel();
	}
}

//-----------------------------------------------------------------------------------------------
void RopeSimulation3D::UpdateGPU()
{
	//Initial Updates
	m_renderer->DispatchComputeShader(m_csInitialUpdates, m_dispatchThreadX, m_dispatchThreadY, m_dispatchThreadZ);
	for (int solverIterationIndex = 0; solverIterationIndex < m_totalSolverIterations; solverIterationIndex++)
	{
		//Distance/Bending Constraints
		m_renderer->DispatchComputeShader(m_csProjectNonCollisionConstraints, m_dispatchThreadX, m_dispatchThreadY, m_dispatchThreadZ);
		//Update After Non-Collision Constraints
		m_renderer->DispatchComputeShader(m_csUpdateAfterNonCollisionConstraints, m_dispatchThreadX, m_dispatchThreadY, m_dispatchThreadZ);
		//Collisions
		if (m_collisionType == CollisionType::SPHERES)
		{
			//Sphere Collision
			m_renderer->DispatchComputeShader(m_csProjectCollisionConstraintsSpheres, m_numberOfParticlesInRope, m_dispatchThreadY, m_dispatchThreadZ);
		}
		else if (m_collisionType == CollisionType::CAPSULES)
		{
			int capsuleDispatch = int(std::ceil(int(m_numberOfParticlesInRope) / 2));
			//Bit Regions Capsules
			m_renderer->DispatchComputeShader(m_csInitializeBitRegionsCapsules, m_dispatchThreadX, m_dispatchThreadY, m_dispatchThreadZ);
			//Capsule Collision Phase 1
			m_renderer->DispatchComputeShader(m_csCapsuleCollisionsPhaseOne, capsuleDispatch, m_dispatchThreadY, m_dispatchThreadZ);
			//Bit Regions Capsules
			m_renderer->DispatchComputeShader(m_csInitializeBitRegionsCapsules, m_dispatchThreadX, m_dispatchThreadY, m_dispatchThreadZ);
			//Capsule Collision Phase 2
			m_renderer->DispatchComputeShader(m_csCapsuleCollisionsPhaseTwo, capsuleDispatch, m_dispatchThreadY, m_dispatchThreadZ);
		}
	}
	//Final Updates
	m_renderer->DispatchComputeShader(m_csFinalUpdates, m_dispatchThreadX, m_dispatchThreadY, m_dispatchThreadZ);
}

//-----------------------------------------------------------------------------------------------
void RopeSimulation3D::Render() const
{
	if (m_isDebugRope == false)
	{
		m_renderer->SetModelConstants();
		m_renderer->SetRasterizerMode(RasterizerMode::SOLID_CULL_NONE);
		Texture* diffuseTexture = m_renderer->CreateOrGetTextureFromFile("Data/Images/Rope_Diffuse.jpg");
		Texture* normalTexture = m_renderer->CreateOrGetTextureFromFile("Data/Images/Rope_Normal.png");
		Texture* specGlossEmitTexture = m_renderer->CreateOrGetTextureFromFile("Data/Images/Brick_SpecGlossEmit.png");
		m_renderer->BindTextures(diffuseTexture, normalTexture, specGlossEmitTexture);
		m_renderer->DrawStructuredBuffer(m_totalVerts, m_renderShader, m_structuredBufferVerts, 3, nullptr, 0, PrimitiveTopology::D3D11_PRIMITIVE_TOPOLOGY_TRIANGLELIST);
	}
	else
	{
		DebugRenderParticles();
	}

	if (m_isDebugBitRegions)
	{
		DebugRenderBitRegions();
	}
}

//-----------------------------------------------------------------------------------------------
void RopeSimulation3D::InitializeGPUCollisionObjects()
{
	if (m_collisionObjects.size() == 0)
		return;
	if (m_sbAABBs)
		delete m_sbAABBs;
		m_sbAABBs = nullptr;	
	if (m_sbOBBs)
		delete m_sbOBBs;
		m_sbOBBs = nullptr;
	if (m_sbCylinders)
		delete m_sbCylinders;
		m_sbCylinders = nullptr;
	if (m_sbCapsules)
		delete m_sbCapsules;
		m_sbCapsules = nullptr;
	if (m_sbSpheres)
		delete m_sbSpheres;
		m_sbSpheres = nullptr;
	if (m_cbRopeData)
		delete m_cbRopeData;
		m_cbRopeData = nullptr;
	std::vector<AABBCollisionObject> aabbs;
	std::vector<OBBCollisionObject> obbs;
	std::vector<CylinderCollisionObject> cylinders;
	std::vector<CapsuleCollisionObject> capsules;
	std::vector<SphereCollisionObject> spheres;
	for (int objectIndex = 0; objectIndex < m_totalCollisionObjects; objectIndex++)
	{
		if (objectIndex < m_totalAABBs)
		{
			aabbs.push_back(*dynamic_cast<AABBCollisionObject*>(m_collisionObjects[objectIndex]));
		}
		else if (objectIndex < m_totalAABBs + m_totalOBBs)
		{
			obbs.push_back(*dynamic_cast<OBBCollisionObject*>(m_collisionObjects[objectIndex]));
		}
		else if (objectIndex < m_totalAABBs + m_totalOBBs + m_totalCylinders)
		{
			cylinders.push_back(*dynamic_cast<CylinderCollisionObject*>(m_collisionObjects[objectIndex]));
		}
		else if (objectIndex < m_totalAABBs + m_totalOBBs + m_totalCylinders + m_totalCapsules)
		{
			capsules.push_back(*dynamic_cast<CapsuleCollisionObject*>(m_collisionObjects[objectIndex]));
		}
		else if (objectIndex < m_totalAABBs + m_totalOBBs + m_totalCylinders + m_totalCapsules + m_totalSpheres)
		{
			spheres.push_back(*dynamic_cast<SphereCollisionObject*>(m_collisionObjects[objectIndex]));
		}
	}
	if (aabbs.size() != 0)
	{
		m_sbAABBs = m_renderer->CreateStructuredBuffer(sizeof(AABBCollisionObject) * aabbs.size(), sizeof(AABBCollisionObject), aabbs.data());
		m_renderer->BindStructuredBufferUAVCS(1, m_sbAABBs);
	}
	if (obbs.size() != 0)
	{
		m_sbOBBs = m_renderer->CreateStructuredBuffer(sizeof(OBBCollisionObject) * obbs.size(), sizeof(OBBCollisionObject), obbs.data());
		m_renderer->BindStructuredBufferUAVCS(2, m_sbOBBs);
	}
	if (cylinders.size() != 0)
	{
		m_sbCylinders = m_renderer->CreateStructuredBuffer(sizeof(CylinderCollisionObject) * cylinders.size(), sizeof(CylinderCollisionObject), cylinders.data());
		m_renderer->BindStructuredBufferUAVCS(3, m_sbCylinders);
	}
	if (capsules.size() != 0)
	{
		m_sbCapsules = m_renderer->CreateStructuredBuffer(sizeof(CapsuleCollisionObject) * capsules.size(), sizeof(CapsuleCollisionObject), capsules.data());
		m_renderer->BindStructuredBufferUAVCS(4, m_sbCapsules);
	}
	if (spheres.size() != 0)
	{
		m_sbSpheres = m_renderer->CreateStructuredBuffer(sizeof(SphereCollisionObject) * spheres.size(), sizeof(SphereCollisionObject), spheres.data());
		m_renderer->BindStructuredBufferUAVCS(5, m_sbSpheres);
	}
	//Update rope constant buffer
	m_constantBufferVars.m_totalCollisionObjects = m_totalCollisionObjects;
	m_constantBufferVars.m_totalAABBs = m_totalAABBs;
	m_constantBufferVars.m_totalOBBs = m_totalOBBs;
	m_constantBufferVars.m_totalCylinders = m_totalCylinders;
	m_constantBufferVars.m_totalCapsules = m_totalCapsules;
	m_constantBufferVars.m_totalSpheres = m_totalSpheres;
	m_cbRopeData = m_renderer->CreateConstantBuffer(sizeof(RopeSimualtionConstantBufferVariables));
	m_renderer->CopyCPUToGPU(&m_constantBufferVars, sizeof(RopeSimualtionConstantBufferVariables), m_cbRopeData);

	//Dispatch Logic
	m_renderer->BindConstantBuffer(0, m_cbRopeData);
	m_renderer->DispatchComputeShader(m_csInitializeCollisionObjects, m_totalCollisionObjects, 1, 1);
	m_renderer->UnbindStructuredBufferUAVCS(1);
	m_renderer->UnbindStructuredBufferUAVCS(2);
	m_renderer->UnbindStructuredBufferUAVCS(3);
	m_renderer->UnbindStructuredBufferUAVCS(4);
	m_renderer->UnbindStructuredBufferUAVCS(5);
}

//-----------------------------------------------------------------------------------------------
float RopeSimulation3D::GetCurrentLengthOfTheRope()
{
	float currentLength = 0.0f;
	for (int particleIndex = 1; particleIndex < m_particles.m_positions.size(); particleIndex++)
	{
		currentLength += (m_particles.m_positions[particleIndex - 1] - m_particles.m_positions[particleIndex]).GetLength();
	}
	return currentLength;
}

//-----------------------------------------------------------------------------------------------
void RopeSimulation3D::ClearShapeReferences()
{
	for (int collisionObjectIndex = 0; collisionObjectIndex < m_collisionObjects.size(); collisionObjectIndex++)
	{
		if (m_collisionObjects[collisionObjectIndex])
		{
			delete m_collisionObjects[collisionObjectIndex];
			m_collisionObjects[collisionObjectIndex] = nullptr;
		}
	}

	m_collisionObjects.clear();
}


//-----------------------------------------------------------------------------------------------
void RopeSimulation3D::UpdateGrabbedRopeParticle(int sentParticleIndex, Vec3 const& newPosition)
{
	//Position Updates
	m_grabbedParticlePosition = newPosition;
	m_grabbedParticleIndex = sentParticleIndex;
	m_particles.m_positions[sentParticleIndex] = newPosition;
	m_particles.m_proposedPositions[sentParticleIndex] = newPosition;
}

//-----------------------------------------------------------------------------------------------
void RopeSimulation3D::AttachRopeParticle(int const& particleIndex)
{
	m_particles.m_isAttached[particleIndex] = true;

	bool isAttached = false;
	for (int attachedIndex = 0; attachedIndex < m_attachedParticleIndices.size(); attachedIndex++)
	{
		if (particleIndex == m_attachedParticleIndices[attachedIndex])
		{
			isAttached = true;
		}
	}

	if (isAttached == false)
	{
		m_attachedParticleIndices.push_back(particleIndex);
	}
}

//-----------------------------------------------------------------------------------------------
void RopeSimulation3D::UnattachRopeParticle(int const& particleIndex)
{
	m_particles.m_isAttached[particleIndex] = false;
	m_grabbedParticleIndex = -1;
	m_attachedParticleIndices.erase(std::remove(m_attachedParticleIndices.begin(), m_attachedParticleIndices.end(), particleIndex), m_attachedParticleIndices.end());
}

//-----------------------------------------------------------------------------------------------
void RopeSimulation3D::UpdateCollisionObjectBitRegions()
{
	BitRegionDetectionAllCollisionObjects();
}

//-----------------------------------------------------------------------------------------------
void RopeSimulation3D::InitializeShaders()
{
	m_constantBufferVars.m_ropeRadius = m_ropeRadius;
	m_constantBufferVars.m_gravityCoefficient = m_gravityCoefficient;
	m_constantBufferVars.m_physicsTimestep = m_physicsTimestep;
	m_constantBufferVars.m_totalParticles = int(m_particles.m_positions.size());
	m_constantBufferVars.m_dampingCoefficient = m_dampingCoefficient;
	m_constantBufferVars.m_totalSolverIterations = m_totalSolverIterations;
	m_constantBufferVars.m_desiredDistance = m_desiredDistance;
	m_constantBufferVars.m_compressionCoefficient = m_compressionCoefficient;
	m_constantBufferVars.m_stretchingCoefficient = m_stretchingCoefficient;
	m_constantBufferVars.m_kineticFrictionCoefficient = m_kineticFrictionCoefficient;
	m_constantBufferVars.m_bendingConstraintDistance = m_bendingConstraintDistance;
	m_constantBufferVars.m_bendingCoefficient = m_bendingCoefficient;
	m_constantBufferVars.m_worldBoundsMins = m_worldBounds.m_mins;
	m_constantBufferVars.m_worldBoundsMaxs = m_worldBounds.m_maxs;
	m_constantBufferVars.m_totalCollisionObjects = m_totalCollisionObjects;
	m_constantBufferVars.m_totalAABBs = m_totalAABBs;
	m_constantBufferVars.m_totalOBBs = m_totalOBBs;
	m_constantBufferVars.m_totalCylinders = m_totalCylinders;
	m_constantBufferVars.m_totalCapsules = m_totalCapsules;
	m_constantBufferVars.m_totalSpheres = m_totalSpheres;

	GameStructuredBufferVariables gameSBVariables;
	gameSBVariables.m_isRopeBeingGrabbed = false;

	//Constant Buffer
	m_cbRopeData = m_renderer->CreateConstantBuffer(sizeof(RopeSimualtionConstantBufferVariables));
	m_renderer->CopyCPUToGPU(&m_constantBufferVars, sizeof(RopeSimualtionConstantBufferVariables), m_cbRopeData);

	//Structured Buffers
	m_sbGameInteraction = m_renderer->CreateStructuredBuffer(sizeof(gameSBVariables), sizeof(gameSBVariables), &gameSBVariables);
	m_sbParticlePositions = m_renderer->CreateStructuredBuffer(m_particles.m_positions.size() * sizeof(Vec3), sizeof(Vec3), m_particles.m_positions.data());
	m_sbParticleVelocities = m_renderer->CreateStructuredBuffer(m_particles.m_velocities.size() * sizeof(Vec3), sizeof(Vec3), m_particles.m_velocities.data());
	m_sbParticleProposedPositions = m_renderer->CreateStructuredBuffer(m_particles.m_proposedPositions.size() * sizeof(Vec3), sizeof(Vec3), m_particles.m_proposedPositions.data());
	m_sbParticleJacobiCorrections = m_renderer->CreateStructuredBuffer(m_particles.m_jacobiCorrections.size() * sizeof(Vec4), sizeof(Vec4), m_particles.m_jacobiCorrections.data());
	m_sbParticleCollisionNormals = m_renderer->CreateStructuredBuffer(m_particles.m_collisionNormals.size() * sizeof(Vec3), sizeof(Vec3), m_particles.m_collisionNormals.data());
	m_sbParticleMacroBitRegions = m_renderer->CreateStructuredBuffer(m_particles.m_macroBitRegions.size() * sizeof(uint64_t), sizeof(uint64_t), m_particles.m_macroBitRegions.data());
	m_sbParticleMicroBitRegions = m_renderer->CreateStructuredBuffer(m_particles.m_microBitRegions.size() * sizeof(uint64_t), sizeof(uint64_t), m_particles.m_microBitRegions.data());
	m_sbParticleMasses = m_renderer->CreateStructuredBuffer(m_particles.m_masses.size() * sizeof(float), sizeof(float), m_particles.m_masses.data());
	m_sbParticleInverseMasses = m_renderer->CreateStructuredBuffer(m_particles.m_inverseMasses.size() * sizeof(float), sizeof(float), m_particles.m_inverseMasses.data());
	m_sbParticleIsAttached = m_renderer->CreateStructuredBuffer(m_particles.m_isAttached.size() * sizeof(int), sizeof(int), m_particles.m_isAttached.data());
	m_sbRopeCapsules = m_renderer->CreateStructuredBuffer(m_collisionCapsules.size() * sizeof(CapsuleCollisionObject), sizeof(CapsuleCollisionObject), m_collisionCapsules.data());

	//Compute Shaders
	m_csGameInteraction = m_renderer->CreateComputeShader("Data/Shaders/CSUpdateFromGame.hlsl", "CSMain");
	m_csInitializeCollisionObjects = m_renderer->CreateComputeShader("Data/Shaders/CSInitializeCollisionObjects.hlsl", "CSMain");
	m_csInitialUpdates = m_renderer->CreateComputeShader("Data/Shaders/CSInitialUpdates.hlsl", "CSMain");
	m_csProjectNonCollisionConstraints = m_renderer->CreateComputeShader("Data/Shaders/CSProjectNonCollisionConstraints.hlsl", "CSMain");
	m_csUpdateAfterNonCollisionConstraints = m_renderer->CreateComputeShader("Data/Shaders/CSUpdateAfterNonCollisionConstraints.hlsl", "CSMain");
	m_csProjectCollisionConstraintsSpheres = m_renderer->CreateComputeShader("Data/Shaders/CSProjectCollisionConstraintsSpheres.hlsl", "CSMain");
	m_csInitializeBitRegionsCapsules = m_renderer->CreateComputeShader("Data/Shaders/CSInitializeBitRegionsCapsules.hlsl", "CSMain");
	m_csCapsuleCollisionsPhaseOne = m_renderer->CreateComputeShader("Data/Shaders/CSCapsuleCollisionsPhaseOne.hlsl", "CSMain");
	m_csCapsuleCollisionsPhaseTwo = m_renderer->CreateComputeShader("Data/Shaders/CSCapsuleCollisionsPhaseTwo.hlsl", "CSMain");
	m_csUpdateParticlesFromCapsules = m_renderer->CreateComputeShader("Data/Shaders/CSUpdateParticlesFromCapsules.hlsl", "CSMain");
	m_csFinalUpdates = m_renderer->CreateComputeShader("Data/Shaders/CSFinalUpdates.hlsl", "CSMain");

	//Queries
	m_startGPUQuery = m_renderer->CreateQuery(true);
	m_endGPUQuery = m_renderer->CreateQuery(true);

	//Creating Vertex_PCUTBN structured buffer
	std::vector<GPUVertex_PCUTBN> GPUVerts;
	m_totalVerts = 864 * int(m_collisionCapsules.size());
	GPUVerts.resize(m_totalVerts);
	m_structuredBufferVerts = m_renderer->CreateStructuredBuffer(GPUVerts.size() * sizeof(GPUVertex_PCUTBN), sizeof(GPUVertex_PCUTBN), GPUVerts.data());
	m_csAddVerts = m_renderer->CreateComputeShader("Data/Shaders/CSAddVerts.hlsl", "CSMain");

	//Render Shader
	m_renderShader = m_renderer->CreateShader("Data/Shaders/RopeLitTBN.hlsl", VertexType::VERTEX_PCUTBN, false, true);

	//Geometry Shaders
	m_geometryShaderCylinderSides = m_renderer->CreateShader("Data/Shaders/GSCylinderSides.hlsl", VertexType::VERTEX_PCUTBN, true);
	m_geometryShaderHemisphereTop = m_renderer->CreateShader("Data/Shaders/GSHemisphereTop.hlsl", VertexType::VERTEX_PCUTBN, true);
	m_geometryShaderHemisphereBottom = m_renderer->CreateShader("Data/Shaders/GSHemisphereBottom.hlsl", VertexType::VERTEX_PCUTBN, true);
}

//-----------------------------------------------------------------------------------------------
void RopeSimulation3D::UpdateGPUBuffers()
{
	bool shouldUpdate = false;
	if (m_numberOfParticlesInRope != m_constantBufferVars.m_totalParticles)
	{
		m_constantBufferVars.m_totalParticles = m_numberOfParticlesInRope;
		std::vector<GPUVertex_PCUTBN> GPUVerts;
		m_totalVerts = 864 * int(m_collisionCapsules.size());
		GPUVerts.resize(m_totalVerts);
		delete m_structuredBufferVerts;
		m_structuredBufferVerts = m_renderer->CreateStructuredBuffer(GPUVerts.size() * sizeof(GPUVertex_PCUTBN), sizeof(GPUVertex_PCUTBN), GPUVerts.data());
		if (m_isGPUSimulated)
		{
			delete m_sbParticlePositions;
			delete m_sbParticleProposedPositions;
			delete m_sbParticleVelocities;
			delete m_sbParticleIsAttached;
			delete m_sbParticleMasses;
			delete m_sbParticleInverseMasses;
			delete m_sbParticleMacroBitRegions;
			delete m_sbParticleMicroBitRegions;
			delete m_sbParticleCollisionNormals;
			delete m_sbParticleJacobiCorrections;
			delete m_sbRopeCapsules;
			m_sbParticlePositions = m_renderer->CreateStructuredBuffer(m_particles.m_positions.size() * sizeof(Vec3), sizeof(Vec3), m_particles.m_positions.data());
			m_sbParticleVelocities = m_renderer->CreateStructuredBuffer(m_particles.m_velocities.size() * sizeof(Vec3), sizeof(Vec3), m_particles.m_velocities.data());
			m_sbParticleProposedPositions = m_renderer->CreateStructuredBuffer(m_particles.m_proposedPositions.size() * sizeof(Vec3), sizeof(Vec3), m_particles.m_proposedPositions.data());
			m_sbParticleJacobiCorrections = m_renderer->CreateStructuredBuffer(m_particles.m_jacobiCorrections.size() * sizeof(Vec4), sizeof(Vec4), m_particles.m_jacobiCorrections.data());
			m_sbParticleCollisionNormals = m_renderer->CreateStructuredBuffer(m_particles.m_collisionNormals.size() * sizeof(Vec3), sizeof(Vec3), m_particles.m_collisionNormals.data());
			m_sbParticleMacroBitRegions = m_renderer->CreateStructuredBuffer(m_particles.m_macroBitRegions.size() * sizeof(uint64_t), sizeof(uint64_t), m_particles.m_macroBitRegions.data());
			m_sbParticleMicroBitRegions = m_renderer->CreateStructuredBuffer(m_particles.m_microBitRegions.size() * sizeof(uint64_t), sizeof(uint64_t), m_particles.m_microBitRegions.data());
			m_sbParticleMasses = m_renderer->CreateStructuredBuffer(m_particles.m_masses.size() * sizeof(float), sizeof(float), m_particles.m_masses.data());
			m_sbParticleInverseMasses = m_renderer->CreateStructuredBuffer(m_particles.m_inverseMasses.size() * sizeof(float), sizeof(float), m_particles.m_inverseMasses.data());
			m_sbParticleIsAttached = m_renderer->CreateStructuredBuffer(m_particles.m_isAttached.size() * sizeof(int), sizeof(int), m_particles.m_isAttached.data());
			m_sbRopeCapsules = m_renderer->CreateStructuredBuffer(m_collisionCapsules.size() * sizeof(CapsuleCollisionObject), sizeof(CapsuleCollisionObject), m_collisionCapsules.data());
		}
		shouldUpdate = true;
	}
	if (m_totalSolverIterations != m_constantBufferVars.m_totalSolverIterations)
	{
		m_constantBufferVars.m_totalSolverIterations = m_totalSolverIterations;
		shouldUpdate = true;
	}
	if (m_desiredDistance != m_constantBufferVars.m_desiredDistance)
	{
		m_constantBufferVars.m_desiredDistance = m_desiredDistance;
		shouldUpdate = true;
	}
	if (m_compressionCoefficient != m_constantBufferVars.m_compressionCoefficient)
	{
		m_constantBufferVars.m_compressionCoefficient = m_compressionCoefficient;
		shouldUpdate = true;
	}
	if (m_stretchingCoefficient != m_constantBufferVars.m_stretchingCoefficient)
	{
		m_constantBufferVars.m_stretchingCoefficient = m_stretchingCoefficient;
		shouldUpdate = true;
	}
	if (m_bendingConstraintDistance != m_constantBufferVars.m_bendingConstraintDistance)
	{
		m_constantBufferVars.m_bendingConstraintDistance = m_bendingConstraintDistance;
		shouldUpdate = true;
	}
	if (m_bendingCoefficient != m_constantBufferVars.m_bendingCoefficient)
	{
		m_constantBufferVars.m_bendingCoefficient = m_bendingCoefficient;
		m_constantBufferVars.m_bendingConstraintDistance = m_bendingConstraintDistance;
		shouldUpdate = true;
	}
	if (m_totalCollisionObjects != m_constantBufferVars.m_totalCollisionObjects)
	{
		m_constantBufferVars.m_totalCollisionObjects = m_totalCollisionObjects;
		shouldUpdate = true;
	}
	if (m_totalAABBs != m_constantBufferVars.m_totalAABBs)
	{
		m_constantBufferVars.m_totalAABBs = m_totalAABBs;
		shouldUpdate = true;
	}
	if (m_totalOBBs != m_constantBufferVars.m_totalOBBs)
	{
		m_constantBufferVars.m_totalOBBs = m_totalOBBs;
		shouldUpdate = true;
	}
	if (m_totalCylinders != m_constantBufferVars.m_totalCylinders)
	{
		m_constantBufferVars.m_totalCylinders = m_totalCylinders;
		shouldUpdate = true;
	}
	if (m_totalCapsules != m_constantBufferVars.m_totalCapsules)
	{
		m_constantBufferVars.m_totalCapsules = m_totalCapsules;
		shouldUpdate = true;
	}
	if (m_totalSpheres != m_constantBufferVars.m_totalSpheres)
	{
		m_constantBufferVars.m_totalSpheres = m_totalSpheres;
		shouldUpdate = true;
	}
	if (int(m_isSelfCollisionEnabled) != m_constantBufferVars.m_isSelfCollisionEnabled)
	{
		m_constantBufferVars.m_isSelfCollisionEnabled = m_isSelfCollisionEnabled;
		shouldUpdate = true;
	}
	if (m_hasGPUSwitchOccured)
	{
		if(m_isGPUSimulated)
		{
			delete m_sbParticlePositions;
			delete m_sbParticleProposedPositions;
			delete m_sbParticleVelocities;
			delete m_sbParticleIsAttached;
			delete m_sbParticleMasses;
			delete m_sbParticleInverseMasses;
			delete m_sbParticleMacroBitRegions;
			delete m_sbParticleMicroBitRegions;
			delete m_sbParticleCollisionNormals;
			delete m_sbParticleJacobiCorrections;
			delete m_sbRopeCapsules;
			m_sbParticlePositions = m_renderer->CreateStructuredBuffer(m_particles.m_positions.size() * sizeof(Vec3), sizeof(Vec3), m_particles.m_positions.data());
			m_sbParticleVelocities = m_renderer->CreateStructuredBuffer(m_particles.m_velocities.size() * sizeof(Vec3), sizeof(Vec3), m_particles.m_velocities.data());
			m_sbParticleProposedPositions = m_renderer->CreateStructuredBuffer(m_particles.m_proposedPositions.size() * sizeof(Vec3), sizeof(Vec3), m_particles.m_proposedPositions.data());
			m_sbParticleJacobiCorrections = m_renderer->CreateStructuredBuffer(m_particles.m_jacobiCorrections.size() * sizeof(Vec4), sizeof(Vec4), m_particles.m_jacobiCorrections.data());
			m_sbParticleCollisionNormals = m_renderer->CreateStructuredBuffer(m_particles.m_collisionNormals.size() * sizeof(Vec3), sizeof(Vec3), m_particles.m_collisionNormals.data());
			m_sbParticleMacroBitRegions = m_renderer->CreateStructuredBuffer(m_particles.m_macroBitRegions.size() * sizeof(uint64_t), sizeof(uint64_t), m_particles.m_macroBitRegions.data());
			m_sbParticleMicroBitRegions = m_renderer->CreateStructuredBuffer(m_particles.m_microBitRegions.size() * sizeof(uint64_t), sizeof(uint64_t), m_particles.m_microBitRegions.data());
			m_sbParticleMasses = m_renderer->CreateStructuredBuffer(m_particles.m_masses.size() * sizeof(float), sizeof(float), m_particles.m_masses.data());
			m_sbParticleInverseMasses = m_renderer->CreateStructuredBuffer(m_particles.m_inverseMasses.size() * sizeof(float), sizeof(float), m_particles.m_inverseMasses.data());
			m_sbParticleIsAttached = m_renderer->CreateStructuredBuffer(m_particles.m_isAttached.size() * sizeof(int), sizeof(int), m_particles.m_isAttached.data());
			m_sbRopeCapsules = m_renderer->CreateStructuredBuffer(m_collisionCapsules.size() * sizeof(CapsuleCollisionObject), sizeof(CapsuleCollisionObject), m_collisionCapsules.data());
			shouldUpdate = true;
		}
		else
		{
			m_renderer->CopyGPUToCPU(m_particles.m_positions.data(), m_particles.m_positions.size() * sizeof(Vec3), m_sbParticlePositions);
			m_renderer->CopyGPUToCPU(m_particles.m_velocities.data(), m_particles.m_velocities.size() * sizeof(Vec3), m_sbParticleVelocities);
			m_renderer->CopyGPUToCPU(m_particles.m_masses.data(), m_particles.m_masses.size() * sizeof(float), m_sbParticleMasses);
			m_renderer->CopyGPUToCPU(m_particles.m_inverseMasses.data(), m_particles.m_inverseMasses.size() * sizeof(float), m_sbParticleInverseMasses);
			m_renderer->CopyGPUToCPU(m_particles.m_isAttached.data(), m_particles.m_isAttached.size() * sizeof(int), m_sbParticleIsAttached);
			/*m_renderer->CopyGPUToCPU(m_collisionCapsules.data(), m_particles.m_macroBitRegions.size() * sizeof(CapsuleCollisionObject), m_sbRopeCapsules);
			m_renderer->CopyGPUToCPU(m_particles.m_jacobiCorrections.data(), m_particles.m_jacobiCorrections.size() * sizeof(Vec4), m_sbParticleJacobiCorrections);
			m_renderer->CopyGPUToCPU(m_particles.m_macroBitRegions.data(), m_particles.m_macroBitRegions.size() * sizeof(uint64_t), m_sbParticleMacroBitRegions);
			m_renderer->CopyGPUToCPU(m_particles.m_microBitRegions.data(), m_particles.m_microBitRegions.size() * sizeof(uint64_t), m_sbParticleMicroBitRegions);
			m_renderer->CopyGPUToCPU(m_particles.m_proposedPositions.data(), m_particles.m_proposedPositions.size() * sizeof(Vec3), m_sbParticleProposedPositions);*/
		}
		m_hasGPUSwitchOccured = false;
	}


	if (shouldUpdate)
	{
		m_renderer->CopyCPUToGPU(&m_constantBufferVars, sizeof(m_constantBufferVars), m_cbRopeData);
	}
}

//-----------------------------------------------------------------------------------------------
void RopeSimulation3D::UpdateGaussSeidel()
{
	//Propose Positions
	for (int particleIndex = 0; particleIndex < m_particles.m_positions.size(); particleIndex++)
	{
		if (m_particles.m_isAttached[particleIndex] == 1)
		{
			m_particles.m_proposedPositions[particleIndex] = m_particles.m_positions[particleIndex];
			UpdateCollisionCapsulesFromParticle(particleIndex);
			continue;
		}

		Vec3 acceleration = (Vec3(0.0f, 0.0f, -m_gravityCoefficient));
		Vec3 velocity = m_particles.m_velocities[particleIndex];
		velocity += acceleration * m_physicsTimestep;
		velocity *= m_dampingCoefficient;
		m_particles.m_proposedPositions[particleIndex] = m_particles.m_positions[particleIndex] + velocity * m_physicsTimestep;
		UpdateCollisionCapsulesFromParticle(particleIndex);
		UnaddCollidingRopeParticle(particleIndex);
	}

	//Constraint Projection
	for (int solverIndex = 0; solverIndex < m_totalSolverIterations; solverIndex++)
	{
		ProjectConstraintsGaussSeidel();
	}

	//Velocity and Friction Updates
	for (int particleIndex = 0; particleIndex < m_particles.m_positions.size(); particleIndex++)
	{
		m_particles.m_velocities[particleIndex] = (m_particles.m_proposedPositions[particleIndex] - m_particles.m_positions[particleIndex]) / m_physicsTimestep;

		if (m_particles.m_collisionNormals[particleIndex] != Vec3(0.0, 0.0, 0.0))
		{
			//Static Friction
			if (m_particles.m_velocities[particleIndex].GetLength() < 0.15f && DotProduct3D(m_particles.m_collisionNormals[particleIndex], Vec3(0.0f, 0.0f, 1.0f)) != 0.0f && m_particles.m_isSelfCollision[particleIndex] == 0)
			{
				m_particles.m_velocities[particleIndex] = Vec3();
				m_particles.m_collisionNormals[particleIndex] = Vec3(); 
				continue;
			}
			else if (m_particles.m_velocities[particleIndex].GetLength() < 0.075f && DotProduct3D(m_particles.m_collisionNormals[particleIndex], Vec3(0.0f, 0.0f, 1.0f)) != 0.0f && m_particles.m_isSelfCollision[particleIndex] == 0)
			{
				m_particles.m_velocities[particleIndex] = Vec3();
				m_particles.m_collisionNormals[particleIndex] = Vec3();
				continue;
			}

			//Kinetic Friction
			Vec3 tangentalFriction = GetProjectedOnto3D(m_particles.m_velocities[particleIndex], m_particles.m_collisionNormals[particleIndex]) - m_particles.m_velocities[particleIndex];
			m_particles.m_velocities[particleIndex] += tangentalFriction * m_kineticFrictionCoefficient;
			m_particles.m_collisionNormals[particleIndex] = Vec3(); 
			m_particles.m_isSelfCollision[particleIndex] = 0;
		}

		m_particles.m_positions[particleIndex] = m_particles.m_proposedPositions[particleIndex];
	}
}

//-----------------------------------------------------------------------------------------------
void RopeSimulation3D::ProjectConstraintsGaussSeidel()
{
	//Distance and Bending Constraints
	for (int constraintIndex = 0; constraintIndex < m_distanceConstraints.size(); constraintIndex++)
	{
		ProjectDistanceConstraintGaussSeidel(constraintIndex);
	}
	for (int constraintIndex = 0; constraintIndex < m_bendingConstraints.size(); constraintIndex++)
	{
		ProjectBendingConstraintGaussSeidel(constraintIndex);
	}

	//Collision Constraints
	if (m_collisionType == CollisionType::SPHERES)
	{
		BitRegionDetectionAllParticles();

		for (int particleIndex = 0; particleIndex < m_particles.m_positions.size(); particleIndex++)
		{
			ProjectCollisionConstraintsSpheresGaussSeidel(particleIndex);
		}
	}
	else if (m_collisionType == CollisionType::CAPSULES)
	{
		BitRegionDetectionAllCapsules();

		for (int capsuleIndex = 0; capsuleIndex < m_collisionCapsules.size(); capsuleIndex++)
		{
			Capsule3& capsule = m_collisionCapsules[capsuleIndex].m_capsule;
			capsule.m_bone.m_start = m_particles.m_proposedPositions[capsuleIndex];
			capsule.m_bone.m_end = m_particles.m_proposedPositions[capsuleIndex + 1];
			ProjectCollisionConstraintsCapsulesGaussSeidel(capsuleIndex);
		}
	}
}

//-----------------------------------------------------------------------------------------------
void RopeSimulation3D::ProjectDistanceConstraintGaussSeidel(int constraintIndex)
{
	//Initializations
	Constraint3D& constraint = m_distanceConstraints[constraintIndex];

	//Calculates direction and overflow along with weight Coefficients
	Vec3 displacement = m_particles.m_proposedPositions[constraint.m_indices[1]] - m_particles.m_proposedPositions[constraint.m_indices[0]];
	float desiredDistance = 0.0f;
	if (m_isHierarchical)
	{
		desiredDistance = abs(constraint.m_indices[1] - constraint.m_indices[0]) * m_desiredDistance;
	}
	else
	{
		desiredDistance = (m_desiredDistance);
	}
	float distanceConstraint = (displacement.GetLength() - desiredDistance);

	if (constraint.m_constraintEquality == Constraint3DEquality::EQUALITY)
	{
		if (distanceConstraint == 0.0f)
		{
			return;
		}
	}
	else if (constraint.m_constraintEquality == Constraint3DEquality::INEQUALITY_GREATER)
	{
		if (distanceConstraint <= 0.0f)
		{
			return;
		}
	}
	else if (constraint.m_constraintEquality == Constraint3DEquality::INEQUALITY_LESS)
	{
		if (distanceConstraint >= 0.0f)
		{
			return;
		}
	}

	Vec3 gradient = displacement.GetNormalized();
	float coefficientValue = 0.0f;
	if (distanceConstraint < 0.0f)
	{
		coefficientValue = m_compressionCoefficient;
	}
	else
	{
		coefficientValue = m_stretchingCoefficient;
	}

	if (m_particles.m_isAttached[constraint.m_indices[0]] == false && m_particles.m_isAttached[constraint.m_indices[1]] == false)
	{
		float particleAWeightCoefficient = (m_particles.m_inverseMasses[constraint.m_indices[0]] / (m_particles.m_inverseMasses[constraint.m_indices[0]] + m_particles.m_inverseMasses[constraint.m_indices[1]]));
		float particleBWeightCoefficient = (m_particles.m_inverseMasses[constraint.m_indices[1]] / (m_particles.m_inverseMasses[constraint.m_indices[0]] + m_particles.m_inverseMasses[constraint.m_indices[1]]));

		Vec3 deltaParticleA = distanceConstraint * particleAWeightCoefficient * gradient;
		Vec3 deltaParticleB = distanceConstraint * particleBWeightCoefficient * gradient;

		m_particles.m_proposedPositions[constraint.m_indices[0]] += coefficientValue * deltaParticleA;
		m_particles.m_proposedPositions[constraint.m_indices[1]] -= coefficientValue * deltaParticleB;
	}
	else if (m_particles.m_isAttached[constraint.m_indices[0]] == 1 && m_particles.m_isAttached[constraint.m_indices[1]] == 0)
	{
		Vec3 deltaParticleB = distanceConstraint * gradient;
		m_particles.m_proposedPositions[constraint.m_indices[1]] -= coefficientValue * deltaParticleB;
	}
	else if (m_particles.m_isAttached[constraint.m_indices[0]] == 0 && m_particles.m_isAttached[constraint.m_indices[1]] == 1)
	{
		Vec3 deltaParticleA = distanceConstraint * gradient;
		m_particles.m_proposedPositions[constraint.m_indices[0]] += coefficientValue * deltaParticleA;
	}
}

//-----------------------------------------------------------------------------------------------
void RopeSimulation3D::ProjectBendingConstraintGaussSeidel(int constraintIndex)
{
	//Initializations
	Constraint3D& constraint = m_bendingConstraints[constraintIndex];

	//Calculates direction and overflow along with weight Coefficients
	Vec3 displacement = m_particles.m_proposedPositions[constraint.m_indices[2]] - m_particles.m_proposedPositions[constraint.m_indices[0]];
	float distanceConstraint = displacement.GetLength() - m_bendingConstraintDistance;

	//Check if less than the minimum distance, if so no need to constrain
	if (distanceConstraint > 0.0f)
	{
		return;
	}

	Vec3 gradient = displacement.GetNormalized();
	Vec3 directionCheck = (m_particles.m_proposedPositions[constraint.m_indices[1]] - m_particles.m_proposedPositions[constraint.m_indices[0]]).GetNormalized();

	//Because this is a cheap method to check for bending need to ensure the direction inst the same
	if (gradient == directionCheck)
	{
		return;
	}

	if (m_particles.m_isAttached[constraint.m_indices[0]] == 0 && m_particles.m_isAttached[constraint.m_indices[2]] == 0)
	{
		float pointAWeightCoefficient = (m_particles.m_inverseMasses[constraint.m_indices[0]] / (m_particles.m_inverseMasses[constraint.m_indices[0]] + m_particles.m_inverseMasses[constraint.m_indices[2]]));
		float pointCWeightCoefficient = (m_particles.m_inverseMasses[constraint.m_indices[2]] / (m_particles.m_inverseMasses[constraint.m_indices[0]] + m_particles.m_inverseMasses[constraint.m_indices[2]]));

		Vec3 deltaPointA = pointAWeightCoefficient * distanceConstraint * gradient;
		Vec3 deltaPointC = pointCWeightCoefficient * distanceConstraint * gradient;

		m_particles.m_proposedPositions[constraint.m_indices[0]] += m_bendingCoefficient * deltaPointA;
		m_particles.m_proposedPositions[constraint.m_indices[2]] -= m_bendingCoefficient * deltaPointC;
	}
	else if (m_particles.m_isAttached[constraint.m_indices[0]] == 1 && m_particles.m_isAttached[constraint.m_indices[2]] == 0)
	{
		Vec3 deltaPointC = distanceConstraint * gradient;
		m_particles.m_proposedPositions[constraint.m_indices[2]] -= m_bendingCoefficient * deltaPointC;
	}
	else if (m_particles.m_isAttached[constraint.m_indices[0]] == 0 && m_particles.m_isAttached[constraint.m_indices[2]] == 1)
	{
		Vec3 deltaPointA = distanceConstraint * gradient;
		m_particles.m_proposedPositions[constraint.m_indices[0]] += m_bendingCoefficient * deltaPointA;
	}
}

//-----------------------------------------------------------------------------------------------
void RopeSimulation3D::ProjectCollisionConstraintsSpheresGaussSeidel(int sentParticleIndex)
{
	//Self Collisions
	if (m_isSelfCollisionEnabled)
	{
		for (int particleIndex = 0; particleIndex < m_particles.m_positions.size(); particleIndex++)
		{
			if (particleIndex != sentParticleIndex && particleIndex != sentParticleIndex - 1 && particleIndex != sentParticleIndex + 1)
			{
				if ((m_particles.m_macroBitRegions[sentParticleIndex] & m_particles.m_macroBitRegions[particleIndex]) != 0)
				{
					if ((m_particles.m_microBitRegions[sentParticleIndex] & m_particles.m_microBitRegions[particleIndex]) != 0)
					{
						Vec3 sentPreviousPosition = m_particles.m_proposedPositions[sentParticleIndex];
						Vec3 currentPreviousPosition = m_particles.m_proposedPositions[particleIndex];
						if (PushSphereOutOfSphere3D(m_particles.m_proposedPositions[sentParticleIndex], m_ropeRadius, m_particles.m_proposedPositions[particleIndex], m_ropeRadius))
						{
							Vec3 collisionNormal = (m_particles.m_proposedPositions[sentParticleIndex] - sentPreviousPosition).GetNormalized();
							if (collisionNormal.x != 0.0 || collisionNormal.y != 0.0 || collisionNormal.z != 0)
							{
								m_particles.m_collisionNormals[sentParticleIndex] = collisionNormal;
								m_particles.m_isSelfCollision[sentParticleIndex] = 1;
							}
							collisionNormal = (m_particles.m_proposedPositions[particleIndex] - currentPreviousPosition).GetNormalized();
							if (collisionNormal.x != 0.0 || collisionNormal.y != 0.0 || collisionNormal.z != 0)
							{
								m_particles.m_collisionNormals[particleIndex] = collisionNormal;
								m_particles.m_isSelfCollision[particleIndex] = 1;
							}
							//Update Bit Field
							BitRegionDetectionSingleParticle(sentParticleIndex);
							BitRegionDetectionSingleParticle(particleIndex);
						}
					}
				}
			}
		}
	}

	//Collision Objects
	for (int collisionObjectIndex = 0; collisionObjectIndex < m_collisionObjects.size(); collisionObjectIndex++)
	{
		CollisionObject*& collisionObject = m_collisionObjects[collisionObjectIndex];
		Vec3 previousPosition = m_particles.m_proposedPositions[sentParticleIndex];
		if (collisionObject)
		{
			SphereCollisionObject* sphere = dynamic_cast<SphereCollisionObject*>(collisionObject);
			if (sphere) //Sphere Collisions
			{
				if ((sphere->m_macroBitRegions & m_particles.m_macroBitRegions[sentParticleIndex]) != 0)
				{
					if ((sphere->m_microBitRegions & m_particles.m_microBitRegions[sentParticleIndex]) != 0)
					{
						if (PushSphereOutOfFixedSphere3D(m_particles.m_proposedPositions[sentParticleIndex], m_ropeRadius, sphere->m_sphere.m_center, sphere->m_sphere.m_radius))
						{
							m_particles.m_collisionNormals[sentParticleIndex] = (m_particles.m_proposedPositions[sentParticleIndex] - previousPosition).GetNormalized();
							m_particles.m_isSelfCollision[sentParticleIndex] = 0;
							BitRegionDetectionSingleParticle(sentParticleIndex);
						}
					}
				}
				continue;
			}
			AABBCollisionObject* aabb = dynamic_cast<AABBCollisionObject*>(collisionObject);
			if (aabb) //AABB Collisions
			{
				if ((aabb->m_macroBitRegions & m_particles.m_macroBitRegions[sentParticleIndex]) != 0)
				{
					if ((aabb->m_microBitRegions & m_particles.m_microBitRegions[sentParticleIndex]) != 0)
					{
						//Bounding Disc Check
						if (DoSpheresOverlap(m_particles.m_proposedPositions[sentParticleIndex], m_ropeRadius, aabb->m_boundingDiscCenter, aabb->m_boundingDiscRadius))
						{
							if (PushSphereOutOfFixedAABB3D(m_particles.m_proposedPositions[sentParticleIndex], m_ropeRadius, aabb->m_aabb))
							{
								Vec3 collisionNormal = (m_particles.m_proposedPositions[sentParticleIndex] - previousPosition).GetNormalized();
								if (collisionNormal.x != 0.0 || collisionNormal.y != 0.0 || collisionNormal.z != 0)
								{
									m_particles.m_collisionNormals[sentParticleIndex] = collisionNormal;
									m_particles.m_isSelfCollision[sentParticleIndex] = 0;
								}
								BitRegionDetectionSingleParticle(sentParticleIndex);
							}
						}
					}
				}
				continue;
			}
			OBBCollisionObject* obb = dynamic_cast<OBBCollisionObject*>(collisionObject);
			if (obb) //OBB Collisions
			{
				if ((obb->m_macroBitRegions & m_particles.m_macroBitRegions[sentParticleIndex]) != 0)
				{
					if ((obb->m_microBitRegions & m_particles.m_microBitRegions[sentParticleIndex]) != 0)
					{
						//Bounding Disc Check
						if (DoSpheresOverlap(m_particles.m_proposedPositions[sentParticleIndex], m_ropeRadius, obb->m_boundingDiscCenter, obb->m_boundingDiscRadius))
						{
							if (PushDiscOutOfFixedOBB3D(m_particles.m_proposedPositions[sentParticleIndex], m_ropeRadius, obb->m_obb))
							{
								Vec3 collisionNormal = (m_particles.m_proposedPositions[sentParticleIndex] - previousPosition).GetNormalized();
								if (collisionNormal.x != 0.0 || collisionNormal.y != 0.0 || collisionNormal.z != 0)
								{
									m_particles.m_collisionNormals[sentParticleIndex] = collisionNormal;
									m_particles.m_isSelfCollision[sentParticleIndex] = 0;
								}
								BitRegionDetectionSingleParticle(sentParticleIndex);
							}
						}
					}
				}
				continue;
			}
			CapsuleCollisionObject* capsule = dynamic_cast<CapsuleCollisionObject*>(collisionObject);
			if (capsule) //Capsule Collisions
			{
				if ((capsule->m_macroBitRegions & m_particles.m_macroBitRegions[sentParticleIndex]) != 0)
				{
					if ((capsule->m_microBitRegions & m_particles.m_microBitRegions[sentParticleIndex]) != 0)
					{
						//Bounding Disc Check
						if (DoSpheresOverlap(m_particles.m_proposedPositions[sentParticleIndex], m_ropeRadius, capsule->m_boundingDiscCenter, capsule->m_boundingDiscRadius))
						{
							if (PushSphereOutOfFixedCapsule3D(m_particles.m_proposedPositions[sentParticleIndex], m_ropeRadius, capsule->m_capsule))
							{
								Vec3 collisionNormal = (m_particles.m_proposedPositions[sentParticleIndex] - previousPosition).GetNormalized();
								if (collisionNormal.x != 0.0 || collisionNormal.y != 0.0 || collisionNormal.z != 0)
								{
									m_particles.m_collisionNormals[sentParticleIndex] = collisionNormal;
									m_particles.m_isSelfCollision[sentParticleIndex] = 0;
								}
								BitRegionDetectionSingleParticle(sentParticleIndex);
							}
						}
					}
				}
				continue;
			}
			CylinderCollisionObject* cylinder = dynamic_cast<CylinderCollisionObject*>(collisionObject);
			if (cylinder) //Cylinder Collisions
			{
				if ((cylinder->m_macroBitRegions & m_particles.m_macroBitRegions[sentParticleIndex]) != 0)
				{
					if ((cylinder->m_microBitRegions & m_particles.m_microBitRegions[sentParticleIndex]) != 0)
					{
						//Bounding Disc Check
						if (DoSpheresOverlap(m_particles.m_proposedPositions[sentParticleIndex], m_ropeRadius, cylinder->m_boundingDiscCenter, cylinder->m_boundingDiscRadius))
						{
							if (PushSphereOutOfFixedCylinder3D(m_particles.m_proposedPositions[sentParticleIndex], m_ropeRadius, cylinder->m_cylinder))
							{
								Vec3 collisionNormal = (m_particles.m_proposedPositions[sentParticleIndex] - previousPosition).GetNormalized();
								if (collisionNormal.x != 0.0 || collisionNormal.y != 0.0 || collisionNormal.z != 0)
								{
									m_particles.m_collisionNormals[sentParticleIndex] = collisionNormal;
									m_particles.m_isSelfCollision[sentParticleIndex] = 0;
								}
								BitRegionDetectionSingleParticle(sentParticleIndex);
							}
						}
					}
				}
				continue;
			}
		}
	}


	//World Bounds Collisions
	ProjectWorldBoundsConstraintsSpheresGaussSeidel(sentParticleIndex);
}

//-----------------------------------------------------------------------------------------------
void RopeSimulation3D::ProjectWorldBoundsConstraintsSpheresGaussSeidel(int sentParticleIndex)
{
	//Min Z
	if (m_particles.m_proposedPositions[sentParticleIndex].z < m_worldBounds.m_mins.z + m_ropeRadius)
	{
		m_particles.m_proposedPositions[sentParticleIndex].z = m_worldBounds.m_mins.z + m_ropeRadius;
		m_particles.m_collisionNormals[sentParticleIndex] = Vec3(0.0f, 0.0f, 1.0f);
		m_particles.m_isSelfCollision[sentParticleIndex] = 0;
		BitRegionDetectionSingleParticle(sentParticleIndex);
	}
	//Max Z
	if (m_particles.m_proposedPositions[sentParticleIndex].z > m_worldBounds.m_maxs.z - m_ropeRadius)
	{
		m_particles.m_proposedPositions[sentParticleIndex].z = m_worldBounds.m_maxs.z - m_ropeRadius;
		m_particles.m_collisionNormals[sentParticleIndex] = Vec3(0.0f, 0.0f, -1.0f);
		m_particles.m_isSelfCollision[sentParticleIndex] = 0;
		BitRegionDetectionSingleParticle(sentParticleIndex);
	}
	//Min X
	if (m_particles.m_proposedPositions[sentParticleIndex].x < m_worldBounds.m_mins.x + m_ropeRadius)
	{
		m_particles.m_proposedPositions[sentParticleIndex].x = m_worldBounds.m_mins.x + m_ropeRadius;
		m_particles.m_collisionNormals[sentParticleIndex] = Vec3(1.0f, 0.0f, 0.0f);
		m_particles.m_isSelfCollision[sentParticleIndex] = 0;
		BitRegionDetectionSingleParticle(sentParticleIndex);
	}
	//Max X
	if (m_particles.m_proposedPositions[sentParticleIndex].x > m_worldBounds.m_maxs.x - m_ropeRadius)
	{
		m_particles.m_proposedPositions[sentParticleIndex].x = m_worldBounds.m_maxs.x - m_ropeRadius;
		m_particles.m_collisionNormals[sentParticleIndex] = Vec3(-1.0f, 0.0f, 0.0f);
		m_particles.m_isSelfCollision[sentParticleIndex] = 0;
		BitRegionDetectionSingleParticle(sentParticleIndex);
	}
	//Min Y
	if (m_particles.m_proposedPositions[sentParticleIndex].y < m_worldBounds.m_mins.y + m_ropeRadius)
	{
		m_particles.m_proposedPositions[sentParticleIndex].y = m_worldBounds.m_mins.y + m_ropeRadius;
		m_particles.m_collisionNormals[sentParticleIndex] = Vec3(0.0f, 1.0f, 0.0f);
		m_particles.m_isSelfCollision[sentParticleIndex] = 0;
		BitRegionDetectionSingleParticle(sentParticleIndex);
	}
	//Max Y
	if (m_particles.m_proposedPositions[sentParticleIndex].y > m_worldBounds.m_maxs.y - m_ropeRadius)
	{
		m_particles.m_proposedPositions[sentParticleIndex].y = m_worldBounds.m_maxs.y - m_ropeRadius;
		m_particles.m_collisionNormals[sentParticleIndex] = Vec3(0.0f, -1.0f, 0.0f);
		m_particles.m_isSelfCollision[sentParticleIndex] = 0;
		BitRegionDetectionSingleParticle(sentParticleIndex);
	}
}

//-----------------------------------------------------------------------------------------------
void RopeSimulation3D::ProjectCollisionConstraintsCapsulesGaussSeidel(int sentCapsuleIndex)
{
	//Initializations
	Capsule3& sentCapsule = m_collisionCapsules[sentCapsuleIndex].m_capsule;
	UnaddCollidingRopeParticle(sentCapsuleIndex);

	//Self Collisions
	if (m_isSelfCollisionEnabled)
	{
		for (int capsuleIndex = 0; capsuleIndex < m_collisionCapsules.size(); capsuleIndex++)
		{
			if (capsuleIndex == sentCapsuleIndex || capsuleIndex == sentCapsuleIndex + 1 || capsuleIndex == sentCapsuleIndex - 1
				|| capsuleIndex == sentCapsuleIndex + 2 || capsuleIndex == sentCapsuleIndex - 2)
			{
				continue;
			}

			if ((m_collisionCapsules[sentCapsuleIndex].m_macroBitRegions & m_collisionCapsules[capsuleIndex].m_macroBitRegions) != 0)
			{
				if ((m_collisionCapsules[sentCapsuleIndex].m_microBitRegions & m_collisionCapsules[capsuleIndex].m_microBitRegions) != 0)
				{
					Capsule3& capsule = m_collisionCapsules[capsuleIndex].m_capsule;
					capsule.m_bone.m_start = m_particles.m_proposedPositions[capsuleIndex];
					capsule.m_bone.m_end = m_particles.m_proposedPositions[capsuleIndex + 1];

					Vec3 sentCapsuleCenter = (sentCapsule.m_bone.m_start + sentCapsule.m_bone.m_end) * 0.5f;
					Vec3 capsuleCenter = (capsule.m_bone.m_start + capsule.m_bone.m_end) * 0.5f;
					float sentCapsuleBoundingRadius = (sentCapsule.m_bone.m_start - sentCapsuleCenter).GetLength() + sentCapsule.m_radius;
					float capsuleBoundingRadius = (capsule.m_bone.m_start - capsuleCenter).GetLength() + capsule.m_radius;

					if (!DoSpheresOverlap(sentCapsuleCenter, sentCapsuleBoundingRadius, capsuleCenter, capsuleBoundingRadius))
						continue;

					Vec3 originalStartSent = sentCapsule.m_bone.m_start;
					Vec3 originalEndSent = sentCapsule.m_bone.m_end;
					Vec3 originalStartCurrent = capsule.m_bone.m_start;
					Vec3 originalEndCurrent = capsule.m_bone.m_end;
					if (PushCapsuleOutOfCapsule3D(sentCapsule, capsule))
					{
						if (m_particles.m_isAttached[sentCapsuleIndex])
						{
							sentCapsule.m_bone.m_start = originalStartSent;
						}
						if (m_particles.m_isAttached[sentCapsuleIndex + 1])
						{
							sentCapsule.m_bone.m_end = originalEndSent;
						}
						if (m_particles.m_isAttached[capsuleIndex])
						{
							capsule.m_bone.m_start = originalStartCurrent;
						}
						if (m_particles.m_isAttached[capsuleIndex + 1])
						{
							capsule.m_bone.m_end = originalEndCurrent;
						}

						//Collision Normals
						UpdateProposedParticlesFromCollisionCapsule(capsuleIndex);
						BitRegionDetectionSingleCapsule(sentCapsuleIndex);
						BitRegionDetectionSingleCapsule(capsuleIndex);
						Vec3 collisionNormal = (sentCapsule.m_bone.m_start - originalStartSent).GetNormalized();
						if (collisionNormal.x != 0.0 || collisionNormal.y != 0.0 || collisionNormal.z != 0)
							m_particles.m_collisionNormals[sentCapsuleIndex] = collisionNormal;
							m_particles.m_isSelfCollision[sentCapsuleIndex] = 1;
						collisionNormal = (sentCapsule.m_bone.m_end - originalEndSent).GetNormalized();
						if (collisionNormal.x != 0.0 || collisionNormal.y != 0.0 || collisionNormal.z != 0)
							m_particles.m_collisionNormals[sentCapsuleIndex + 1] = collisionNormal;
							m_particles.m_isSelfCollision[sentCapsuleIndex + 1] = 1;
						collisionNormal = (capsule.m_bone.m_start - originalStartCurrent).GetNormalized();
						if (collisionNormal.x != 0.0 || collisionNormal.y != 0.0 || collisionNormal.z != 0)
							m_particles.m_collisionNormals[capsuleIndex] = collisionNormal;
							m_particles.m_isSelfCollision[capsuleIndex] = 1;
						collisionNormal = (capsule.m_bone.m_end - originalEndCurrent).GetNormalized();
						if (collisionNormal.x != 0.0 || collisionNormal.y != 0.0 || collisionNormal.z != 0)
							m_particles.m_collisionNormals[capsuleIndex + 1] = collisionNormal;
							m_particles.m_isSelfCollision[capsuleIndex + 1] = 1;
					}
				}
			}
		}
	}

	//Collision Objects
	for (int collisionObjectIndex = 0; collisionObjectIndex < m_collisionObjects.size(); collisionObjectIndex++)
	{
		CollisionObject*& collisionObject = m_collisionObjects[collisionObjectIndex];
		Vec3 originalStart = sentCapsule.m_bone.m_start;
		Vec3 originalEnd = sentCapsule.m_bone.m_end;
		Vec3 capsuleBoundingDiscCenter = (originalStart + originalEnd) * 0.5f;
		float capsuleBoundingDiscRadius = ((originalStart + originalEnd).GetLength() + m_ropeRadius * 2.0f) * 0.5f;
		if (collisionObject)
		{
			SphereCollisionObject* sphere = dynamic_cast<SphereCollisionObject*>(collisionObject);
			if (sphere) //Sphere Collisions
			{
				if ((sphere->m_macroBitRegions & m_collisionCapsules[sentCapsuleIndex].m_macroBitRegions) != 0)
				{
					if ((sphere->m_microBitRegions & m_collisionCapsules[sentCapsuleIndex].m_microBitRegions) != 0)
					{
						if (PushCapsuleOutOfFixedSphere3D(sentCapsule, sphere->m_sphere.m_center, sphere->m_sphere.m_radius))
						{
							if (m_particles.m_isAttached[sentCapsuleIndex])
							{
								sentCapsule.m_bone.m_start = originalStart;
							}
							if (m_particles.m_isAttached[sentCapsuleIndex + 1])
							{
								sentCapsule.m_bone.m_end = originalEnd;
							}

							//Collision Normals
							BitRegionDetectionSingleCapsule(sentCapsuleIndex);
							Vec3 collisionNormal = (sentCapsule.m_bone.m_start - originalStart).GetNormalized();
							if (collisionNormal.x != 0.0 || collisionNormal.y != 0.0 || collisionNormal.z != 0)
								m_particles.m_collisionNormals[sentCapsuleIndex] = collisionNormal;
								m_particles.m_isSelfCollision[sentCapsuleIndex] = 0;
							collisionNormal = (sentCapsule.m_bone.m_end - originalEnd).GetNormalized();
							if (collisionNormal.x != 0.0 || collisionNormal.y != 0.0 || collisionNormal.z != 0)
								m_particles.m_collisionNormals[sentCapsuleIndex + 1] = collisionNormal;
								m_particles.m_isSelfCollision[sentCapsuleIndex + 1] = 0;
						}
					}
				}
				continue;
			}
			AABBCollisionObject* aabb = dynamic_cast<AABBCollisionObject*>(collisionObject);
			if (aabb) //AABB Collisions
			{
				if ((aabb->m_macroBitRegions & m_collisionCapsules[sentCapsuleIndex].m_macroBitRegions) != 0)
				{
					if ((aabb->m_microBitRegions & m_collisionCapsules[sentCapsuleIndex].m_microBitRegions) != 0)
					{
						//Bounding Disk Check
						if (!DoSpheresOverlap(capsuleBoundingDiscCenter, capsuleBoundingDiscRadius, aabb->m_boundingDiscCenter, aabb->m_boundingDiscRadius))
							continue;

						if (PushCapsuleOutOfFixedAABB3D(sentCapsule, aabb->m_aabb))
						{
							if (m_particles.m_isAttached[sentCapsuleIndex])
							{
								sentCapsule.m_bone.m_start = originalStart;
							}
							if (m_particles.m_isAttached[sentCapsuleIndex + 1])
							{
								sentCapsule.m_bone.m_end = originalEnd;
							}

							//Collision Normals
							BitRegionDetectionSingleCapsule(sentCapsuleIndex);
							Vec3 collisionNormal = (sentCapsule.m_bone.m_start - originalStart).GetNormalized();
							if (collisionNormal.x != 0.0 || collisionNormal.y != 0.0 || collisionNormal.z != 0)
								m_particles.m_collisionNormals[sentCapsuleIndex] = collisionNormal;
								m_particles.m_isSelfCollision[sentCapsuleIndex] = 0;
							collisionNormal = (sentCapsule.m_bone.m_end - originalEnd).GetNormalized();
							if (collisionNormal.x != 0.0 || collisionNormal.y != 0.0 || collisionNormal.z != 0)
								m_particles.m_collisionNormals[sentCapsuleIndex + 1] = collisionNormal;
								m_particles.m_isSelfCollision[sentCapsuleIndex + 1] = 0;
						}
					}
				}
				continue;
			}
			OBBCollisionObject* obb = dynamic_cast<OBBCollisionObject*>(collisionObject);
			if (obb) //OBB Collisions
			{
				if ((obb->m_macroBitRegions & m_collisionCapsules[sentCapsuleIndex].m_macroBitRegions) != 0)
				{
					if ((obb->m_microBitRegions & m_collisionCapsules[sentCapsuleIndex].m_microBitRegions) != 0)
					{
						//Bounding Disk Check
						if (!DoSpheresOverlap(capsuleBoundingDiscCenter, capsuleBoundingDiscRadius, obb->m_boundingDiscCenter, obb->m_boundingDiscRadius))
							continue;

						if (PushCapsuleOutOfFixedOBB3D(sentCapsule, obb->m_obb))
						{
							if (m_particles.m_isAttached[sentCapsuleIndex])
							{
								sentCapsule.m_bone.m_start = originalStart;
							}
							if (m_particles.m_isAttached[sentCapsuleIndex + 1])
							{
								sentCapsule.m_bone.m_end = originalEnd;
							}

							//Collision Normals
							BitRegionDetectionSingleCapsule(sentCapsuleIndex);
							Vec3 collisionNormal = (sentCapsule.m_bone.m_start - originalStart).GetNormalized();
							if (collisionNormal.x != 0.0 || collisionNormal.y != 0.0 || collisionNormal.z != 0)
								m_particles.m_collisionNormals[sentCapsuleIndex] = collisionNormal;
								m_particles.m_isSelfCollision[sentCapsuleIndex] = 0;
							collisionNormal = (sentCapsule.m_bone.m_end - originalEnd).GetNormalized();
							if (collisionNormal.x != 0.0 || collisionNormal.y != 0.0 || collisionNormal.z != 0)
								m_particles.m_collisionNormals[sentCapsuleIndex + 1] = collisionNormal;
								m_particles.m_isSelfCollision[sentCapsuleIndex + 1] = 0;
						}
					}
				}
				continue;
			}
			CapsuleCollisionObject* capsule = dynamic_cast<CapsuleCollisionObject*>(collisionObject);
			if (capsule) //Capsule Collisions
			{
				if ((capsule->m_macroBitRegions & m_collisionCapsules[sentCapsuleIndex].m_macroBitRegions) != 0)
				{
					if ((capsule->m_microBitRegions & m_collisionCapsules[sentCapsuleIndex].m_microBitRegions) != 0)
					{
						//Bounding Disk Check
						if (!DoSpheresOverlap(capsuleBoundingDiscCenter, capsuleBoundingDiscRadius, capsule->m_boundingDiscCenter, capsule->m_boundingDiscRadius))
							continue;

						if (PushCapsuleOutOfFixedCapsule3D(sentCapsule, capsule->m_capsule))
						{
							if (m_particles.m_isAttached[sentCapsuleIndex])
							{
								sentCapsule.m_bone.m_start = originalStart;
							}
							if (m_particles.m_isAttached[sentCapsuleIndex + 1])
							{
								sentCapsule.m_bone.m_end = originalEnd;
							}

							//Collision Normals
							BitRegionDetectionSingleCapsule(sentCapsuleIndex);
							Vec3 collisionNormal = (sentCapsule.m_bone.m_start - originalStart).GetNormalized();
							if (collisionNormal.x != 0.0 || collisionNormal.y != 0.0 || collisionNormal.z != 0)
								m_particles.m_collisionNormals[sentCapsuleIndex] = collisionNormal;
								m_particles.m_isSelfCollision[sentCapsuleIndex] = 0;
							collisionNormal = (sentCapsule.m_bone.m_end - originalEnd).GetNormalized();
							if (collisionNormal.x != 0.0 || collisionNormal.y != 0.0 || collisionNormal.z != 0)
								m_particles.m_collisionNormals[sentCapsuleIndex + 1] = collisionNormal;
								m_particles.m_isSelfCollision[sentCapsuleIndex + 1] = 0;
						}
					}
				}
				continue;
			}
			CylinderCollisionObject* cylinder = dynamic_cast<CylinderCollisionObject*>(collisionObject);
			if (cylinder) //Cylinder Collisions
			{
				if ((cylinder->m_macroBitRegions & m_collisionCapsules[sentCapsuleIndex].m_macroBitRegions) != 0)
				{
					if ((cylinder->m_microBitRegions & m_collisionCapsules[sentCapsuleIndex].m_microBitRegions) != 0)
					{
						//Bounding Disk Check
						if (!DoSpheresOverlap(capsuleBoundingDiscCenter, capsuleBoundingDiscRadius, cylinder->m_boundingDiscCenter, cylinder->m_boundingDiscRadius))
							continue;

						if (PushCapsuleOutOfFixedCylinder3D(sentCapsule, cylinder->m_cylinder))
						{
							if (m_particles.m_isAttached[sentCapsuleIndex])
							{
								sentCapsule.m_bone.m_start = originalStart;
							}
							if (m_particles.m_isAttached[sentCapsuleIndex + 1])
							{
								sentCapsule.m_bone.m_end = originalEnd;
							}

							//Collision Normals
							BitRegionDetectionSingleCapsule(sentCapsuleIndex);
							Vec3 collisionNormal = (sentCapsule.m_bone.m_start - originalStart).GetNormalized();
							if (collisionNormal.x != 0.0 || collisionNormal.y != 0.0 || collisionNormal.z != 0)
								m_particles.m_collisionNormals[sentCapsuleIndex] = collisionNormal;
								m_particles.m_isSelfCollision[sentCapsuleIndex] = 0;
							collisionNormal = (sentCapsule.m_bone.m_end - originalEnd).GetNormalized();
							if (collisionNormal.x != 0.0 || collisionNormal.y != 0.0 || collisionNormal.z != 0)
								m_particles.m_collisionNormals[sentCapsuleIndex + 1] = collisionNormal;
								m_particles.m_isSelfCollision[sentCapsuleIndex + 1] = 0;
						}
					}
				}
				continue;
			}
		}
	}

	//World Bounds Collisions
	m_collisionCapsules[sentCapsuleIndex].m_capsule = sentCapsule;
	Vec3 originalStart = sentCapsule.m_bone.m_start;
	Vec3 originalEnd = sentCapsule.m_bone.m_end;
	ProjectWorldBoundsConstraintsCapsulesGaussSeidel(sentCapsuleIndex);
	if (m_collisionCapsules[sentCapsuleIndex].m_capsule.m_bone.m_start != originalStart || m_collisionCapsules[sentCapsuleIndex].m_capsule.m_bone.m_end != originalEnd)
	{
		//Collision Normals
		Vec3 collisionNormal = (sentCapsule.m_bone.m_start - originalStart).GetNormalized();
		if (collisionNormal.x != 0.0 || collisionNormal.y != 0.0 || collisionNormal.z != 0)
			m_particles.m_collisionNormals[sentCapsuleIndex] = collisionNormal;
			m_particles.m_isSelfCollision[sentCapsuleIndex] = 0;
		collisionNormal = (sentCapsule.m_bone.m_end - originalEnd).GetNormalized();
		if (collisionNormal.x != 0.0 || collisionNormal.y != 0.0 || collisionNormal.z != 0)
			m_particles.m_collisionNormals[sentCapsuleIndex + 1] = collisionNormal;
			m_particles.m_isSelfCollision[sentCapsuleIndex + 1] = 0;
	}

	//Update Proposed Particles
	UpdateProposedParticlesFromCollisionCapsule(sentCapsuleIndex);
}

//-----------------------------------------------------------------------------------------------
void RopeSimulation3D::ProjectWorldBoundsConstraintsCapsulesGaussSeidel(int sentCapsuleIndex)
{
	//Initializations
	Capsule3& sentCapsule = m_collisionCapsules[sentCapsuleIndex].m_capsule;

	//START CAPSULE
	//Min Z
	if (sentCapsule.m_bone.m_start.z < m_worldBounds.m_mins.z + m_ropeRadius)
	{
		sentCapsule.m_bone.m_start.z = m_worldBounds.m_mins.z + m_ropeRadius;
	}
	//Max Z
	if (sentCapsule.m_bone.m_start.z > m_worldBounds.m_maxs.z - m_ropeRadius)
	{
		sentCapsule.m_bone.m_start.z = m_worldBounds.m_maxs.z - m_ropeRadius;
	}
	//Min X
	if (sentCapsule.m_bone.m_start.x < m_worldBounds.m_mins.x + m_ropeRadius)
	{
		sentCapsule.m_bone.m_start.x = m_worldBounds.m_mins.x + m_ropeRadius;
	}
	//Max X
	if (sentCapsule.m_bone.m_start.x > m_worldBounds.m_maxs.x - m_ropeRadius)
	{
		sentCapsule.m_bone.m_start.x = m_worldBounds.m_maxs.x - m_ropeRadius;
	}
	//Min Y
	if (sentCapsule.m_bone.m_start.y < m_worldBounds.m_mins.y + m_ropeRadius)
	{
		sentCapsule.m_bone.m_start.y = m_worldBounds.m_mins.y + m_ropeRadius;
	}
	//Max Y
	if (sentCapsule.m_bone.m_start.y > m_worldBounds.m_maxs.y - m_ropeRadius)
	{
		sentCapsule.m_bone.m_start.y = m_worldBounds.m_maxs.y - m_ropeRadius;
	}

	//END CAPSULE
	//Min Z
	if (sentCapsule.m_bone.m_end.z < m_worldBounds.m_mins.z + m_ropeRadius)
	{
		sentCapsule.m_bone.m_end.z = m_worldBounds.m_mins.z + m_ropeRadius;
	}
	//Max Z				   
	if (sentCapsule.m_bone.m_end.z > m_worldBounds.m_maxs.z - m_ropeRadius)
	{
		sentCapsule.m_bone.m_end.z = m_worldBounds.m_maxs.z - m_ropeRadius;
	}
	//Min X				   
	if (sentCapsule.m_bone.m_end.x < m_worldBounds.m_mins.x + m_ropeRadius)
	{
		sentCapsule.m_bone.m_end.x = m_worldBounds.m_mins.x + m_ropeRadius;
	}
	//Max X				   
	if (sentCapsule.m_bone.m_end.x > m_worldBounds.m_maxs.x - m_ropeRadius)
	{
		sentCapsule.m_bone.m_end.x = m_worldBounds.m_maxs.x - m_ropeRadius;
	}
	//Min Y				   
	if (sentCapsule.m_bone.m_end.y < m_worldBounds.m_mins.y + m_ropeRadius)
	{
		sentCapsule.m_bone.m_end.y = m_worldBounds.m_mins.y + m_ropeRadius;
	}
	//Max Y				   
	if (sentCapsule.m_bone.m_end.y > m_worldBounds.m_maxs.y - m_ropeRadius)
	{
		sentCapsule.m_bone.m_end.y = m_worldBounds.m_maxs.y - m_ropeRadius;
	}
}

//-----------------------------------------------------------------------------------------------
void RopeSimulation3D::UpdateJacobi()
{
	for (int particleIndex = 0; particleIndex < m_particles.m_positions.size(); particleIndex++)
	{
		if (m_particles.m_isAttached[particleIndex] == 1)
		{
			m_particles.m_proposedPositions[particleIndex] = m_particles.m_positions[particleIndex];
			UpdateCollisionCapsulesFromParticle(particleIndex);
			continue;
		}

		//Calculate next velocity (semi-implicit Euler)
		Vec3 acceleration = (Vec3(0.0f, 0.0f, -m_gravityCoefficient));
		Vec3 velocity = m_particles.m_velocities[particleIndex];
		velocity += acceleration * m_physicsTimestep;

		//Damp Velocities
		velocity *= m_dampingCoefficient;

		//Calculate Proposed Positions and Update Collision Capsules
		Vec3 deltaPosition = velocity * m_physicsTimestep;
		m_particles.m_proposedPositions[particleIndex] = m_particles.m_positions[particleIndex] + deltaPosition;
		UpdateCollisionCapsulesFromParticle(particleIndex);
	}

	//Constraint Projection
	m_collisionCount = 0;
	for (int solverIterationIndex = 0; solverIterationIndex < m_totalSolverIterations; solverIterationIndex++)
	{
		ProjectConstraintsJacobi();
	}

	//Loop through and set projected positions and velocities
	for (int particleIndex = 0; particleIndex < m_particles.m_positions.size(); particleIndex++)
	{
		//Calculate new velocity
		m_particles.m_velocities[particleIndex] = (m_particles.m_proposedPositions[particleIndex] - m_particles.m_positions[particleIndex]) / m_physicsTimestep;

		//Friction Logic
		if (m_particles.m_collisionNormals[particleIndex] != Vec3(0.0, 0.0, 0.0))
		{
			//Static Friction
			if (m_particles.m_velocities[particleIndex].GetLength() < 0.01f)
			{
				m_particles.m_velocities[particleIndex] = Vec3();
				m_particles.m_collisionNormals[particleIndex] = Vec3();
				continue;
			}

			//Kinetic Friction
			Vec3 tangentalFriction = GetProjectedOnto3D(m_particles.m_velocities[particleIndex], m_particles.m_collisionNormals[particleIndex]) - m_particles.m_velocities[particleIndex];
			m_particles.m_velocities[particleIndex] += tangentalFriction * m_kineticFrictionCoefficient;
			m_particles.m_collisionNormals[particleIndex] = Vec3();
		}

		m_particles.m_positions[particleIndex] = m_particles.m_proposedPositions[particleIndex];
	}
}

//-----------------------------------------------------------------------------------------------
void RopeSimulation3D::ProjectConstraintsJacobi()
{
	//Distance and Bending Constraints
	for (int constraintIndex = 0; constraintIndex < m_distanceConstraints.size(); constraintIndex++)
	{
		ProjectDistanceConstraintJacobi(constraintIndex);
	}
	for (int constraintIndex = 0; constraintIndex < m_bendingConstraints.size(); constraintIndex++)
	{
		ProjectBendingConstraintJacobi(constraintIndex);
	}

	//Update the delta prior to collisions 
	for (int particleIndex = 0; particleIndex < m_particles.m_positions.size(); particleIndex++)
	{
		if (m_particles.m_jacobiCorrections[particleIndex].w != 0.0)
		{
			m_particles.m_proposedPositions[particleIndex] += Vec3(m_particles.m_jacobiCorrections[particleIndex].x, m_particles.m_jacobiCorrections[particleIndex].y, m_particles.m_jacobiCorrections[particleIndex].z) / float(m_particles.m_jacobiCorrections[particleIndex].w);
			m_particles.m_jacobiCorrections[particleIndex] = Vec4();
		}
	}

	//Collision Constraints
	if (m_collisionType == CollisionType::SPHERES)
	{
		BitRegionDetectionAllParticles();

		for (int particleIndex = 0; particleIndex < m_particles.m_positions.size(); particleIndex++)
		{
			ProjectCollisionConstraintsSpheresJacobi(particleIndex);
		}
	}
	else if (m_collisionType == CollisionType::CAPSULES)
	{
		//Phase 1 graph coloring
		BitRegionDetectionAllCapsules();
		for (int capsuleIndex = 0; capsuleIndex < m_collisionCapsules.size(); capsuleIndex++)
		{
			CapsuleCollisionObject& capsuleCollisionObject = m_collisionCapsules[capsuleIndex];
			capsuleCollisionObject.m_collisionNormalStart = Vec3();
			capsuleCollisionObject.m_collisionNormalEnd = Vec3();
			capsuleCollisionObject.m_isCollidingStart = 0;
			capsuleCollisionObject.m_isCollidingEnd = 0;
			capsuleCollisionObject.m_jacobiConstraintTotalStart = 0;
			capsuleCollisionObject.m_jacobiConstraintTotalEnd = 0;
			capsuleCollisionObject.m_jacobiCorrectionStart = Vec3();
			capsuleCollisionObject.m_jacobiCorrectionEnd = Vec3();
			capsuleCollisionObject.m_capsule.m_bone.m_start = m_particles.m_proposedPositions[capsuleIndex];
			capsuleCollisionObject.m_capsule.m_bone.m_end = m_particles.m_proposedPositions[capsuleIndex + 1];
		}

		for (int capsuleIndex = 0; capsuleIndex < m_collisionCapsules.size(); capsuleIndex++)
		{	
			if (capsuleIndex % 2 == 0)
			{
				ProjectCollisionConstraintsCapsulesJacobi(capsuleIndex);
			}
		}

		//Update Particles After first graph color
		for (int particleIndex = 0; particleIndex < m_numberOfParticlesInRope; particleIndex++)
		{
			if (m_particles.m_isAttached[particleIndex] == 1)
			{
				continue;
			}

			if (particleIndex != 0)
			{
				if (m_collisionCapsules[particleIndex - 1].m_isCollidingEnd == 1 && m_particles.m_jacobiCorrections[particleIndex].w == 0)
				{
					m_particles.m_jacobiCorrections[particleIndex].x += m_collisionCapsules[particleIndex - 1].m_jacobiCorrectionEnd.x;
					m_particles.m_jacobiCorrections[particleIndex].y += m_collisionCapsules[particleIndex - 1].m_jacobiCorrectionEnd.y;
					m_particles.m_jacobiCorrections[particleIndex].z += m_collisionCapsules[particleIndex - 1].m_jacobiCorrectionEnd.z;
					m_particles.m_jacobiCorrections[particleIndex].w += m_collisionCapsules[particleIndex - 1].m_jacobiConstraintTotalEnd;
					m_particles.m_collisionNormals[particleIndex] = m_collisionCapsules[particleIndex - 1].m_collisionNormalEnd;
				}
			}
			if (particleIndex != m_numberOfParticlesInRope - 1)
			{
				if (m_collisionCapsules[particleIndex].m_isCollidingStart == 1 && m_particles.m_jacobiCorrections[particleIndex].w == 0)
				{
					m_particles.m_jacobiCorrections[particleIndex].x += m_collisionCapsules[particleIndex].m_jacobiCorrectionStart.x;
					m_particles.m_jacobiCorrections[particleIndex].y += m_collisionCapsules[particleIndex].m_jacobiCorrectionStart.y;
					m_particles.m_jacobiCorrections[particleIndex].z += m_collisionCapsules[particleIndex].m_jacobiCorrectionStart.z;
					m_particles.m_jacobiCorrections[particleIndex].w += m_collisionCapsules[particleIndex].m_jacobiConstraintTotalStart;
					m_particles.m_collisionNormals[particleIndex] = m_collisionCapsules[particleIndex].m_collisionNormalStart;
				}
			}
		}

		//Update the delta
		for (int particleIndex = 0; particleIndex < m_particles.m_positions.size(); particleIndex++)
		{
			if (m_particles.m_jacobiCorrections[particleIndex].w != 0.0)
			{
				m_particles.m_proposedPositions[particleIndex] += Vec3(m_particles.m_jacobiCorrections[particleIndex].x, m_particles.m_jacobiCorrections[particleIndex].y, m_particles.m_jacobiCorrections[particleIndex].z) / float(m_particles.m_jacobiCorrections[particleIndex].w);
				m_particles.m_jacobiCorrections[particleIndex] = Vec4();
			}
		}



		//Second Phase of graph coloring
		BitRegionDetectionAllCapsules();
		for (int capsuleIndex = 0; capsuleIndex < m_collisionCapsules.size(); capsuleIndex++)
		{
			CapsuleCollisionObject& capsuleCollisionObject = m_collisionCapsules[capsuleIndex];
			capsuleCollisionObject.m_collisionNormalStart = Vec3();
			capsuleCollisionObject.m_collisionNormalEnd = Vec3();
			capsuleCollisionObject.m_isCollidingStart = 0;
			capsuleCollisionObject.m_isCollidingEnd = 0;
			capsuleCollisionObject.m_jacobiConstraintTotalStart = 0;
			capsuleCollisionObject.m_jacobiConstraintTotalEnd = 0;
			capsuleCollisionObject.m_jacobiCorrectionStart = Vec3();
			capsuleCollisionObject.m_jacobiCorrectionEnd = Vec3();
			capsuleCollisionObject.m_capsule.m_bone.m_start = m_particles.m_proposedPositions[capsuleIndex];
			capsuleCollisionObject.m_capsule.m_bone.m_end = m_particles.m_proposedPositions[capsuleIndex + 1];
		}
		for (int capsuleIndex = 0; capsuleIndex < m_collisionCapsules.size(); capsuleIndex++)
		{
			if (capsuleIndex % 2 == 1)
			{
				ProjectCollisionConstraintsCapsulesJacobi(capsuleIndex);
			}
		}

		//Update Particles After second graph color
		for (int particleIndex = 0; particleIndex < m_numberOfParticlesInRope; particleIndex++)
		{
			if (m_particles.m_isAttached[particleIndex] == 1)
			{
				continue;
			}

			if (particleIndex != 0)
			{
				if (m_collisionCapsules[particleIndex - 1].m_isCollidingEnd == 1 && m_particles.m_jacobiCorrections[particleIndex].w == 0)
				{
					m_particles.m_jacobiCorrections[particleIndex].x += m_collisionCapsules[particleIndex - 1].m_jacobiCorrectionEnd.x;
					m_particles.m_jacobiCorrections[particleIndex].y += m_collisionCapsules[particleIndex - 1].m_jacobiCorrectionEnd.y;
					m_particles.m_jacobiCorrections[particleIndex].z += m_collisionCapsules[particleIndex - 1].m_jacobiCorrectionEnd.z;
					m_particles.m_jacobiCorrections[particleIndex].w += m_collisionCapsules[particleIndex - 1].m_jacobiConstraintTotalEnd;
					m_particles.m_collisionNormals[particleIndex] = m_collisionCapsules[particleIndex - 1].m_collisionNormalEnd;
				}
			}
			if (particleIndex != m_numberOfParticlesInRope - 1)
			{
				if (m_collisionCapsules[particleIndex].m_isCollidingStart == 1 && m_particles.m_jacobiCorrections[particleIndex].w == 0)
				{
					m_particles.m_jacobiCorrections[particleIndex].x += m_collisionCapsules[particleIndex].m_jacobiCorrectionStart.x;
					m_particles.m_jacobiCorrections[particleIndex].y += m_collisionCapsules[particleIndex].m_jacobiCorrectionStart.y;
					m_particles.m_jacobiCorrections[particleIndex].z += m_collisionCapsules[particleIndex].m_jacobiCorrectionStart.z;
					m_particles.m_jacobiCorrections[particleIndex].w += m_collisionCapsules[particleIndex].m_jacobiConstraintTotalStart;
					m_particles.m_collisionNormals[particleIndex] = m_collisionCapsules[particleIndex].m_collisionNormalStart;
				}
			}
		}
	}

	//Update the delta
	for (int particleIndex = 0; particleIndex < m_particles.m_positions.size(); particleIndex++)
	{
		if (m_particles.m_jacobiCorrections[particleIndex].w != 0.0)
		{
			m_particles.m_proposedPositions[particleIndex] += Vec3(m_particles.m_jacobiCorrections[particleIndex].x, m_particles.m_jacobiCorrections[particleIndex].y, m_particles.m_jacobiCorrections[particleIndex].z) / float(m_particles.m_jacobiCorrections[particleIndex].w);
			m_particles.m_jacobiCorrections[particleIndex] = Vec4();
		}
	}
}

//-----------------------------------------------------------------------------------------------
void RopeSimulation3D::ProjectDistanceConstraintJacobi(int constraintIndex)
{
	//Initializations
	Constraint3D& constraint = m_distanceConstraints[constraintIndex];

	//Calculates direction and overflow along with weight Coefficients
	Vec3 displacement = m_particles.m_proposedPositions[constraint.m_indices[1]] - m_particles.m_proposedPositions[constraint.m_indices[0]];
	float desiredDistance = 0.0f;
	if (m_isHierarchical)
	{
		desiredDistance = abs(constraint.m_indices[1] - constraint.m_indices[0]) * m_desiredDistance;
	}
	else
	{
		desiredDistance = (m_desiredDistance);
	}
	float distanceConstraint = (displacement.GetLength() - desiredDistance);

	if (constraint.m_constraintEquality == Constraint3DEquality::EQUALITY)
	{
		if (distanceConstraint == 0.0f)
		{
			return;
		}
	}
	else if (constraint.m_constraintEquality == Constraint3DEquality::INEQUALITY_GREATER)
	{
		if (distanceConstraint <= 0.0f)
		{
			return;
		}
	}
	else if (constraint.m_constraintEquality == Constraint3DEquality::INEQUALITY_LESS)
	{
		if (distanceConstraint >= 0.0f)
		{
			return;
		}
	}

	Vec3 gradient = displacement.GetNormalized();
	float coefficientValue = 0.0f;
	if (distanceConstraint < 0.0f)
	{
		coefficientValue = m_compressionCoefficient;
	}
	else
	{
		coefficientValue = m_stretchingCoefficient;
	}

	if (m_particles.m_isAttached[constraint.m_indices[0]] == 0 && m_particles.m_isAttached[constraint.m_indices[1]] == 0)
	{
		float particleAWeightCoefficient = (m_particles.m_inverseMasses[constraint.m_indices[0]] / (m_particles.m_inverseMasses[constraint.m_indices[0]] + m_particles.m_inverseMasses[constraint.m_indices[1]]));
		float particleBWeightCoefficient = (m_particles.m_inverseMasses[constraint.m_indices[1]] / (m_particles.m_inverseMasses[constraint.m_indices[0]] + m_particles.m_inverseMasses[constraint.m_indices[1]]));

		Vec3 deltaParticleA = distanceConstraint * particleAWeightCoefficient * gradient * coefficientValue;
		Vec3 deltaParticleB = distanceConstraint * particleBWeightCoefficient * gradient * coefficientValue;

		m_particles.m_jacobiCorrections[constraint.m_indices[0]].x += deltaParticleA.x;
		m_particles.m_jacobiCorrections[constraint.m_indices[0]].y += deltaParticleA.y;
		m_particles.m_jacobiCorrections[constraint.m_indices[0]].z += deltaParticleA.z;
		m_particles.m_jacobiCorrections[constraint.m_indices[0]].w++;
		m_particles.m_jacobiCorrections[constraint.m_indices[1]].x -= deltaParticleB.x;
		m_particles.m_jacobiCorrections[constraint.m_indices[1]].y -= deltaParticleB.y;
		m_particles.m_jacobiCorrections[constraint.m_indices[1]].z -= deltaParticleB.z;
		m_particles.m_jacobiCorrections[constraint.m_indices[1]].w++;
	}
	else if (m_particles.m_isAttached[constraint.m_indices[0]] == 1 && m_particles.m_isAttached[constraint.m_indices[1]] == 0)
	{
		Vec3 deltaParticleB = distanceConstraint * gradient * coefficientValue;
		m_particles.m_jacobiCorrections[constraint.m_indices[1]].x -= deltaParticleB.x;
		m_particles.m_jacobiCorrections[constraint.m_indices[1]].y -= deltaParticleB.y;
		m_particles.m_jacobiCorrections[constraint.m_indices[1]].z -= deltaParticleB.z;
		m_particles.m_jacobiCorrections[constraint.m_indices[1]].w++;
	}
	else if (m_particles.m_isAttached[constraint.m_indices[0]] == 0 && m_particles.m_isAttached[constraint.m_indices[1]] == 1)
	{
		Vec3 deltaParticleA = distanceConstraint * gradient * coefficientValue;
		m_particles.m_jacobiCorrections[constraint.m_indices[0]].x += deltaParticleA.x;
		m_particles.m_jacobiCorrections[constraint.m_indices[0]].y += deltaParticleA.y;
		m_particles.m_jacobiCorrections[constraint.m_indices[0]].z += deltaParticleA.z;
		m_particles.m_jacobiCorrections[constraint.m_indices[0]].w++;
	}
}

//-----------------------------------------------------------------------------------------------
void RopeSimulation3D::ProjectBendingConstraintJacobi(int constraintIndex)
{
	//Initializations
	Constraint3D& constraint = m_bendingConstraints[constraintIndex];

	//Calculates direction and overflow along with weight Coefficients
	Vec3 displacement = m_particles.m_proposedPositions[constraint.m_indices[2]] - m_particles.m_proposedPositions[constraint.m_indices[0]];
	float distanceConstraint = displacement.GetLength() - m_bendingConstraintDistance;

	//Check if less than the minimum distance, if so no need to constrain
	if (distanceConstraint > 0.0f)
	{
		return;
	}

	Vec3 gradient = displacement.GetNormalized();
	Vec3 directionCheck = (m_particles.m_proposedPositions[constraint.m_indices[1]] - m_particles.m_proposedPositions[constraint.m_indices[0]]).GetNormalized();

	//Because this is a cheap method to check for bending need to ensure the direction inst the same
	if (gradient == directionCheck)
	{
		return;
	}

	if (m_particles.m_isAttached[constraint.m_indices[0]] == 0 && m_particles.m_isAttached[constraint.m_indices[2]] == 0)
	{
		float pointAWeightCoefficient = (m_particles.m_inverseMasses[constraint.m_indices[0]] / (m_particles.m_inverseMasses[constraint.m_indices[0]] + m_particles.m_inverseMasses[constraint.m_indices[2]]));
		float pointCWeightCoefficient = (m_particles.m_inverseMasses[constraint.m_indices[2]] / (m_particles.m_inverseMasses[constraint.m_indices[0]] + m_particles.m_inverseMasses[constraint.m_indices[2]]));

		Vec3 deltaPointA = pointAWeightCoefficient * distanceConstraint * gradient * m_bendingCoefficient;
		Vec3 deltaPointC = pointCWeightCoefficient * distanceConstraint * gradient * m_bendingCoefficient;

		m_particles.m_jacobiCorrections[constraint.m_indices[0]].x += deltaPointA.x;
		m_particles.m_jacobiCorrections[constraint.m_indices[0]].y += deltaPointA.y;
		m_particles.m_jacobiCorrections[constraint.m_indices[0]].z += deltaPointA.z;
		m_particles.m_jacobiCorrections[constraint.m_indices[0]].w++;
		m_particles.m_jacobiCorrections[constraint.m_indices[2]].x -= deltaPointC.x;
		m_particles.m_jacobiCorrections[constraint.m_indices[2]].y -= deltaPointC.y;
		m_particles.m_jacobiCorrections[constraint.m_indices[2]].z -= deltaPointC.z;
		m_particles.m_jacobiCorrections[constraint.m_indices[2]].w++;
	}
	else if (m_particles.m_isAttached[constraint.m_indices[0]] == 1 && m_particles.m_isAttached[constraint.m_indices[2]] == 0)
	{
		Vec3 deltaPointC = distanceConstraint * gradient * m_bendingCoefficient;
		m_particles.m_jacobiCorrections[constraint.m_indices[2]].x -= deltaPointC.x;
		m_particles.m_jacobiCorrections[constraint.m_indices[2]].y -= deltaPointC.y;
		m_particles.m_jacobiCorrections[constraint.m_indices[2]].z -= deltaPointC.z;
		m_particles.m_jacobiCorrections[constraint.m_indices[2]].w++;
	}
	else if (m_particles.m_isAttached[constraint.m_indices[0]] == 0 && m_particles.m_isAttached[constraint.m_indices[2]] == 1)
	{
		Vec3 deltaPointA = distanceConstraint * gradient * m_bendingCoefficient;
		m_particles.m_jacobiCorrections[constraint.m_indices[0]].x += deltaPointA.x;
		m_particles.m_jacobiCorrections[constraint.m_indices[0]].y += deltaPointA.y;
		m_particles.m_jacobiCorrections[constraint.m_indices[0]].z += deltaPointA.z;
		m_particles.m_jacobiCorrections[constraint.m_indices[0]].w++;
	}
}

//-----------------------------------------------------------------------------------------------
void RopeSimulation3D::ProjectCollisionConstraintsSpheresJacobi(int sentParticleIndex)
{
	//Self Collisions
	if (m_isSelfCollisionEnabled)
	{
		for (int particleIndex = 0; particleIndex < m_particles.m_positions.size(); particleIndex++)
		{
			if (particleIndex != sentParticleIndex && particleIndex != sentParticleIndex - 1 && particleIndex != sentParticleIndex + 1)
			{
				if ((m_particles.m_macroBitRegions[sentParticleIndex] & m_particles.m_macroBitRegions[particleIndex]) != 0)
				{
					if ((m_particles.m_microBitRegions[sentParticleIndex] & m_particles.m_microBitRegions[particleIndex]) != 0)
					{
						Vec3 sentNewPosition = m_particles.m_proposedPositions[sentParticleIndex];
						Vec3 currentNewPosition = m_particles.m_proposedPositions[particleIndex];
						if (PushSphereOutOfSphere3D(sentNewPosition, m_ropeRadius, currentNewPosition, m_ropeRadius))
						{
							m_particles.m_collisionNormals[sentParticleIndex] = (sentNewPosition - m_particles.m_proposedPositions[sentParticleIndex]).GetNormalized();
							Vec3 displacement = (sentNewPosition - m_particles.m_proposedPositions[sentParticleIndex]);
							m_particles.m_jacobiCorrections[sentParticleIndex].x += displacement.x;
							m_particles.m_jacobiCorrections[sentParticleIndex].y += displacement.y;
							m_particles.m_jacobiCorrections[sentParticleIndex].z += displacement.z;
							m_particles.m_jacobiCorrections[sentParticleIndex].w++;
						}
					}
				}
			}
		}
	}

	//Collision Objects
	for (int collisionObjectIndex = 0; collisionObjectIndex < m_collisionObjects.size(); collisionObjectIndex++)
	{
		CollisionObject*& collisionObject = m_collisionObjects[collisionObjectIndex];
		Vec3 newPosition = m_particles.m_proposedPositions[sentParticleIndex];
		if (collisionObject)
		{
			SphereCollisionObject* sphere = dynamic_cast<SphereCollisionObject*>(collisionObject);
			if (sphere) //Sphere Collisions
			{
				if ((sphere->m_macroBitRegions & m_particles.m_macroBitRegions[sentParticleIndex]) != 0)
				{
					if ((sphere->m_microBitRegions & m_particles.m_microBitRegions[sentParticleIndex]) != 0)
					{
						if (PushSphereOutOfFixedSphere3D(newPosition, m_ropeRadius, sphere->m_sphere.m_center, sphere->m_sphere.m_radius))
						{
							m_particles.m_collisionNormals[sentParticleIndex] = (newPosition - m_particles.m_proposedPositions[sentParticleIndex]).GetNormalized();
							Vec3 displacement = (newPosition - m_particles.m_proposedPositions[sentParticleIndex]);
							m_particles.m_jacobiCorrections[sentParticleIndex].x += displacement.x;
							m_particles.m_jacobiCorrections[sentParticleIndex].y += displacement.y;
							m_particles.m_jacobiCorrections[sentParticleIndex].z += displacement.z;
							m_particles.m_jacobiCorrections[sentParticleIndex].w++;
						}
					}
				}
				continue;
			}
			AABBCollisionObject* aabb = dynamic_cast<AABBCollisionObject*>(collisionObject);
			if (aabb) //AABB Collisions
			{
				if ((aabb->m_macroBitRegions & m_particles.m_macroBitRegions[sentParticleIndex]) != 0)
				{
					if ((aabb->m_microBitRegions & m_particles.m_microBitRegions[sentParticleIndex]) != 0)
					{
						//Bounding Disc Check
						if (DoSpheresOverlap(m_particles.m_proposedPositions[sentParticleIndex], m_ropeRadius, aabb->m_boundingDiscCenter, aabb->m_boundingDiscRadius))
						{
							if (PushSphereOutOfFixedAABB3D(newPosition, m_ropeRadius, aabb->m_aabb))
							{
								m_particles.m_collisionNormals[sentParticleIndex] = (newPosition - m_particles.m_proposedPositions[sentParticleIndex]).GetNormalized();
								Vec3 displacement = (newPosition - m_particles.m_proposedPositions[sentParticleIndex]);
								m_particles.m_jacobiCorrections[sentParticleIndex].x += displacement.x;
								m_particles.m_jacobiCorrections[sentParticleIndex].y += displacement.y;
								m_particles.m_jacobiCorrections[sentParticleIndex].z += displacement.z;
								m_particles.m_jacobiCorrections[sentParticleIndex].w++;
							}
						}
					}
				}
				continue;
			}
			OBBCollisionObject* obb = dynamic_cast<OBBCollisionObject*>(collisionObject);
			if (obb) //OBB Collisions
			{
				if ((obb->m_macroBitRegions & m_particles.m_macroBitRegions[sentParticleIndex]) != 0)
				{
					if ((obb->m_microBitRegions & m_particles.m_microBitRegions[sentParticleIndex]) != 0)
					{
						//Bounding Disc Check
						if (DoSpheresOverlap(m_particles.m_proposedPositions[sentParticleIndex], m_ropeRadius, obb->m_boundingDiscCenter, obb->m_boundingDiscRadius))
						{
							if (PushDiscOutOfFixedOBB3D(newPosition, m_ropeRadius, obb->m_obb))
							{
								m_particles.m_collisionNormals[sentParticleIndex] = (newPosition - m_particles.m_proposedPositions[sentParticleIndex]).GetNormalized();
								Vec3 displacement = (newPosition - m_particles.m_proposedPositions[sentParticleIndex]);
								m_particles.m_jacobiCorrections[sentParticleIndex].x += displacement.x;
								m_particles.m_jacobiCorrections[sentParticleIndex].y += displacement.y;
								m_particles.m_jacobiCorrections[sentParticleIndex].z += displacement.z;
								m_particles.m_jacobiCorrections[sentParticleIndex].w++;
							}
						}
					}
				}
				continue;
			}
			CapsuleCollisionObject* capsule = dynamic_cast<CapsuleCollisionObject*>(collisionObject);
			if (capsule) //Capsule Collisions
			{
				if ((capsule->m_macroBitRegions & m_particles.m_macroBitRegions[sentParticleIndex]) != 0)
				{
					if ((capsule->m_microBitRegions & m_particles.m_microBitRegions[sentParticleIndex]) != 0)
					{
						//Bounding Disc Check
						if (DoSpheresOverlap(m_particles.m_proposedPositions[sentParticleIndex], m_ropeRadius, capsule->m_boundingDiscCenter, capsule->m_boundingDiscRadius))
						{
							if (PushSphereOutOfFixedCapsule3D(newPosition, m_ropeRadius, capsule->m_capsule))
							{
								m_particles.m_collisionNormals[sentParticleIndex] = (newPosition - m_particles.m_proposedPositions[sentParticleIndex]).GetNormalized();
								Vec3 displacement = (newPosition - m_particles.m_proposedPositions[sentParticleIndex]);
								m_particles.m_jacobiCorrections[sentParticleIndex].x += displacement.x;
								m_particles.m_jacobiCorrections[sentParticleIndex].y += displacement.y;
								m_particles.m_jacobiCorrections[sentParticleIndex].z += displacement.z;
								m_particles.m_jacobiCorrections[sentParticleIndex].w++;
							}
						}
					}
				}
				continue;
			}
			CylinderCollisionObject* cylinder = dynamic_cast<CylinderCollisionObject*>(collisionObject);
			if (cylinder) //Cylinder Collisions
			{
				if ((cylinder->m_macroBitRegions & m_particles.m_macroBitRegions[sentParticleIndex]) != 0)
				{
					if ((cylinder->m_microBitRegions & m_particles.m_microBitRegions[sentParticleIndex]) != 0)
					{
						//Bounding Disc Check
						if (DoSpheresOverlap(m_particles.m_proposedPositions[sentParticleIndex], m_ropeRadius, cylinder->m_boundingDiscCenter, cylinder->m_boundingDiscRadius))
						{
							if (PushSphereOutOfFixedCylinder3D(newPosition, m_ropeRadius, cylinder->m_cylinder))
							{
								m_particles.m_collisionNormals[sentParticleIndex] = (newPosition - m_particles.m_proposedPositions[sentParticleIndex]).GetNormalized();
								Vec3 displacement = (newPosition - m_particles.m_proposedPositions[sentParticleIndex]);
								m_particles.m_jacobiCorrections[sentParticleIndex].x += displacement.x;
								m_particles.m_jacobiCorrections[sentParticleIndex].y += displacement.y;
								m_particles.m_jacobiCorrections[sentParticleIndex].z += displacement.z;
								m_particles.m_jacobiCorrections[sentParticleIndex].w++;
							}
						}
					}
				}
				continue;
			}
		}
	}

	//World Bounds Collisions
	ProjectWorldBoundsConstraintsSpheresJacobi(sentParticleIndex);
}

//-----------------------------------------------------------------------------------------------
void RopeSimulation3D::ProjectWorldBoundsConstraintsSpheresJacobi(int sentParticleIndex)
{
	//Initializations
	Vec3 newPosition = m_particles.m_proposedPositions[sentParticleIndex];

	//Min Z
	if (m_particles.m_proposedPositions[sentParticleIndex].z < m_worldBounds.m_mins.z + m_ropeRadius)
	{
		newPosition.z = m_worldBounds.m_mins.z + m_ropeRadius;
		m_particles.m_collisionNormals[sentParticleIndex] = Vec3(0.0f, 0.0f, 1.0f);
		m_particles.m_jacobiCorrections[sentParticleIndex].w++;
	}
	//Max Z
	if (m_particles.m_proposedPositions[sentParticleIndex].z > m_worldBounds.m_maxs.z - m_ropeRadius)
	{
		newPosition.z = m_worldBounds.m_maxs.z - m_ropeRadius;
		m_particles.m_collisionNormals[sentParticleIndex] = Vec3(0.0f, 0.0f, -1.0f);
		m_particles.m_jacobiCorrections[sentParticleIndex].w++;
	}
	//Min X
	if (m_particles.m_proposedPositions[sentParticleIndex].x < m_worldBounds.m_mins.x + m_ropeRadius)
	{
		newPosition.x = m_worldBounds.m_mins.x + m_ropeRadius;
		m_particles.m_collisionNormals[sentParticleIndex] = Vec3(1.0f, 0.0f, 0.0f);
		m_particles.m_jacobiCorrections[sentParticleIndex].w++;
	}
	//Max X
	if (m_particles.m_proposedPositions[sentParticleIndex].x > m_worldBounds.m_maxs.x - m_ropeRadius)
	{
		newPosition.x = m_worldBounds.m_maxs.x - m_ropeRadius;
		m_particles.m_collisionNormals[sentParticleIndex] = Vec3(-1.0f, 0.0f, 0.0f);
		m_particles.m_jacobiCorrections[sentParticleIndex].w++;
	}
	//Min Y
	if (m_particles.m_proposedPositions[sentParticleIndex].y < m_worldBounds.m_mins.y + m_ropeRadius)
	{
		newPosition.y = m_worldBounds.m_mins.y + m_ropeRadius;
		m_particles.m_collisionNormals[sentParticleIndex] = Vec3(0.0f, 1.0f, 0.0f);
		m_particles.m_jacobiCorrections[sentParticleIndex].w++;
	}
	//Max Y
	if (m_particles.m_proposedPositions[sentParticleIndex].y > m_worldBounds.m_maxs.y - m_ropeRadius)
	{
		newPosition.y = m_worldBounds.m_maxs.y - m_ropeRadius;
		m_particles.m_collisionNormals[sentParticleIndex] = Vec3(0.0f, -1.0f, 0.0f);
		m_particles.m_jacobiCorrections[sentParticleIndex].w++;
	}
	
	Vec3 displacement = (newPosition - m_particles.m_proposedPositions[sentParticleIndex]);
	m_particles.m_jacobiCorrections[sentParticleIndex].x += displacement.x;
	m_particles.m_jacobiCorrections[sentParticleIndex].y += displacement.y;
	m_particles.m_jacobiCorrections[sentParticleIndex].z += displacement.z;
}

//-----------------------------------------------------------------------------------------------
void RopeSimulation3D::ProjectCollisionConstraintsCapsulesJacobi(int sentCapsuleIndex)
{
	//Initializations
	CapsuleCollisionObject& sentCollisionObject = m_collisionCapsules[sentCapsuleIndex];
	Capsule3& sentCapsule = m_collisionCapsules[sentCapsuleIndex].m_capsule;
	UnaddCollidingRopeParticle(sentCapsuleIndex);

	//Self Collisions
	if (m_isSelfCollisionEnabled)
	{
		for (int capsuleIndex = 0; capsuleIndex < m_collisionCapsules.size(); capsuleIndex++)
		{
			if (capsuleIndex == sentCapsuleIndex || capsuleIndex == sentCapsuleIndex + 1 || capsuleIndex == sentCapsuleIndex - 1
				|| capsuleIndex == sentCapsuleIndex + 2 || capsuleIndex == sentCapsuleIndex - 2)
			{
				continue;
			}

			if ((m_collisionCapsules[sentCapsuleIndex].m_macroBitRegions & m_collisionCapsules[capsuleIndex].m_macroBitRegions) != 0)
			{
				if ((m_collisionCapsules[sentCapsuleIndex].m_microBitRegions & m_collisionCapsules[capsuleIndex].m_microBitRegions) != 0)
				{
					Capsule3& capsule = m_collisionCapsules[capsuleIndex].m_capsule;
					capsule.m_bone.m_start = m_particles.m_proposedPositions[capsuleIndex];
					capsule.m_bone.m_end = m_particles.m_proposedPositions[capsuleIndex + 1];

					Vec3 sentCapsuleCenter = (sentCapsule.m_bone.m_start + sentCapsule.m_bone.m_end) * 0.5f;
					Vec3 capsuleCenter = (capsule.m_bone.m_start + capsule.m_bone.m_end) * 0.5f;
					float sentCapsuleBoundingRadius = (sentCapsule.m_bone.m_start - sentCapsuleCenter).GetLength() + sentCapsule.m_radius;
					float capsuleBoundingRadius = (capsule.m_bone.m_start - capsuleCenter).GetLength() + capsule.m_radius;

					if (!DoSpheresOverlap(sentCapsuleCenter, sentCapsuleBoundingRadius, capsuleCenter, capsuleBoundingRadius))
						continue;

					Vec3 originalStart = sentCapsule.m_bone.m_start;
					Vec3 originalEnd = sentCapsule.m_bone.m_end;
					if (PushCapsuleOutOfCapsule3D(sentCapsule, capsule))
					{
						if (m_particles.m_isAttached[sentCapsuleIndex])
						{
							sentCapsule.m_bone.m_start = originalStart;
						}
						if (m_particles.m_isAttached[sentCapsuleIndex + 1])
						{
							sentCapsule.m_bone.m_end = originalEnd;
						}

						sentCollisionObject.m_collisionNormalStart = (sentCapsule.m_bone.m_start - originalStart).GetNormalized();
						sentCollisionObject.m_collisionNormalEnd = (sentCapsule.m_bone.m_end - originalEnd).GetNormalized();
						sentCollisionObject.m_jacobiCorrectionStart += (sentCapsule.m_bone.m_start - originalStart);
						sentCollisionObject.m_jacobiCorrectionEnd += (sentCapsule.m_bone.m_end - originalEnd);
						sentCollisionObject.m_jacobiConstraintTotalStart++;
						sentCollisionObject.m_jacobiConstraintTotalEnd++;
						sentCollisionObject.m_isCollidingStart = 1;
						sentCollisionObject.m_isCollidingEnd = 1;
					}
				}
			}
		}
	}

	//Collision Objects
	for (int collisionObjectIndex = 0; collisionObjectIndex < m_collisionObjects.size(); collisionObjectIndex++)
	{
		CollisionObject*& collisionObject = m_collisionObjects[collisionObjectIndex];
		Vec3 originalStart = sentCapsule.m_bone.m_start;
		Vec3 originalEnd = sentCapsule.m_bone.m_end;
		Vec3 capsuleBoundingDiscCenter = (originalStart + originalEnd) * 0.5f;
		float capsuleBoundingDiscRadius = ((originalStart + originalEnd).GetLength() + m_ropeRadius * 2.0f) * 0.5f;
		if (collisionObject)
		{
			SphereCollisionObject* sphere = dynamic_cast<SphereCollisionObject*>(collisionObject);
			if (sphere) //Sphere Collisions
			{
				if ((sphere->m_macroBitRegions & m_collisionCapsules[sentCapsuleIndex].m_macroBitRegions) != 0)
				{
					if ((sphere->m_microBitRegions & m_collisionCapsules[sentCapsuleIndex].m_microBitRegions) != 0)
					{
						if (PushCapsuleOutOfFixedSphere3D(sentCapsule, sphere->m_sphere.m_center, sphere->m_sphere.m_radius))
						{
							if (m_particles.m_isAttached[sentCapsuleIndex])
							{
								sentCapsule.m_bone.m_start = originalStart;
							}
							if (m_particles.m_isAttached[sentCapsuleIndex + 1])
							{
								sentCapsule.m_bone.m_end = originalEnd;
							}

							sentCollisionObject.m_collisionNormalStart = (sentCapsule.m_bone.m_start - originalStart).GetNormalized();
							sentCollisionObject.m_collisionNormalEnd = (sentCapsule.m_bone.m_end - originalEnd).GetNormalized();
							sentCollisionObject.m_jacobiCorrectionStart += (sentCapsule.m_bone.m_start - originalStart);
							sentCollisionObject.m_jacobiCorrectionEnd += (sentCapsule.m_bone.m_end - originalEnd);
							sentCollisionObject.m_jacobiConstraintTotalStart++;
							sentCollisionObject.m_jacobiConstraintTotalEnd++;
							sentCollisionObject.m_isCollidingStart = 1;
							sentCollisionObject.m_isCollidingEnd = 1;
						}
					}
				}
				continue;
			}
			AABBCollisionObject* aabb = dynamic_cast<AABBCollisionObject*>(collisionObject);
			if (aabb) //AABB Collisions
			{
				if ((aabb->m_macroBitRegions & sentCollisionObject.m_macroBitRegions) != 0)
				{
					if ((aabb->m_microBitRegions & sentCollisionObject.m_microBitRegions) != 0)
					{
						//Bounding Disk Check
						if (!DoSpheresOverlap(capsuleBoundingDiscCenter, capsuleBoundingDiscRadius, aabb->m_boundingDiscCenter, aabb->m_boundingDiscRadius))
							continue;

						if (PushCapsuleOutOfFixedAABB3D(sentCapsule, aabb->m_aabb))
						{
							if (m_particles.m_isAttached[sentCapsuleIndex])
							{
								sentCapsule.m_bone.m_start = originalStart;
							}
							if (m_particles.m_isAttached[sentCapsuleIndex + 1])
							{
								sentCapsule.m_bone.m_end = originalEnd;
							}

							sentCollisionObject.m_collisionNormalStart = (sentCapsule.m_bone.m_start - originalStart).GetNormalized();
							sentCollisionObject.m_collisionNormalEnd = (sentCapsule.m_bone.m_end - originalEnd).GetNormalized();
							sentCollisionObject.m_jacobiCorrectionStart += (sentCapsule.m_bone.m_start - originalStart);
							sentCollisionObject.m_jacobiCorrectionEnd += (sentCapsule.m_bone.m_end - originalEnd);
							sentCollisionObject.m_jacobiConstraintTotalStart++;
							sentCollisionObject.m_jacobiConstraintTotalEnd++;
							sentCollisionObject.m_isCollidingStart = 1;
							sentCollisionObject.m_isCollidingEnd = 1;
							/*m_collisionCount++;
							DebugAddWorldBillboardText(Stringf("%d", m_collisionCount), sentCapsule.m_bone.m_start, 0.01f, Vec2(0.5f, 0.5f), 0.1f, Rgba8::BLACK, Rgba8::BLACK);
							DebugAddWorldBillboardText(Stringf("%d", m_collisionCount), sentCapsule.m_bone.m_end, 0.01f, Vec2(0.5f, 0.5f), 0.1f, Rgba8::BLACK, Rgba8::BLACK);*/
						}
					}
				}
				continue;
			}
			OBBCollisionObject* obb = dynamic_cast<OBBCollisionObject*>(collisionObject);
			if (obb) //OBB Collisions
			{
				if ((obb->m_macroBitRegions & m_collisionCapsules[sentCapsuleIndex].m_macroBitRegions) != 0)
				{
					if ((obb->m_microBitRegions & m_collisionCapsules[sentCapsuleIndex].m_microBitRegions) != 0)
					{
						//Bounding Disk Check
						if (!DoSpheresOverlap(capsuleBoundingDiscCenter, capsuleBoundingDiscRadius, obb->m_boundingDiscCenter, obb->m_boundingDiscRadius))
							continue;

						if (PushCapsuleOutOfFixedOBB3D(sentCapsule, obb->m_obb))
						{
							if (m_particles.m_isAttached[sentCapsuleIndex])
							{
								sentCapsule.m_bone.m_start = originalStart;
							}
							if (m_particles.m_isAttached[sentCapsuleIndex + 1])
							{
								sentCapsule.m_bone.m_end = originalEnd;
							}

							sentCollisionObject.m_collisionNormalStart = (sentCapsule.m_bone.m_start - originalStart).GetNormalized();
							sentCollisionObject.m_collisionNormalEnd = (sentCapsule.m_bone.m_end - originalEnd).GetNormalized();
							sentCollisionObject.m_jacobiCorrectionStart += (sentCapsule.m_bone.m_start - originalStart);
							sentCollisionObject.m_jacobiCorrectionEnd += (sentCapsule.m_bone.m_end - originalEnd);
							sentCollisionObject.m_jacobiConstraintTotalStart++;
							sentCollisionObject.m_jacobiConstraintTotalEnd++;
							sentCollisionObject.m_isCollidingStart = 1;
							sentCollisionObject.m_isCollidingEnd = 1;
						}
					}
				}
				continue;
			}
			CapsuleCollisionObject* capsule = dynamic_cast<CapsuleCollisionObject*>(collisionObject);
			if (capsule) //Capsule Collisions
			{
				if ((capsule->m_macroBitRegions & m_collisionCapsules[sentCapsuleIndex].m_macroBitRegions) != 0)
				{
					if ((capsule->m_microBitRegions & m_collisionCapsules[sentCapsuleIndex].m_microBitRegions) != 0)
					{
						//Bounding Disk Check
						if (!DoSpheresOverlap(capsuleBoundingDiscCenter, capsuleBoundingDiscRadius, capsule->m_boundingDiscCenter, capsule->m_boundingDiscRadius))
							continue;

						if (PushCapsuleOutOfFixedCapsule3D(sentCapsule, capsule->m_capsule))
						{
							if (m_particles.m_isAttached[sentCapsuleIndex])
							{
								sentCapsule.m_bone.m_start = originalStart;
							}
							if (m_particles.m_isAttached[sentCapsuleIndex + 1])
							{
								sentCapsule.m_bone.m_end = originalEnd;
							}

							sentCollisionObject.m_collisionNormalStart = (sentCapsule.m_bone.m_start - originalStart).GetNormalized();
							sentCollisionObject.m_collisionNormalEnd = (sentCapsule.m_bone.m_end - originalEnd).GetNormalized();
							sentCollisionObject.m_jacobiCorrectionStart += (sentCapsule.m_bone.m_start - originalStart);
							sentCollisionObject.m_jacobiCorrectionEnd += (sentCapsule.m_bone.m_end - originalEnd);
							sentCollisionObject.m_jacobiConstraintTotalStart++;
							sentCollisionObject.m_jacobiConstraintTotalEnd++;
							sentCollisionObject.m_isCollidingStart = 1;
							sentCollisionObject.m_isCollidingEnd = 1;
						}
					}
				}
				continue;
			}
			CylinderCollisionObject* cylinder = dynamic_cast<CylinderCollisionObject*>(collisionObject);
			if (cylinder) //Cylinder Collisions
			{
				if ((cylinder->m_macroBitRegions & m_collisionCapsules[sentCapsuleIndex].m_macroBitRegions) != 0)
				{
					if ((cylinder->m_microBitRegions & m_collisionCapsules[sentCapsuleIndex].m_microBitRegions) != 0)
					{
						//Bounding Disk Check
						if (!DoSpheresOverlap(capsuleBoundingDiscCenter, capsuleBoundingDiscRadius, cylinder->m_boundingDiscCenter, cylinder->m_boundingDiscRadius))
							continue;

						if (PushCapsuleOutOfFixedCylinder3D(sentCapsule, cylinder->m_cylinder))
						{
							if (m_particles.m_isAttached[sentCapsuleIndex])
							{
								sentCapsule.m_bone.m_start = originalStart;
							}
							if (m_particles.m_isAttached[sentCapsuleIndex + 1])
							{
								sentCapsule.m_bone.m_end = originalEnd;
							}

							sentCollisionObject.m_collisionNormalStart = (sentCapsule.m_bone.m_start - originalStart).GetNormalized();
							sentCollisionObject.m_collisionNormalEnd = (sentCapsule.m_bone.m_end - originalEnd).GetNormalized();
							sentCollisionObject.m_jacobiCorrectionStart += (sentCapsule.m_bone.m_start - originalStart);
							sentCollisionObject.m_jacobiCorrectionEnd += (sentCapsule.m_bone.m_end - originalEnd);
							sentCollisionObject.m_jacobiConstraintTotalStart++;
							sentCollisionObject.m_jacobiConstraintTotalEnd++;
							sentCollisionObject.m_isCollidingStart = 1;
							sentCollisionObject.m_isCollidingEnd = 1;
						}
					}
				}
				continue;
			}
		}
	}

	//World Bounds Collisions
	Vec3 originalStart = sentCapsule.m_bone.m_start;
	Vec3 originalEnd = sentCapsule.m_bone.m_end;
	ProjectWorldBoundsConstraintsCapsulesJacobi(sentCapsuleIndex);
	if (sentCapsule.m_bone.m_start != originalStart)
	{
		sentCollisionObject.m_collisionNormalStart = (sentCapsule.m_bone.m_start - originalStart).GetNormalized();
		sentCollisionObject.m_jacobiCorrectionStart += (sentCapsule.m_bone.m_start - originalStart);
		sentCollisionObject.m_jacobiConstraintTotalStart++;
		sentCollisionObject.m_isCollidingStart = 1;
	}
	if (sentCapsule.m_bone.m_end != originalEnd)
	{
		sentCollisionObject.m_collisionNormalEnd = (sentCapsule.m_bone.m_end - originalEnd).GetNormalized();
		sentCollisionObject.m_jacobiCorrectionEnd += (sentCapsule.m_bone.m_end - originalEnd);
		sentCollisionObject.m_jacobiConstraintTotalEnd++;
		sentCollisionObject.m_isCollidingEnd = 1;
	}
}

//-----------------------------------------------------------------------------------------------
void RopeSimulation3D::ProjectWorldBoundsConstraintsCapsulesJacobi(int sentCapsuleIndex)
{	
	//Initializations
	Capsule3& sentCapsule = m_collisionCapsules[sentCapsuleIndex].m_capsule;

	//START CAPSULE
	//Min Z
	if (sentCapsule.m_bone.m_start.z < m_worldBounds.m_mins.z + m_ropeRadius)
	{
		sentCapsule.m_bone.m_start.z = m_worldBounds.m_mins.z + m_ropeRadius;
	}
	//Max Z
	if (sentCapsule.m_bone.m_start.z > m_worldBounds.m_maxs.z - m_ropeRadius)
	{
		sentCapsule.m_bone.m_start.z = m_worldBounds.m_maxs.z - m_ropeRadius;
	}
	//Min X
	if (sentCapsule.m_bone.m_start.x < m_worldBounds.m_mins.x + m_ropeRadius)
	{
		sentCapsule.m_bone.m_start.x = m_worldBounds.m_mins.x + m_ropeRadius;
	}
	//Max X
	if (sentCapsule.m_bone.m_start.x > m_worldBounds.m_maxs.x - m_ropeRadius)
	{
		sentCapsule.m_bone.m_start.x = m_worldBounds.m_maxs.x - m_ropeRadius;
	}
	//Min Y
	if (sentCapsule.m_bone.m_start.y < m_worldBounds.m_mins.y + m_ropeRadius)
	{
		sentCapsule.m_bone.m_start.y = m_worldBounds.m_mins.y + m_ropeRadius;
	}
	//Max Y
	if (sentCapsule.m_bone.m_start.y > m_worldBounds.m_maxs.y - m_ropeRadius)
	{
		sentCapsule.m_bone.m_start.y = m_worldBounds.m_maxs.y - m_ropeRadius;
	}

	//END CAPSULE
	//Min Z
	if (sentCapsule.m_bone.m_end.z < m_worldBounds.m_mins.z + m_ropeRadius)
	{
		sentCapsule.m_bone.m_end.z = m_worldBounds.m_mins.z + m_ropeRadius;
	}
	//Max Z				   
	if (sentCapsule.m_bone.m_end.z > m_worldBounds.m_maxs.z - m_ropeRadius)
	{
		sentCapsule.m_bone.m_end.z = m_worldBounds.m_maxs.z - m_ropeRadius;
	}
	//Min X				   
	if (sentCapsule.m_bone.m_end.x < m_worldBounds.m_mins.x + m_ropeRadius)
	{
		sentCapsule.m_bone.m_end.x = m_worldBounds.m_mins.x + m_ropeRadius;
	}
	//Max X				   
	if (sentCapsule.m_bone.m_end.x > m_worldBounds.m_maxs.x - m_ropeRadius)
	{
		sentCapsule.m_bone.m_end.x = m_worldBounds.m_maxs.x - m_ropeRadius;
	}
	//Min Y				   
	if (sentCapsule.m_bone.m_end.y < m_worldBounds.m_mins.y + m_ropeRadius)
	{
		sentCapsule.m_bone.m_end.y = m_worldBounds.m_mins.y + m_ropeRadius;
	}
	//Max Y				   
	if (sentCapsule.m_bone.m_end.y > m_worldBounds.m_maxs.y - m_ropeRadius)
	{
		sentCapsule.m_bone.m_end.y = m_worldBounds.m_maxs.y - m_ropeRadius;
	}
}

//-----------------------------------------------------------------------------------------------
void RopeSimulation3D::InitializeBitRegions()
{
	//Bit Region Initializations
	m_macroBitRegionBounds.reserve(64);
	m_microBitRegionBounds.resize(64);

	AABB3 previousBounds;
	float xStep = abs(m_worldBounds.m_maxs.x - m_worldBounds.m_mins.x) / 8.0f;
	float yStep = abs(m_worldBounds.m_maxs.y - m_worldBounds.m_mins.y) / 8.0f;

	for (int bitRegionIndex = 0; bitRegionIndex < 64; bitRegionIndex++)
	{
		AABB3 bounds;

		if (bitRegionIndex == 0)
		{
			bounds.m_mins = Vec3(m_worldBounds.m_mins.x, m_worldBounds.m_mins.y, m_worldBounds.m_mins.z);
			bounds.m_maxs = Vec3(m_worldBounds.m_mins.x + xStep, m_worldBounds.m_mins.y + yStep, m_worldBounds.m_maxs.z);
		}
		else if(bitRegionIndex % 8 == 0)
		{
			bounds.m_mins = Vec3(m_worldBounds.m_mins.x, previousBounds.m_maxs.y, m_worldBounds.m_mins.z);
			bounds.m_maxs = Vec3(m_worldBounds.m_mins.x + xStep, previousBounds.m_maxs.y + yStep, m_worldBounds.m_maxs.z);
		}
		else
		{
			bounds.m_mins = Vec3(previousBounds.m_maxs.x, previousBounds.m_mins.y, m_worldBounds.m_mins.z);
			bounds.m_maxs = Vec3(previousBounds.m_maxs.x + xStep, previousBounds.m_maxs.y, m_worldBounds.m_maxs.z);
		}

		m_macroBitRegionBounds.push_back(bounds);
		previousBounds = bounds;
	}

	for (int macroIndex = 0; macroIndex < m_macroBitRegionBounds.size(); macroIndex++)
	{
		AABB3 macroBounds = m_macroBitRegionBounds[macroIndex];
		previousBounds = AABB3();

		xStep = abs(m_macroBitRegionBounds[macroIndex].m_maxs.x - m_macroBitRegionBounds[macroIndex].m_mins.x) / 8.0f;
		yStep = abs(m_macroBitRegionBounds[macroIndex].m_maxs.y - m_macroBitRegionBounds[macroIndex].m_mins.y) / 8.0f;

		for (int bitRegionIndex = 0; bitRegionIndex < 64; bitRegionIndex++)
		{
			AABB3 bounds;

			if (bitRegionIndex == 0)
			{
				bounds.m_mins = Vec3(macroBounds.m_mins.x, macroBounds.m_mins.y, macroBounds.m_mins.z);
				bounds.m_maxs = Vec3(macroBounds.m_mins.x + xStep, macroBounds.m_mins.y + yStep, macroBounds.m_maxs.z);
			}
			else if (bitRegionIndex % 8 == 0)
			{
				bounds.m_mins = Vec3(macroBounds.m_mins.x, previousBounds.m_maxs.y, macroBounds.m_mins.z);
				bounds.m_maxs = Vec3(macroBounds.m_mins.x + xStep, previousBounds.m_maxs.y + yStep, macroBounds.m_maxs.z);
			}
			else
			{
				bounds.m_mins = Vec3(previousBounds.m_maxs.x, previousBounds.m_mins.y, macroBounds.m_mins.z);
				bounds.m_maxs = Vec3(previousBounds.m_maxs.x + xStep, previousBounds.m_maxs.y, macroBounds.m_maxs.z);
			}

			m_microBitRegionBounds[macroIndex].push_back(bounds);
			previousBounds = bounds;
		}
	}

	//Debug VertexB buffer
	std::vector<Vertex_PCU> verts;
	AddVertsForLineList(verts, m_worldBounds.m_mins, Vec3(m_worldBounds.m_mins.x, m_worldBounds.m_mins.y, m_worldBounds.m_maxs.z), Rgba8::GREEN);
	AddVertsForLineList(verts, m_worldBounds.m_mins, Vec3(m_worldBounds.m_mins.x, m_worldBounds.m_maxs.y, m_worldBounds.m_mins.z), Rgba8::GREEN);
	AddVertsForLineList(verts, m_worldBounds.m_mins, Vec3(m_worldBounds.m_maxs.x, m_worldBounds.m_mins.y, m_worldBounds.m_mins.z), Rgba8::GREEN);
	AddVertsForLineList(verts, m_worldBounds.m_maxs, Vec3(m_worldBounds.m_maxs.x, m_worldBounds.m_maxs.y, m_worldBounds.m_mins.z), Rgba8::GREEN);
	AddVertsForLineList(verts, m_worldBounds.m_maxs, Vec3(m_worldBounds.m_maxs.x, m_worldBounds.m_mins.y, m_worldBounds.m_maxs.z), Rgba8::GREEN);
	AddVertsForLineList(verts, m_worldBounds.m_maxs, Vec3(m_worldBounds.m_mins.x, m_worldBounds.m_maxs.y, m_worldBounds.m_maxs.z), Rgba8::GREEN);
	AddVertsForLineList(verts, Vec3(m_worldBounds.m_maxs.x, m_worldBounds.m_mins.y, m_worldBounds.m_mins.z), Vec3(m_worldBounds.m_maxs.x, m_worldBounds.m_maxs.y, m_worldBounds.m_mins.z), Rgba8::GREEN);
	AddVertsForLineList(verts, Vec3(m_worldBounds.m_mins.x, m_worldBounds.m_maxs.y, m_worldBounds.m_mins.z), Vec3(m_worldBounds.m_maxs.x, m_worldBounds.m_maxs.y, m_worldBounds.m_mins.z), Rgba8::GREEN);
	AddVertsForLineList(verts, Vec3(m_worldBounds.m_mins.x, m_worldBounds.m_maxs.y, m_worldBounds.m_mins.z), Vec3(m_worldBounds.m_mins.x, m_worldBounds.m_maxs.y, m_worldBounds.m_maxs.z), Rgba8::GREEN);
	AddVertsForLineList(verts, Vec3(m_worldBounds.m_maxs.x, m_worldBounds.m_mins.y, m_worldBounds.m_mins.z), Vec3(m_worldBounds.m_maxs.x, m_worldBounds.m_mins.y, m_worldBounds.m_maxs.z), Rgba8::GREEN);
	AddVertsForLineList(verts, Vec3(m_worldBounds.m_mins.x, m_worldBounds.m_mins.y, m_worldBounds.m_maxs.z), Vec3(m_worldBounds.m_maxs.x, m_worldBounds.m_mins.y, m_worldBounds.m_maxs.z), Rgba8::GREEN);
	AddVertsForLineList(verts, Vec3(m_worldBounds.m_mins.x, m_worldBounds.m_mins.y, m_worldBounds.m_maxs.z), Vec3(m_worldBounds.m_mins.x, m_worldBounds.m_maxs.y, m_worldBounds.m_maxs.z), Rgba8::GREEN);

	for (int bitRegionIndex = 0; bitRegionIndex < m_macroBitRegionBounds.size(); bitRegionIndex++)
	{
		for (int microRegionIndex = 0; microRegionIndex < m_microBitRegionBounds[bitRegionIndex].size(); microRegionIndex++)
		{
			AABB3 const& microBounds = m_microBitRegionBounds[bitRegionIndex][microRegionIndex];
			AddVertsForLineList(verts, microBounds.m_mins, Vec3(microBounds.m_mins.x, microBounds.m_mins.y, 0.0f), Rgba8::RED);
			AddVertsForLineList(verts, microBounds.m_mins, Vec3(microBounds.m_mins.x, microBounds.m_maxs.y, 0.0f), Rgba8::RED);
			AddVertsForLineList(verts, microBounds.m_mins, Vec3(microBounds.m_maxs.x, microBounds.m_mins.y, 0.0f), Rgba8::RED);
			AddVertsForLineList(verts, Vec3(microBounds.m_maxs.x, microBounds.m_maxs.y, 0.0f), Vec3(microBounds.m_maxs.x, microBounds.m_maxs.y, 0.0f), Rgba8::RED);
			AddVertsForLineList(verts, Vec3(microBounds.m_maxs.x, microBounds.m_maxs.y, 0.0f), Vec3(microBounds.m_maxs.x, microBounds.m_mins.y, 0.0f), Rgba8::RED);
			AddVertsForLineList(verts, Vec3(microBounds.m_maxs.x, microBounds.m_maxs.y, 0.0f), Vec3(microBounds.m_mins.x, microBounds.m_maxs.y, 0.0f), Rgba8::RED);
			AddVertsForLineList(verts, Vec3(microBounds.m_maxs.x, microBounds.m_mins.y, 0.0f), Vec3(microBounds.m_maxs.x, microBounds.m_maxs.y, 0.0f), Rgba8::RED);
			AddVertsForLineList(verts, Vec3(microBounds.m_mins.x, microBounds.m_maxs.y, 0.0f), Vec3(microBounds.m_maxs.x, microBounds.m_maxs.y, 0.0f), Rgba8::RED);
			AddVertsForLineList(verts, Vec3(microBounds.m_mins.x, microBounds.m_maxs.y, 0.0f), Vec3(microBounds.m_mins.x, microBounds.m_maxs.y, 0.0f), Rgba8::RED);
			AddVertsForLineList(verts, Vec3(microBounds.m_maxs.x, microBounds.m_mins.y, 0.0f), Vec3(microBounds.m_maxs.x, microBounds.m_mins.y, 0.0f), Rgba8::RED);
			AddVertsForLineList(verts, Vec3(microBounds.m_mins.x, microBounds.m_mins.y, 0.0f), Vec3(microBounds.m_maxs.x, microBounds.m_mins.y, 0.0f), Rgba8::RED);
			AddVertsForLineList(verts, Vec3(microBounds.m_mins.x, microBounds.m_mins.y, 0.0f), Vec3(microBounds.m_mins.x, microBounds.m_maxs.y, 0.0f), Rgba8::RED);
		}

		AABB3 const& macroBounds = m_macroBitRegionBounds[bitRegionIndex];
		AddVertsForLineList(verts, macroBounds.m_mins, Vec3(macroBounds.m_mins.x, macroBounds.m_mins.y, 0.0f), Rgba8::BLUE);
		AddVertsForLineList(verts, macroBounds.m_mins, Vec3(macroBounds.m_mins.x, macroBounds.m_maxs.y, 0.0f), Rgba8::BLUE);
		AddVertsForLineList(verts, macroBounds.m_mins, Vec3(macroBounds.m_maxs.x, macroBounds.m_mins.y, 0.0f), Rgba8::BLUE);
		AddVertsForLineList(verts, Vec3(macroBounds.m_maxs.x, macroBounds.m_maxs.y, 0.0f), Vec3(macroBounds.m_maxs.x, macroBounds.m_maxs.y, 0.0f), Rgba8::BLUE);
		AddVertsForLineList(verts, Vec3(macroBounds.m_maxs.x, macroBounds.m_maxs.y, 0.0f), Vec3(macroBounds.m_maxs.x, macroBounds.m_mins.y, 0.0f), Rgba8::BLUE);
		AddVertsForLineList(verts, Vec3(macroBounds.m_maxs.x, macroBounds.m_maxs.y, 0.0f), Vec3(macroBounds.m_mins.x, macroBounds.m_maxs.y, 0.0f), Rgba8::BLUE);
		AddVertsForLineList(verts, Vec3(macroBounds.m_maxs.x, macroBounds.m_mins.y, 0.0f), Vec3(macroBounds.m_maxs.x, macroBounds.m_maxs.y, 0.0f), Rgba8::BLUE);
		AddVertsForLineList(verts, Vec3(macroBounds.m_mins.x, macroBounds.m_maxs.y, 0.0f), Vec3(macroBounds.m_maxs.x, macroBounds.m_maxs.y, 0.0f), Rgba8::BLUE);
		AddVertsForLineList(verts, Vec3(macroBounds.m_mins.x, macroBounds.m_maxs.y, 0.0f), Vec3(macroBounds.m_mins.x, macroBounds.m_maxs.y, 0.0f), Rgba8::BLUE);
		AddVertsForLineList(verts, Vec3(macroBounds.m_maxs.x, macroBounds.m_mins.y, 0.0f), Vec3(macroBounds.m_maxs.x, macroBounds.m_mins.y, 0.0f), Rgba8::BLUE);
		AddVertsForLineList(verts, Vec3(macroBounds.m_mins.x, macroBounds.m_mins.y, 0.0f), Vec3(macroBounds.m_maxs.x, macroBounds.m_mins.y, 0.0f), Rgba8::BLUE);
		AddVertsForLineList(verts, Vec3(macroBounds.m_mins.x, macroBounds.m_mins.y, 0.0f), Vec3(macroBounds.m_mins.x, macroBounds.m_maxs.y, 0.0f), Rgba8::BLUE);
	}
	m_bitRegionDebugVertCount = int(verts.size());
	m_bitRegionVertexBuffer = m_renderer->CreateVertexBuffer(sizeof(Vertex_PCU) * verts.size(), sizeof(Vertex_PCU));
	m_renderer->CopyCPUToGPU(verts.data(), sizeof(Vertex_PCU)* verts.size(), m_bitRegionVertexBuffer);
}

//-----------------------------------------------------------------------------------------------
void RopeSimulation3D::BitRegionDetectionAllParticles()
{
	//Loop through each Particle and set bit regions on macro and micro scale
	for (int particleIndex = 0; particleIndex < m_particles.m_positions.size(); particleIndex++)
	{
		BitRegionDetectionSingleParticle(particleIndex);
	}
}

//-----------------------------------------------------------------------------------------------
void RopeSimulation3D::BitRegionDetectionSingleParticle(int const& sentParticleIndex)
{
	//Initializations
	m_particles.m_macroBitRegions[sentParticleIndex] = 0;
	m_particles.m_microBitRegions[sentParticleIndex] = 0;
	Vec3 center = m_particles.m_proposedPositions[sentParticleIndex];
	
	//Macro
	float minX = (center.x - m_ropeRadius) * m_macroScaleX;
	float minY = (center.y - m_ropeRadius) * m_macroScaleY;
	float maxX = (center.x + m_ropeRadius) * m_macroScaleX;
	float maxY = (center.y + m_ropeRadius) * m_macroScaleY;

	IntVec3 minXMinY = IntVec3(int(minX), int(minY), 0);
	IntVec3 minXMaxY = IntVec3(int(minX), int(maxY), 0);
	IntVec3 maxXMinY = IntVec3(int(maxX), int(minY), 0);
	IntVec3 maxXMaxY = IntVec3(int(maxX), int(maxY), 0);
	int region1 = GetRegionIndexForCoords(minXMinY);
	int region2 = GetRegionIndexForCoords(minXMaxY);
	int region3 = GetRegionIndexForCoords(maxXMinY);
	int region4 = GetRegionIndexForCoords(maxXMaxY);
	region4;
	uint64_t mask = 0;
	int regionDifference12 = region2 - region1;
	int currentRegion = region1;
	int regionDifferenceX = region3 - region1;
	int regionDifferenceY = regionDifference12 >> 3;
	AssignBitRegionsParticle(sentParticleIndex, currentRegion, regionDifferenceX, regionDifferenceY, false);

	//Micro
	IntVec3 minXMinYMicro = IntVec3(int((minX - minXMinY.x) * m_microScaleX), int((minY - minXMinY.y) * m_microScaleY), 0);
	IntVec3 minXMaxYMicro = IntVec3(int((minX - minXMaxY.x) * m_microScaleX), int((maxY - minXMaxY.y) * m_microScaleY), 0);
	IntVec3 maxXMinYMicro = IntVec3(int((maxX - maxXMinY.x) * m_microScaleX), int((minY - maxXMinY.y) * m_microScaleY), 0);
	IntVec3 maxXMaxYMicro = IntVec3(int((maxX - maxXMaxY.x) * m_microScaleX), int((maxY - maxXMaxY.y) * m_microScaleY), 0);
	int region1Micro = GetRegionIndexForCoords(minXMinYMicro);
	int region2Micro = GetRegionIndexForCoords(minXMaxYMicro);
	int region3Micro = GetRegionIndexForCoords(maxXMinYMicro);
	int region4Micro = GetRegionIndexForCoords(maxXMaxYMicro);

	//All regions are the same CORRECT
	if (region1 == region2 && region1 == region3)
	{
		mask = 0;
		currentRegion = region1Micro;
		regionDifference12 = region2Micro - region1Micro;
		regionDifferenceX = region3Micro - region1Micro;
		regionDifferenceY = regionDifference12 >> 3;
		AssignBitRegionsParticle(sentParticleIndex, currentRegion, regionDifferenceX, regionDifferenceY, true);
	}
	//Y regions are different
	else if (region1 != region2 && region1 == region3)
	{
		//Spans across multiple Y macro regions CORRECT
		if (region2 - region1 > 8)
		{
			regionDifferenceY = 7;
			regionDifferenceX = region3Micro - region1Micro;
			currentRegion = region1Micro - (region1Micro >> 3) * 8;
			AssignBitRegionsParticle(sentParticleIndex, currentRegion, regionDifferenceX, regionDifferenceY, true);
		}
		//Span across 1 Y macro region
		else
		{
			//region 1/3 to top CORRECT
			regionDifferenceX = region3Micro - region1Micro;
			regionDifferenceY = 7 - (region1Micro >> 3);
			currentRegion = region1Micro;
			AssignBitRegionsParticle(sentParticleIndex, currentRegion, regionDifferenceX, regionDifferenceY, true);
			//bottom to region 2/4 CORRECT
			regionDifferenceX = region4Micro - region2Micro;
			regionDifferenceY = (region2Micro >> 3);
			currentRegion = region2Micro - (region2Micro >> 3) * 8;
			AssignBitRegionsParticle(sentParticleIndex, currentRegion, regionDifferenceX, regionDifferenceY, true);
		}
	}
	//X regions are different
	else if (region1 == region2 && region1 != region3)
	{
		//Spans across multiple X macro regions CORRECT
		if (region3 - region1 > 1)
		{
			regionDifferenceX = 7;
			regionDifferenceY = region2Micro - region1Micro;
			currentRegion = (region1Micro >> 3) * 8;
			AssignBitRegionsParticle(sentParticleIndex, currentRegion, regionDifferenceX, regionDifferenceY, true);
		}
		//Span across 1 X macro region
		else
		{
			//region 1/2 to right CORRECT
			regionDifferenceX = ((region1Micro >> 3) * 8 + 7) - region1Micro;
			regionDifferenceY = region2Micro - region1Micro;
			currentRegion = region1Micro;
			AssignBitRegionsParticle(sentParticleIndex, currentRegion, regionDifferenceX, regionDifferenceY, true);
			//left to region 3/4 CORRECT
			currentRegion = (region3Micro >> 3) * 8;
			regionDifferenceX = region3Micro - currentRegion;
			regionDifferenceY = region2Micro - region1Micro;
			AssignBitRegionsParticle(sentParticleIndex, currentRegion, regionDifferenceX, regionDifferenceY, true);
		}
	}
	//All macro regions are different
	else if (region1 != region2 && region1 != region3)
	{
		//Spans across multiple X and Y regions CORRECT
		if (region2 - region1 > 8 && region3 - region1 > 1)
		{
			mask = 0;
			mask = ~mask;
			m_particles.m_microBitRegions[sentParticleIndex] = mask;
		}
		//Spans across 1 Y and multiple X regions
		else if (region2 - region1 == 8 && region3 - region1 > 1)
		{
			//region 1/3 to top CORRECT
			regionDifferenceX = 7;
			regionDifferenceY = 7 - (region1Micro >> 3);
			currentRegion = (region1Micro >> 3) * 8;
			AssignBitRegionsParticle(sentParticleIndex, currentRegion, regionDifferenceX, regionDifferenceY, true);
			//bottom to region 2/4 CORRECT
			regionDifferenceX = 7;
			regionDifferenceY = (region2Micro >> 3);
			currentRegion = 0;
			AssignBitRegionsParticle(sentParticleIndex, currentRegion, regionDifferenceX, regionDifferenceY, true);
		}
		//Spans across 1 X and multiple Y regions
		else if (region2 - region1 > 8 && region3 - region1 == 1)
		{
			//region 1/2 to right CORRECT		
			regionDifferenceX = ((region1Micro >> 3) * 8 + 7) - region1Micro;
			regionDifferenceY = 7;
			currentRegion = region1Micro - (region1Micro >> 3) * 8;
			AssignBitRegionsParticle(sentParticleIndex, currentRegion, regionDifferenceX, regionDifferenceY, true);
			//left to region 3/4 					
			currentRegion = 0;
			regionDifferenceX = region3Micro - (region3Micro >> 3) * 8;
			regionDifferenceY = 7;
			AssignBitRegionsParticle(sentParticleIndex, currentRegion, regionDifferenceX, regionDifferenceY, true);
		}
		//Spans across 1 X and 1 Y region
		else if (region2 - region1 == 8 && region3 - region1 == 1)
		{
			//region1 to right/top
			regionDifferenceX = ((region1Micro >> 3) * 8 + 7) - region1Micro;
			regionDifferenceY = 7 - (region1Micro >> 3);
			currentRegion = region1Micro;
			AssignBitRegionsParticle(sentParticleIndex, currentRegion, regionDifferenceX, regionDifferenceY, true);
			//left/top to region 3
			regionDifferenceX = region3Micro - (region3Micro >> 3) * 8;
			regionDifferenceY = 7 - (region3Micro >> 3);
			currentRegion = (region3Micro >> 3) * 8;
			AssignBitRegionsParticle(sentParticleIndex, currentRegion, regionDifferenceX, regionDifferenceY, true);
			//bottom/right to region 2
			regionDifferenceX = ((region2Micro >> 3) * 8 + 7) - region2Micro;
			regionDifferenceY = (region2Micro >> 3);
			currentRegion = region2Micro - (region2Micro >> 3) * 8;
			AssignBitRegionsParticle(sentParticleIndex, currentRegion, regionDifferenceX, regionDifferenceY, true);
			//bottom/left to region 4
			regionDifferenceX = region4Micro - (region4Micro >> 3) * 8;
			regionDifferenceY = (region4Micro >> 3);
			currentRegion = 0;
			AssignBitRegionsParticle(sentParticleIndex, currentRegion, regionDifferenceX, regionDifferenceY, true);
		}
	}
}

//-----------------------------------------------------------------------------------------------
void RopeSimulation3D::BitRegionDetectionAllCapsules()
{
	for (int capsuleIndex = 0; capsuleIndex < m_collisionCapsules.size(); capsuleIndex++)
	{
		Capsule3& capsule = m_collisionCapsules[capsuleIndex].m_capsule;
		capsule.m_bone.m_start = m_particles.m_proposedPositions[capsuleIndex];
		capsule.m_bone.m_end = m_particles.m_proposedPositions[capsuleIndex + 1];
		BitRegionDetectionSingleCapsule(capsuleIndex);
	}
}

//-----------------------------------------------------------------------------------------------
void RopeSimulation3D::BitRegionDetectionSingleCapsule(int const& sentCapsuleIndex)
{
	//Initializations
	CapsuleCollisionObject& sentCapsule = m_collisionCapsules[sentCapsuleIndex];
	sentCapsule.m_macroBitRegions = 0;
	sentCapsule.m_microBitRegions = 0;

	//Macro
	float minX = FLT_MAX; float minY = FLT_MAX; float maxX = 0.0f; float maxY = 0.0f;
	float radius = sentCapsule.m_capsule.m_radius;
	Vec3 start = sentCapsule.m_capsule.m_bone.m_start;
	Vec3 end = sentCapsule.m_capsule.m_bone.m_end;
	float minX1 = (start.x - radius) * m_macroScaleX;
	float minY1 = (start.y - radius) * m_macroScaleY;
	float maxX1 = (start.x + radius) * m_macroScaleX;
	float maxY1 = (start.y + radius) * m_macroScaleY;
	float minX2 = (end.x - radius) * m_macroScaleX;
	float minY2 = (end.y - radius) * m_macroScaleY;
	float maxX2 = (end.x + radius) * m_macroScaleX;
	float maxY2 = (end.y + radius) * m_macroScaleY;

	if (minX1 < minX2)
		minX = minX1;
	else
		minX = minX2;
	if (minY1 < minY2)
		minY = minY1;
	else
		minY = minY2;
	if (maxX1 > maxX2)
		maxX = maxX1;
	else
		maxX = maxX2;
	if (maxY1 > maxY2)
		maxY = maxY1;
	else
		maxY = maxY2;

	//Macro CORRECT
	IntVec3 minXMinY = IntVec3(int(minX), int(minY), 0);
	IntVec3 minXMaxY = IntVec3(int(minX), int(maxY), 0);
	IntVec3 maxXMinY = IntVec3(int(maxX), int(minY), 0);
	IntVec3 maxXMaxY = IntVec3(int(maxX), int(maxY), 0);
	int region1 = GetRegionIndexForCoords(minXMinY);
	int region2 = GetRegionIndexForCoords(minXMaxY);
	int region3 = GetRegionIndexForCoords(maxXMinY);
	int region4 = GetRegionIndexForCoords(maxXMaxY);
	region4;
	uint64_t mask = 0;
	int regionDifference12 = region2 - region1;
	int currentRegion = region1;
	int regionDifferenceX = region3 - region1;
	int regionDifferenceY = regionDifference12 >> 3;
	AssignBitRegionsCapsule(sentCapsuleIndex, currentRegion, regionDifferenceX, regionDifferenceY, false);

	//Micro
	IntVec3 minXMinYMicro = IntVec3(int((minX - minXMinY.x) * m_microScaleX), int((minY - minXMinY.y) * m_microScaleY), 0);
	IntVec3 minXMaxYMicro = IntVec3(int((minX - minXMaxY.x) * m_microScaleX), int((maxY - minXMaxY.y) * m_microScaleY), 0);
	IntVec3 maxXMinYMicro = IntVec3(int((maxX - maxXMinY.x) * m_microScaleX), int((minY - maxXMinY.y) * m_microScaleY), 0);
	IntVec3 maxXMaxYMicro = IntVec3(int((maxX - maxXMaxY.x) * m_microScaleX), int((maxY - maxXMaxY.y) * m_microScaleY), 0);
	int region1Micro = GetRegionIndexForCoords(minXMinYMicro);
	int region2Micro = GetRegionIndexForCoords(minXMaxYMicro);
	int region3Micro = GetRegionIndexForCoords(maxXMinYMicro);
	int region4Micro = GetRegionIndexForCoords(maxXMaxYMicro);

	//All regions are the same //CORRECT
	if (region1 == region2 && region1 == region3)
	{
		mask = 0;
		currentRegion = region1Micro;
		regionDifference12 = region2Micro - region1Micro;
		regionDifferenceX = region3Micro - region1Micro;
		regionDifferenceY = regionDifference12 >> 3;
		AssignBitRegionsCapsule(sentCapsuleIndex, currentRegion, regionDifferenceX, regionDifferenceY, true);
	}
	//Y regions are different //CORRECT
	else if (region1 != region2 && region1 == region3)
	{
		//Spans across multiple Y macro regions  //CORRECT
		if (region2 - region1 > 8)
		{
			regionDifferenceY = 7;
			regionDifferenceX = region3Micro - region1Micro;
			currentRegion = region1Micro - (region1Micro >> 3) * 8;
			AssignBitRegionsCapsule(sentCapsuleIndex, currentRegion, regionDifferenceX, regionDifferenceY, true);
		}
		//Span across 1 Y macro region //CORRECT
		else
		{
			//region 1/3 to top CORRECT
			regionDifferenceX = region3Micro - region1Micro;
			regionDifferenceY = 7 - (region1Micro >> 3);
			currentRegion = region1Micro;
			AssignBitRegionsCapsule(sentCapsuleIndex, currentRegion, regionDifferenceX, regionDifferenceY, true);
			//bottom to region 2/4 CORRECT
			regionDifferenceX = region4Micro - region2Micro;
			regionDifferenceY = (region2Micro >> 3);
			currentRegion = region2Micro - (region2Micro >> 3) * 8;
			AssignBitRegionsCapsule(sentCapsuleIndex, currentRegion, regionDifferenceX, regionDifferenceY, true);
		}
	}
	//X regions are different
	else if (region1 == region2 && region1 != region3)
	{
		//Spans across multiple X macro regions //CORRECT
		if (region3 - region1 > 1)
		{
			regionDifferenceX = 7;
			regionDifferenceY = region2Micro - region1Micro;
			currentRegion = (region1Micro >> 3) * 8;
			AssignBitRegionsCapsule(sentCapsuleIndex, currentRegion, regionDifferenceX, regionDifferenceY, true);
		}
		//Span across 1 X macro region
		else
		{
			//region 1/2 to right //CORRECT
			regionDifferenceX = ((region1Micro >> 3) * 8 + 7) - region1Micro;
			regionDifferenceY = region2Micro - region1Micro;
			currentRegion = region1Micro;
			AssignBitRegionsCapsule(sentCapsuleIndex, currentRegion, regionDifferenceX, regionDifferenceY, true);
			//left to region 3/4 //CORRECT
			currentRegion = (region3Micro >> 3) * 8;
			regionDifferenceX = region3Micro - currentRegion;
			regionDifferenceY = region2Micro - region1Micro;
			AssignBitRegionsCapsule(sentCapsuleIndex, currentRegion, regionDifferenceX, regionDifferenceY, true);
		}
	}
	//All macro regions are different
	else if (region1 != region2 && region1 != region3)
	{
		//Spans across multiple X and Y regions //CORRECT
		if (region2 - region1 > 8 && region3 - region1 > 1)
		{
			mask = 0;
			mask = ~mask;
			sentCapsule.m_microBitRegions = mask;
		}
		//Spans across 1 Y and multiple X regions //CORRECT
		else if (region2 - region1 == 8 && region3 - region1 > 1)
		{
			//region 1/3 to top CORRECT
			regionDifferenceX = 7;
			regionDifferenceY = 7 - (region1Micro >> 3);
			currentRegion = (region1Micro >> 3) * 8;
			AssignBitRegionsCapsule(sentCapsuleIndex, currentRegion, regionDifferenceX, regionDifferenceY, true);
			//bottom to region 2/4 CORRECT
			regionDifferenceX = 7;
			regionDifferenceY = (region2Micro >> 3);
			currentRegion = 0;
			AssignBitRegionsCapsule(sentCapsuleIndex, currentRegion, regionDifferenceX, regionDifferenceY, true);
		}
		//Spans across 1 X and multiple Y regions //CORRECT
		else if (region2 - region1 > 8 && region3 - region1 == 1)
		{
			//region 1/2 to right		
			regionDifferenceX = ((region1Micro >> 3) * 8 + 7) - region1Micro;
			regionDifferenceY = 7;
			currentRegion = region1Micro - (region1Micro >> 3) * 8;
			AssignBitRegionsCapsule(sentCapsuleIndex, currentRegion, regionDifferenceX, regionDifferenceY, true);
			//left to region 3/4 					
			currentRegion = 0;
			regionDifferenceX = region3Micro - (region3Micro >> 3) * 8;
			regionDifferenceY = 7;
			AssignBitRegionsCapsule(sentCapsuleIndex, currentRegion, regionDifferenceX, regionDifferenceY, true);
		}
		//Spans across 1 X and 1 Y region //CORRECT
		else if (region2 - region1 == 8 && region3 - region1 == 1)
		{
			//region1 to right/top
			regionDifferenceX = ((region1Micro >> 3) * 8 + 7) - region1Micro;
			regionDifferenceY = 7 - (region1Micro >> 3);
			currentRegion = region1Micro;
			AssignBitRegionsCapsule(sentCapsuleIndex, currentRegion, regionDifferenceX, regionDifferenceY, true);
			//left/top to region 3
			regionDifferenceX = region3Micro - (region3Micro >> 3) * 8;
			regionDifferenceY = 7 - (region3Micro >> 3);
			currentRegion = (region3Micro >> 3) * 8;
			AssignBitRegionsCapsule(sentCapsuleIndex, currentRegion, regionDifferenceX, regionDifferenceY, true);
			//bottom/right to region 2
			regionDifferenceX = ((region2Micro >> 3) * 8 + 7) - region2Micro;
			regionDifferenceY = (region2Micro >> 3);
			currentRegion = region2Micro - (region2Micro >> 3) * 8;
			AssignBitRegionsCapsule(sentCapsuleIndex, currentRegion, regionDifferenceX, regionDifferenceY, true);
			//bottom/left to region 4
			regionDifferenceX = region4Micro - (region4Micro >> 3) * 8;
			regionDifferenceY = (region4Micro >> 3);
			currentRegion = 0;
			AssignBitRegionsCapsule(sentCapsuleIndex, currentRegion, regionDifferenceX, regionDifferenceY, true);
		}
	}
}

//-----------------------------------------------------------------------------------------------
void RopeSimulation3D::BitRegionDetectionAllCollisionObjects()
{
	for (int collisionObjectIndex = 0; collisionObjectIndex < m_collisionObjects.size(); collisionObjectIndex++)
	{
		//Initializations
		CollisionObject*& collisionObject = m_collisionObjects[collisionObjectIndex];
		float minX = 0.0f;
		float minY = 0.0f;
		float maxX = 0.0f;
		float maxY = 0.0f;

		if (collisionObject)
		{
			SphereCollisionObject* sphere = dynamic_cast<SphereCollisionObject*>(collisionObject);
			if (sphere) //Sphere Collisions
			{
				minX = (sphere->m_sphere.m_center.x - sphere->m_sphere.m_radius) * m_macroScaleX;
				minY = (sphere->m_sphere.m_center.y - sphere->m_sphere.m_radius) * m_macroScaleY;
				maxX = (sphere->m_sphere.m_center.x + sphere->m_sphere.m_radius) * m_macroScaleX;
				maxY = (sphere->m_sphere.m_center.y + sphere->m_sphere.m_radius) * m_macroScaleY;
			}
			AABBCollisionObject* aabb = dynamic_cast<AABBCollisionObject*>(collisionObject);
			if (aabb) //AABB Collisions
			{
				if (aabb->m_aabb.m_mins.x < aabb->m_aabb.m_maxs.x)
					minX = aabb->m_aabb.m_mins.x * m_macroScaleX;
				else
					minX = aabb->m_aabb.m_maxs.x * m_macroScaleX;
				if (aabb->m_aabb.m_mins.y < aabb->m_aabb.m_maxs.y)
					minY = aabb->m_aabb.m_mins.y * m_macroScaleY;
				else
					minY = aabb->m_aabb.m_maxs.y * m_macroScaleY;
				if (aabb->m_aabb.m_mins.x > aabb->m_aabb.m_maxs.x)
					maxX = aabb->m_aabb.m_mins.x * m_macroScaleX;
				else
					maxX = aabb->m_aabb.m_maxs.x * m_macroScaleX;
				if (aabb->m_aabb.m_mins.y > aabb->m_aabb.m_maxs.y)
					maxY = aabb->m_aabb.m_mins.y * m_macroScaleY;
				else
					maxY = aabb->m_aabb.m_maxs.y * m_macroScaleY;
			}
			OBBCollisionObject* obb = dynamic_cast<OBBCollisionObject*>(collisionObject);
			if (obb) //OBB Collisions
			{
				Vec3 center = obb->m_obb.m_center;
				Vec3 ibasis = obb->m_obb.m_iBasisNormal;
				Vec3 jbasis = obb->m_obb.m_jBasisNormal;
				Vec3 kbasis = obb->m_obb.m_kBasisNormal;
				float x = obb->m_obb.m_halfDimensions.x;
				float y = obb->m_obb.m_halfDimensions.y;
				float z = obb->m_obb.m_halfDimensions.z;
				std::vector<Vec3> corners;
				corners.push_back((ibasis * x + jbasis * y + kbasis * z) + center);
				corners.push_back((ibasis * x + jbasis * -y + kbasis * z) + center);
				corners.push_back((ibasis * x + jbasis * y + kbasis * -z) + center);
				corners.push_back((ibasis * x + jbasis * -y + kbasis * -z) + center);
				corners.push_back((ibasis * -x + jbasis * y + kbasis * z) + center);
				corners.push_back((ibasis * -x + jbasis * -y + kbasis * z) + center);
				corners.push_back((ibasis * -x + jbasis * y + kbasis * -z) + center);
				corners.push_back((ibasis * -x + jbasis * -y + kbasis * -z) + center);

				minX = FLT_MAX; minY = FLT_MAX; maxX = 0.0f; maxY = 0.0f;
				for (int index = 0; index < corners.size(); index++)
				{
					if (corners[index].x < minX)
						minX = corners[index].x;
					if (corners[index].x > maxX)
						maxX = corners[index].x;
					if (corners[index].y < minY)
						minY = corners[index].y;
					if (corners[index].y > maxY)
						maxY = corners[index].y;
				}
				minX *= m_macroScaleX;
				maxX *= m_macroScaleX;
				minY *= m_macroScaleY;
				maxY *= m_macroScaleY;
			}
			CapsuleCollisionObject* capsule = dynamic_cast<CapsuleCollisionObject*>(collisionObject);
			if (capsule) //Capsule Collisions
			{
				float radius = capsule->m_capsule.m_radius;
				Vec3 start = capsule->m_capsule.m_bone.m_start;
				Vec3 end = capsule->m_capsule.m_bone.m_end;
				float minX1 = (start.x - radius) * m_macroScaleX;
				float minY1 = (start.y - radius) * m_macroScaleY;
				float maxX1 = (start.x + radius) * m_macroScaleX;
				float maxY1 = (start.y + radius) * m_macroScaleY;
				float minX2 = (end.x - radius) * m_macroScaleX;
				float minY2 = (end.y - radius) * m_macroScaleY;
				float maxX2 = (end.x + radius) * m_macroScaleX;
				float maxY2 = (end.y + radius) * m_macroScaleY;

				if (minX1 < minX2)
					minX = minX1;
				else
					minX = minX2;
				if (minY1 < minY2)
					minY = minY1;
				else
					minY = minY2;
				if (maxX1 > maxX2)
					maxX = maxX1;
				else
					maxX = maxX2;
				if (maxY1 > maxY2)
					maxY = maxY1;
				else
					maxY = maxY2;
			}
			CylinderCollisionObject* cylinder = dynamic_cast<CylinderCollisionObject*>(collisionObject);
			if (cylinder) //Cylinder Collisions
			{
				float radius = cylinder->m_cylinder.m_radius;
				Vec3 start = cylinder->m_cylinder.m_start;
				Vec3 end = cylinder->m_cylinder.m_end;
				float minX1 = (start.x - radius) * m_macroScaleX;
				float minY1 = (start.y - radius) * m_macroScaleY;
				float maxX1 = (start.x + radius) * m_macroScaleX;
				float maxY1 = (start.y + radius) * m_macroScaleY;
				float minX2 = (end.x - radius) * m_macroScaleX;
				float minY2 = (end.y - radius) * m_macroScaleY;
				float maxX2 = (end.x + radius) * m_macroScaleX;
				float maxY2 = (end.y + radius) * m_macroScaleY;

				if (minX1 < minX2)
					minX = minX1;
				else
					minX = minX2;
				if (minY1 < minY2)
					minY = minY1;
				else
					minY = minY2;
				if (maxX1 > maxX2)
					maxX = maxX1;
				else
					maxX = maxX2;
				if (maxY1 > maxY2)
					maxY = maxY1;
				else
					maxY = maxY2;
			}

			//Initializations
			minX = GetClamped(minX, 0.0f, 7.0f);
			maxX = GetClamped(maxX, 0.0f, 7.0f);
			minY = GetClamped(minY, 0.0f, 8.0f);
			maxY = GetClamped(maxY, 0.0f, 8.0f);
			if (collisionObject)
			{
				collisionObject->m_macroBitRegions = 0;
				collisionObject->m_microBitRegions = 0;
			}

			//Macro CORRECT
			IntVec3 minXMinY = IntVec3(int(minX), int(minY), 0);
			IntVec3 minXMaxY = IntVec3(int(minX), int(maxY), 0);
			IntVec3 maxXMinY = IntVec3(int(maxX), int(minY), 0);
			IntVec3 maxXMaxY = IntVec3(int(maxX), int(maxY), 0);
			int region1 = GetRegionIndexForCoords(minXMinY);
			int region2 = GetRegionIndexForCoords(minXMaxY);
			int region3 = GetRegionIndexForCoords(maxXMinY);
			int region4 = GetRegionIndexForCoords(maxXMaxY);
			region4;
			uint64_t mask = 0;
			int regionDifference12 = region2 - region1;
			int currentRegion = region1;
			int regionDifferenceX = region3 - region1;
			int regionDifferenceY = regionDifference12 >> 3;
			AssignBitRegionsCollisionObject(collisionObjectIndex, currentRegion, regionDifferenceX, regionDifferenceY, false);

			//Micro
			IntVec3 minXMinYMicro = IntVec3(int((minX - minXMinY.x) * m_microScaleX), int((minY - minXMinY.y) * m_microScaleY), 0);
			IntVec3 minXMaxYMicro = IntVec3(int((minX - minXMaxY.x) * m_microScaleX), int((maxY - minXMaxY.y) * m_microScaleY), 0);
			IntVec3 maxXMinYMicro = IntVec3(int((maxX - maxXMinY.x) * m_microScaleX), int((minY - maxXMinY.y) * m_microScaleY), 0);
			IntVec3 maxXMaxYMicro = IntVec3(int((maxX - maxXMaxY.x) * m_microScaleX), int((maxY - maxXMaxY.y) * m_microScaleY), 0);
			int region1Micro = GetRegionIndexForCoords(minXMinYMicro);
			int region2Micro = GetRegionIndexForCoords(minXMaxYMicro);
			int region3Micro = GetRegionIndexForCoords(maxXMinYMicro);
			int region4Micro = GetRegionIndexForCoords(maxXMaxYMicro);

			//All regions are the same //CORRECT
			if (region1 == region2 && region1 == region3)
			{
				mask = 0;
				currentRegion = region1Micro;
				regionDifference12 = region2Micro - region1Micro;
				regionDifferenceX = region3Micro - region1Micro;
				regionDifferenceY = regionDifference12 >> 3;
				AssignBitRegionsCollisionObject(collisionObjectIndex, currentRegion, regionDifferenceX, regionDifferenceY, true);
			}
			//Y regions are different //CORRECT
			else if (region1 != region2 && region1 == region3)
			{
				//Spans across multiple Y macro regions  //CORRECT
				if (region2 - region1 > 8)
				{
					regionDifferenceY = 7;
					regionDifferenceX = region3Micro - region1Micro;
					currentRegion = region1Micro - (region1Micro >> 3) * 8;
					AssignBitRegionsCollisionObject(collisionObjectIndex, currentRegion, regionDifferenceX, regionDifferenceY, true);
				}
				//Span across 1 Y macro region //CORRECT
				else
				{
					//region 1/3 to top CORRECT
					regionDifferenceX = region3Micro - region1Micro;
					regionDifferenceY = 7 - (region1Micro >> 3);
					currentRegion = region1Micro;
					AssignBitRegionsCollisionObject(collisionObjectIndex, currentRegion, regionDifferenceX, regionDifferenceY, true);
					//bottom to region 2/4 CORRECT
					regionDifferenceX = region4Micro - region2Micro;
					regionDifferenceY = (region2Micro >> 3);
					currentRegion = region2Micro - (region2Micro >> 3) * 8;
					AssignBitRegionsCollisionObject(collisionObjectIndex, currentRegion, regionDifferenceX, regionDifferenceY, true);
				}
			}
			//X regions are different
			else if (region1 == region2 && region1 != region3)
			{
				//Spans across multiple X macro regions //CORRECT
				if (region3 - region1 > 1)
				{
					regionDifferenceX = 7;
					regionDifferenceY = region2Micro - region1Micro;
					currentRegion = (region1Micro >> 3) * 8;
					AssignBitRegionsCollisionObject(collisionObjectIndex, currentRegion, regionDifferenceX, regionDifferenceY, true);
				}
				//Span across 1 X macro region
				else
				{
					//region 1/2 to right //CORRECT
					regionDifferenceX = ((region1Micro >> 3) * 8 + 7) - region1Micro;
					regionDifferenceY = region2Micro - region1Micro;
					currentRegion = region1Micro;
					AssignBitRegionsCollisionObject(collisionObjectIndex, currentRegion, regionDifferenceX, regionDifferenceY, true);
					//left to region 3/4 //CORRECT
					currentRegion = (region3Micro >> 3) * 8;
					regionDifferenceX = region3Micro - currentRegion;
					regionDifferenceY = region2Micro - region1Micro;
					AssignBitRegionsCollisionObject(collisionObjectIndex, currentRegion, regionDifferenceX, regionDifferenceY, true);
				}
			}
			//All macro regions are different
			else if (region1 != region2 && region1 != region3)
			{
				//Spans across multiple X and Y regions //CORRECT
				if (region2 - region1 > 8 && region3 - region1 > 1)
				{
					mask = 0;
					mask = ~mask;
					collisionObject->m_microBitRegions = mask;
				}
				//Spans across 1 Y and multiple X regions //CORRECT
				else if (region2 - region1 == 8 && region3 - region1 > 1)
				{
					//region 1/3 to top CORRECT
					regionDifferenceX = 7;
					regionDifferenceY = 7 - (region1Micro >> 3);
					currentRegion = (region1Micro >> 3) * 8;
					AssignBitRegionsCollisionObject(collisionObjectIndex, currentRegion, regionDifferenceX, regionDifferenceY, true);
					//bottom to region 2/4 CORRECT
					regionDifferenceX = 7;
					regionDifferenceY = (region2Micro >> 3);
					currentRegion = 0;
					AssignBitRegionsCollisionObject(collisionObjectIndex, currentRegion, regionDifferenceX, regionDifferenceY, true);
				}
				//Spans across 1 X and multiple Y regions //CORRECT
				else if (region2 - region1 > 8 && region3 - region1 == 1)
				{
					//region 1/2 to right		
					regionDifferenceX = ((region1Micro >> 3) * 8 + 7) - region1Micro;
					regionDifferenceY = 7;
					currentRegion = region1Micro - (region1Micro >> 3) * 8;
					AssignBitRegionsCollisionObject(collisionObjectIndex, currentRegion, regionDifferenceX, regionDifferenceY, true);
					//left to region 3/4 					
					currentRegion = 0;
					regionDifferenceX = region3Micro - (region3Micro >> 3) * 8;
					regionDifferenceY = 7;
					AssignBitRegionsCollisionObject(collisionObjectIndex, currentRegion, regionDifferenceX, regionDifferenceY, true);
				}
				//Spans across 1 X and 1 Y region //CORRECT
				else if (region2 - region1 == 8 && region3 - region1 == 1)
				{
					//region1 to right/top
					regionDifferenceX = ((region1Micro >> 3) * 8 + 7) - region1Micro;
					regionDifferenceY = 7 - (region1Micro >> 3);
					currentRegion = region1Micro;
					AssignBitRegionsCollisionObject(collisionObjectIndex, currentRegion, regionDifferenceX, regionDifferenceY, true);
					//left/top to region 3
					regionDifferenceX = region3Micro - (region3Micro >> 3) * 8;
					regionDifferenceY = 7 - (region3Micro >> 3);
					currentRegion = (region3Micro >> 3) * 8;
					AssignBitRegionsCollisionObject(collisionObjectIndex, currentRegion, regionDifferenceX, regionDifferenceY, true);
					//bottom/right to region 2
					regionDifferenceX = ((region2Micro >> 3) * 8 + 7) - region2Micro;
					regionDifferenceY = (region2Micro >> 3);
					currentRegion = region2Micro - (region2Micro >> 3) * 8;
					AssignBitRegionsCollisionObject(collisionObjectIndex, currentRegion, regionDifferenceX, regionDifferenceY, true);
					//bottom/left to region 4
					regionDifferenceX = region4Micro - (region4Micro >> 3) * 8;
					regionDifferenceY = (region4Micro >> 3);
					currentRegion = 0;
					AssignBitRegionsCollisionObject(collisionObjectIndex, currentRegion, regionDifferenceX, regionDifferenceY, true);
				}
			}
		}
	}
}

//-----------------------------------------------------------------------------------------------
void RopeSimulation3D::AssignBitRegionsParticle(int sentParticleIndex, int currentRegion, int regionDifferenceX, int regionDifferenceY, bool isMicro)
{
	uint64_t mask = 0;
	for (int regionIndexY = 0; regionIndexY <= regionDifferenceY; regionIndexY++)
	{
		for (int regionIndexX = 0; regionIndexX <= regionDifferenceX; regionIndexX++)
		{
			mask = 1ULL << (currentRegion + regionIndexX);
			if(isMicro == false)
				m_particles.m_macroBitRegions[sentParticleIndex] |= mask;
			else
				m_particles.m_microBitRegions[sentParticleIndex] |= mask;
		}
		currentRegion += 8;
	}
}

//-----------------------------------------------------------------------------------------------
void RopeSimulation3D::AssignBitRegionsCapsule(int sentCapsuleIndex, int currentRegion, int regionDifferenceX, int regionDifferenceY, bool isMicro)
{
	CapsuleCollisionObject& sentCapsule = m_collisionCapsules[sentCapsuleIndex];
	uint64_t mask = 0;
	for (int regionIndexY = 0; regionIndexY <= regionDifferenceY; regionIndexY++)
	{
		for (int regionIndexX = 0; regionIndexX <= regionDifferenceX; regionIndexX++)
		{
			mask = 1ULL << (currentRegion + regionIndexX);
			if (isMicro == false)
				sentCapsule.m_macroBitRegions |= mask;
			else
				sentCapsule.m_microBitRegions |= mask;
		}
		currentRegion += 8;
	}
}

//-----------------------------------------------------------------------------------------------
void RopeSimulation3D::AssignBitRegionsCollisionObject(int sentCollisionIndex, int currentRegion, int regionDifferenceX, int regionDifferenceY, bool isMicro)
{
	CollisionObject*& sentCollisionObject = m_collisionObjects[sentCollisionIndex];
	uint64_t mask = 0;
	for (int regionIndexY = 0; regionIndexY <= regionDifferenceY; regionIndexY++)
	{
		for (int regionIndexX = 0; regionIndexX <= regionDifferenceX; regionIndexX++)
		{
			mask = 1ULL << (currentRegion + regionIndexX);
			if (isMicro == false)
				sentCollisionObject->m_macroBitRegions |= mask;
			else
				sentCollisionObject->m_microBitRegions |= mask;
		}
		currentRegion += 8;
	}
}

//-----------------------------------------------------------------------------------------------
void RopeSimulation3D::DebugRenderParticles() const
{
	if (m_sbParticlePositions)
	{
		m_renderer->SetModelConstants();
		m_renderer->BindTexture(m_renderer->CreateOrGetTextureFromFile("Data/Images/RopeTexture.jpg"));
		m_renderer->SetRasterizerMode(RasterizerMode::SOLID_CULL_BACK);
		m_renderer->DrawGeometryShader(int(m_particles.m_positions.size()), m_geometryShaderCylinderSides, m_sbParticlePositions, 1, m_cbRopeData, 0, PrimitiveTopology::D3D11_PRIMITIVE_TOPOLOGY_POINTLIST);
		m_renderer->DrawGeometryShader(int(m_particles.m_positions.size()), m_geometryShaderHemisphereTop, m_sbParticlePositions, 1, m_cbRopeData, 0, PrimitiveTopology::D3D11_PRIMITIVE_TOPOLOGY_POINTLIST);
		m_renderer->DrawGeometryShader(int(m_particles.m_positions.size()), m_geometryShaderHemisphereBottom, m_sbParticlePositions, 1, m_cbRopeData, 0, PrimitiveTopology::D3D11_PRIMITIVE_TOPOLOGY_POINTLIST);
	}
}

//-----------------------------------------------------------------------------------------------
void RopeSimulation3D::DebugRenderBitRegions() const
{
	m_renderer->SetModelConstants();
	m_renderer->BindTexture(nullptr);
	m_renderer->DrawVertexBuffer(m_bitRegionVertexBuffer, m_bitRegionDebugVertCount, PrimitiveTopology::D3D11_PRIMITIVE_TOPOLOGY_LINELIST);
}

//-----------------------------------------------------------------------------------------------
int RopeSimulation3D::GetRegionIndexForCoords(IntVec3 const& coords)
{
	return coords.x | (coords.y << 3);
}

//-----------------------------------------------------------------------------------------------
IntVec3 RopeSimulation3D::GetCoordsForRegionIndex(int const& region)
{
	IntVec3 coords;
	coords.x = region & 7; 
	coords.y = region >> 3; 
	coords.z = 0; 
	return coords;
}

//-----------------------------------------------------------------------------------------------
void RopeSimulation3D::UpdateCollisionCapsulesFromParticle(int const& sentParticleIndex)
{	
	//Update Collision Capsules Based On Sent Particle
	if (sentParticleIndex != 0)
	{
		m_collisionCapsules[sentParticleIndex - 1].m_capsule.m_bone.m_end = m_particles.m_proposedPositions[sentParticleIndex];
	}
	if (sentParticleIndex != int(m_particles.m_positions.size()) - 1)
	{
		m_collisionCapsules[sentParticleIndex].m_capsule.m_bone.m_start = m_particles.m_proposedPositions[sentParticleIndex];
	}
}

//-----------------------------------------------------------------------------------------------
void RopeSimulation3D::UpdateProposedParticlesFromCollisionCapsule(int const& sentCapsuleIndex)
{
	//Initializations
	Capsule3& sentCapsule = m_collisionCapsules[sentCapsuleIndex].m_capsule;
	
	m_particles.m_proposedPositions[sentCapsuleIndex] = sentCapsule.m_bone.m_start;
	m_particles.m_proposedPositions[sentCapsuleIndex + 1] = sentCapsule.m_bone.m_end;
}

//-----------------------------------------------------------------------------------------------
void RopeSimulation3D::UpdateNeighboringCollisionCapsules(int const& sentCapsuleIndex)
{
	//Initializations
	Capsule3& sentCapsule = m_collisionCapsules[sentCapsuleIndex].m_capsule;

	if (sentCapsuleIndex != 0)
	{
		m_collisionCapsules[sentCapsuleIndex - 1].m_capsule.m_bone.m_end = sentCapsule.m_bone.m_start;
	}
	if (sentCapsuleIndex != int(m_collisionCapsules.size()) - 1)
	{
		m_collisionCapsules[sentCapsuleIndex + 1].m_capsule.m_bone.m_start = sentCapsule.m_bone.m_end;
	}
}

//-----------------------------------------------------------------------------------------------
void RopeSimulation3D::AddCollidingRopeParticle(int const& particleIndex)
{
	bool isColliding = false;
	for (int collidingIndex = 0; collidingIndex < m_collisionParticleIndices.size(); collidingIndex++)
	{
		if (particleIndex == m_collisionParticleIndices[collidingIndex])
		{
			isColliding = true;
		}
	}

	if (isColliding == false)
	{
		m_collisionParticleIndices.push_back(particleIndex);
	}
}

//-----------------------------------------------------------------------------------------------
void RopeSimulation3D::UnaddCollidingRopeParticle(int const& particleIndex)
{
	m_collisionParticleIndices.erase(std::remove(m_collisionParticleIndices.begin(),
		m_collisionParticleIndices.end(), particleIndex), m_collisionParticleIndices.end());
}

//-----------------------------------------------------------------------------------------------
int RopeSimulation3D::GetPreviousAttachmentOrCollisionParticleIndex(int const& particleIndex) const
{
	int previousAttachedParticleIndex = 0;

	//Get Attachment and Collision Particles
	for (int attachedIndex = 0; attachedIndex < m_attachedParticleIndices.size(); attachedIndex++)
	{
		if (m_attachedParticleIndices[attachedIndex] < particleIndex &&
			m_attachedParticleIndices[attachedIndex] > previousAttachedParticleIndex)
		{
			previousAttachedParticleIndex = m_attachedParticleIndices[attachedIndex];
		}
	}
	for (int collisionIndex = 0; collisionIndex < m_collisionParticleIndices.size(); collisionIndex++)
	{
		if (m_collisionParticleIndices[collisionIndex] < particleIndex &&
			m_collisionParticleIndices[collisionIndex] > previousAttachedParticleIndex)
		{
			previousAttachedParticleIndex = m_collisionParticleIndices[collisionIndex];
		}
	}

	return previousAttachedParticleIndex;
}

//-----------------------------------------------------------------------------------------------
int RopeSimulation3D::GetNextAttachmentOrCollisionParticleIndex(int const& particleIndex) const
{
	int nextAttachedParticleIndex = int(m_particles.m_positions.size());

	for (int attachedIndex = 0; attachedIndex < m_attachedParticleIndices.size(); attachedIndex++)
	{
		if (m_attachedParticleIndices[attachedIndex] > particleIndex &&
			m_attachedParticleIndices[attachedIndex] < nextAttachedParticleIndex)
		{
			nextAttachedParticleIndex = m_attachedParticleIndices[attachedIndex];
		}
	}
	for (int collisionIndex = 0; collisionIndex < m_collisionParticleIndices.size(); collisionIndex++)
	{
		if (m_collisionParticleIndices[collisionIndex] > particleIndex &&
			m_collisionParticleIndices[collisionIndex] < nextAttachedParticleIndex)
		{
			nextAttachedParticleIndex = m_collisionParticleIndices[collisionIndex];
		}
	}

	return nextAttachedParticleIndex;
}

//-----------------------------------------------------------------------------------------------
void RopeSimulation3D::GenerateConstraints()
{
	DeleteConstraints();

	//Distance Constraints
	for (int particleIndex = 1; particleIndex < m_particles.m_positions.size(); particleIndex++)
	{
		Constraint3D constraint = Constraint3D();
		constraint.m_cardinality = 2;
		constraint.m_constraintEquality = Constraint3DEquality::EQUALITY;
		constraint.m_constraintType = Constraint3DType::DISTANCE;
		constraint.m_indices.push_back(particleIndex - 1);
		constraint.m_indices.push_back(particleIndex);
		constraint.m_stiffnessParameter = m_stretchingCoefficient;
		m_distanceConstraints.push_back(constraint);
	}

	//Bending Constraints
	for (int particleIndex = 1; particleIndex < m_particles.m_positions.size(); particleIndex++)
	{
		if (particleIndex + 1 >= m_particles.m_positions.size())
			break;

		Constraint3D constraint = Constraint3D();
		constraint.m_cardinality = 3;
		constraint.m_constraintEquality = Constraint3DEquality::EQUALITY;
		constraint.m_constraintType = Constraint3DType::BENDING;
		constraint.m_indices.push_back(particleIndex - 1);
		constraint.m_indices.push_back(particleIndex);
		constraint.m_indices.push_back(particleIndex + 1);
		constraint.m_stiffnessParameter = m_bendingCoefficient;
		m_bendingConstraints.push_back(constraint);
	}
}

//-----------------------------------------------------------------------------------------------
void RopeSimulation3D::DeleteConstraints()
{
	m_distanceConstraints.clear();
	m_bendingConstraints.clear();
}

//-----------------------------------------------------------------------------------------------
CollisionObject::CollisionObject(Vec3 const& boundingDiscCenter, float boundingDiscRadius)
	:m_boundingDiscCenter(boundingDiscCenter)
	,m_boundingDiscRadius(boundingDiscRadius)
{
}

//-----------------------------------------------------------------------------------------------
SphereCollisionObject::SphereCollisionObject(Vec3 const& boundingDiscCenter, float boundingDiscRadius, Sphere3 const& sphere)
	:CollisionObject(boundingDiscCenter, boundingDiscRadius)
	,m_sphere(sphere)
{
}

//-----------------------------------------------------------------------------------------------
AABBCollisionObject::AABBCollisionObject(Vec3 const& boundingDiscCenter, float boundingDiscRadius, AABB3 const& aabb)
	: CollisionObject(boundingDiscCenter, boundingDiscRadius)
	, m_aabb(aabb) 
{
}

//-----------------------------------------------------------------------------------------------
OBBCollisionObject::OBBCollisionObject(Vec3 const& boundingDiscCenter, float boundingDiscRadius, OBB3 const& obb)
	: CollisionObject(boundingDiscCenter, boundingDiscRadius)
	, m_obb(obb) 
{
}

//-----------------------------------------------------------------------------------------------
CapsuleCollisionObject::CapsuleCollisionObject(Vec3 const& boundingDiscCenter, float boundingDiscRadius, Capsule3 const& capsule)
	: CollisionObject(boundingDiscCenter, boundingDiscRadius)
	, m_capsule(capsule) 
{
}

//-----------------------------------------------------------------------------------------------
CylinderCollisionObject::CylinderCollisionObject(Vec3 const& boundingDiscCenter, float boundingDiscRadius, Cylinder3 const& cylinder)
	: CollisionObject(boundingDiscCenter, boundingDiscRadius)
	, m_cylinder(cylinder) 
{
}
