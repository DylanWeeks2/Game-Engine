#include "ClothSimulation3D.hpp"
#include "Engine/Math/MathUtils.hpp"
#include "Engine/Core/Vertex_PCU.hpp"
#include "Engine/Core/VertexUtils.hpp"
#include "Engine/Renderer/Renderer.hpp"
#include "Engine/Core/Time.hpp"
#include "Engine/Core/Vertex_PCUTBN.hpp"
#include <vector>

//-----------------------------------------------------------------------------------------------
ClothSimulation3D::ClothSimulation3D(Renderer* renderer, AABB3 worldBounds, Vec2 dimensions, int totalNumberOfParticles, 
	int totalSolverIterations, Vec3 attachPointTopLeft, Vec3 attachPointTopRight)
	:m_renderer(renderer)
	,m_worldBounds(worldBounds)
	,m_dimensions(dimensions)
	,m_totalNumberOfParticles(totalNumberOfParticles)
	, m_totalSolverIterations(totalSolverIterations)
{
	InitializeShaders();

	m_numberOfRows = int(sqrtf(float(totalNumberOfParticles)));
	m_numberOfParticlesPerRow = m_numberOfRows;
	m_originalHorizontalDistance = dimensions.x / m_numberOfParticlesPerRow;
	m_originalVerticalDistance = dimensions.y / m_numberOfRows;
	m_originalDiagonalDistance = sqrtf(m_originalHorizontalDistance * m_originalHorizontalDistance + m_originalVerticalDistance * m_originalVerticalDistance);
	Vec3 directionRight = (attachPointTopRight - attachPointTopLeft).GetNormalized();
	Vec3 directionDown = Vec3(0.0f, 0.0f, 1.0f);
	Vec3 zero = Vec3(0.0, 0.0, 0.0);

	for (int rowIndex = 0; rowIndex < m_numberOfRows; rowIndex++)
	{
		for (int columnIndex = 0; columnIndex < m_numberOfParticlesPerRow; columnIndex++)
		{
			m_particles.m_positions.push_back(attachPointTopLeft + (directionRight * (columnIndex * m_originalHorizontalDistance)) + (directionDown * (rowIndex * m_originalVerticalDistance)));
			m_particles.m_velocities.push_back(zero);
			m_particles.m_proposedPositions.push_back(zero);
			m_particles.m_jacobiCorrections.push_back(Vec4(0.0, 0.0, 0.0, 0.0));
			m_particles.m_collisionNormals.push_back(zero);
			m_particles.m_macroBitRegions.push_back(0);
			m_particles.m_microBitRegions.push_back(0);
			m_particles.m_masses.push_back(1.0f);
			m_particles.m_inverseMasses.push_back(1.0f);
			if((rowIndex == 0 && columnIndex == 0) || (rowIndex == 0 && columnIndex == m_numberOfParticlesPerRow - 1))
				m_particles.m_isAttached.push_back(1);
			else
				m_particles.m_isAttached.push_back(0);
			m_particles.m_isSelfCollision.push_back(0);
		}
	}

	//Horizontal Distance Constraint
	for (int particleIndex = 0; particleIndex < m_totalNumberOfParticles; particleIndex++)
	{
		std::vector<int> indices;
		indices.push_back(particleIndex);
		indices.push_back(particleIndex + 1);

		Constraint3D constraint;
		constraint.m_cardinality = 2;
		constraint.m_constraintEquality = Constraint3DEquality::EQUALITY;
		constraint.m_constraintType = Constraint3DType::DISTANCE;
		constraint.m_stiffnessParameter = 1.0f;
		constraint.m_indices = indices;
		if ((particleIndex + 1) % (m_numberOfParticlesPerRow) != 0)
		{
			m_distanceConstraints.push_back(constraint);
			m_distanceConstraintsOriginalDists.push_back(m_originalHorizontalDistance);
		}
	}

	//Vertical Distance Constraint
	for (int particleIndex = 0; particleIndex < m_totalNumberOfParticles; particleIndex++)
	{
		std::vector<int> indices;
		indices.push_back(particleIndex);
		indices.push_back(particleIndex + m_numberOfParticlesPerRow);

		Constraint3D constraint;
		constraint.m_cardinality = 2;
		constraint.m_constraintEquality = Constraint3DEquality::EQUALITY;
		constraint.m_constraintType = Constraint3DType::DISTANCE;
		constraint.m_stiffnessParameter = 1.0f;
		constraint.m_indices = indices;
		if (particleIndex < (m_numberOfRows - 1) * m_numberOfParticlesPerRow)
		{
			m_distanceConstraints.push_back(constraint);
			m_distanceConstraintsOriginalDists.push_back(m_originalVerticalDistance);
		}
	}


	//Diagonal Right Distance Constraint
	for (int particleIndex = 0; particleIndex < m_totalNumberOfParticles; particleIndex++)
	{
		std::vector<int> indices;
		indices.push_back(particleIndex);
		indices.push_back(particleIndex + m_numberOfParticlesPerRow + 1);

		Constraint3D constraint;
		constraint.m_cardinality = 2;
		constraint.m_constraintEquality = Constraint3DEquality::EQUALITY;
		constraint.m_constraintType = Constraint3DType::DISTANCE;
		constraint.m_stiffnessParameter = 1.0f;
		constraint.m_indices = indices;
		if (particleIndex < (m_numberOfRows - 1) * m_numberOfParticlesPerRow)
			if ((particleIndex + m_numberOfParticlesPerRow + 1) % (m_numberOfParticlesPerRow) != 0)
			{
				m_distanceConstraints.push_back(constraint);
				m_distanceConstraintsOriginalDists.push_back(m_originalDiagonalDistance);
			}
	}

	//Diagonal Left Distance Constraint
	for (int particleIndex = 0; particleIndex < m_totalNumberOfParticles; particleIndex++)
	{
		std::vector<int> indices;
		indices.push_back(particleIndex);
		indices.push_back(particleIndex + m_numberOfParticlesPerRow - 1);

		Constraint3D constraint;
		constraint.m_cardinality = 2;
		constraint.m_constraintEquality = Constraint3DEquality::EQUALITY;
		constraint.m_constraintType = Constraint3DType::DISTANCE;
		constraint.m_stiffnessParameter = 1.0f;
		constraint.m_indices = indices;
		if (particleIndex < (m_numberOfRows - 1) * m_numberOfParticlesPerRow)
			if ((particleIndex) % (m_numberOfParticlesPerRow) != 0)
			{
				m_distanceConstraints.push_back(constraint);
				m_distanceConstraintsOriginalDists.push_back(m_originalDiagonalDistance);
			}
	}
}

//-----------------------------------------------------------------------------------------------
void ClothSimulation3D::Update(float deltaSeconds)
{
	m_simulationStartTime = float(GetCurrentTimeSeconds());
	m_physicsDebt += deltaSeconds;
	while (m_physicsDebt > m_physicsTimestep)
	{
		UpdateCPU();
		m_physicsDebt -= m_physicsTimestep;
	}
	m_simulationEndTime = float(GetCurrentTimeSeconds());
}

//-----------------------------------------------------------------------------------------------
void ClothSimulation3D::Render() const
{
	if (m_isDebugCloth)
	{
		RenderDebugVerts();
		//return;
	}
	
	//Normal Render 
	AABB2 uvs;
	float uvVertAmount = 1.0f / m_numberOfRows;
	float uvHorzAmount = 1.0f / m_numberOfParticlesPerRow;
	std::vector<Vertex_PCUTBN> verts;
	for (int particleIndex = 0; particleIndex < m_totalNumberOfParticles; particleIndex++)
	{
		if (particleIndex < (m_numberOfRows - 1) * m_numberOfParticlesPerRow)
		{
			if ((particleIndex + m_numberOfParticlesPerRow + 1) % (m_numberOfParticlesPerRow) != 0)
			{
				Vec3 bottomLeft = m_particles.m_positions[particleIndex + m_numberOfParticlesPerRow];
				Vec3 bottomRight = m_particles.m_positions[particleIndex + m_numberOfParticlesPerRow + 1];
				Vec3 topLeft = m_particles.m_positions[particleIndex];
				Vec3 topRight = m_particles.m_positions[particleIndex + 1];
				
				if ((particleIndex) % m_numberOfRows == 0)
				{
					uvs.m_mins = Vec2(0.0f, uvs.m_maxs.y);
					uvs.m_maxs = Vec2(uvs.m_mins.x + uvHorzAmount, uvs.m_mins.y + uvVertAmount);
				}
				else
				{
					uvs.m_mins = Vec2(uvs.m_maxs.x, uvs.m_mins.y);
					uvs.m_maxs = Vec2(uvs.m_mins.x + uvHorzAmount, uvs.m_mins.y + uvVertAmount);
				}
				AddVertsForQuad3D(verts, bottomLeft, bottomRight, topLeft, topRight, Rgba8::YELLOW, uvs);
			}
		}
	}

	CalculateTangentSpaceVectors(verts);

	m_renderer->SetRasterizerMode(RasterizerMode::SOLID_CULL_NONE);
	Texture* diffuseTexture = m_renderer->CreateOrGetTextureFromFile("Data/Images/Fabric_Diffuse.png");
	Texture* normalTexture = m_renderer->CreateOrGetTextureFromFile("Data/Images/Fabric_Normal.png");
	Texture* specGlossEmitTexture = m_renderer->CreateOrGetTextureFromFile("Data/Images/Brick_SpecGlossEmit.png");
	m_renderer->BindTextures(diffuseTexture, normalTexture, specGlossEmitTexture);
	m_renderer->SetModelConstants();
	m_renderer->DrawVertexArray(int(verts.size()), verts.data(), m_renderShader);
}

//-----------------------------------------------------------------------------------------------
void ClothSimulation3D::UpdateGrabbedClothParticle(int sentParticleIndex, Vec3 const& newPosition)
{
	//Position Updates
	m_grabbedParticlePosition = newPosition;
	m_grabbedParticleIndex = sentParticleIndex;
	m_particles.m_positions[sentParticleIndex] = newPosition;
	m_particles.m_proposedPositions[sentParticleIndex] = newPosition;
}

//-----------------------------------------------------------------------------------------------
void ClothSimulation3D::UpdateCPU()
{
	UpdateGaussSeidel();
}

//-----------------------------------------------------------------------------------------------
void ClothSimulation3D::UpdateGaussSeidel()
{
	//Propose Positions
	for (int particleIndex = 0; particleIndex < m_particles.m_positions.size(); particleIndex++)
	{
		if (m_particles.m_isAttached[particleIndex] == 1)
		{
			m_particles.m_proposedPositions[particleIndex] = m_particles.m_positions[particleIndex];
			continue;
		}

		Vec3 acceleration = (Vec3(0.0f, 0.0f, -m_gravityCoefficient));
		Vec3 velocity = m_particles.m_velocities[particleIndex];
		velocity += acceleration * m_physicsTimestep;
		velocity *= m_dampingCoefficient;
		m_particles.m_proposedPositions[particleIndex] = m_particles.m_positions[particleIndex] + velocity * m_physicsTimestep;
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
void ClothSimulation3D::ProjectConstraintsGaussSeidel()
{
	for (int constraintIndex = 0; constraintIndex < m_distanceConstraints.size(); constraintIndex++)
	{
		ProjectDistanceConstraintGaussSeidel(constraintIndex);
	}

	//Collisions
	for (int particleIndex = 0; particleIndex < m_particles.m_positions.size(); particleIndex++)
	{
		ProjectWorldBoundsConstraintsSpheresGaussSeidel(particleIndex);
	}
}

//-----------------------------------------------------------------------------------------------
void ClothSimulation3D::ProjectDistanceConstraintGaussSeidel(int constraintIndex)
{
	//Initializations
	Constraint3D& constraint = m_distanceConstraints[constraintIndex];

	//Calculates direction and overflow along with weight Coefficients
	Vec3 displacement = m_particles.m_proposedPositions[constraint.m_indices[1]] - m_particles.m_proposedPositions[constraint.m_indices[0]];
	float desiredDistance = 0.0f;
	desiredDistance = (m_distanceConstraintsOriginalDists[constraintIndex]);
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
		coefficientValue = 1.0f;
	}
	else
	{
		coefficientValue = 1.0f;
	}

	if (m_particles.m_isAttached[constraint.m_indices[0]] == 0 && m_particles.m_isAttached[constraint.m_indices[1]] == 0)
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
void ClothSimulation3D::ProjectWorldBoundsConstraintsSpheresGaussSeidel(int sentParticleIndex)
{
	//Min Z
	if (m_particles.m_proposedPositions[sentParticleIndex].z < m_worldBounds.m_mins.z + m_clothThickness)
	{
		m_particles.m_proposedPositions[sentParticleIndex].z = m_worldBounds.m_mins.z + m_clothThickness;
		m_particles.m_collisionNormals[sentParticleIndex] = Vec3(0.0f, 0.0f, 1.0f);
		m_particles.m_isSelfCollision[sentParticleIndex] = 0;
	}
	//Max Z
	if (m_particles.m_proposedPositions[sentParticleIndex].z > m_worldBounds.m_maxs.z - m_clothThickness)
	{
		m_particles.m_proposedPositions[sentParticleIndex].z = m_worldBounds.m_maxs.z - m_clothThickness;
		m_particles.m_collisionNormals[sentParticleIndex] = Vec3(0.0f, 0.0f, -1.0f);
		m_particles.m_isSelfCollision[sentParticleIndex] = 0;
	}
	//Min X
	if (m_particles.m_proposedPositions[sentParticleIndex].x < m_worldBounds.m_mins.x + m_clothThickness)
	{
		m_particles.m_proposedPositions[sentParticleIndex].x = m_worldBounds.m_mins.x + m_clothThickness;
		m_particles.m_collisionNormals[sentParticleIndex] = Vec3(1.0f, 0.0f, 0.0f);
		m_particles.m_isSelfCollision[sentParticleIndex] = 0;
	}
	//Max X
	if (m_particles.m_proposedPositions[sentParticleIndex].x > m_worldBounds.m_maxs.x - m_clothThickness)
	{
		m_particles.m_proposedPositions[sentParticleIndex].x = m_worldBounds.m_maxs.x - m_clothThickness;
		m_particles.m_collisionNormals[sentParticleIndex] = Vec3(-1.0f, 0.0f, 0.0f);
		m_particles.m_isSelfCollision[sentParticleIndex] = 0;
	}
	//Min Y
	if (m_particles.m_proposedPositions[sentParticleIndex].y < m_worldBounds.m_mins.y + m_clothThickness)
	{
		m_particles.m_proposedPositions[sentParticleIndex].y = m_worldBounds.m_mins.y + m_clothThickness;
		m_particles.m_collisionNormals[sentParticleIndex] = Vec3(0.0f, 1.0f, 0.0f);
		m_particles.m_isSelfCollision[sentParticleIndex] = 0;
	}
	//Max Y
	if (m_particles.m_proposedPositions[sentParticleIndex].y > m_worldBounds.m_maxs.y - m_clothThickness)
	{
		m_particles.m_proposedPositions[sentParticleIndex].y = m_worldBounds.m_maxs.y - m_clothThickness;
		m_particles.m_collisionNormals[sentParticleIndex] = Vec3(0.0f, -1.0f, 0.0f);
		m_particles.m_isSelfCollision[sentParticleIndex] = 0;
	}
}

//-----------------------------------------------------------------------------------------------
void ClothSimulation3D::RenderDebugVerts() const
{
	std::vector<Vertex_PCU> verts;
	for (int particleIndex = 0; particleIndex < m_totalNumberOfParticles; particleIndex++)
	{
		Rgba8 color = Rgba8::RED;
		if (m_particles.m_isAttached[particleIndex])
		{
			color = Rgba8::BLUE;
		}
		else
		{
			if ((particleIndex) % m_numberOfRows == 0)
			{
				color = Rgba8::GREEN;
			}
		}
		AddVertsForSphere3D(verts, m_particles.m_positions[particleIndex], 0.01f, color);
	}

	for (int constraintIndex = 0; constraintIndex < m_distanceConstraints.size(); constraintIndex++)
	{
		Vec3 p1 = m_particles.m_positions[m_distanceConstraints[constraintIndex].m_indices[0]];
		Vec3 p2 = m_particles.m_positions[m_distanceConstraints[constraintIndex].m_indices[1]];;
		AddVertsForLineList(verts, p1, p2, Rgba8::ORANGE);
	}


	m_renderer->BindTexture(nullptr);
	m_renderer->SetModelConstants();
	m_renderer->DrawVertexArray(int(verts.size()), verts.data(), PrimitiveTopology::D3D11_PRIMITIVE_TOPOLOGY_LINELIST);
}

//-----------------------------------------------------------------------------------------------
void ClothSimulation3D::InitializeShaders()
{
	//Render Shader
 	m_renderShader = m_renderer->CreateShader("Data/Shaders/SpriteLitTBN.hlsl", VertexType::VERTEX_PCUTBN, false, false);
}
