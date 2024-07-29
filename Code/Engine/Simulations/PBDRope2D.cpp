#include "PBDRope2D.hpp"
#include "Engine/Core/Vertex_PCU.hpp"
#include "Engine/Core/VertexUtils.hpp"
#include "Engine/Math/LineSegment2.hpp"
#include "Engine/Core/ErrorWarningAssert.hpp"
#include "Engine/Simulations/Spring2D.hpp"
#include "Engine/Simulations/Particle2D.hpp"
#include "Engine/Core/EngineCommon.hpp"
#include "Engine/Core/Time.hpp"
#include "Engine/Math/MathUtils.hpp"
#include "Engine/Math/CatmullRomSpline2D.hpp"
#include "Engine/Math/CubicHermiteCurve2D.hpp"


//-----------------------------------------------------------------------------------------------
PBDRope2D::PBDRope2D(int totalPoints, float totalMassOfRope, float dampingCoefficient, float stretchCoefficient,
	float bendingCoefficient, float staticFrictionCoefficient, float kineticFrictionCoefficient,
	int totalSolverIterations, Vec2 start, Vec2 end, float physicsTimestep, bool isDebugMode)
	:m_numberOfPointsInRope(totalPoints)
	,m_dampingCoefficient(dampingCoefficient)
	,m_stretchingCoefficient(stretchCoefficient)
	,m_bendingCoefficient(bendingCoefficient)
	,m_staticFrictionCoefficient(staticFrictionCoefficient)
	,m_kineticFrictionCoefficient(kineticFrictionCoefficient)
	,m_totalSolverIterations(totalSolverIterations)
	,m_ropeStartPosition(start)
	,m_ropeEndPosition(end)
	,m_totalLengthOfRope((start - end).GetLength())
	,m_isDebugMode(isDebugMode)
	,m_physicsTimestep(physicsTimestep)
{
	Vec2 displacement = end - start;
	Vec2 direction = displacement.GetNormalized();
	float magnitude = displacement.GetLength();
	m_desiredDistance = magnitude / (totalPoints - 1);
	m_bendingConstraintDistance = m_desiredDistance * (2.0f);
	float massPerPoint = (totalMassOfRope / totalPoints);
	float inverseMassPerPoint = 1.0f / massPerPoint;
	Particle2D* particleA = nullptr;
	Particle2D* particleB = nullptr;
	for (int particleIndex = 0; particleIndex < totalPoints; particleIndex++)
	{	
		if (particleIndex == 0)
		{
			continue;
		}
		else if (particleIndex == 1)
		{
			particleA = new Particle2D(massPerPoint, inverseMassPerPoint, start +
				(direction * ((particleIndex - 1) * m_desiredDistance)), Vec2(), true);
			m_particles.push_back(particleA);
		}
		else
		{
			particleA = particleB;
		}

		particleB = new Particle2D(massPerPoint, inverseMassPerPoint, start +
			(direction * (particleIndex * m_desiredDistance)), Vec2());
		m_particles.push_back(particleB);

		Capsule2 capsule;
		capsule.m_bone = LineSegment2(particleA->m_proposedPosition, particleB->m_proposedPosition);
		capsule.m_radius = m_ropeRadius;
		capsule.m_inverseMass = 1.0f / (massPerPoint + massPerPoint);
		m_selfCollisionCapsules.push_back(capsule);
	}

	m_shapes = new Shapes2D();
}

//-----------------------------------------------------------------------------------------------
PBDRope2D::PBDRope2D()
{
}

//-----------------------------------------------------------------------------------------------
PBDRope2D::~PBDRope2D()
{
	for (int particleIndex = 0; particleIndex < m_particles.size(); particleIndex++)
	{
		Particle2D* particle = m_particles[particleIndex];
		if (particle)
		{
			delete particle;
			particle = nullptr;
		}
	}

	for (int particleIndex = 0; particleIndex < m_particles.size(); particleIndex++)
	{
		Particle2D* particle = m_particles[particleIndex];
		if (particle)
		{
			delete particle;
			particle = nullptr;
		}
	}
}

//-----------------------------------------------------------------------------------------------
void PBDRope2D::Startup()
{
}

//-----------------------------------------------------------------------------------------------
void PBDRope2D::Shutdown()
{
}

//-----------------------------------------------------------------------------------------------
void PBDRope2D::Update(float deltaSeconds)
{
	//float startTime = float(GetCurrentTimeSeconds());
	m_physicsDebt += deltaSeconds;
	while (m_physicsDebt > m_physicsTimestep)
	{
		//Loop to estimate new velocities, proposed positions, and generate collisions
		for (int particleIndex = 0; particleIndex < m_particles.size(); particleIndex++)
		{
			Particle2D* currentPoint = m_particles[particleIndex];
			if (currentPoint->m_isLocked == true)
			{
				continue;
			}

			//Calculate next velocity (semi-implicit Euler)
			Vec2 acceleration = (Vec2(0.0f, -m_gravityCoefficient));
			Vec2 velocity = currentPoint->m_velocity;
			velocity += acceleration * m_physicsTimestep;

			//Damp Velocities (TO-DO: ADD MORE SOPHISTICATED DAMPING LATER)
			velocity *= m_dampingCoefficient;

			//Calculate Proposed Positions
			currentPoint->m_proposedPosition = currentPoint->m_position + velocity * m_physicsTimestep;
		}

		//Loop through constraints and project them
		for (int solverIndex = 0; solverIndex < m_totalSolverIterations; solverIndex++)
		{
			ProjectConstraints();
		}

		//Loop through and set projected positions and velocities
		for (int particleIndex = 0; particleIndex < m_particles.size(); particleIndex++)
		{
			Particle2D* currentPoint = m_particles[particleIndex];
			currentPoint->m_velocity = (currentPoint->m_proposedPosition - currentPoint->m_position) / m_physicsTimestep;

			if (currentPoint->m_forcesAsFloat != 0.0f)
			{
				currentPoint->m_velocity += (currentPoint->m_velocity.GetNormalized() * -1.0f)
					* currentPoint->m_forcesAsFloat * m_physicsTimestep;
			}

			currentPoint->m_position = currentPoint->m_proposedPosition;
			currentPoint->m_forcesAsFloat = 0.0f;
		}

		m_physicsDebt -= m_physicsTimestep;
	}
	//float endTime = float(GetCurrentTimeSeconds());
}

//-----------------------------------------------------------------------------------------------
void PBDRope2D::Render(std::vector<Vertex_PCU>& verts) const
{
	verts.reserve((m_particles.size() - 1) * size_t(6));
	
	std::vector<Vec2> positions;
	positions.reserve(m_particles.size());
	for (int particleIndex = 0; particleIndex < m_particles.size(); particleIndex++)
	{
		positions.push_back(m_particles[particleIndex]->m_position);
		/*if (particleIndex == 0)
		{
			continue;
		}*/

		/*Particle2D* previousPoint = m_particles[particleIndex - 1];
		Particle2D* currentPoint = m_particles[particleIndex];
		LineSegment2 line;
		line.m_start = previousPoint->m_position;
		line.m_end = currentPoint->m_position;
		Capsule2 capsule;
		capsule.m_bone = line;
		capsule.m_radius = m_ropeRadius;
		AddVertsForCapsule2D(verts, capsule, Rgba8(142, 89, 60, 255));*/

		

		/*if (m_isDebugMode)
		{
			AddVertsForDisc2D(verts, previousPoint->m_position, m_ropeRadius * 3.0f, Rgba8::WHITE);
			AddVertsForDisc2D(verts, currentPoint->m_position, m_ropeRadius * 3.0f, Rgba8::WHITE);
		}*/
	}

	CatmullRomSpline2D newSpline(positions);
	for (int curveIndex = 0; curveIndex < newSpline.m_hermiteCurves.size(); curveIndex++)
	{
		CubicHermiteCurve2D* previousCurve = nullptr;
		if (curveIndex != 0)
		{
			previousCurve = newSpline.m_hermiteCurves[curveIndex - 1];
		}
		CubicHermiteCurve2D* currentCurve = newSpline.m_hermiteCurves[curveIndex];
		if (currentCurve != nullptr && previousCurve != nullptr)
		{
			LineSegment2 line1; LineSegment2 line2;
			line1.m_start = previousCurve->m_endPos;
			line1.m_end = currentCurve->m_startPos;
			line2.m_start = currentCurve->m_startPos;
			line2.m_end = currentCurve->m_endPos;
			Capsule2 capsule1;
			capsule1.m_bone = line1;
			capsule1.m_radius = m_ropeRadius; 
			Capsule2 capsule2;
			capsule2.m_bone = line1;
			capsule2.m_radius = m_ropeRadius;
			AddVertsForCapsule2D(verts, capsule1, Rgba8(142, 89, 60, 255));
			AddVertsForCapsule2D(verts, capsule2, Rgba8(142, 89, 60, 255));
			/*AddVertsForLineSegment2D(verts, line1, 0.05f, Rgba8(100, 130, 200, 255));
			AddVertsForLineSegment2D(verts, line2, 0.05f, Rgba8(100, 130, 200, 255));*/
		}
		else if (currentCurve != nullptr && previousCurve == nullptr)
		{
			LineSegment2 line;
			line.m_start = currentCurve->m_startPos;
			line.m_end = currentCurve->m_endPos;
			Capsule2 capsule1;
			capsule1.m_bone = line;
			capsule1.m_radius = m_ropeRadius;
			AddVertsForCapsule2D(verts, capsule1, Rgba8(142, 89, 60, 255));
			//AddVertsForLineSegment2D(verts, line, 0.05f, Rgba8(100, 130, 200, 255));
		}
	}
}

//-----------------------------------------------------------------------------------------------
float PBDRope2D::GetCurrentLengthOfTheRope()
{
	float currentLength = 0.0f;
	for (int particleIndex = 0; particleIndex < m_particles.size(); particleIndex++)
	{
		Particle2D* particleA = nullptr;
		Particle2D* particleB = m_particles[particleIndex];

		if (particleIndex != 0)
		{
			particleA = m_particles[particleIndex - 1];
		}

		if (particleA)
		{
			currentLength += (particleA->m_position - particleB->m_position).GetLength();
		}
	}
	return currentLength;
}

//-----------------------------------------------------------------------------------------------
void PBDRope2D::ClearShapeReferences()
{
	m_shapes->m_discs.clear();
	m_shapes->m_capsules.clear();
	m_shapes->m_aabbs.clear();
	m_shapes->m_obbs.clear();
}

//-----------------------------------------------------------------------------------------------
void PBDRope2D::ProjectConstraints()
{	 
	//Points to reference for ease of access
	/*Particle2D* particleStart = m_particles[0];
	Particle2D* particlePreviousToEnd = m_particles[m_particles.size() - 2];
	Particle2D* particleEnd = m_particles[m_particles.size() - 1];*/

	//Loop to project each constraint on each particle
	for (int particleIndex = 0; particleIndex < m_particles.size(); particleIndex++)
	{
		Particle2D* particleA = nullptr;
		Particle2D* particleB = m_particles[particleIndex];
		Particle2D* particleC = nullptr;
		
		if (particleIndex != 0)
		{
			particleA = m_particles[particleIndex - 1];
		}
		if (particleIndex != int(m_particles.size() - 1))
		{
			particleC = m_particles[particleIndex + 1];
		}

		//Constraints Projection
		if (particleB->m_isLocked == false)
		{
			ProjectCollisionConstraints(particleB);
		}
		if (particleA)
		{
			ProjectDistanceConstraint(particleA, particleB);
		}
		if (particleA && particleC)
		{
			ProjectBendingConstraint(particleA, particleC);
		}
	}
}

//-----------------------------------------------------------------------------------------------
void PBDRope2D::ProjectDistanceConstraint(Particle2D* particleA, Particle2D* particleB)
{
	//Calculates direction and overflow along with weight Coefficients
	Vec2 displacement = particleB->m_proposedPosition - particleA->m_proposedPosition;
	float distanceConstraint = (displacement.GetLength() - m_desiredDistance);

	if (distanceConstraint == 0)
	{
		return;
	}

	Vec2 gradient = displacement.GetNormalized();

	if (particleA->m_isLocked == false && particleB->m_isLocked == false)
	{
		float particleAWeightCoefficient = (particleA->m_inverseMass / (particleA->m_inverseMass + particleB->m_inverseMass));
		float particleBWeightCoefficient = (particleB->m_inverseMass / (particleA->m_inverseMass + particleB->m_inverseMass));

		Vec2 deltaPointA = particleAWeightCoefficient * distanceConstraint * gradient;
		Vec2 deltaPointB = particleBWeightCoefficient * distanceConstraint * gradient;

		particleA->m_proposedPosition += deltaPointA * m_stretchingCoefficient;
		particleB->m_proposedPosition -= deltaPointB * m_stretchingCoefficient;
	}
	else if (particleA->m_isLocked == true && particleB->m_isLocked == false)
	{
		Vec2 deltaPointB = distanceConstraint * gradient;
		particleB->m_proposedPosition -= deltaPointB * m_stretchingCoefficient;
	}
	else if (particleA->m_isLocked == false && particleB->m_isLocked == true)
	{
		Vec2 deltaPointA = distanceConstraint * gradient;
		particleA->m_proposedPosition += deltaPointA * m_stretchingCoefficient;
	}
}

//-----------------------------------------------------------------------------------------------
void PBDRope2D::ProjectBendingConstraint(Particle2D* particleA, Particle2D* particleC)
{
	//Calculates direction and overflow along with weight Coefficients
	Vec2 displacement = particleC->m_proposedPosition - particleA->m_proposedPosition;
	float distanceConstraint = displacement.GetLength() - m_bendingConstraintDistance;
	
	//Check if less than the minimum distance, if so no need to constrain
	if (distanceConstraint > 0.0f)
	{
		return;
	}

	Vec2 gradient = displacement.GetNormalized();

	if (particleA->m_isLocked == false && particleC->m_isLocked == false)
	{
		float particleAWeightCoefficient = (particleA->m_inverseMass / (particleA->m_inverseMass + particleC->m_inverseMass));
		float particleCWeightCoefficient = (particleC->m_inverseMass / (particleA->m_inverseMass + particleC->m_inverseMass));

		Vec2 deltaPointA = m_bendingCoefficient * particleAWeightCoefficient * distanceConstraint * gradient;
		Vec2 deltaPointC = m_bendingCoefficient * particleCWeightCoefficient * distanceConstraint * gradient;

		particleA->m_proposedPosition += deltaPointA;
		particleC->m_proposedPosition -= deltaPointC;
	}
	else if (particleA->m_isLocked == true && particleC->m_isLocked == false)
	{
		Vec2 deltaPointC = m_bendingCoefficient * distanceConstraint * gradient;
		particleC->m_proposedPosition -= deltaPointC;
	}
	else if (particleA->m_isLocked == false && particleC->m_isLocked == true)
	{
		Vec2 deltaPointA = m_bendingCoefficient * distanceConstraint * gradient;
		particleA->m_proposedPosition += deltaPointA;
	}
}

//-----------------------------------------------------------------------------------------------
void PBDRope2D::ProjectCollisionConstraints(Particle2D* particle)
{
	//Floor Collisions
	if (particle->m_proposedPosition.y < m_ropeRadius)
	{
		particle->m_proposedPosition.y = m_ropeRadius;

		if (particle->m_velocity != Vec2())
		{
			float normalForce = particle->m_mass * m_gravityCoefficient;
			particle->m_forcesAsFloat += m_kineticFrictionCoefficient * normalForce;
		}
	}

	//Discs Collisions
	for (int discIndex = 0; discIndex < m_shapes->m_discs.size(); discIndex++)
	{
		Disc2& disc = m_shapes->m_discs[discIndex];
		if (PushDiscOutOfFixedDisc2D(particle->m_proposedPosition, m_ropeRadius, disc.m_center, disc.m_radius))
		{
			if (particle->m_velocity != Vec2())
			{
				float normalForce = particle->m_mass * m_gravityCoefficient;
				particle->m_forcesAsFloat += m_kineticFrictionCoefficient * normalForce;
			}
		}
	}

	//AABB Collisions
	for (int aabbIndex = 0; aabbIndex < m_shapes->m_aabbs.size(); aabbIndex++)
	{
		AABB2& aabb = m_shapes->m_aabbs[aabbIndex];
		if (PushDiscOutOfFixedAABB2D(particle->m_proposedPosition, m_ropeRadius, aabb))
		{
			if (particle->m_velocity != Vec2())
			{
				float normalForce = particle->m_mass * m_gravityCoefficient;
				particle->m_forcesAsFloat += m_kineticFrictionCoefficient * normalForce;
			}
		}
	}

	//OBB Collisions
	for (int obbIndex = 0; obbIndex < m_shapes->m_obbs.size(); obbIndex++)
	{
		OBB2& obb = m_shapes->m_obbs[obbIndex];
		if (PushDiscOutOfFixedOBB2D(particle->m_proposedPosition, m_ropeRadius, obb))
		{
			if (particle->m_velocity != Vec2())
			{
				float normalForce = particle->m_mass * m_gravityCoefficient;
				particle->m_forcesAsFloat += m_kineticFrictionCoefficient * normalForce;
			}
		}
	}

	//Capsule Collisions
	for (int capsuleIndex = 0; capsuleIndex < m_shapes->m_capsules.size(); capsuleIndex++)
	{
		Capsule2& capsule = m_shapes->m_capsules[capsuleIndex];
		if (PushDiscOutOfFixedCapsule2D(particle->m_proposedPosition, m_ropeRadius, capsule))
		{
			if (particle->m_velocity != Vec2())
			{
				float normalForce = particle->m_mass * m_gravityCoefficient;
				particle->m_forcesAsFloat += m_kineticFrictionCoefficient * normalForce;
			}
		}
	}

	//SELF COLLISIONS
	/*int capsuleIndex = 0;
	for (int particleIndex = 0; particleIndex < m_numberOfPointsInRope; particleIndex++)
	{
		if (particleIndex == 0)
		{
			continue;
		}

		Particle2D* particleA = m_particles[particleIndex - 1];
		Particle2D* particleB = m_particles[particleIndex];

		if (particle == particleA || particle == particleB)
		{
			continue;
		}

		Capsule2& capsule = m_selfCollisionCapsules[capsuleIndex];
		capsule.m_bone.m_start = particleA->m_proposedPosition;
		capsule.m_bone.m_end = particleB->m_proposedPosition;
		PushDiscOutOfMobileCapsule2D(particle->m_proposedPosition, m_ropeRadius, particle->m_inverseMass, capsule);
		particleA->m_proposedPosition = capsule.m_bone.m_start;
		particleB->m_proposedPosition = capsule.m_bone.m_end;
		capsuleIndex++;
	}*/
}

//-----------------------------------------------------------------------------------------------
Shapes2D::Shapes2D()
{
}

//-----------------------------------------------------------------------------------------------
Shapes2D::~Shapes2D()
{
}
