#include "MassSpringRope2D.hpp"
#include "Engine/Core/Vertex_PCU.hpp"
#include "Engine/Core/VertexUtils.hpp"
#include "Engine/Math/LineSegment2.hpp"
#include "Engine/Core/ErrorWarningAssert.hpp"
#include "Engine/Simulations/Spring2D.hpp"
#include "Engine/Simulations/Particle2D.hpp"
#include "Engine/Core/EngineCommon.hpp"
#include "Engine/Core/Time.hpp"
#include "Engine/Math/Capsule2.hpp"

//-----------------------------------------------------------------------------------------------
MassSpringRope2D::MassSpringRope2D(IntegrationMethod integrationMethod, int totalPoints, 
	float stiffnessConstant, float dampingConstant, float massOfEachPoint, Vec2 start, Vec2 end)
	:m_integrationMethod(integrationMethod)
	,m_numberOfPointsInRope(totalPoints)
	,m_stiffnessConstant(stiffnessConstant)
	,m_dampingConstant(dampingConstant)
	,m_ropeStartPosition(start)
	,m_ropeEndPosition(end)
{
	Vec2 displacement = end - start;
	Vec2 direction = displacement.GetNormalized();
	float magnitude = displacement.GetLength();
	float lengthPerSpring = magnitude / (totalPoints - 1);
	float massPerPoint = (massOfEachPoint / totalPoints);
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
				(direction * ((particleIndex - 1) * lengthPerSpring)),
				Vec2(), true);
			m_particles.push_back(particleA);
		}
		else
		{
			particleA = particleB;
		}

		particleB = new Particle2D(massPerPoint, inverseMassPerPoint, start +
			(direction * (particleIndex * lengthPerSpring)),
			Vec2());
		m_particles.push_back(particleB);
		/*if (particleIndex == totalPoints - 1)
		{
			particleB->m_isLocked = true;
		}*/
		Spring2D* spring = new Spring2D(particleA, particleB, m_stiffnessConstant, 
			(particleA->m_position - particleB->m_position).GetLength());
		m_springs.push_back(spring);		
	}
}

//-----------------------------------------------------------------------------------------------
MassSpringRope2D::MassSpringRope2D()
{

}

//-----------------------------------------------------------------------------------------------
MassSpringRope2D::~MassSpringRope2D()
{
}

//-----------------------------------------------------------------------------------------------
void MassSpringRope2D::Startup()
{
}

//-----------------------------------------------------------------------------------------------
void MassSpringRope2D::Shutdown()
{
}

//-----------------------------------------------------------------------------------------------
void MassSpringRope2D::Update(float deltaSeconds)
{
	//float startTime = float(GetCurrentTimeSeconds());
	m_physicsDebt += deltaSeconds;
	while (m_physicsDebt > m_physicsTimestep)
	{
		if (m_integrationMethod == IntegrationMethod::EXPLICIT_EULER)
		{
			UpdateExplicitEuler();
		}
		else if (m_integrationMethod == IntegrationMethod::SEMI_IMPLICIT_EULER)
		{
			UpdateSemiImplicitEuler();
		}
		else if (m_integrationMethod == IntegrationMethod::MIDPOINT)
		{
			UpdateMidparticle();
		}
		else if (m_integrationMethod == IntegrationMethod::RUNGE_KUTTA)
		{
			UpdateRungeKutta();
		}
		else if (m_integrationMethod == IntegrationMethod::VERLET)
		{
			UpdateVerlet();
		}
		m_physicsDebt -= m_physicsTimestep;
	}
	//float endTime = float(GetCurrentTimeSeconds());
}

//-----------------------------------------------------------------------------------------------
void MassSpringRope2D::Render(std::vector<Vertex_PCU>& verts) const
{
	verts.reserve(m_springs.size() * size_t(6));
	for (int springIndex = 0; springIndex < m_springs.size(); springIndex++)
	{
		Spring2D*const spring = m_springs[springIndex];
		if (spring != nullptr)
		{
			LineSegment2 line;
			line.m_start = spring->m_particleA->m_position;
			line.m_end = spring->m_particleB->m_position;
			Capsule2 capsule;
			capsule.m_bone = line;
			capsule.m_radius = m_ropeRadius;
			AddVertsForCapsule2D(verts, capsule, Rgba8(142, 89, 60, 255));
			if (m_isDebugMode)
			{
				AddVertsForDisc2D(verts, spring->m_particleA->m_position, m_ropeRadius * 3.0f, Rgba8::WHITE);
				AddVertsForDisc2D(verts, spring->m_particleB->m_position, m_ropeRadius * 3.0f, Rgba8::WHITE);
			}
		}
	}
}

//-----------------------------------------------------------------------------------------------
void MassSpringRope2D::UpdateExplicitEuler()
{
	for (int springIndex = 0; springIndex < m_springs.size(); springIndex++)
	{
		Spring2D*& spring = m_springs[springIndex];
		if (spring != nullptr)
		{
			//Calculate forces using hookes law
			Vec2 displacement = spring->m_particleB->m_position - spring->m_particleA->m_position;
			float magnitude = displacement.GetLength();
			Vec2 direction = displacement.GetNormalized();
			float length = magnitude - spring->m_initialLength;
			Vec2 springForceA = spring->m_stiffness * direction * length;
			Vec2 springForceB = springForceA * -1.0f;
			Vec2 dampingForce = (spring->m_particleB->m_velocity - spring->m_particleA->m_velocity) * m_dampingConstant;

			spring->m_particleA->m_forces += (springForceA + dampingForce);
			spring->m_particleB->m_forces += (springForceB - dampingForce);
		}
	}

	for (int particleIndex = 0; particleIndex < m_particles.size(); particleIndex++)
	{
		Particle2D*& particle = m_particles[particleIndex];
		if (particle != nullptr)
		{
			if (particle->m_isLocked == false)
			{
				//Apply Forces Through Explicit Euler Integration
				Vec2 acceleration = (particle->m_forces + Vec2(0.0f, -m_gravityConstant));
				particle->m_position += particle->m_velocity * m_physicsTimestep;
				particle->m_velocity +=  acceleration * m_physicsTimestep;
			}

			particle->m_forces = Vec2();
		}
	}
}

//-----------------------------------------------------------------------------------------------
void MassSpringRope2D::UpdateSemiImplicitEuler()
{
	for (int springIndex = 0; springIndex < m_springs.size(); springIndex++)
	{
		Spring2D*& spring = m_springs[springIndex];
		if (spring != nullptr)
		{
			//Calculate forces using hookes law
			Vec2 displacement = spring->m_particleB->m_position - spring->m_particleA->m_position;
			float magnitude = displacement.GetLength();
			Vec2 direction = displacement.GetNormalized();
			float length = magnitude - spring->m_initialLength;
			Vec2 springForceA = spring->m_stiffness * direction * length;
			Vec2 springForceB = springForceA * -1.0f;
			Vec2 dampingForce = (spring->m_particleB->m_velocity - spring->m_particleA->m_velocity) * m_dampingConstant;

			spring->m_particleA->m_forces += (springForceA + dampingForce);
			spring->m_particleB->m_forces += (springForceB - dampingForce);
		}
	}

	for (int particleIndex = 0; particleIndex < m_particles.size(); particleIndex++)
	{
		Particle2D*& particle = m_particles[particleIndex];
		if (particle != nullptr)
		{
			if (particle->m_isLocked == false)
			{
				//Apply Forces Through Semi-Implicit Euler Integration
				Vec2 acceleration = (particle->m_forces * particle->m_inverseMass + Vec2(0.0f, -m_gravityConstant));
				particle->m_velocity += acceleration * m_physicsTimestep;
				particle->m_position += particle->m_velocity * m_physicsTimestep;
			}

			particle->m_forces = Vec2();
		}
	}
}

//-----------------------------------------------------------------------------------------------
void MassSpringRope2D::UpdateMidparticle()
{
	for (int springIndex = 0; springIndex < m_springs.size(); springIndex++)
	{
		Spring2D*& spring = m_springs[springIndex];
		if (spring != nullptr)
		{
			//Calculate forces using hookes law
			Vec2 displacement = spring->m_particleB->m_position - spring->m_particleA->m_position;
			float magnitude = displacement.GetLength();
			Vec2 direction = displacement.GetNormalized();
			float length = magnitude - spring->m_initialLength;
			Vec2 springForceA = spring->m_stiffness * direction * length;
			Vec2 springForceB = springForceA * -1.0f;
			Vec2 dampingForce = (spring->m_particleB->m_velocity - spring->m_particleA->m_velocity) * m_dampingConstant;

			spring->m_particleA->m_forces += (springForceA + dampingForce);
			spring->m_particleB->m_forces += (springForceB - dampingForce);
		}
	}

	for (int particleIndex = 0; particleIndex < m_particles.size(); particleIndex++)
	{
		Particle2D*& particle = m_particles[particleIndex];
		if (particle != nullptr)
		{
			if (particle->m_isLocked == false)
			{
				Vec2 acceleration = (particle->m_forces * particle->m_inverseMass + Vec2(0.0f, -m_gravityConstant));
				Vec2 originalVelocity = particle->m_velocity;
				particle->m_velocity += acceleration * m_physicsTimestep;
				particle->m_position += 0.5f * (particle->m_velocity + originalVelocity) * m_physicsTimestep;
			}

			particle->m_forces = Vec2();
		}
	}
}

//-----------------------------------------------------------------------------------------------
void MassSpringRope2D::UpdateRungeKutta()
{
	for (int springIndex = 0; springIndex < m_springs.size(); springIndex++)
	{
		Spring2D*& spring = m_springs[springIndex];
		if (spring != nullptr)
		{
			//Calculate forces using hookes law
			Vec2 displacement = spring->m_particleB->m_position - spring->m_particleA->m_position;
			float magnitude = displacement.GetLength();
			Vec2 direction = displacement.GetNormalized();
			float length = magnitude - spring->m_initialLength;
			Vec2 springForceA = spring->m_stiffness * direction * length;
			Vec2 springForceB = springForceA * -1.0f;
			Vec2 dampingForce = (spring->m_particleB->m_velocity - spring->m_particleA->m_velocity) * m_dampingConstant;

			spring->m_particleA->m_forces += (springForceA + dampingForce);
			spring->m_particleB->m_forces += (springForceB - dampingForce);
		}
	}

	for (int particleIndex = 0; particleIndex < m_particles.size(); particleIndex++)
	{
		Particle2D*& particle = m_particles[particleIndex];
		if (particle != nullptr)
		{
			if (particle->m_isLocked == false)
			{
				Vec2 acceleration = (particle->m_forces * particle->m_inverseMass + Vec2(0.0f, -m_gravityConstant));
				Vec2 k1 = particle->m_velocity + acceleration * m_physicsTimestep;
				Vec2 k2 = 0.5f * (particle->m_velocity + k1);
				Vec2 k3 = 0.5f * (particle->m_velocity + k2);
				Vec2 k4 = particle->m_velocity + k3;
				particle->m_position += ((k1 + 2 * k2 + 2 * k3 + k4) / 6.0f) * m_physicsTimestep;
				particle->m_velocity = k1;
			}

			particle->m_forces = Vec2();
		}
	}
}

//-----------------------------------------------------------------------------------------------
void MassSpringRope2D::UpdateVerlet()
{
	for (int particleIndex = 0; particleIndex < m_particles.size(); particleIndex++)
	{
		Particle2D*& particle = m_particles[particleIndex];
		if (particle != nullptr)
		{
			if (particle->m_isLocked == false)
			{
				//Position manipulation only Verlet integration
				Vec2 acceleration = (Vec2(0.0f, -m_gravityConstant));
				Vec2 tempPosition = particle->m_position;
				particle->m_position += (particle->m_position - particle->m_previousPosition) * (1.0f - m_dampingConstant) +
					(acceleration * (m_physicsTimestep * m_physicsTimestep));
				particle->m_previousPosition = tempPosition;
			}

			particle->m_forces = Vec2();
		}
	}

	
	for(int sovlerIndex = 0; sovlerIndex < m_verletSolverIterations; sovlerIndex++)
	{
		for (int springIndex = 0; springIndex < m_springs.size(); springIndex++)
		{
			Spring2D*& spring = m_springs[springIndex];
			if (spring != nullptr)
			{
				//Calculates direction and overflow along with weight constants
				Vec2 displacement = spring->m_particleB->m_position - spring->m_particleA->m_position;
				float distanceConstraint = displacement.GetLength() - spring->m_initialLength;
				Vec2 direction = displacement.GetNormalized();

				if (spring->m_particleA->m_isLocked == false && spring->m_particleB->m_isLocked == false)
				{
					float particleAWeightConstant = (spring->m_particleA->m_inverseMass / (spring->m_particleA->m_inverseMass + spring->m_particleB->m_inverseMass));
					float particleBWeightConstant = (spring->m_particleB->m_inverseMass / (spring->m_particleA->m_inverseMass + spring->m_particleB->m_inverseMass));

					Vec2 deltaPointA = particleAWeightConstant * distanceConstraint * direction;
					Vec2 deltaPointB = particleBWeightConstant * distanceConstraint * direction;

					spring->m_particleA->m_position += deltaPointA;
					spring->m_particleB->m_position -= deltaPointB;
				}
				else if (spring->m_particleA->m_isLocked == true && spring->m_particleB->m_isLocked == false)
				{
					Vec2 deltaPointB = distanceConstraint * direction;
					spring->m_particleB->m_position -= deltaPointB;
				}
				else if (spring->m_particleA->m_isLocked == false && spring->m_particleB->m_isLocked == true)
				{
					Vec2 deltaPointA = distanceConstraint * direction;
					spring->m_particleA->m_position += deltaPointA;
				}
			}
		}
	}

}
