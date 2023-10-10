#include "MassSpringRope2D.hpp"
#include "Engine/Core/Vertex_PCU.hpp"
#include "Engine/Core/VertexUtils.hpp"
#include "Engine/Math/LineSegment2.hpp"
#include "Engine/Core/ErrorWarningAssert.hpp"
#include "Engine/Simulations/Spring2D.hpp"
#include "Engine/Simulations/Point2D.hpp"
#include "Engine/Core/EngineCommon.hpp"
#include "Engine/Core/Time.hpp"


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
	Point2D* pointA = nullptr;
	Point2D* pointB = nullptr;
	for (int pointIndex = 0; pointIndex < totalPoints; pointIndex++)
	{
		if (pointIndex == 0)
		{
			continue;
		}
		else if (pointIndex == 1)
		{
			pointA = new Point2D(massPerPoint, inverseMassPerPoint, start +
				(direction * ((pointIndex - 1) * lengthPerSpring)),
				Vec2(), true);
			m_points.push_back(pointA);
		}
		else
		{
			pointA = pointB;
		}

		pointB = new Point2D(massPerPoint, inverseMassPerPoint, start +
			(direction * (pointIndex * lengthPerSpring)),
			Vec2());
		m_points.push_back(pointB);
		/*if (pointIndex == totalPoints - 1)
		{
			pointB->m_isLocked = true;
		}*/
		Spring2D* spring = new Spring2D(pointA, pointB, m_stiffnessConstant, 
			(pointA->m_position - pointB->m_position).GetLength());
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
			UpdateMidpoint();
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
			line.m_start = spring->m_pointA->m_position;
			line.m_end = spring->m_pointB->m_position;
			AddVertsForLineSegment2D(verts, line, 0.05f, Rgba8(142, 89, 60, 255));
			//AddVertsForDisc2D(verts, spring->m_pointA->m_position, 0.025f, Rgba8(142, 89, 60, 255));
			//AddVertsForDisc2D(verts, spring->m_pointB->m_position, 0.025f, Rgba8(142, 89, 60, 255));
			if (m_isDebugMode)
			{
				AddVertsForDisc2D(verts, spring->m_pointA->m_position, 0.125f, Rgba8::WHITE);
				AddVertsForDisc2D(verts, spring->m_pointB->m_position, 0.125f, Rgba8::WHITE);
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
			Vec2 displacement = spring->m_pointB->m_position - spring->m_pointA->m_position;
			float magnitude = displacement.GetLength();
			Vec2 direction = displacement.GetNormalized();
			float length = magnitude - spring->m_initialLength;
			Vec2 springForceA = spring->m_stiffness * direction * length;
			Vec2 springForceB = springForceA * -1.0f;
			Vec2 dampingForce = (spring->m_pointB->m_velocity - spring->m_pointA->m_velocity) * m_dampingConstant;

			spring->m_pointA->m_forces += (springForceA + dampingForce);
			spring->m_pointB->m_forces += (springForceB - dampingForce);
		}
	}

	for (int pointIndex = 0; pointIndex < m_points.size(); pointIndex++)
	{
		Point2D*& point = m_points[pointIndex];
		if (point != nullptr)
		{
			if (point->m_isLocked == false)
			{
				//Apply Forces Through Explicit Euler Integration
				Vec2 acceleration = (point->m_forces + Vec2(0.0f, -m_gravityConstant));
				point->m_position += point->m_velocity * m_physicsTimestep;
				point->m_velocity +=  acceleration * m_physicsTimestep;
			}

			point->m_forces = Vec2();
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
			Vec2 displacement = spring->m_pointB->m_position - spring->m_pointA->m_position;
			float magnitude = displacement.GetLength();
			Vec2 direction = displacement.GetNormalized();
			float length = magnitude - spring->m_initialLength;
			Vec2 springForceA = spring->m_stiffness * direction * length;
			Vec2 springForceB = springForceA * -1.0f;
			Vec2 dampingForce = (spring->m_pointB->m_velocity - spring->m_pointA->m_velocity) * m_dampingConstant;

			spring->m_pointA->m_forces += (springForceA + dampingForce);
			spring->m_pointB->m_forces += (springForceB - dampingForce);
		}
	}

	for (int pointIndex = 0; pointIndex < m_points.size(); pointIndex++)
	{
		Point2D*& point = m_points[pointIndex];
		if (point != nullptr)
		{
			if (point->m_isLocked == false)
			{
				//Apply Forces Through Semi-Implicit Euler Integration
				Vec2 acceleration = (point->m_forces * point->m_inverseMass + Vec2(0.0f, -m_gravityConstant));
				point->m_velocity += acceleration * m_physicsTimestep;
				point->m_position += point->m_velocity * m_physicsTimestep;
			}

			point->m_forces = Vec2();
		}
	}
}

//-----------------------------------------------------------------------------------------------
void MassSpringRope2D::UpdateMidpoint()
{
	for (int springIndex = 0; springIndex < m_springs.size(); springIndex++)
	{
		Spring2D*& spring = m_springs[springIndex];
		if (spring != nullptr)
		{
			//Calculate forces using hookes law
			Vec2 displacement = spring->m_pointB->m_position - spring->m_pointA->m_position;
			float magnitude = displacement.GetLength();
			Vec2 direction = displacement.GetNormalized();
			float length = magnitude - spring->m_initialLength;
			Vec2 springForceA = spring->m_stiffness * direction * length;
			Vec2 springForceB = springForceA * -1.0f;
			Vec2 dampingForce = (spring->m_pointB->m_velocity - spring->m_pointA->m_velocity) * m_dampingConstant;

			spring->m_pointA->m_forces += (springForceA + dampingForce);
			spring->m_pointB->m_forces += (springForceB - dampingForce);
		}
	}

	for (int pointIndex = 0; pointIndex < m_points.size(); pointIndex++)
	{
		Point2D*& point = m_points[pointIndex];
		if (point != nullptr)
		{
			if (point->m_isLocked == false)
			{
				Vec2 acceleration = (point->m_forces * point->m_inverseMass + Vec2(0.0f, -m_gravityConstant));
				Vec2 originalVelocity = point->m_velocity;
				point->m_velocity += acceleration * m_physicsTimestep;
				point->m_position += 0.5f * (point->m_velocity + originalVelocity) * m_physicsTimestep;
			}

			point->m_forces = Vec2();
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
			Vec2 displacement = spring->m_pointB->m_position - spring->m_pointA->m_position;
			float magnitude = displacement.GetLength();
			Vec2 direction = displacement.GetNormalized();
			float length = magnitude - spring->m_initialLength;
			Vec2 springForceA = spring->m_stiffness * direction * length;
			Vec2 springForceB = springForceA * -1.0f;
			Vec2 dampingForce = (spring->m_pointB->m_velocity - spring->m_pointA->m_velocity) * m_dampingConstant;

			spring->m_pointA->m_forces += (springForceA + dampingForce);
			spring->m_pointB->m_forces += (springForceB - dampingForce);
		}
	}

	for (int pointIndex = 0; pointIndex < m_points.size(); pointIndex++)
	{
		Point2D*& point = m_points[pointIndex];
		if (point != nullptr)
		{
			if (point->m_isLocked == false)
			{
				Vec2 acceleration = (point->m_forces * point->m_inverseMass + Vec2(0.0f, -m_gravityConstant));
				Vec2 k1 = point->m_velocity + acceleration * m_physicsTimestep;
				Vec2 k2 = 0.5f * (point->m_velocity + k1);
				Vec2 k3 = 0.5f * (point->m_velocity + k2);
				Vec2 k4 = point->m_velocity + k3;
				point->m_position += ((k1 + 2 * k2 + 2 * k3 + k4) / 6.0f) * m_physicsTimestep;
				point->m_velocity = k1;
			}

			point->m_forces = Vec2();
		}
	}
}

//-----------------------------------------------------------------------------------------------
void MassSpringRope2D::UpdateVerlet()
{
	for (int pointIndex = 0; pointIndex < m_points.size(); pointIndex++)
	{
		Point2D*& point = m_points[pointIndex];
		if (point != nullptr)
		{
			if (point->m_isLocked == false)
			{
				//Position manipulation only Verlet integration
				Vec2 acceleration = (Vec2(0.0f, -m_gravityConstant));
				Vec2 tempPosition = point->m_position;
				point->m_position += (point->m_position - point->m_previousPosition) * (1.0f - m_dampingConstant) +
					(acceleration * (m_physicsTimestep * m_physicsTimestep));
				point->m_previousPosition = tempPosition;
			}

			point->m_forces = Vec2();
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
				Vec2 displacement = spring->m_pointB->m_position - spring->m_pointA->m_position;
				float distanceConstraint = displacement.GetLength() - spring->m_initialLength;
				Vec2 direction = displacement.GetNormalized();

				if (spring->m_pointA->m_isLocked == false && spring->m_pointB->m_isLocked == false)
				{
					float pointAWeightConstant = (spring->m_pointA->m_inverseMass / (spring->m_pointA->m_inverseMass + spring->m_pointB->m_inverseMass));
					float pointBWeightConstant = (spring->m_pointB->m_inverseMass / (spring->m_pointA->m_inverseMass + spring->m_pointB->m_inverseMass));

					Vec2 deltaPointA = pointAWeightConstant * distanceConstraint * direction;
					Vec2 deltaPointB = pointBWeightConstant * distanceConstraint * direction;

					spring->m_pointA->m_position += deltaPointA;
					spring->m_pointB->m_position -= deltaPointB;
				}
				else if (spring->m_pointA->m_isLocked == true && spring->m_pointB->m_isLocked == false)
				{
					Vec2 deltaPointB = distanceConstraint * direction;
					spring->m_pointB->m_position -= deltaPointB;
				}
				else if (spring->m_pointA->m_isLocked == false && spring->m_pointB->m_isLocked == true)
				{
					Vec2 deltaPointA = distanceConstraint * direction;
					spring->m_pointA->m_position += deltaPointA;
				}
			}
		}
	}

}
