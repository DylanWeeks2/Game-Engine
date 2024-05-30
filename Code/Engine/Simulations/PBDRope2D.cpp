#include "PBDRope2D.hpp"
#include "Engine/Core/Vertex_PCU.hpp"
#include "Engine/Core/VertexUtils.hpp"
#include "Engine/Math/LineSegment2.hpp"
#include "Engine/Core/ErrorWarningAssert.hpp"
#include "Engine/Simulations/Spring2D.hpp"
#include "Engine/Simulations/Point2D.hpp"
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
				(direction * ((pointIndex - 1) * m_desiredDistance)), Vec2(), true);
			m_points.push_back(pointA);
		}
		else
		{
			pointA = pointB;
		}

		pointB = new Point2D(massPerPoint, inverseMassPerPoint, start +
			(direction * (pointIndex * m_desiredDistance)), Vec2());
		m_points.push_back(pointB);

		Capsule2 capsule;
		capsule.m_bone = LineSegment2(pointA->m_proposedPosition, pointB->m_proposedPosition);
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
	for (int pointIndex = 0; pointIndex < m_points.size(); pointIndex++)
	{
		Point2D* point = m_points[pointIndex];
		if (point)
		{
			delete point;
			point = nullptr;
		}
	}

	for (int pointIndex = 0; pointIndex < m_points.size(); pointIndex++)
	{
		Point2D* point = m_points[pointIndex];
		if (point)
		{
			delete point;
			point = nullptr;
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
		for (int pointIndex = 0; pointIndex < m_points.size(); pointIndex++)
		{
			Point2D* currentPoint = m_points[pointIndex];
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
		for (int pointIndex = 0; pointIndex < m_points.size(); pointIndex++)
		{
			Point2D* currentPoint = m_points[pointIndex];
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
	verts.reserve((m_points.size() - 1) * size_t(6));
	
	std::vector<Vec2> positions;
	positions.reserve(m_points.size());
	for (int pointIndex = 0; pointIndex < m_points.size(); pointIndex++)
	{
		positions.push_back(m_points[pointIndex]->m_position);
		/*if (pointIndex == 0)
		{
			continue;
		}*/

		/*Point2D* previousPoint = m_points[pointIndex - 1];
		Point2D* currentPoint = m_points[pointIndex];
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
	for (int pointIndex = 0; pointIndex < m_points.size(); pointIndex++)
	{
		Point2D* pointA = nullptr;
		Point2D* pointB = m_points[pointIndex];

		if (pointIndex != 0)
		{
			pointA = m_points[pointIndex - 1];
		}

		if (pointA)
		{
			currentLength += (pointA->m_position - pointB->m_position).GetLength();
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
	/*Point2D* pointStart = m_points[0];
	Point2D* pointPreviousToEnd = m_points[m_points.size() - 2];
	Point2D* pointEnd = m_points[m_points.size() - 1];*/

	//Loop to project each constraint on each point
	for (int pointIndex = 0; pointIndex < m_points.size(); pointIndex++)
	{
		Point2D* pointA = nullptr;
		Point2D* pointB = m_points[pointIndex];
		Point2D* pointC = nullptr;
		
		if (pointIndex != 0)
		{
			pointA = m_points[pointIndex - 1];
		}
		if (pointIndex != int(m_points.size() - 1))
		{
			pointC = m_points[pointIndex + 1];
		}

		//Constraints Projection
		if (pointB->m_isLocked == false)
		{
			ProjectCollisionConstraints(pointB);
		}
		if (pointA)
		{
			ProjectDistanceConstraint(pointA, pointB);
		}
		if (pointA && pointC)
		{
			ProjectBendingConstraint(pointA, pointC);
		}
	}
}

//-----------------------------------------------------------------------------------------------
void PBDRope2D::ProjectDistanceConstraint(Point2D* pointA, Point2D* pointB)
{
	//Calculates direction and overflow along with weight Coefficients
	Vec2 displacement = pointB->m_proposedPosition - pointA->m_proposedPosition;
	float distanceConstraint = (displacement.GetLength() - m_desiredDistance);

	if (distanceConstraint == 0)
	{
		return;
	}

	Vec2 gradient = displacement.GetNormalized();

	if (pointA->m_isLocked == false && pointB->m_isLocked == false)
	{
		float pointAWeightCoefficient = (pointA->m_inverseMass / (pointA->m_inverseMass + pointB->m_inverseMass));
		float pointBWeightCoefficient = (pointB->m_inverseMass / (pointA->m_inverseMass + pointB->m_inverseMass));

		Vec2 deltaPointA = pointAWeightCoefficient * distanceConstraint * gradient;
		Vec2 deltaPointB = pointBWeightCoefficient * distanceConstraint * gradient;

		pointA->m_proposedPosition += deltaPointA * m_stretchingCoefficient;
		pointB->m_proposedPosition -= deltaPointB * m_stretchingCoefficient;
	}
	else if (pointA->m_isLocked == true && pointB->m_isLocked == false)
	{
		Vec2 deltaPointB = distanceConstraint * gradient;
		pointB->m_proposedPosition -= deltaPointB * m_stretchingCoefficient;
	}
	else if (pointA->m_isLocked == false && pointB->m_isLocked == true)
	{
		Vec2 deltaPointA = distanceConstraint * gradient;
		pointA->m_proposedPosition += deltaPointA * m_stretchingCoefficient;
	}
}

//-----------------------------------------------------------------------------------------------
void PBDRope2D::ProjectBendingConstraint(Point2D* pointA, Point2D* pointC)
{
	//Calculates direction and overflow along with weight Coefficients
	Vec2 displacement = pointC->m_proposedPosition - pointA->m_proposedPosition;
	float distanceConstraint = displacement.GetLength() - m_bendingConstraintDistance;
	
	//Check if less than the minimum distance, if so no need to constrain
	if (distanceConstraint > 0.0f)
	{
		return;
	}

	Vec2 gradient = displacement.GetNormalized();

	if (pointA->m_isLocked == false && pointC->m_isLocked == false)
	{
		float pointAWeightCoefficient = (pointA->m_inverseMass / (pointA->m_inverseMass + pointC->m_inverseMass));
		float pointCWeightCoefficient = (pointC->m_inverseMass / (pointA->m_inverseMass + pointC->m_inverseMass));

		Vec2 deltaPointA = m_bendingCoefficient * pointAWeightCoefficient * distanceConstraint * gradient;
		Vec2 deltaPointC = m_bendingCoefficient * pointCWeightCoefficient * distanceConstraint * gradient;

		pointA->m_proposedPosition += deltaPointA;
		pointC->m_proposedPosition -= deltaPointC;
	}
	else if (pointA->m_isLocked == true && pointC->m_isLocked == false)
	{
		Vec2 deltaPointC = m_bendingCoefficient * distanceConstraint * gradient;
		pointC->m_proposedPosition -= deltaPointC;
	}
	else if (pointA->m_isLocked == false && pointC->m_isLocked == true)
	{
		Vec2 deltaPointA = m_bendingCoefficient * distanceConstraint * gradient;
		pointA->m_proposedPosition += deltaPointA;
	}
}

//-----------------------------------------------------------------------------------------------
void PBDRope2D::ProjectCollisionConstraints(Point2D* point)
{
	//Floor Collisions
	if (point->m_proposedPosition.y < m_ropeRadius)
	{
		point->m_proposedPosition.y = m_ropeRadius;

		if (point->m_velocity != Vec2())
		{
			float normalForce = point->m_mass * m_gravityCoefficient;
			point->m_forcesAsFloat += m_kineticFrictionCoefficient * normalForce;
		}
	}

	//Discs Collisions
	for (int discIndex = 0; discIndex < m_shapes->m_discs.size(); discIndex++)
	{
		Disc2& disc = m_shapes->m_discs[discIndex];
		if (PushDiscOutOfFixedDisc2D(point->m_proposedPosition, m_ropeRadius, disc.m_center, disc.m_radius))
		{
			if (point->m_velocity != Vec2())
			{
				float normalForce = point->m_mass * m_gravityCoefficient;
				point->m_forcesAsFloat += m_kineticFrictionCoefficient * normalForce;
			}
		}
	}

	//AABB Collisions
	for (int aabbIndex = 0; aabbIndex < m_shapes->m_aabbs.size(); aabbIndex++)
	{
		AABB2& aabb = m_shapes->m_aabbs[aabbIndex];
		if (PushDiscOutOfFixedAABB2D(point->m_proposedPosition, m_ropeRadius, aabb))
		{
			if (point->m_velocity != Vec2())
			{
				float normalForce = point->m_mass * m_gravityCoefficient;
				point->m_forcesAsFloat += m_kineticFrictionCoefficient * normalForce;
			}
		}
	}

	//OBB Collisions
	for (int obbIndex = 0; obbIndex < m_shapes->m_obbs.size(); obbIndex++)
	{
		OBB2& obb = m_shapes->m_obbs[obbIndex];
		if (PushDiscOutOfFixedOBB2D(point->m_proposedPosition, m_ropeRadius, obb))
		{
			if (point->m_velocity != Vec2())
			{
				float normalForce = point->m_mass * m_gravityCoefficient;
				point->m_forcesAsFloat += m_kineticFrictionCoefficient * normalForce;
			}
		}
	}

	//Capsule Collisions
	for (int capsuleIndex = 0; capsuleIndex < m_shapes->m_capsules.size(); capsuleIndex++)
	{
		Capsule2& capsule = m_shapes->m_capsules[capsuleIndex];
		if (PushDiscOutOfFixedCapsule2D(point->m_proposedPosition, m_ropeRadius, capsule))
		{
			if (point->m_velocity != Vec2())
			{
				float normalForce = point->m_mass * m_gravityCoefficient;
				point->m_forcesAsFloat += m_kineticFrictionCoefficient * normalForce;
			}
		}
	}

	//SELF COLLISIONS
	/*int capsuleIndex = 0;
	for (int pointIndex = 0; pointIndex < m_numberOfPointsInRope; pointIndex++)
	{
		if (pointIndex == 0)
		{
			continue;
		}

		Point2D* pointA = m_points[pointIndex - 1];
		Point2D* pointB = m_points[pointIndex];

		if (point == pointA || point == pointB)
		{
			continue;
		}

		Capsule2& capsule = m_selfCollisionCapsules[capsuleIndex];
		capsule.m_bone.m_start = pointA->m_proposedPosition;
		capsule.m_bone.m_end = pointB->m_proposedPosition;
		PushDiscOutOfMobileCapsule2D(point->m_proposedPosition, m_ropeRadius, point->m_inverseMass, capsule);
		pointA->m_proposedPosition = capsule.m_bone.m_start;
		pointB->m_proposedPosition = capsule.m_bone.m_end;
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
