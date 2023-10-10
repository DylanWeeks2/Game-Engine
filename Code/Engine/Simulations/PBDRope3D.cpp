#include "PBDRope3D.hpp"
#include "Engine/Core/Vertex_PCU.hpp"
#include "Engine/Core/VertexUtils.hpp"
#include "Engine/Core/ErrorWarningAssert.hpp"
#include "Engine/Simulations/Point3D.hpp"
#include "Engine/Core/EngineCommon.hpp"
#include "Engine/Core/Time.hpp"
#include "Engine/Math/MathUtils.hpp"
#include "Engine/Math/LineSegment3.hpp"
#include "Engine/Math/Mat44.hpp"

//-----------------------------------------------------------------------------------------------
PBDRope3D::PBDRope3D(int totalPoints, float totalMassOfRope, float dampingCoefficient, float stretchCoefficient,
	float compressionCoefficient, float bendingCoefficient, float staticFrictionCoefficient, float kineticFrictionCoefficient,
	int totalSolverIterations, Vec3 start, Vec3 end, float physicsTimestep, bool isDebugMode)
	: m_numberOfPointsInRope(totalPoints)
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
	, m_isDebugMode(isDebugMode)
	, m_physicsTimestep(physicsTimestep)
{
	Vec3 displacement = end - start;
	Vec3 direction = displacement.GetNormalized();
	float magnitude = displacement.GetLength();
	m_desiredDistance = magnitude / (totalPoints - 1);
	m_bendingConstraintDistance = m_desiredDistance * (2.0f);
	float massPerPoint = (totalMassOfRope / totalPoints);
	float inverseMassPerPoint = 1.0f / massPerPoint;
	Point3D* pointA = nullptr;
	Point3D* pointB = nullptr;
	Point3D* lockedPoint = nullptr;
	for (int pointIndex = 0; pointIndex < totalPoints; pointIndex++)
	{
		if (pointIndex == 0)
		{
			continue;
		}
		else if (pointIndex == 1)
		{
			pointA = new Point3D(massPerPoint, inverseMassPerPoint, start +
				(direction * ((pointIndex - 1) * m_desiredDistance)), Vec3(), true);
			lockedPoint = pointA;
			m_points.push_back(pointA);
		}
		else
		{
			pointA = pointB;
		}

		pointB = new Point3D(massPerPoint, inverseMassPerPoint, start +
			(direction * (pointIndex * m_desiredDistance)), Vec3());
		m_points.push_back(pointB);

		/*Capsule2 capsule;
		capsule.m_bone = LineSegment2(pointA->m_proposedPosition, pointB->m_proposedPosition);
		capsule.m_radius = m_ropeRadius;
		capsule.m_inverseMass = 1.0f / (massPerPoint + massPerPoint);
		m_selfCollisionCapsules.push_back(capsule);*/
	}

	m_shapes = new Shapes3D();
}

//-----------------------------------------------------------------------------------------------
PBDRope3D::PBDRope3D()
{
}

//-----------------------------------------------------------------------------------------------
PBDRope3D::~PBDRope3D()
{
	for (int pointIndex = 0; pointIndex < m_points.size(); pointIndex++)
	{
		Point3D* point = m_points[pointIndex];
		if (point)
		{
			delete point;
			point = nullptr;
		}
	}
}

//-----------------------------------------------------------------------------------------------
void PBDRope3D::Startup()
{
}

//-----------------------------------------------------------------------------------------------
void PBDRope3D::Shutdown()
{
}

//-----------------------------------------------------------------------------------------------
void PBDRope3D::Update(float deltaSeconds)
{
	//float startTime = float(GetCurrentTimeSeconds());
	m_physicsDebt += deltaSeconds;
	while (m_physicsDebt > m_physicsTimestep)
	{
		//Loop to estimate new velocities, proposed positions, and generate collisions
		for (int pointIndex = 0; pointIndex < m_points.size(); pointIndex++)
		{
			Point3D* currentPoint = m_points[pointIndex];
			if (currentPoint->m_isLocked == true || currentPoint->m_isGrabbed == true)
			{
				continue;
			}

			//Calculate next velocity (semi-implicit Euler)
			Vec3 acceleration = (Vec3(0.0f, 0.0f, -m_gravityCoefficient));
			Vec3 velocity = currentPoint->m_velocity;
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
			Point3D* currentPoint = m_points[pointIndex];
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
void PBDRope3D::Render(std::vector<Vertex_PCU>& verts) const
{
	verts.reserve((m_points.size() - 1) * size_t(6));
	for (int pointIndex = 0; pointIndex < m_points.size(); pointIndex++)
	{
		if (pointIndex == 0)
		{
			continue;
		}

		Point3D* previousPoint = m_points[pointIndex - 1];
		Point3D* currentPoint = m_points[pointIndex];
		std::vector<Vertex_PCU> lineVerts;
		AddVertsForLineSegment3D(lineVerts, previousPoint->m_position, currentPoint->m_position, m_ropeRadius * 2.0f, Rgba8(142, 89, 60, 255));
		for (int index = 0; index < lineVerts.size(); index++)
		{
			verts.push_back(lineVerts[index]);
		}

		if (m_isDebugMode)
		{
			AddVertsForSphere3D(verts, previousPoint->m_position, m_ropeRadius * 3.0f, Rgba8::WHITE);
			AddVertsForSphere3D(verts, currentPoint->m_position, m_ropeRadius * 3.0f, Rgba8::WHITE);
		}
	}
}

//-----------------------------------------------------------------------------------------------
float PBDRope3D::GetCurrentLengthOfTheRope()
{
	float currentLength = 0.0f;
	for (int pointIndex = 0; pointIndex < m_points.size(); pointIndex++)
	{
		Point3D* pointA = nullptr;
		Point3D* pointB = m_points[pointIndex];

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
void PBDRope3D::ClearShapeReferences()
{
	m_shapes->m_spheres.clear();
	m_shapes->m_capsules.clear();
	m_shapes->m_aabbs.clear();
	m_shapes->m_obbs.clear();
}

//-----------------------------------------------------------------------------------------------
Mat44 PBDRope3D::GetModelMatrix() const
{
	Mat44 modelMatrix;
	modelMatrix = m_orientation.GetAsMatrix_XFwd_YLeft_ZUp();
	modelMatrix.SetTranslation3D(m_points[0]->m_position);
	return modelMatrix;
}

//-----------------------------------------------------------------------------------------------
void PBDRope3D::ProjectConstraints()
{
	//Points to reference for ease of access
	/*Point2D* pointStart = m_points[0];
	Point2D* pointPreviousToEnd = m_points[m_points.size() - 2];
	Point2D* pointEnd = m_points[m_points.size() - 1];*/
	
	//Loop to project each constraint on each point
	for (int pointIndex = 0; pointIndex < m_points.size(); pointIndex++)
	{
		Point3D* pointA = nullptr;
		Point3D* pointB = m_points[pointIndex];
		Point3D* pointC = nullptr;

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
			ProjectBendingConstraint(pointA, pointB, pointC);
		}
	}
}

//-----------------------------------------------------------------------------------------------
void PBDRope3D::ProjectDistanceConstraint(Point3D* pointA, Point3D* pointB)
{
	//Calculates direction and overflow along with weight Coefficients
	Vec3 displacement = pointB->m_proposedPosition - pointA->m_proposedPosition;
	float distanceConstraint = (displacement.GetLength() - m_desiredDistance);

	if (distanceConstraint == 0)
	{
		return;
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

	if (pointA->m_isLocked == false && pointB->m_isLocked == false)
	{
		float pointAWeightCoefficient = (pointA->m_inverseMass / (pointA->m_inverseMass + pointB->m_inverseMass));
		float pointBWeightCoefficient = (pointB->m_inverseMass / (pointA->m_inverseMass + pointB->m_inverseMass));

		Vec3 deltaPointA = coefficientValue * pointAWeightCoefficient * distanceConstraint * gradient;
		Vec3 deltaPointB = coefficientValue * pointBWeightCoefficient * distanceConstraint * gradient;

		pointA->m_proposedPosition += deltaPointA;
		pointB->m_proposedPosition -= deltaPointB;
	}
	else if (pointA->m_isLocked == true && pointB->m_isLocked == false)
	{
		Vec3 deltaPointB = coefficientValue * distanceConstraint * gradient;
		pointB->m_proposedPosition -= deltaPointB;
	}
	else if (pointA->m_isLocked == false && pointB->m_isLocked == true)
	{
		Vec3 deltaPointA = coefficientValue * distanceConstraint * gradient;
		pointA->m_proposedPosition += deltaPointA;
	}
}

//-----------------------------------------------------------------------------------------------
void PBDRope3D::ProjectBendingConstraint(Point3D* pointA, Point3D* pointB, Point3D* pointC)
{
	//Calculates direction and overflow along with weight Coefficients
	Vec3 displacement = pointC->m_proposedPosition - pointA->m_proposedPosition;
	float distanceConstraint = displacement.GetLength() - m_bendingConstraintDistance;

	//Check if less than the minimum distance, if so no need to constrain
	if (distanceConstraint > 0.0f)
	{
		return;
	}

	Vec3 gradient = displacement.GetNormalized();
	Vec3 directionCheck = (pointB->m_proposedPosition - pointA->m_proposedPosition).GetNormalized();

	//Because this is a cheap method to check for bending need to ensure the direction inst the same
	if (gradient == directionCheck)
	{
		return;
	}

	if (pointA->m_isLocked == false && pointC->m_isLocked == false)
	{
		float pointAWeightCoefficient = (pointA->m_inverseMass / (pointA->m_inverseMass + pointC->m_inverseMass));
		float pointCWeightCoefficient = (pointC->m_inverseMass / (pointA->m_inverseMass + pointC->m_inverseMass));

		Vec3 deltaPointA = m_bendingCoefficient * pointAWeightCoefficient * distanceConstraint * gradient;
		Vec3 deltaPointC = m_bendingCoefficient * pointCWeightCoefficient * distanceConstraint * gradient;

		pointA->m_proposedPosition += deltaPointA;
		pointC->m_proposedPosition -= deltaPointC;
	}
	else if (pointA->m_isLocked == true && pointC->m_isLocked == false)
	{
		Vec3 deltaPointC = m_bendingCoefficient * distanceConstraint * gradient;
		pointC->m_proposedPosition -= deltaPointC;
	}
	else if (pointA->m_isLocked == false && pointC->m_isLocked == true)
	{
		Vec3 deltaPointA = m_bendingCoefficient * distanceConstraint * gradient;
		pointA->m_proposedPosition += deltaPointA;
	}
}

//-----------------------------------------------------------------------------------------------
void PBDRope3D::ProjectCollisionConstraints(Point3D* point)
{
	//Floor Collisions
	if (point->m_proposedPosition.z < m_ropeRadius)
	{
		point->m_proposedPosition.z = m_ropeRadius;

		/*if (point->m_velocity != Vec3())
		{
			float normalForce = point->m_mass * m_gravityCoefficient;
			point->m_forcesAsFloat += m_kineticFrictionCoefficient * normalForce;
		}*/
	}

	//AABB Collisions
	for (int aabbIndex = 0; aabbIndex < m_shapes->m_aabbs.size(); aabbIndex++)
	{
		AABB3& aabb = m_shapes->m_aabbs[aabbIndex];
		if (PushSphereOutOfFixedAABB3D(point->m_proposedPosition, m_ropeRadius, aabb))
		{
			/*if (point->m_velocity != Vec3())
			{
				float normalForce = point->m_mass * m_gravityCoefficient;
				point->m_forcesAsFloat += m_kineticFrictionCoefficient * normalForce;
			}*/
		}
	}

	//Sphere Collisions
	for (int sphereIndex = 0; sphereIndex < m_shapes->m_spheres.size(); sphereIndex++)
	{
		Sphere3& sphere = m_shapes->m_spheres[sphereIndex];
		if (PushSphereOutOfFixedSphere3D(point->m_proposedPosition, m_ropeRadius, sphere.m_center, sphere.m_radius))
		{
			//Friction Logic
		}
	}

	////OBB Collisions
	//for (int obbIndex = 0; obbIndex < m_shapes->m_obbs.size(); obbIndex++)
	//{
	//	OBB2& obb = m_shapes->m_obbs[obbIndex];
	//	if (PushDiscOutOfFixedOBB2D(point->m_proposedPosition, m_ropeRadius, obb))
	//	{
	//		if (point->m_velocity != Vec2())
	//		{
	//			float normalForce = point->m_mass * m_gravityCoefficient;
	//			point->m_forcesAsFloat += m_kineticFrictionCoefficient * normalForce;
	//		}
	//	}
	//}

	////Capsule Collisions
	//for (int capsuleIndex = 0; capsuleIndex < m_shapes->m_capsules.size(); capsuleIndex++)
	//{
	//	Capsule2& capsule = m_shapes->m_capsules[capsuleIndex];
	//	if (PushDiscOutOfFixedCapsule2D(point->m_proposedPosition, m_ropeRadius, capsule))
	//	{
	//		if (point->m_velocity != Vec2())
	//		{
	//			float normalForce = point->m_mass * m_gravityCoefficient;
	//			point->m_forcesAsFloat += m_kineticFrictionCoefficient * normalForce;
	//		}
	//	}
	//}

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
Shapes3D::Shapes3D()
{
}

//-----------------------------------------------------------------------------------------------
Shapes3D::~Shapes3D()
{
}
