#include "PhysicsScene3D.hpp"
#include "RigidBody3D.hpp"
#include "Collider3D.hpp"
#include "Engine/Math/Vec2.hpp"
#include "Engine/Math/ConvexPoly2D.hpp"
#include "Engine/Math/MathUtils.hpp"
#include "Engine/Core/DebugRender.hpp"
#include "Engine/Core/ErrorWarningAssert.hpp"
#include <algorithm>

//-----------------------------------------------------------------------------------------------
PhysicsScene3D::PhysicsScene3D(AABB3 const& worldBounds)
	:m_worldBounds(worldBounds)
{
}

//-----------------------------------------------------------------------------------------------
void PhysicsScene3D::Update(float deltaSeconds)
{
	m_physicsDebt += deltaSeconds;
	while (m_physicsDebt > m_physicsTimestep)
	{
		UpdateRigidBodies();
		DetectCollisions();
		ResolveCollisions();
		m_physicsDebt -= m_physicsTimestep;
	}
}

//-----------------------------------------------------------------------------------------------
void PhysicsScene3D::AddRigidBody(RigidBody3D* rigidBody)
{
	m_rigidBodies.push_back(rigidBody);
}

//-----------------------------------------------------------------------------------------------
bool PhysicsScene3D::RemoveRigidBody(RigidBody3D* rigidBody)
{
	auto it = std::find(m_rigidBodies.begin(), m_rigidBodies.end(), rigidBody);
	if (it != m_rigidBodies.end())
	{
		m_rigidBodies.erase(it);
		return true;
	}

	return false;
}

//-----------------------------------------------------------------------------------------------
void PhysicsScene3D::UpdateRigidBodies()
{
	for (int rigidBodyIndex = 0; rigidBodyIndex < m_rigidBodies.size(); rigidBodyIndex++)
	{
		RigidBody3D*& rigidBody = m_rigidBodies[rigidBodyIndex];
		if (rigidBody)
		{
			rigidBody->Update(m_physicsTimestep);
		}
	}
}

//-----------------------------------------------------------------------------------------------
void PhysicsScene3D::DetectCollisions()
{
	m_contactManifolds.clear();
	DetectCollisionsRigidBodies(); 
	DetectCollisionsWorldBounds();
}

//-----------------------------------------------------------------------------------------------
bool PhysicsScene3D::DetectCollisionsWorldBounds()
{
	// TODO: TURN THIS INTO AN ARRAY OF BOUNDING PLANES
	for (int index = 0; index < m_rigidBodies.size(); index++)
	{
		RigidBody3D*& a = m_rigidBodies[index];
		if (a)
		{
			RigidBodyVSGroundPlane(a);
		}
	}
	bool collision = false;
	return collision;
}

//-----------------------------------------------------------------------------------------------
void PhysicsScene3D::DetectCollisionsRigidBodies()
{
	for (int indexA = 0; indexA < m_rigidBodies.size(); indexA++)
	{
		RigidBody3D*& a = m_rigidBodies[indexA];
		if (a)
		{
			for (int indexB = indexA; indexB < m_rigidBodies.size(); indexB++)
			{
				RigidBody3D*& b = m_rigidBodies[indexB];
				if (b && b != a)
				{
					if (DoesExistingManifoldExist(a, b))
					{
						continue;
					}

					bool collision = BroadPhaseCheck(a, b);
					if (collision == false)
					{
						continue;
					}

					collision = NarrowPhaseCheck(a, b);
					if (collision == false)
					{
						continue;
					}

					GenerateContactData(a, b);
				}
			}
		}
	}
}

//-----------------------------------------------------------------------------------------------
bool PhysicsScene3D::DoesExistingManifoldExist(RigidBody3D* a, RigidBody3D* b)
{
	for (int index = 0; index < m_contactManifolds.size(); index++)
	{
		if ((m_contactManifolds[index]->m_a == a || m_contactManifolds[index]->m_a == b) &&
			(m_contactManifolds[index]->m_b == a || m_contactManifolds[index]->m_b == b))
		{
			return true;
		}
	}
	return false;
}

//-----------------------------------------------------------------------------------------------
bool PhysicsScene3D::RigidBodyVSGroundPlane(RigidBody3D* a)
{
	std::vector<Vec3> contactPoints = a->m_collider->GetFurthestPointsInDireciton(Vec3(0.0f, 0.0f, -1.0f));
	if (contactPoints.size() == 0)
	{
		a = a;
	}
	float penetration = DotProduct3D(contactPoints[0], Vec3(0.0f, 0.0f, 1.0f));

	if (penetration < 0.0f)
	{
		ContactManifold3D* contact = new ContactManifold3D();
		contact->m_a = a;
		contact->m_contactPoints = contactPoints;
		contact->m_contactNormal = Vec3(0.0f, 0.0f, 1.0f);
		contact->m_penetrationDepth = -penetration;
		m_contactManifolds.push_back(contact);
		return true;
	}

	return false;
}

//-----------------------------------------------------------------------------------------------
bool PhysicsScene3D::BroadPhaseCheck(RigidBody3D* a, RigidBody3D* b)
{
	//TODO: ADD BIT-BUCKET AND BVH SPATIAL PARTITIONING (MAYBE QUADTREES)
	if (DoSpheresOverlap(a->m_position, a->m_collider->m_boundingSphereRadius, b->m_position, b->m_collider->m_boundingSphereRadius))
	{
		return true;
	}

	return false;
}

//-----------------------------------------------------------------------------------------------
bool PhysicsScene3D::NarrowPhaseCheck(RigidBody3D* a, RigidBody3D* b)
{
	if (m_isSAT)
	{
		return SAT(a, b);
	}
	else
	{
		return GJK_EPA(a, b);
	}
}

//-----------------------------------------------------------------------------------------------
bool PhysicsScene3D::SAT(RigidBody3D* a, RigidBody3D* b)
{
	//TODO - FINZLIZE SAT AND COMPARE RESULTS TO GJK
	//Test A faces
	float largestDistA = -10000000000000.0f;
	int largestIndexA = -1;
	for (int planeIndex = 0; planeIndex < a->m_collider->m_hull->m_boundingPlanes.size(); planeIndex++)
	{
		//Get World Space Plane
		Plane3D plane = a->m_collider->m_hull->m_boundingPlanes[planeIndex];
		Vec3 pointOnPlane = a->m_collider->m_hull->m_boundingPolys[planeIndex].m_ccwOrderedPoints[0];
		pointOnPlane = a->m_rotation.TransformPosition3D(pointOnPlane);
		pointOnPlane += a->m_position;
		plane.m_normal = a->m_rotation.TransformVectorQuantity3D(plane.m_normal);
		plane.m_distanceFromOrigin = DotProduct3D(plane.m_normal, pointOnPlane);

		//Get furthest point in direction on b
		Vec3 furthestPointB = b->m_collider->GetFurthestPointInDireciton(plane.m_normal * -1.0f);

		//Calculate altitude
		float currentAlt = DotProduct3D(furthestPointB, plane.m_normal) - plane.m_distanceFromOrigin;
		if (currentAlt > largestDistA)
		{
			largestDistA = currentAlt;
			largestIndexA = planeIndex;
		}
	}

	if (largestDistA > 0.0f)
	{
		return false;
	}

	//Test B Faces
	float largestDistB = -10000000000000.0f;
	int largestIndexB = -1;
	for (int planeIndex = 0; planeIndex < b->m_collider->m_hull->m_boundingPlanes.size(); planeIndex++)
	{
		//Get World Space Plane
		Plane3D plane = b->m_collider->m_hull->m_boundingPlanes[planeIndex];
		Vec3 pointOnPlane = b->m_collider->m_hull->m_boundingPolys[planeIndex].m_ccwOrderedPoints[0];
		pointOnPlane = b->m_rotation.TransformPosition3D(pointOnPlane);
		pointOnPlane += b->m_position;
		plane.m_normal = b->m_rotation.TransformVectorQuantity3D(plane.m_normal);
		plane.m_distanceFromOrigin = DotProduct3D(plane.m_normal, pointOnPlane);

		//Get furthest point in direction on b
		Vec3 furthestPointA = a->m_collider->GetFurthestPointInDireciton(plane.m_normal * -1.0f);

		//Calculate altitude
		float currentAlt = DotProduct3D(furthestPointA, plane.m_normal) - plane.m_distanceFromOrigin;
		if (currentAlt > largestDistB)
		{
			largestDistB = currentAlt;
			largestIndexB = planeIndex;
		}
	}

	if (largestDistB > 0.0f)
	{
		return false;
	}


	//Test Edges
	std::vector<LineSegment3> worldEdgesA = a->m_collider->m_hull->m_boundingEdges;
	std::vector<LineSegment3> worldEdgesB = b->m_collider->m_hull->m_boundingEdges;
	for (int edgeIndex = 0; edgeIndex < worldEdgesA.size(); edgeIndex++)
	{
		LineSegment3& line = worldEdgesA[edgeIndex];
		line.m_start = a->m_rotation.TransformPosition3D(line.m_start);
		line.m_start += a->m_position;
		line.m_end = a->m_rotation.TransformPosition3D(line.m_end);
		line.m_end += a->m_position;
	}
	for (int edgeIndex = 0; edgeIndex < worldEdgesB.size(); edgeIndex++)
	{
		LineSegment3& line = worldEdgesB[edgeIndex];
		line.m_start = b->m_rotation.TransformPosition3D(line.m_start);
		line.m_start += b->m_position;
		line.m_end = b->m_rotation.TransformPosition3D(line.m_end);
		line.m_end += b->m_position;
	}


	float largestDistEdge = -10000000000000.0f;
	int largestIndexEdgeA = -1;
	int largestIndexEdgeB = -1;
	Vec3 largestNormal;
	Vec3 largestPoint;
	for (int indexA = 0; indexA < worldEdgesA.size(); indexA++)
	{
		LineSegment3& lineA = worldEdgesA[indexA];
		Vec3 lineDirecitonA = (lineA.m_end - lineA.m_start).GetNormalized();
		for (int indexB = 0; indexB < worldEdgesB.size(); indexB++)
		{
			LineSegment3& lineB = worldEdgesB[indexB];
			Vec3 lineDirecitonB = (lineB.m_end - lineB.m_start).GetNormalized();
			Vec3 normal = CrossProduct3D(lineDirecitonA, lineDirecitonB).GetNormalized();
			if (DotProduct3D(normal, lineA.m_start - a->m_position) <= 0.0f)
			{
				normal *= -1.0f;
			}

			//DebugAddWorldArrow(lineA.m_start, lineA.m_start + normal, 0.02f, 0.00001f);

			Plane3D plane(normal, DotProduct3D(normal, lineA.m_start));
			Vec3 furthestPointB = b->m_collider->GetFurthestPointInDireciton(plane.m_normal * -1.0f);

			
			float currentAlt = DotProduct3D(plane.m_normal, furthestPointB) - plane.m_distanceFromOrigin;
			if (currentAlt > largestDistEdge)
			{
				largestDistEdge = currentAlt;
				largestIndexEdgeA = indexA;
				largestIndexEdgeB = indexB; 
				largestNormal = normal;
				largestPoint = furthestPointB;
			}
		}
	}

	//AABB3 point;
	//point.SetDimensions(Vec3(0.025f, 0.025f, 0.025f));
	//point.SetCenter(worldEdgesA[largestIndexEdgeA].m_start);
	//DebugAddWorldWireAABB3(point, 0.01f, Rgba8::CYAN);
	//point.SetCenter(worldEdgesA[largestIndexEdgeA].m_end);
	//DebugAddWorldWireAABB3(point, 0.01f, Rgba8::BLUE);
	//DebugAddWorldArrow(worldEdgesA[largestIndexEdgeA].m_start, worldEdgesA[largestIndexEdgeA].m_start + largestNormal, 0.02f, 0.00001f);
	//point.SetCenter(worldEdgesB[largestIndexEdgeB].m_start);
	//DebugAddWorldWireAABB3(point, 0.01f, Rgba8::ORANGE);
	//point.SetCenter(worldEdgesB[largestIndexEdgeB].m_end);
	//DebugAddWorldWireAABB3(point, 0.01f, Rgba8::RED);
	//point.SetCenter(largestPoint);
	//DebugAddWorldWireAABB3(point, 0.01f, Rgba8::GREEN);
	//DebuggerPrintf("%f\n", largestDistEdge);


	if (largestDistEdge > 0.0f)
	{
		return false;
	}

	return true;
}

//-----------------------------------------------------------------------------------------------
bool PhysicsScene3D::GJK_EPA(RigidBody3D* a, RigidBody3D* b)
{
	//GJK Section
	Vec3 origin;
	std::vector<Vec3> points;
	Vec3 direction = (b->m_position - a->m_position).GetNormalized();
	Vec3 point1 = GJK3DSupportFunciton(a, b, direction);
	Vec3 point2 = GJK3DSupportFunciton(a, b, direction * -1.0f);
	Vec3 line12 = point2 - point1;
	Vec3 newDireciton = TripleProduct3D((line12), (point1 * -1.0f), (line12));
	if (newDireciton == Vec3())
	{
		newDireciton = Vec3(CrossProduct3D((line12), (point1 * -1.0f)).GetNormalized());
		if (newDireciton == Vec3())
		{
			newDireciton = Vec3(1.0f, 0.0f, 0.0f);
		}
	}
	Vec3 point3 = GJK3DSupportFunciton(a, b, newDireciton);
	Vec3 triangleNormal = CrossProduct3D(point3 - point2, point1 - point2).GetNormalized();
	Vec3 point4 = GJK3DSupportFunciton(a, b, triangleNormal * -1.0f);
	points.push_back(point1);
	points.push_back(point2);
	points.push_back(point3);
	points.push_back(point4);

	ConvexPoly3D poly1;
	poly1.m_ccwOrderedPoints.push_back(point1);
	poly1.m_ccwOrderedPoints.push_back(point2);
	poly1.m_ccwOrderedPoints.push_back(point3);
	ConvexPoly3D poly2;
	poly2.m_ccwOrderedPoints.push_back(point1);
	poly2.m_ccwOrderedPoints.push_back(point3);
	poly2.m_ccwOrderedPoints.push_back(point4);
	ConvexPoly3D poly3;
	poly3.m_ccwOrderedPoints.push_back(point3);
	poly3.m_ccwOrderedPoints.push_back(point2);
	poly3.m_ccwOrderedPoints.push_back(point4);
	ConvexPoly3D poly4;
	poly4.m_ccwOrderedPoints.push_back(point1);
	poly4.m_ccwOrderedPoints.push_back(point4);
	poly4.m_ccwOrderedPoints.push_back(point2);
	Plane3D plane1(poly1);
	Plane3D plane2(poly2);
	Plane3D plane3(poly3);
	Plane3D plane4(poly4);

	//First Simplex
	ConvexHull3D hull;
	hull.m_boundingPlanes.push_back(plane1);
	hull.m_boundingPlanes.push_back(plane2);
	hull.m_boundingPlanes.push_back(plane3);
	hull.m_boundingPlanes.push_back(plane4);
	hull.m_boundingPolys.push_back(poly1);
	hull.m_boundingPolys.push_back(poly2);
	hull.m_boundingPolys.push_back(poly3);
	hull.m_boundingPolys.push_back(poly4);
	hull.CalculateNewEpsilon(points);

	int breakCounter = 0;
	while (hull.IsPointInside(origin) == false)
	{
		//Find the plane where the point is outside of
		int outsidePlaneIndex = -1;
		float highestAlt = -100000000000.0f;
		for (int index = 0; index < hull.m_boundingPlanes.size(); index++)
		{
			float alt = hull.m_boundingPlanes[index].GetAltitude(origin);
			if (alt > highestAlt)
			{
				highestAlt = alt;
				outsidePlaneIndex = index;
			}
		}

		//Early out check
		Vec3 currDir = hull.m_boundingPlanes[outsidePlaneIndex].m_normal;
		Vec3 newPoint = GJK3DSupportFunciton(a, b, currDir);
		float relation = DotProduct3D(newPoint, currDir);
		if (relation <= 0.0f)
		{
			return false;
		}

		//Find the point that is not part of the plane
		int indexToEliminate = -1;
		for (int index = 0; index < points.size(); index++)
		{
			Vec3 currentPoint = points[index];
			auto it = std::find(hull.m_boundingPolys[outsidePlaneIndex].m_ccwOrderedPoints.begin(), hull.m_boundingPolys[outsidePlaneIndex].m_ccwOrderedPoints.end(), currentPoint);
			if (it == hull.m_boundingPolys[outsidePlaneIndex].m_ccwOrderedPoints.end())
			{
				indexToEliminate = index;
				break;
			}
		}
		points[indexToEliminate] = newPoint;
		hull.CalculateNewEpsilon(points);

		//Update planes and polys
		hull.m_boundingPlanes[outsidePlaneIndex].m_normal *= -1.0f;
		hull.m_boundingPlanes[outsidePlaneIndex].m_distanceFromOrigin *= -1.0f;
		Vec3 tempPoint = hull.m_boundingPolys[outsidePlaneIndex].m_ccwOrderedPoints[1];
		hull.m_boundingPolys[outsidePlaneIndex].m_ccwOrderedPoints[1] = hull.m_boundingPolys[outsidePlaneIndex].m_ccwOrderedPoints[2];
		hull.m_boundingPolys[outsidePlaneIndex].m_ccwOrderedPoints[2] = tempPoint;

		int counter = 0;
		for (int index = 0; index < hull.m_boundingPlanes.size(); index++)
		{
			if (index != outsidePlaneIndex)
			{
				if (counter == 0)
				{
					hull.m_boundingPolys[index].m_ccwOrderedPoints[0] = hull.m_boundingPolys[outsidePlaneIndex].m_ccwOrderedPoints[0];
					hull.m_boundingPolys[index].m_ccwOrderedPoints[1] = newPoint;
					hull.m_boundingPolys[index].m_ccwOrderedPoints[2] = hull.m_boundingPolys[outsidePlaneIndex].m_ccwOrderedPoints[1];

				}
				else if (counter == 1)
				{
					hull.m_boundingPolys[index].m_ccwOrderedPoints[0] = hull.m_boundingPolys[outsidePlaneIndex].m_ccwOrderedPoints[0];
					hull.m_boundingPolys[index].m_ccwOrderedPoints[1] = hull.m_boundingPolys[outsidePlaneIndex].m_ccwOrderedPoints[2];
					hull.m_boundingPolys[index].m_ccwOrderedPoints[2] = newPoint;
				}
				else
				{
					hull.m_boundingPolys[index].m_ccwOrderedPoints[0] = hull.m_boundingPolys[outsidePlaneIndex].m_ccwOrderedPoints[2];
					hull.m_boundingPolys[index].m_ccwOrderedPoints[1] = hull.m_boundingPolys[outsidePlaneIndex].m_ccwOrderedPoints[1];
					hull.m_boundingPolys[index].m_ccwOrderedPoints[2] = newPoint;
				}
				hull.m_boundingPlanes[index] = Plane3D(hull.m_boundingPolys[index]);
				counter++;
			}
		}

		if (breakCounter >= 100)
		{
			return false;
		}
		breakCounter++;
	}

	//EPA Section
	breakCounter = 0;
	hull.m_conflictLists.resize(4);
	Vec3 supportPoint;
	int closestIndex = -1;
	while (true)
	{
		float closestDist = FLT_MAX;
		for (int index = 0; index < hull.m_boundingPolys.size(); index++)
		{
			float alt = fabsf(hull.m_boundingPlanes[index].GetAltitude(origin));
			if (alt < closestDist)
			{
				closestDist = alt;
				closestIndex = index;
			}
		}

		supportPoint = GJK3DSupportFunciton(a, b, hull.m_boundingPlanes[closestIndex].m_normal);
		if (hull.IsPointInside(supportPoint))
		{
			break;
		}

		//Partitioning of Points and Faces
		hull.m_pointsToPartition.push_back(supportPoint);
		for (int pointIndex = 0; pointIndex < hull.m_pointsToPartition.size(); pointIndex++)
		{
			Vec3& point = hull.m_pointsToPartition[pointIndex];
			float minDist = FLT_MAX;
			int closestPlaneIndex = -1;
			for (int planeIndex = 0; planeIndex < hull.m_boundingPlanes.size(); planeIndex++)
			{
				Plane3D& plane = hull.m_boundingPlanes[planeIndex];
				float altitude = DotProduct3D(point, plane.m_normal) - plane.m_distanceFromOrigin;
				if (altitude < minDist && altitude > 0.0f)
				{
					minDist = altitude;
					closestPlaneIndex = planeIndex;
				}
			}

			hull.m_conflictLists[closestPlaneIndex].push_back(Vec2(float(pointIndex), minDist));
		}
		hull.IterativeQuickullGeneration(true);

		if (breakCounter >= 100)
		{
			return false;
		}
		breakCounter++;
	}

	Vec3 nearestPointToOrigin = GetNearestPointOnPlane3D(origin, hull.m_boundingPlanes[closestIndex]);

	Vec3 penetration = nearestPointToOrigin;
	ContactManifold3D* contact = new ContactManifold3D();
	contact->m_a = a;
	contact->m_b = b;
	contact->m_penetrationDepth = penetration.GetLength();
	contact->m_contactNormal = penetration.GetNormalized();
	m_contactManifolds.push_back(contact);
	return true;
}

//-----------------------------------------------------------------------------------------------
void PhysicsScene3D::GenerateContactData(RigidBody3D* a, RigidBody3D* b)
{
	ContactManifold3D* contact = m_contactManifolds[m_contactManifolds.size() - 1];

	//Identify the significant faces
	Vec3 vertexA = a->m_collider->GetFurthestPointInDireciton(contact->m_contactNormal) - a->m_position;
	Vec3 vertexB = b->m_collider->GetFurthestPointInDireciton(contact->m_contactNormal * -1.0f) - b->m_position;
	ConvexHull3D* hullA = a->m_collider->m_hull;
	ConvexHull3D* hullB = b->m_collider->m_hull;

	float bestRelationA = -1000000000.0f;
	int bestPlaneIndexA = -1;
	for (int index = 0; index < hullA->m_boundingPolys.size(); index++)
	{
		Vec3 newPlaneNormal = a->m_rotation.TransformVectorQuantity3D(hullA->m_boundingPlanes[index].m_normal);
		float currentAlt = DotProduct3D(newPlaneNormal, vertexA) - hullA->m_boundingPlanes[index].m_distanceFromOrigin;
		if (currentAlt <= hullA->m_epsilon && currentAlt >= -hullA->m_epsilon)
		{
			float currentRelation = DotProduct3D(newPlaneNormal, contact->m_contactNormal);
			if (currentRelation > bestRelationA)
			{
				bestRelationA = currentRelation;
				bestPlaneIndexA = index;
			}
		}
	}

	float bestRelationB = -1000000000.0f;
	int bestPlaneIndexB = -1;
	for (int index = 0; index < hullB->m_boundingPlanes.size(); index++)
	{
		Vec3 newPlaneNormal = b->m_rotation.TransformVectorQuantity3D(hullB->m_boundingPlanes[index].m_normal);
		float currentAlt = DotProduct3D(newPlaneNormal, vertexB) - hullB->m_boundingPlanes[index].m_distanceFromOrigin;
		if (currentAlt <= hullB->m_epsilon && currentAlt >= -hullB->m_epsilon)
		{
			float currentRelation = DotProduct3D(newPlaneNormal, contact->m_contactNormal * -1.0f);
			if (currentRelation > bestRelationB)
			{
				bestRelationB = currentRelation;
				bestPlaneIndexB = index;
			}
		}
	}

	//Compute contact points based on reference or incident bodies
	if (bestRelationA >= bestRelationB)
	{
		Vec3 refPoint = vertexA + a->m_position;
		Vec3 incPoint = vertexB + b->m_position;
		ComputeContactPoints(contact->m_contactNormal, a, refPoint, bestPlaneIndexA, b, incPoint, bestPlaneIndexB, contact);
	}
	else
	{
		Vec3 refPoint = vertexB + b->m_position;
		Vec3 incPoint = vertexA + a->m_position;
		ComputeContactPoints(contact->m_contactNormal * -1.0f, b, refPoint, bestPlaneIndexB, a, incPoint, bestPlaneIndexA, contact);
	}
}

//-----------------------------------------------------------------------------------------------
void PhysicsScene3D::ComputeContactPoints(Vec3 const& contactNormal, RigidBody3D* referenceBody, Vec3& referenceVert, int referenceIndex,
	RigidBody3D* incidentBody, Vec3& incidentVert, int incidentIndex, ContactManifold3D* contact)
{
	incidentVert;
	ConvexHull3D* referenceHull = referenceBody->m_collider->m_hull;
	ConvexHull3D* incidentHull = incidentBody->m_collider->m_hull;
	ConvexPoly3D referencePoly = referenceHull->m_boundingPolys[referenceIndex];
	ConvexPoly3D incidentPoly = incidentHull->m_boundingPolys[incidentIndex];
	Plane3D referencePlane = referenceHull->m_boundingPlanes[referenceIndex];
	Plane3D incidentPlane = incidentHull->m_boundingPlanes[incidentIndex];
	Plane3D collisionPlane(contactNormal, DotProduct3D(contactNormal, referenceVert));

	//Project Points into collision plane
	ConvexPoly3D referencePolyProjected;
	referencePolyProjected.m_ccwOrderedPoints.reserve(referencePoly.m_ccwOrderedPoints.size());
	for (int index = 0; index < referencePoly.m_ccwOrderedPoints.size(); index++)
	{
		referencePoly.m_ccwOrderedPoints[index] = referenceBody->m_rotation.TransformPosition3D(referencePoly.m_ccwOrderedPoints[index]);
		referencePoly.m_ccwOrderedPoints[index] += referenceBody->m_position;
		float alt = DotProduct3D(referencePoly.m_ccwOrderedPoints[index], collisionPlane.m_normal) - collisionPlane.m_distanceFromOrigin;
		referencePolyProjected.m_ccwOrderedPoints.push_back(referencePoly.m_ccwOrderedPoints[index] - collisionPlane.m_normal * alt);
	}
	//DebugAddWorldConvexPoly3D(referencePolyProjected, 0.0001f, Rgba8::CYAN);

	ConvexPoly3D reversedIncPolyProjected;
	reversedIncPolyProjected.m_ccwOrderedPoints.reserve(incidentPoly.m_ccwOrderedPoints.size());
	for (int index = int(incidentPoly.m_ccwOrderedPoints.size()) - 1; index >= 0; index--)
	{
		incidentPoly.m_ccwOrderedPoints[index] = incidentBody->m_rotation.TransformPosition3D(incidentPoly.m_ccwOrderedPoints[index]);
		incidentPoly.m_ccwOrderedPoints[index] += incidentBody->m_position;
		float alt = DotProduct3D(incidentPoly.m_ccwOrderedPoints[index], collisionPlane.m_normal) - collisionPlane.m_distanceFromOrigin;
		Vec3 projectedPoint = incidentPoly.m_ccwOrderedPoints[index] - collisionPlane.m_normal * alt;
		reversedIncPolyProjected.m_ccwOrderedPoints.push_back(projectedPoint);
	}
	//DebugAddWorldConvexPoly3D(reversedIncPolyProjected, 0.0001f, Rgba8::ORANGE);

	referencePlane.m_normal = referenceBody->m_rotation.TransformVectorQuantity3D(referencePlane.m_normal);
	incidentPlane.m_normal = incidentBody->m_rotation.TransformVectorQuantity3D(incidentPlane.m_normal);
	referencePlane.m_distanceFromOrigin = DotProduct3D(referencePlane.m_normal, referencePoly.m_ccwOrderedPoints[0]);
	incidentPlane.m_distanceFromOrigin = DotProduct3D(incidentPlane.m_normal, incidentPoly.m_ccwOrderedPoints[0]);
	Vec3 collisionPlaneOrigin = collisionPlane.m_normal * collisionPlane.m_distanceFromOrigin;	
	//DebugAddWorldArrow(referenceVert, referenceVert + collisionPlane.m_normal, 0.02f, 0.00001f, Rgba8::MEDIUM_GREEN);

	Vec3 u = CrossProduct3D(Vec3(1.0f, 0.0f, 0.0f), collisionPlane.m_normal).GetNormalized();
	if (u == Vec3())
	{
		u = Vec3(0.0f, 1.0f, 0.0f);
	}
	Vec3 v = CrossProduct3D(collisionPlane.m_normal, u).GetNormalized();

	//2D Plane Space Coords
	ConvexPoly2D refPoly2D;
	for (int index = 0; index < referencePolyProjected.m_ccwOrderedPoints.size(); index++)
	{
		float x = DotProduct3D((referencePolyProjected.m_ccwOrderedPoints[index] - collisionPlaneOrigin), u);
		float y = DotProduct3D((referencePolyProjected.m_ccwOrderedPoints[index] - collisionPlaneOrigin), v);
		Vec2 point2D(x, y);
		refPoly2D.m_ccwOrderedPoints.push_back(point2D);
	}
	ConvexPoly2D incPoly2D;
	for (int index = 0; index < reversedIncPolyProjected.m_ccwOrderedPoints.size(); index++)
	{
		float x = DotProduct3D((reversedIncPolyProjected.m_ccwOrderedPoints[index] - collisionPlaneOrigin), u);
		float y = DotProduct3D((reversedIncPolyProjected.m_ccwOrderedPoints[index] - collisionPlaneOrigin), v);
		Vec2 point2D(x, y);
		incPoly2D.m_ccwOrderedPoints.push_back(point2D);
	}

	//Convert back to 3D world space
	ConvexPoly2D clippedPoly2D = GetClippedPolygon2D(refPoly2D, incPoly2D);
	ConvexPoly3D clippedPoly3D;
	for (int index = 0; index < clippedPoly2D.m_ccwOrderedPoints.size(); index++)
	{
		Vec2 point2D = clippedPoly2D.m_ccwOrderedPoints[index];
		clippedPoly3D.m_ccwOrderedPoints.push_back(collisionPlaneOrigin + (u * point2D.x) + (v * point2D.y));
	}

	//DebugAddWorldConvexPoly3D(clippedPoly3D, 0.0001f, Rgba8::MAGENTA);

	//Calculate plane epsilons
	float maxX = -100000000000.0f;
	float maxY = -100000000000.0f;
	float maxZ = -100000000000.0f;
	for (int index = 0; index < referencePoly.m_ccwOrderedPoints.size(); index++)
	{
		float x = fabsf(referencePoly.m_ccwOrderedPoints[index].x);
		float y = fabsf(referencePoly.m_ccwOrderedPoints[index].y);
		float z = fabsf(referencePoly.m_ccwOrderedPoints[index].z);
		if (x > maxX)
		{
			maxX = x;
		}
		if (y > maxY)
		{
			maxY = y;
		}
		if (z > maxZ)
		{
			maxZ = z;
		}
	}
	float referenceEpsilon = 3.0f * (maxX + maxY + maxZ) * FLT_EPSILON;

	maxX = -100000000000.0f;
	maxY = -100000000000.0f;
	maxZ = -100000000000.0f;
	for (int index = 0; index < incidentPoly.m_ccwOrderedPoints.size(); index++)
	{
		float x = fabsf(incidentPoly.m_ccwOrderedPoints[index].x);
		float y = fabsf(incidentPoly.m_ccwOrderedPoints[index].y);
		float z = fabsf(incidentPoly.m_ccwOrderedPoints[index].z);
		if (x > maxX)
		{
			maxX = x;
		}
		if (y > maxY)
		{
			maxY = y;
		}
		if (z > maxZ)
		{
			maxZ = z;
		}
	}
	float incidentEpsilon = 3.0f * (maxX + maxY + maxZ) * FLT_EPSILON;

	//Check if the clipped points are behind or on both planes
	for (int index = 0; index < clippedPoly3D.m_ccwOrderedPoints.size(); index++)
	{
		Vec3& point = clippedPoly3D.m_ccwOrderedPoints[index];
		float refAlt = DotProduct3D(referencePlane.m_normal, point) - referencePlane.m_distanceFromOrigin;
		float incAlt = DotProduct3D(incidentPlane.m_normal, point) - incidentPlane.m_distanceFromOrigin;

		if (refAlt <= referenceEpsilon && incAlt <= incidentEpsilon)
		{
			contact->m_contactPoints.push_back(point);
		}
	}
}

//-----------------------------------------------------------------------------------------------
void PhysicsScene3D::ResolveCollisions()
{
	//Check all contact manifolds
	for (int contactIndex = 0; contactIndex < m_contactManifolds.size(); contactIndex++)
	{
		ContactManifold3D* contact = m_contactManifolds[contactIndex];
		if (contact)
		{
			//Collisions vs world boundaries
			if (contact->m_a && contact->m_b == nullptr)
			{
				contact->m_a->m_position += contact->m_contactNormal * contact->m_penetrationDepth;
				for (int i = 0; i < contact->m_contactPoints.size(); i++)
				{
					Vec3& contactPoint = contact->m_contactPoints[i];
					Vec3 velocityOfPoint = contact->m_a->m_velocity + CrossProduct3D(contact->m_a->m_angularVelocity, (contactPoint - contact->m_a->m_position));
					float relativeVelocity = DotProduct3D(contact->m_contactNormal, velocityOfPoint);

					float term1 = 1.0f / contact->m_a->m_mass;
					float term2 = 0.0f;
					float term3 = DotProduct3D(contact->m_contactNormal, CrossProduct3D(contact->m_a->m_inverseInertiaTensor.TransformVectorQuantity3D(CrossProduct3D((contactPoint - contact->m_a->m_position), contact->m_contactNormal)), (contactPoint - contact->m_a->m_position)));
					float term4 = 0.0f;
					float numerator = -1.0f * (1 + 0.5f) * relativeVelocity;
					float denomenator = term1 + term2 + term3 + term4;
					float j = numerator / denomenator;
					j /= float(contact->m_contactPoints.size());
					Vec3 impusle = j * contact->m_contactNormal;
					contact->m_a->ApplyImpulse(impusle, contactPoint);

					//TODO: NEED ACCURATE FRICTION IMPULSES (THIS IS NOT RIGHT)
					//Friction
					Vec3 tangentalVel = velocityOfPoint - (relativeVelocity * contact->m_contactNormal);
					Vec3 friction = tangentalVel * contact->m_a->m_Uk;
					contact->m_a->ApplyImpulse(-1.0f * friction, contactPoint);
				}
			}
			//Collisions vs rigid bodies
			else
			{
				contact->m_a->m_position += contact->m_contactNormal * (contact->m_penetrationDepth * -0.5f);
				contact->m_b->m_position += contact->m_contactNormal * (contact->m_penetrationDepth * 0.5f);
				for (int i = 0; i < contact->m_contactPoints.size(); i++)
				{
					Vec3& contactPoint = contact->m_contactPoints[i];
					Vec3 velocityOfPointA = contact->m_a->m_velocity + CrossProduct3D(contact->m_a->m_angularVelocity, (contactPoint - contact->m_a->m_position));
					Vec3 velocityOfPointB = contact->m_b->m_velocity + CrossProduct3D(contact->m_b->m_angularVelocity, (contactPoint - contact->m_b->m_position));
					float relativeVelocity = DotProduct3D(contact->m_contactNormal, velocityOfPointA - velocityOfPointB);

					float term1 = 1.0f / contact->m_a->m_mass;
					float term2 = 1.0f / contact->m_b->m_mass;
					float term3 = DotProduct3D(contact->m_contactNormal, CrossProduct3D(contact->m_a->m_inverseInertiaTensor.TransformVectorQuantity3D(CrossProduct3D((contactPoint - contact->m_a->m_position), contact->m_contactNormal)), (contactPoint - contact->m_a->m_position)));
					float term4 = DotProduct3D(contact->m_contactNormal, CrossProduct3D(contact->m_b->m_inverseInertiaTensor.TransformVectorQuantity3D(CrossProduct3D((contactPoint - contact->m_b->m_position), contact->m_contactNormal)), (contactPoint - contact->m_b->m_position)));;
					float numerator = -1.0f * (1 + 0.5f) * relativeVelocity;
					float denomenator = term1 + term2 + term3 + term4;
					float j = numerator / denomenator;
					Vec3 force = j * contact->m_contactNormal;
					contact->m_a->ApplyImpulse(force, contactPoint);
					contact->m_b->ApplyImpulse(-1.0f * force, contactPoint);


					//TODO: NEED ACCURATE FRICTION IMPULSES (THIS IS NOT RIGHT)
					//Friction
					Vec3 tangentalVel = velocityOfPointA - (relativeVelocity * contact->m_contactNormal);
					Vec3 friction = tangentalVel * contact->m_a->m_Uk;
					contact->m_a->ApplyImpulse(-0.1f * friction, contactPoint);

					tangentalVel = velocityOfPointB - (relativeVelocity * contact->m_contactNormal);
					friction = tangentalVel * contact->m_b->m_Uk;
					contact->m_b->ApplyImpulse(-0.1f * friction, contactPoint);
				}
			}
		}
	}
}
