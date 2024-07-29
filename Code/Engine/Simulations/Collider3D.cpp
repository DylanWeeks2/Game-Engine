#include "Engine/Simulations/Collider3D.hpp"
#include "Engine/Math/MathUtils.hpp"


//-----------------------------------------------------------------------------------------------
Collider3D::Collider3D(RigidBody3D* rigidBody)
	:m_rigidBody(rigidBody)
{
}

//-----------------------------------------------------------------------------------------------
Vec3 Collider3D::GetFurthestPointInDireciton(Vec3 const& direciton)
{
	Vec3 returnPoint;
	float furthestDist = -100000000000000000.0f;
	for (int pointIndex = 0; pointIndex < m_hull->m_boundingPoints.size(); pointIndex++)
	{
		Vec3 currentPoint = m_hull->m_boundingPoints[pointIndex];
		currentPoint = m_rigidBody->m_rotation.TransformPosition3D(currentPoint);
		currentPoint += m_rigidBody->m_position;
		float currentDist = DotProduct3D(direciton, currentPoint);
		if (currentDist > furthestDist)
		{
			returnPoint = currentPoint;
			furthestDist = currentDist;
		}
	}

	return returnPoint;
}

//-----------------------------------------------------------------------------------------------
std::vector<Vec3> Collider3D::GetFurthestPointsInDireciton(Vec3 const& direciton)
{
	std::vector<Vec3> returnPoints;
	float furthestDist = -100000000000000000.0f;
	for (int pointIndex = 0; pointIndex < m_hull->m_boundingPoints.size(); pointIndex++)
	{
		Vec3 currentPoint = m_hull->m_boundingPoints[pointIndex];
		currentPoint = m_rigidBody->m_rotation.TransformPosition3D(currentPoint);
		currentPoint += m_rigidBody->m_position;
		float currentDist = DotProduct3D(direciton, currentPoint);
		if (currentDist > furthestDist)
		{
			returnPoints.clear();
			returnPoints.push_back(currentPoint);
			furthestDist = currentDist;
		}
		else if (currentDist == furthestDist)
		{
			returnPoints.push_back(currentPoint);
		}
	}

	return returnPoints;
}

//-----------------------------------------------------------------------------------------------
void Collider3D::GenerateBoundingSphereRadius()
{
	float maxDist = -100000000000000000.0f;
	for (int pointIndex = 0; pointIndex < m_hull->m_boundingPoints.size(); pointIndex++)
	{
		Vec3 currentPoint = m_hull->m_boundingPoints[pointIndex];
		currentPoint = m_rigidBody->m_rotation.TransformPosition3D(currentPoint);
		currentPoint += m_rigidBody->m_position;
		float distSquared = GetDistanceSquared3D(currentPoint, m_rigidBody->m_position);
		if (distSquared > maxDist)
		{
			maxDist = distSquared;
		}
	}

	m_boundingSphereRadius = sqrtf(maxDist);
}
