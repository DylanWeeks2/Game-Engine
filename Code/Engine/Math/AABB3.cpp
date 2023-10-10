#include "AABB3.hpp"

//-----------------------------------------------------------------------------------------------
AABB3::AABB3()
{
}

//-----------------------------------------------------------------------------------------------
AABB3::~AABB3()
{
}

//-----------------------------------------------------------------------------------------------
AABB3::AABB3(AABB3 const& copyFrom)
{
	m_mins = copyFrom.m_mins;
	m_maxs = copyFrom.m_maxs;
}

//-----------------------------------------------------------------------------------------------
AABB3::AABB3(float minX, float minY, float minZ, float maxX, float maxY, float maxZ)
{
	m_mins.x = minX;
	m_mins.y = minY;
	m_mins.z = minZ;

	m_maxs.x = maxX;
	m_maxs.y = maxY;
	m_maxs.z = maxZ;
}

//-----------------------------------------------------------------------------------------------
AABB3::AABB3(Vec3 const& mins, Vec3 const& maxs)
{
	m_mins = mins;
	m_maxs = maxs;
}

//-----------------------------------------------------------------------------------------------
bool AABB3::IsPointInside(Vec3 const& point) const
{
	if (point.x > m_mins.x && point.x < m_maxs.x &&
		point.y > m_mins.y && point.y < m_maxs.y &&
		point.z > m_mins.z && point.z < m_maxs.z)
	{
		return true;
	}

	return false;
}

//-----------------------------------------------------------------------------------------------
Vec3 const AABB3::GetNearestPoint(Vec3 const& referencePosition) const
{
	Vec3 nearestPoint;

	//inside the box
	if (referencePosition.x > m_mins.x && referencePosition.x < m_maxs.x && 
		referencePosition.y > m_mins.y && referencePosition.y < m_maxs.y && 
		referencePosition.z > m_mins.z && referencePosition.z < m_maxs.z)
	{
		nearestPoint = referencePosition;
	}
	
	//X
	if (referencePosition.x < m_maxs.x)
	{
		if (referencePosition.x > m_mins.x)
		{
			nearestPoint.x = referencePosition.x;
		}
		else
		{
			nearestPoint.x = m_mins.x;
		}
	}
	else
	{
		nearestPoint.x = m_maxs.x;
	}
	//Y
	if (referencePosition.y < m_maxs.y)
	{
		if (referencePosition.y > m_mins.y)
		{
			nearestPoint.y = referencePosition.y;
		}
		else
		{
			nearestPoint.y = m_mins.y;
		}
	}
	else
	{
		nearestPoint.y = m_maxs.y;
	}
	//Z
	if (referencePosition.z < m_maxs.z)
	{
		if (referencePosition.z > m_mins.z)
		{
			nearestPoint.z = referencePosition.z;
		}
		else
		{
			nearestPoint.z = m_mins.z;
		}
	}
	else
	{
		nearestPoint.z = m_maxs.z;
	}

	return nearestPoint;
}

//-----------------------------------------------------------------------------------------------
Vec3 const AABB3::GetDimensions() const
{
	Vec3 dimensions(m_maxs.x - m_mins.x, m_maxs.y - m_mins.y, m_maxs.z - m_mins.z);
	return dimensions;
}

//-----------------------------------------------------------------------------------------------
void AABB3::SetCenter(Vec3 const& newCenter)
{
	Vec3 center = GetCenter();
	float halfOfX = m_maxs.x - center.x;
	float halfOfY = m_maxs.y - center.y;
	float halfOfZ = m_maxs.z - center.z;

	m_mins.x = newCenter.x - halfOfX;
	m_maxs.x = newCenter.x + halfOfX;
	m_mins.y = newCenter.y - halfOfY;
	m_maxs.y = newCenter.y + halfOfY;
	m_mins.z = newCenter.z - halfOfZ;
	m_maxs.z = newCenter.z + halfOfZ;
}

//-----------------------------------------------------------------------------------------------
void AABB3::SetDimensions(Vec3 const& newDimensions)
{
	Vec3 center = GetCenter();
	float originalCenterX = center.x;
	float originalCenterY = center.y;
	float originalCenterZ = center.z;

	float halfOfX = newDimensions.x * .5f;
	float halfOfY = newDimensions.y * .5f;
	float halfOfZ = newDimensions.z * .5f;

	m_mins.x = originalCenterX - halfOfX;
	m_maxs.x = originalCenterX + halfOfX;
	m_mins.y = originalCenterY - halfOfY;
	m_maxs.y = originalCenterY + halfOfY;
	m_mins.z = originalCenterZ - halfOfZ;
	m_maxs.z = originalCenterZ + halfOfZ;
}

//-----------------------------------------------------------------------------------------------
Vec3 AABB3::GetCenter() const
{
	Vec3 center = (m_mins + m_maxs) * 0.5f;
	return center;
}
