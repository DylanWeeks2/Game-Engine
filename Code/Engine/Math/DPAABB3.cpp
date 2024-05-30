#include "DPAABB3.hpp"
#include <vector>

//-----------------------------------------------------------------------------------------------
DPAABB3::DPAABB3()
{
}

//-----------------------------------------------------------------------------------------------
DPAABB3::~DPAABB3()
{
}

//-----------------------------------------------------------------------------------------------
DPAABB3::DPAABB3(DPAABB3 const& copyFrom)
{
	m_mins = copyFrom.m_mins;
	m_maxs = copyFrom.m_maxs;
}

//-----------------------------------------------------------------------------------------------
DPAABB3::DPAABB3(double minX, double minY, double minZ, double maxX, double maxY, double maxZ)
{
	m_mins.x = minX;
	m_mins.y = minY;
	m_mins.z = minZ;

	m_maxs.x = maxX;
	m_maxs.y = maxY;
	m_maxs.z = maxZ;
}

//-----------------------------------------------------------------------------------------------
DPAABB3::DPAABB3(DPVec3 const& mins, DPVec3 const& maxs)
{
	m_mins = mins;
	m_maxs = maxs;
}

//-----------------------------------------------------------------------------------------------
bool DPAABB3::IsPointInside(DPVec3 const& point) const
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
DPVec3 const DPAABB3::GetNearestPoint(DPVec3 const& referencePosition) const
{
	DPVec3 nearestPoint;

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
DPVec3 const DPAABB3::GetDimensions() const
{
	DPVec3 dimensions(m_maxs.x - m_mins.x, m_maxs.y - m_mins.y, m_maxs.z - m_mins.z);
	return dimensions;
}

//-----------------------------------------------------------------------------------------------
void DPAABB3::SetCenter(DPVec3 const& newCenter)
{
	DPVec3 center = GetCenter();
	double halfOfX = m_maxs.x - center.x;
	double halfOfY = m_maxs.y - center.y;
	double halfOfZ = m_maxs.z - center.z;

	m_mins.x = newCenter.x - halfOfX;
	m_maxs.x = newCenter.x + halfOfX;
	m_mins.y = newCenter.y - halfOfY;
	m_maxs.y = newCenter.y + halfOfY;
	m_mins.z = newCenter.z - halfOfZ;
	m_maxs.z = newCenter.z + halfOfZ;
}

//-----------------------------------------------------------------------------------------------
void DPAABB3::SetDimensions(DPVec3 const& newDimensions)
{
	DPVec3 center = GetCenter();
	double originalCenterX = center.x;
	double originalCenterY = center.y;
	double originalCenterZ = center.z;

	double halfOfX = newDimensions.x * 0.5f;
	double halfOfY = newDimensions.y * 0.5f;
	double halfOfZ = newDimensions.z * 0.5f;

	m_mins.x = originalCenterX - halfOfX;
	m_maxs.x = originalCenterX + halfOfX;
	m_mins.y = originalCenterY - halfOfY;
	m_maxs.y = originalCenterY + halfOfY;
	m_mins.z = originalCenterZ - halfOfZ;
	m_maxs.z = originalCenterZ + halfOfZ;
}

//-----------------------------------------------------------------------------------------------
DPVec3 const DPAABB3::GetNearestSidePosition(DPVec3 const& referencePosition) const
{
	DPVec3 sidePos = referencePosition;
	DPVec3 center = GetCenter();
	DPVec3 halfDimensions = GetDimensions() * 0.5f;
	std::vector<double> distancesToSides;
	distancesToSides.push_back((center.x + halfDimensions.x) - referencePosition.x);
	distancesToSides.push_back(referencePosition.x - (center.x - halfDimensions.x));
	distancesToSides.push_back((center.y + halfDimensions.y) - referencePosition.y);
	distancesToSides.push_back(referencePosition.y - (center.y - halfDimensions.y));
	distancesToSides.push_back((center.z + halfDimensions.z) - referencePosition.z);
	distancesToSides.push_back(referencePosition.z - (center.z - halfDimensions.z));
	
	double bestDistance = std::numeric_limits<double>::max();
	int bestDistanceIndex = -1;
	for (int distanceIndex = 0; distanceIndex < distancesToSides.size(); distanceIndex++)
	{
		if (distancesToSides[distanceIndex] < bestDistance)
		{
			bestDistance = distancesToSides[distanceIndex];
			bestDistanceIndex = distanceIndex;
		}
	}

	if (bestDistanceIndex == 0)
	{
		sidePos.x += distancesToSides[bestDistanceIndex];
		return sidePos;
	}
	else if (bestDistanceIndex == 1)
	{
		sidePos.x -= distancesToSides[bestDistanceIndex];
		return sidePos;
	}
	else if (bestDistanceIndex == 2)
	{
		sidePos.y += distancesToSides[bestDistanceIndex];
		return sidePos;
	}
	else if (bestDistanceIndex == 3)
	{
		sidePos.y -= distancesToSides[bestDistanceIndex];
		return sidePos;
	}
	else if (bestDistanceIndex == 4)
	{
		sidePos.z += distancesToSides[bestDistanceIndex];
		return sidePos;
	}
	else if (bestDistanceIndex == 5)
	{
		sidePos.z -= distancesToSides[bestDistanceIndex];
		return sidePos;
	}

	return sidePos;
}

//-----------------------------------------------------------------------------------------------
DPVec3 const DPAABB3::GetNearestEdgePosition(DPVec3 const& referencePosition) const
{
	DPVec3 edgePos = referencePosition;
	DPVec3 center = GetCenter();
	DPVec3 halfDimensions = GetDimensions() * 0.5f;
	std::vector<double> distancesToSides;
	distancesToSides.push_back((center.x + halfDimensions.x) - referencePosition.x);
	distancesToSides.push_back(referencePosition.x - (center.x - halfDimensions.x));
	distancesToSides.push_back((center.y + halfDimensions.y) - referencePosition.y);
	distancesToSides.push_back(referencePosition.y - (center.y - halfDimensions.y));
	distancesToSides.push_back((center.z + halfDimensions.z) - referencePosition.z);
	distancesToSides.push_back(referencePosition.z - (center.z - halfDimensions.z));

	double bestDistance = std::numeric_limits<double>::max();
	int bestDistanceIndex = -1;
	double secondBestDistance = std::numeric_limits<double>::max();
	int secondBestDistanceIndex = -1;
	for (int distanceIndex = 0; distanceIndex < distancesToSides.size(); distanceIndex++)
	{
		if (distancesToSides[distanceIndex] < bestDistance)
		{
			secondBestDistance = bestDistance;
			secondBestDistanceIndex = bestDistanceIndex;
			bestDistance = distancesToSides[distanceIndex];
			bestDistanceIndex = distanceIndex;
		}
		else if (distancesToSides[distanceIndex] < secondBestDistance)
		{
			secondBestDistance = distancesToSides[distanceIndex];
			secondBestDistanceIndex = distanceIndex;
		}
	}

	//Side 1
	if (bestDistanceIndex == 0)
	{
		edgePos.x += distancesToSides[bestDistanceIndex];
	}
	else if (bestDistanceIndex == 1)
	{
		edgePos.x -= distancesToSides[bestDistanceIndex];
	}
	else if (bestDistanceIndex == 2)
	{
		edgePos.y += distancesToSides[bestDistanceIndex];
	}
	else if (bestDistanceIndex == 3)
	{
		edgePos.y -= distancesToSides[bestDistanceIndex];
	}
	else if (bestDistanceIndex == 4)
	{
		edgePos.z += distancesToSides[bestDistanceIndex];
	}
	else if (bestDistanceIndex == 5)
	{
		edgePos.z -= distancesToSides[bestDistanceIndex];
	}

	//Side 2
	if (secondBestDistanceIndex == 0)
	{
		edgePos.x += distancesToSides[secondBestDistanceIndex];
		return edgePos;
	}
	else if (secondBestDistanceIndex == 1)
	{
		edgePos.x -= distancesToSides[secondBestDistanceIndex];
		return edgePos;
	}
	else if (secondBestDistanceIndex == 2)
	{
		edgePos.y += distancesToSides[secondBestDistanceIndex];
		return edgePos;
	}
	else if (secondBestDistanceIndex == 3)
	{
		edgePos.y -= distancesToSides[secondBestDistanceIndex];
		return edgePos;
	}
	else if (secondBestDistanceIndex == 4)
	{
		edgePos.z += distancesToSides[secondBestDistanceIndex];
		return edgePos;
	}
	else if (secondBestDistanceIndex == 5)
	{
		edgePos.z -= distancesToSides[secondBestDistanceIndex];
		return edgePos;
	}

	return edgePos;
}

//-----------------------------------------------------------------------------------------------
DPVec3 DPAABB3::GetCenter() const
{
	DPVec3 center = (m_mins + m_maxs) * 0.5f;
	return center;
}
