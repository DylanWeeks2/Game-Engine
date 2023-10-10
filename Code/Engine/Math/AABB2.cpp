#include "AABB2.hpp"
#include "Vec2.hpp"
#include "MathUtils.hpp"

AABB2 AABB2::ZERO_TO_ONE = AABB2(Vec2(0.0f, 0.0f), Vec2(1.0f, 1.0f));

//-----------------------------------------------------------------------------------------------
AABB2::AABB2(AABB2 const& copyFrom)
	:m_mins(copyFrom.m_mins)
	,m_maxs(copyFrom.m_maxs)
{
}

//-----------------------------------------------------------------------------------------------
AABB2::AABB2(float minX, float minY, float maxX, float maxY)
	:m_mins(Vec2(minX, minY))
	,m_maxs(Vec2(maxX, maxY))
{
}

//-----------------------------------------------------------------------------------------------
AABB2::AABB2(Vec2 const& mins, Vec2 const& maxs)
	: m_mins(mins)
	, m_maxs(maxs)
{
}

//-----------------------------------------------------------------------------------------------
bool AABB2::IsPointInside(Vec2 const& point) const
{
	if ((point.x > m_mins.x && point.x < m_maxs.x) && (point.y > m_mins.y && point.y < m_maxs.y))
	{
		return true;
	}

	return false;
}

//-----------------------------------------------------------------------------------------------
Vec2 const AABB2::GetCenter() const
{
	Vec2 center = (m_mins + m_maxs) * 0.5f;
	return center;
}

//-----------------------------------------------------------------------------------------------
Vec2 const AABB2::GetDimensions() const
{
	Vec2 dimensions(m_maxs.x - m_mins.x, m_maxs.y - m_mins.y);
	return dimensions;
}

//-----------------------------------------------------------------------------------------------
Vec2 const AABB2::GetNearestPoint(Vec2 const& referencePosition) const
{
	Vec2 nearestPoint;
	nearestPoint.x = GetClamped(referencePosition.x, m_mins.x, m_maxs.x);
	nearestPoint.y = GetClamped(referencePosition.y, m_mins.y, m_maxs.y);
	return nearestPoint;
}

//-----------------------------------------------------------------------------------------------
Vec2 const AABB2::GetPointAtUV(Vec2 const& uv) const
{
	Vec2 point;
	point.x = RangeMap(uv.x, 0.f, 1.f, m_mins.x, m_maxs.x);
	point.y = RangeMap(uv.y, 0.f, 1.f, m_mins.y, m_maxs.y);

	return point;
}

//-----------------------------------------------------------------------------------------------
Vec2 const AABB2::GetUVForPoint(Vec2 const& point) const
{
	Vec2 uv;
	uv.x = RangeMap(point.x, m_mins.x, m_maxs.x, 0.f, 1.f);
	uv.y = RangeMap(point.y, m_mins.y, m_maxs.y, 0.f, 1.f);
	
	return uv;
}

//-----------------------------------------------------------------------------------------------
void AABB2::Translate(Vec2 const& translationToApply)
{
	m_mins.x += translationToApply.x;
	m_mins.y += translationToApply.y;

	m_maxs.x += translationToApply.x;
	m_maxs.y += translationToApply.y;
}

//-----------------------------------------------------------------------------------------------
void AABB2::SetCenter(Vec2 const& newCenter)
{
	float halfOfWidth = m_maxs.x - GetCenter().x;
	float halfOfHeigth = m_maxs.y - GetCenter().y;

	m_mins.x = newCenter.x - halfOfWidth;
	m_maxs.x = newCenter.x + halfOfWidth;
	m_mins.y = newCenter.y - halfOfHeigth;
	m_maxs.y = newCenter.y + halfOfHeigth;
}

//-----------------------------------------------------------------------------------------------
void AABB2::SetDimensions(Vec2 const& newDimensions)
{
	float originalCenterX = GetCenter().x;
	float originalCenterY = GetCenter().y;

	float halfOfWidth = newDimensions.x * .5f;
	float halfOfHeigth = newDimensions.y * .5f;

	m_mins.x = originalCenterX - halfOfWidth;
	m_maxs.x = originalCenterX + halfOfWidth;
	m_mins.y = originalCenterY - halfOfHeigth;
	m_maxs.y = originalCenterY + halfOfHeigth;
}

//-----------------------------------------------------------------------------------------------
void AABB2::StretchToIncludePoint(Vec2 const& point)
{
	if (point.x < m_mins.x)
	{
		m_mins.x = point.x;
	}
	else if(point.x > m_maxs.x)
	{
		m_maxs.x = point.x;
	}

	if (point.y < m_mins.y)
	{
		m_mins.y = point.y;
	}
	else if (point.y > m_maxs.y)
	{
		m_maxs.y = point.y;
	}
}

//-----------------------------------------------------------------------------------------------
std::vector<AABB2> AABB2::DivideIntoPanes(int numberOfPanes)
{
	std::vector<AABB2> panes;

	int halfPaneNumber = numberOfPanes / 2;
	for (int paneIndex = 0; paneIndex < numberOfPanes; paneIndex++)
	{
		AABB2 newPane;
		//check if even number
		if (numberOfPanes % 2 != 0) //odd
		{
			if (paneIndex < halfPaneNumber + 1)
			{
				newPane.m_mins = Vec2(static_cast<float>(paneIndex) / static_cast<float>(halfPaneNumber + 1) * m_maxs.x, m_maxs.y * 0.5f);
				newPane.m_maxs = Vec2(static_cast<float>(paneIndex + 1) / static_cast<float>(halfPaneNumber + 1) * m_maxs.x, m_maxs.y);
			}
			else
			{
				newPane.m_mins = Vec2(static_cast<float>(paneIndex - (halfPaneNumber + 1)) / static_cast<float>(halfPaneNumber) * m_maxs.x, 0.0f);
				newPane.m_maxs = Vec2(static_cast<float>(paneIndex - (halfPaneNumber + 1) + 1) / static_cast<float>(halfPaneNumber) * m_maxs.x, m_maxs.y * 0.5f);
			}
		}
		else //even
		{
			if (paneIndex < halfPaneNumber)
			{
				newPane.m_mins = Vec2(static_cast<float>(paneIndex) / static_cast<float>(halfPaneNumber) * m_maxs.x, m_maxs.y * 0.5f);
				newPane.m_maxs = Vec2((static_cast<float>(paneIndex + 1) / static_cast<float>(halfPaneNumber)) * m_maxs.x, m_maxs.y);
			}
			else
			{
				newPane.m_mins = Vec2(static_cast<float>(paneIndex - halfPaneNumber) / static_cast<float>(halfPaneNumber) * m_maxs.x, 0.0f);
				newPane.m_maxs = Vec2((static_cast<float>(paneIndex - halfPaneNumber + 1) / static_cast<float>(halfPaneNumber)) * m_maxs.x, m_maxs.y * 0.5f);
			}
		}

		panes.push_back(newPane);
	}

	return panes;
}

//-----------------------------------------------------------------------------------------------
std::vector<AABB2> AABB2::CreateTable(int numberOfRows, int numberOfColumns)
{
	std::vector<AABB2> cells;
	cells.reserve(numberOfColumns * numberOfRows);
	float sizeX = (m_maxs.x - m_mins.x) / numberOfColumns;
	float sizeY = (m_maxs.y - m_mins.y) / numberOfRows;

	for (int yIndex = 0; yIndex < numberOfRows; yIndex++)
	{
		AABB2 cell;
		cell.m_mins.y = m_mins.y + (yIndex * sizeY);
		cell.m_maxs.y = cell.m_mins.y + sizeY;

		for (int xIndex = 0; xIndex < numberOfColumns; xIndex++)
		{
			cell.m_mins.x = m_mins.x + (xIndex * sizeX);
			cell.m_maxs.x = cell.m_mins.x + sizeX;
			cells.push_back(cell);
		}
	}

	return cells;
}

//-----------------------------------------------------------------------------------------------
void AABB2::AddPaddingLeft(float padding)
{
	m_mins.x += padding;
}

//-----------------------------------------------------------------------------------------------
void AABB2::AddPaddingRight(float padding)
{
	m_maxs.x -= padding;
}

//-----------------------------------------------------------------------------------------------
void AABB2::AddPaddingTop(float padding)
{
	m_maxs.y -= padding;
}

//-----------------------------------------------------------------------------------------------
void AABB2::AddPaddingBottom(float padding)
{
	m_mins.y += padding;
}
