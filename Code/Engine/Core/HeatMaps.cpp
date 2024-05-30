#include "HeatMaps.hpp"

//-----------------------------------------------------------------------------------------------
TileHeatMap::TileHeatMap()
{
}

//-----------------------------------------------------------------------------------------------
TileHeatMap::TileHeatMap(IntVec2 const& dimensions)
	:m_dimensions(dimensions)
{
	m_values.resize(dimensions.x * dimensions.y);
}

//-----------------------------------------------------------------------------------------------
void TileHeatMap::SetAllValues(float resetValue)
{
	for (int index = 0; index < m_values.size(); index++)
	{
		m_values[index] = resetValue;
	}
}

//-----------------------------------------------------------------------------------------------
float TileHeatMap::GetHeatValue(IntVec2 const& coord) const
{
	int index = coord.x + (coord.y * m_dimensions.x);
	return m_values[index];
}
//-----------------------------------------------------------------------------------------------

void TileHeatMap::SetHeatValue(IntVec2 const& coord, float heatValue)
{
	int index = coord.x + (coord.y * m_dimensions.x);
	m_values[index] = heatValue;
}

//-----------------------------------------------------------------------------------------------
void TileHeatMap::AddHeatValue(IntVec2 const& coord, float heatValue)
{
	int index = coord.x + (coord.y * m_dimensions.x);
	m_values[index] += heatValue;
}

//-----------------------------------------------------------------------------------------------
float TileHeatMap::GetMaxHeatValue(float maxCost) const
{
	float max = 0.0f;
	for (int index = 0; index < m_values.size(); index++)
	{
		if (m_values[index] >= max && m_values[index] != maxCost)
		{
			max = m_values[index];
		}
	}

	return max;
}
