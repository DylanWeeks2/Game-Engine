#pragma once
#include "Engine/Math/IntVec2.hpp"
#include <vector>


class TileHeatMap
{
public:
	TileHeatMap();
	TileHeatMap(IntVec2 const& dimensions);

	void SetAllValues(float resetValue);
	float GetHeatValue(IntVec2 const& coord) const;
	void SetHeatValue(IntVec2 const& coord, float heatValue);
	void AddHeatValue(IntVec2 const& coord, float heatValue);
	float GetMaxHeatValue(float maxCost) const;

public:
	IntVec2 m_dimensions;
	std::vector<float> m_values = {};
};