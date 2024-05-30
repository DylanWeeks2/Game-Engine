#pragma once
#include "DPVec3.hpp"

//-----------------------------------------------------------------------------------------------
struct DPAABB3
{
public:
	DPVec3 m_mins;
	DPVec3 m_maxs;

public:
	DPAABB3();
	~DPAABB3();
	DPAABB3(DPAABB3 const& copyFrom);
	explicit DPAABB3(double minX, double minY, double minZ, double maxX, double maxY, double maxZ);
	explicit DPAABB3(DPVec3 const& mins, DPVec3 const& maxs);

	bool			IsPointInside(DPVec3 const& point) const;
	DPVec3 const	GetNearestPoint(DPVec3 const& referencePosition) const;
	DPVec3 const	GetDimensions() const;
	void			SetCenter(DPVec3 const& newCenter);
	void			SetDimensions(DPVec3 const& newDimensions);
	DPVec3 const	GetNearestSidePosition(DPVec3 const& referencePosition) const;
	DPVec3 const	GetNearestEdgePosition(DPVec3 const& referencePosition) const;

	DPVec3 GetCenter() const;
};