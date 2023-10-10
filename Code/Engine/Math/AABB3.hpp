#pragma once
#include "Vec3.hpp"

struct AABB3
{
public:
	Vec3 m_mins;
	Vec3 m_maxs;

public:
	AABB3();
	~AABB3();
	AABB3(AABB3 const& copyFrom);
	explicit AABB3(float minX, float minY, float minZ, float maxX, float maxY, float maxZ);
	explicit AABB3(Vec3 const& mins, Vec3 const& maxs);

	bool IsPointInside(Vec3 const& point) const;
	Vec3 const GetNearestPoint(Vec3 const& referencePosition) const;
	Vec3 const GetDimensions() const;
	void SetCenter(Vec3 const& newCenter);
	void SetDimensions(Vec3 const& newDimensions);

	Vec3 GetCenter() const;
};