#pragma once
#include "Vec2.hpp"
#include <vector>

//-----------------------------------------------------------------------------------------------
struct AABB2
{
public:
	Vec2 m_mins;
	Vec2 m_maxs;

public:
	//construction/destruction
	~AABB2() {}
	AABB2() {}
	AABB2(AABB2 const& copyFrom);
	explicit AABB2(float minX, float minY, float maxX, float maxY);
	explicit AABB2(Vec2 const& mins, Vec2 const& maxs);

	//accessors (const methods)
	bool				IsPointInside(Vec2 const& point) const;
	Vec2 const			GetCenter() const;
	Vec2 const			GetDimensions() const;
	Vec2 const			GetNearestPoint(Vec2 const& referencePosition) const;
	Vec2 const			GetPointAtUV(Vec2 const& uv) const;
	Vec2 const			GetUVForPoint(Vec2 const& point) const;

	//mutators (non-const methods)
	void				Translate(Vec2 const& translationToApply);
	void				SetCenter(Vec2 const& newCenter);
	void				SetDimensions(Vec2 const& newDimensions);
	void				StretchToIncludePoint(Vec2 const& point);
	std::vector<AABB2>	DivideIntoPanes(int numberOfPanes);
	std::vector<AABB2>	CreateTable(int numberOfRows, int numberOfColumns);
	void				AddPaddingLeft(float padding);
	void				AddPaddingRight(float padding);
	void				AddPaddingTop(float padding);
	void				AddPaddingBottom(float padding);

public:
	static AABB2 ZERO_TO_ONE;
};