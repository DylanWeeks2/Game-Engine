#pragma once

//-----------------------------------------------------------------------------------------------
struct Point2D;

//-----------------------------------------------------------------------------------------------
struct Spring2D
{
public:
	Spring2D();
	~Spring2D();
	Spring2D(const Spring2D& copyFrom); 
	explicit Spring2D(Point2D* pointA, Point2D* pointB, float stiffness, float initialLength);

public:
	Point2D*	m_pointA = nullptr;
	Point2D*	m_pointB = nullptr;
	float		m_stiffness = 0.0f;
	float		m_initialLength = 0.0f;
};