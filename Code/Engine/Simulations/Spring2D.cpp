#include "Spring2D.hpp"

//-----------------------------------------------------------------------------------------------
Spring2D::Spring2D()
{
}

//-----------------------------------------------------------------------------------------------
Spring2D::~Spring2D()
{
}

//-----------------------------------------------------------------------------------------------
Spring2D::Spring2D(const Spring2D& copyFrom)
{
	m_pointA = copyFrom.m_pointA;
	m_pointB = copyFrom.m_pointB;
	m_stiffness = copyFrom.m_stiffness;
	m_initialLength = copyFrom.m_initialLength;
}

//-----------------------------------------------------------------------------------------------
Spring2D::Spring2D(Point2D* pointA, Point2D* pointB, float stiffness, float initialLength)
	:m_pointA(pointA)
	,m_pointB(pointB)
	,m_stiffness(stiffness)
	,m_initialLength(initialLength)
{
}
