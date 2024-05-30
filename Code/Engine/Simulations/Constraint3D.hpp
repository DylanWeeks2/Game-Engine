#pragma once
#include <vector>

//-----------------------------------------------------------------------------------------------
enum class Constraint3DType
{
	DISTANCE, 
	BENDING,
	OBJECT_COLLISION,
	SELF_COLLISION,
	COUNT
};

//-----------------------------------------------------------------------------------------------
enum class Constraint3DEquality
{
	EQUALITY,
	INEQUALITY_GREATER,
	INEQUALITY_LESS,
	COUNT
};

//-----------------------------------------------------------------------------------------------
struct Constraint3D
{
public:
	Constraint3D() {}
	~Constraint3D() {}
	Constraint3D(const Constraint3D& copyFrom);

public:
	std::vector<int>		m_indices;
	int						m_cardinality = 1;
	Constraint3DType		m_constraintType = Constraint3DType::DISTANCE;
	Constraint3DEquality	m_constraintEquality = Constraint3DEquality::EQUALITY;
	double					m_stiffnessParameter = 1.0f;
};