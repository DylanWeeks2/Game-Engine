#include "Constraint3D.hpp"

//-----------------------------------------------------------------------------------------------
Constraint3D::Constraint3D(const Constraint3D& copyFrom)
{
	m_indices = copyFrom.m_indices;
	m_cardinality = copyFrom.m_cardinality;
	m_constraintEquality = copyFrom.m_constraintEquality;
	m_constraintType = copyFrom.m_constraintType;
	m_stiffnessParameter = copyFrom.m_stiffnessParameter;
}
