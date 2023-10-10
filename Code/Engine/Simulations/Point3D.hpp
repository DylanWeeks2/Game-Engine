#pragma once
#include "Engine/Math/Vec3.hpp"

struct Point3D
{
public:
	Point3D();
	~Point3D();
	Point3D(const Point3D& copyFrom);
	explicit Point3D(float mass, float inverseMass, Vec3 position, Vec3 velocity = Vec3(), bool isLocked = false);

public:
	float m_mass = 0.0f;
	float m_inverseMass = 0.0f;
	float m_forcesAsFloat = 0.0f;
	Vec3 m_position;
	Vec3 m_velocity;
	Vec3 m_forces;
	Vec3 m_previousPosition; //Verlet Integration
	Vec3 m_proposedPosition; //Position Based Dynamics
	bool m_isLocked = false;
	bool m_isGrabbed = false;
};