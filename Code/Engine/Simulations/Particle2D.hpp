#pragma once
#include "Engine/Math/Vec2.hpp"

//-----------------------------------------------------------------------------------------------
struct Particle2D
{
public:
	Particle2D();
	~Particle2D();
	Particle2D(const Particle2D& copyFrom);
	explicit Particle2D(float mass, float inverseMass, Vec2 position, Vec2 velocity = Vec2(), bool isLocked = false);

public:
	float	m_mass = 0.0f;
	float	m_inverseMass = 0.0f;
	float	m_forcesAsFloat = 0.0f;
	Vec2	m_position;
	Vec2	m_velocity;
	Vec2	m_forces;
	Vec2	m_previousPosition; //Verlet Integration
	Vec2	m_proposedPosition; //Position Based Dynamics
	bool	m_isLocked = false;
};