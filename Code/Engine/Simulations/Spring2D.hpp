#pragma once

//-----------------------------------------------------------------------------------------------
struct Particle2D;

//-----------------------------------------------------------------------------------------------
struct Spring2D
{
public:
	Spring2D();
	~Spring2D();
	Spring2D(const Spring2D& copyFrom); 
	explicit Spring2D(Particle2D* particleA, Particle2D* particleB, float stiffness, float initialLength);

public:
	Particle2D*	m_particleA = nullptr;
	Particle2D*	m_particleB = nullptr;
	float		m_stiffness = 0.0f;
	float		m_initialLength = 0.0f;
};