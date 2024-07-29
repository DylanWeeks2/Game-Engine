#pragma once
#include "Engine/Math/Vec3.hpp"
#include "Engine/Math/Mat33.hpp"
#include <vector>
#include <string>

//-----------------------------------------------------------------------------------------------
class	Renderer;
struct	Vertex_PCUTBN;
struct  Collider3D;

//-----------------------------------------------------------------------------------------------
class RigidBody3D
{
public:
	RigidBody3D(std::string xmlFileName, float totalMass);
	RigidBody3D() {}
	~RigidBody3D() {}
	
	void				Update(float deltaSeconds);
	void				ComputeForcesAndTorque();
	void				Integrate(float deltaSeconds);
	void				ApplyForceAndTorque(Vec3 const& force, Vec3 const& impactLocation);
	void				ApplyImpulse(Vec3 const& force, Vec3 const& impactLocation);
	void				DebudRender(Renderer* renderer) const;
	
public:
	Collider3D*			m_collider = nullptr;
	Vec3				m_position;
	Vec3				m_velocity;
	Vec3				m_linearMomentum;
	Vec3				m_force;									// N
	Vec3				m_gravity = Vec3(0.0f, 0.0f, -9.81f);		// m/s^2
	Mat33				m_rotation;		
	Mat33				m_inertiaTensor;
	Mat33				m_inverseInertiaTensor;
	Vec3				m_angularVelocity;							
	Vec3				m_angularMomentum;
	Vec3				m_torque;
	float				m_mass = 0.0f;								// kg
	float				m_Us = 0.9f;								// static friction coefficient
	float				m_Uk = 0.75f;								// kinetic friction coefficient

	Vec3				m_penetration;
	bool				m_isGravityEnabled = false;
};