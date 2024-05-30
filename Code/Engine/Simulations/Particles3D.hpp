#pragma once
#include "Engine/Math/Vec3.hpp"
#include "Engine/Math/Vec4.hpp"
#include <vector>
#include <cstdint>

//SOA Format
//-----------------------------------------------------------------------------------------------
struct Particles3D
{
public:
	Particles3D();
	~Particles3D();
	Particles3D(const Particles3D& copyFrom);

public:
	std::vector<Vec3>		m_positions;
	std::vector<Vec3>		m_velocities;
	std::vector<Vec3>		m_proposedPositions;
	std::vector<Vec4>		m_jacobiCorrections;
	std::vector<Vec3>		m_collisionNormals;
	std::vector<uint64_t>	m_macroBitRegions;
	std::vector<uint64_t>	m_microBitRegions;
	std::vector<float>		m_masses;
	std::vector<float>		m_inverseMasses;
	std::vector<int>		m_isAttached;
	std::vector<int>		m_isSelfCollision;
};