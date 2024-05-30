#pragma once
#include <vector>
#include "Engine/Core/Vertex_PCUTBN.hpp"

//-----------------------------------------------------------------------------------------------
struct Vertex_PCUTBN;

//-----------------------------------------------------------------------------------------------
class CPUMesh
{
public:
	CPUMesh(std::vector<Vertex_PCUTBN>& vertices, std::vector<unsigned int>& indices);
	CPUMesh();
	~CPUMesh();

public:
	std::vector<Vertex_PCUTBN>	m_vertices;
	std::vector<unsigned int>	m_indices;
};