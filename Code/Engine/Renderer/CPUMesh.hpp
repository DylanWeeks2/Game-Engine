#pragma once
#include <vector>
#include "Engine/Core/Vertex_PNCU.hpp"

//-----------------------------------------------------------------------------------------------
struct Vertex_PNCU;

//-----------------------------------------------------------------------------------------------
class CPUMesh
{
public:
	CPUMesh(std::vector<Vertex_PNCU>& vertices, std::vector<int>& indices);
	CPUMesh();
	~CPUMesh();

public:
	std::vector<Vertex_PNCU> m_vertices;
	std::vector<int> m_indices;
};