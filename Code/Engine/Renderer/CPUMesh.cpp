#include "CPUMesh.hpp"

//-----------------------------------------------------------------------------------------------
CPUMesh::CPUMesh(std::vector<Vertex_PCUTBN>& vertices, std::vector<unsigned int>& indices)
	:m_vertices(vertices)
	,m_indices(indices)
{
}

//-----------------------------------------------------------------------------------------------
CPUMesh::CPUMesh()
{
}

//-----------------------------------------------------------------------------------------------
CPUMesh::~CPUMesh()
{
}
