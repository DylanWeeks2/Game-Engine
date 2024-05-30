#pragma once
#include <vector>

//-----------------------------------------------------------------------------------------------
class	VertexBuffer;
class	IndexBuffer;
struct	Vertex_PCUTBN;

//-----------------------------------------------------------------------------------------------
class GPUMesh
{
public:
	GPUMesh();
	~GPUMesh();

public:
	VertexBuffer*	m_vertexBuffer = nullptr;
	IndexBuffer*	m_indexBuffer = nullptr;
};