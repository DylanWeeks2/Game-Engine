#include "GPUMesh.hpp"
#include "VertexBuffer.hpp"
#include "IndexBuffer.hpp"

//-----------------------------------------------------------------------------------------------
GPUMesh::GPUMesh()
{
}

//-----------------------------------------------------------------------------------------------
GPUMesh::~GPUMesh()
{
	delete m_vertexBuffer;
	m_vertexBuffer = nullptr;
	delete m_indexBuffer;
	m_indexBuffer = nullptr;
}
