#include "IndexBuffer.hpp"
#include "Renderer.hpp"
#include "Engine/Core/Vertex_PCU.hpp"
#include <d3d11.h>

#pragma comment (lib, "d3d11.lib")

//-----------------------------------------------------------------------------------------------
IndexBuffer::IndexBuffer(size_t size)
{
	m_size = size;
}

//-----------------------------------------------------------------------------------------------
IndexBuffer::~IndexBuffer()
{
	DX_SAFE_RELEASE(m_buffer);
}

//-----------------------------------------------------------------------------------------------
unsigned int IndexBuffer::GetStride() const
{
	return sizeof(Vertex_PCU);
}
