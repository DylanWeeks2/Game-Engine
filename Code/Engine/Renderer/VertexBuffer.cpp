#include "VertexBuffer.hpp"
#include "Renderer.hpp"
#include "Engine/Core/Vertex_PCU.hpp"
#include <d3d11.h>

#pragma comment (lib, "d3d11.lib")

//-----------------------------------------------------------------------------------------------
VertexBuffer::VertexBuffer(size_t size, unsigned int stride)
	:m_size(size)
	,m_stride(stride)
{
}

//-----------------------------------------------------------------------------------------------
VertexBuffer::~VertexBuffer()
{
	DX_SAFE_RELEASE(m_buffer);
}

//-----------------------------------------------------------------------------------------------
unsigned int VertexBuffer::GetStride() const
{
	return m_stride;
}
