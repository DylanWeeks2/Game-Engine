#include "StructuredBuffer.hpp"
#include "Renderer.hpp"
#include <d3d11.h>

#pragma comment (lib, "d3d11.lib")

//------------------------------------------------------------------------------------------------
StructuredBuffer::StructuredBuffer(size_t size, unsigned int stride)
{
	m_size = size;
	m_stride = stride;
}

//------------------------------------------------------------------------------------------------
StructuredBuffer::~StructuredBuffer()
{
	DX_SAFE_RELEASE(m_SRV);
	DX_SAFE_RELEASE(m_UAV);
	DX_SAFE_RELEASE(m_buffer);
}

//------------------------------------------------------------------------------------------------
unsigned int StructuredBuffer::GetStride()
{
	return m_stride;
}
