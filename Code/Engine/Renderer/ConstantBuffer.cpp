#include "ConstantBuffer.hpp"
#include "Renderer.hpp"
#include <d3d11.h>

#pragma comment (lib, "d3d11.lib")

//------------------------------------------------------------------------------------------------
ConstantBuffer::ConstantBuffer(size_t size)
{
	m_size = size;
}

//------------------------------------------------------------------------------------------------
ConstantBuffer::~ConstantBuffer()
{
	DX_SAFE_RELEASE(m_buffer);
}
