#pragma once

//------------------------------------------------------------------------------------------------
struct ID3D11UnorderedAccessView; 
struct ID3D11ShaderResourceView;
struct ID3D11Buffer;

//------------------------------------------------------------------------------------------------
class StructuredBuffer
{
	friend class Renderer;

public:
	StructuredBuffer(size_t size, unsigned int stride);
	StructuredBuffer(const StructuredBuffer& copy) = delete;
	~StructuredBuffer();

	unsigned int				GetStride();

public:
	ID3D11Buffer*				m_buffer = nullptr;
	ID3D11UnorderedAccessView*	m_UAV = nullptr;
	ID3D11ShaderResourceView*	m_SRV = nullptr;
	size_t						m_size = 0;
	unsigned int				m_stride = 0;
};