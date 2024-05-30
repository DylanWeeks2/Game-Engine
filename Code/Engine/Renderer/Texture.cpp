#include "Texture.hpp"
#include "Renderer.hpp"
#include <d3d11.h>

#pragma comment (lib, "d3d11.lib")

//------------------------------------------------------------------------------------------------
Texture::Texture()
{
}

//------------------------------------------------------------------------------------------------
Texture::~Texture()
{
	DX_SAFE_RELEASE(m_shaderResourceView);
	DX_SAFE_RELEASE(m_texture);
}
