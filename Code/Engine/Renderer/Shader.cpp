#include "Shader.hpp"
#include <d3d11.h>

#pragma comment (lib, "d3d11.lib")

//-----------------------------------------------------------------------------------------------
Shader::Shader(const ShaderConfig& config)
{
	m_config = config;
}

//-----------------------------------------------------------------------------------------------
Shader::~Shader()
{
	DX_SAFE_RELEASE(m_inputLayout);
	DX_SAFE_RELEASE(m_pixelShader);
	DX_SAFE_RELEASE(m_vertexShader);
}

//-----------------------------------------------------------------------------------------------
const std::string& Shader::GetName() const
{
	return m_config.m_name;
}
