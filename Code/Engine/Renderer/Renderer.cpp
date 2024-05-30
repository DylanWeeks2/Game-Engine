#include "ThirdParty/stb/stb_image.h"
#include "Renderer.hpp"
#include "Camera.hpp"
#include "Texture.hpp"
#include "BitmapFont.hpp"
#include "Shader.hpp"
#include "VertexBuffer.hpp"
#include "IndexBuffer.hpp"
#include "PointLight.hpp"
#include "DefaultShader.hpp"
#include "ConstantBuffer.hpp"
#include "StructuredBuffer.hpp"
#include "Query.hpp"
#include "Engine/Core/Image.hpp"
#include "Engine/Core/ErrorWarningAssert.hpp"
#include "Engine/Core/Rgba8.hpp"
#include "Engine/Core/Vertex_PCU.hpp"
#include "Engine/Core/StringUtils.hpp"
#include "Engine/Window/Window.hpp"
#include "Engine/Core/FileUtils.hpp"
#include "Game/EngineBuildPreferences.hpp"
#include "Engine/Core/Vertex_PCUTBN.hpp"
#include "Engine/Math/Vec4.hpp"
#include "Engine/Math/DPMat44.hpp"
#include <windows.h>			// #include this (massive, platform-specific) header in very few places

//-----------------------------------------------------------------------------------------------
//IMGUI
#include "ThirdParty/ImGui/imgui.h"
#include "ThirdParty/ImGui/imgui_impl_win32.h"
#include "ThirdParty/ImGui/imgui_impl_dx11.h"

//-----------------------------------------------------------------------------------------------
//DIRECTX 11 SETUP CODE
#define WIN32_LEAN_AND_MEAN		// Always #define this before #including <windows.h>
#include <windows.h>			// #include this (massive, platform-specific) header in very few places
#include <d3d11.h>
#include <dxgi.h>
#include <d3dcompiler.h>

#pragma comment (lib, "d3d11.lib")
#pragma comment (lib, "dxgi.lib")
#pragma comment (lib, "d3dcompiler.lib")

#if defined(ENGINE_DEBUG_RENDER)
#include <dxgidebug.h>
#pragma comment(lib, "dxguid.lib")
#endif

#if defined(OPAQUE)
#undef OPAQUE
#endif

//-----------------------------------------------------------------------------------------------
struct CameraConstants
{
	Mat44 ProjectionMatrix;
	Mat44 ViewMatrix;
};
static const int k_cameraConstantsSlot = 2;

//-----------------------------------------------------------------------------------------------
struct ModelConstants
{
	Mat44 ModelMatrix;
	float ModelColor[4];
};
static const int k_modelConstantsSlot = 3;

//-----------------------------------------------------------------------------------------------
struct LightConstants
{
	Vec3 SunDirection;
	float SunIntensity;
	float AmbientIntensity;
	Vec3 WorldEyePosition;
	float SunColor[4];
	float AmbientColor[4];
	float PointLightColor[4];
	Vec4 PointLightRanges[100];
	Vec4 PointLightIntensities[100];
	Vec4 PointLightPositions[100];
	int NormalMode;
	int SpecularMode;
	float SpecularIntensity;
	float SpecularPower;
};
static const int k_lightConstantsSlot = 1;

//-----------------------------------------------------------------------------------------------
Renderer::Renderer(RendererConfig const& config)
	:m_config(config)
{
}

//-----------------------------------------------------------------------------------------------
Renderer::~Renderer()
{
}

//-----------------------------------------------------------------------------------------------
void Renderer::Startup()
{
	//DIRECTX 11 BELOW!!

	//Loading DXGI DLL
#if defined(ENGINE_DEBUG_RENDER)
	m_dxgiDebugModule = (void*) ::LoadLibraryA("dxgidebug.dll");
	if (m_dxgiDebugModule == nullptr)
	{
		ERROR_AND_DIE("Could not load dxgidebug.dll.");
	}

	typedef HRESULT(WINAPI* GetDebugModuleCB)(REFIID, void**);
	((GetDebugModuleCB) ::GetProcAddress((HMODULE)m_dxgiDebugModule, "DXGIGetDebugInterface"))(__uuidof(IDXGIDebug), &m_dxgiDebug);

	if (m_dxgiDebug == nullptr)
	{
		ERROR_AND_DIE("Could not load debug module.");
	}
#endif

	//Creating device and the swap chain
	HRESULT hr;

	DXGI_SWAP_CHAIN_DESC swapChainDesc = { 0 };
	swapChainDesc.BufferDesc.Width = m_config.m_window->GetClientDimensions().x;
	swapChainDesc.BufferDesc.Height = m_config.m_window->GetClientDimensions().y;
	swapChainDesc.BufferDesc.Format = DXGI_FORMAT_R8G8B8A8_UNORM;
	swapChainDesc.SampleDesc.Count = 1;
	swapChainDesc.BufferUsage = DXGI_USAGE_RENDER_TARGET_OUTPUT;
	swapChainDesc.BufferCount = 2;
	swapChainDesc.OutputWindow = static_cast<HWND>(m_config.m_window->GetHwnd());
	swapChainDesc.Windowed = true;
	swapChainDesc.SwapEffect = DXGI_SWAP_EFFECT_FLIP_DISCARD;

	unsigned int deviceFlags = 0;
#if defined(ENGINE_DEBUG_RENDER)
	deviceFlags |= D3D11_CREATE_DEVICE_DEBUG;
#endif

	hr = D3D11CreateDeviceAndSwapChain(nullptr, D3D_DRIVER_TYPE_HARDWARE, 0, deviceFlags, nullptr, 0, D3D11_SDK_VERSION, &swapChainDesc, &m_swapChain, &m_device, nullptr, &m_deviceContext);
	if (!SUCCEEDED(hr))
	{
		ERROR_AND_DIE("Could not create D3D11 device and swap chain.");
	}

	//Save a view of the back buffer (code given in the walk through)
	ID3D11Texture2D* backBuffer;
	hr = m_swapChain->GetBuffer(0, __uuidof(ID3D11Texture2D), (void**)&backBuffer);
	if (!SUCCEEDED(hr))
	{
		ERROR_AND_DIE("Could not get swap chain buffer.");
	}

	hr = m_device->CreateRenderTargetView(backBuffer, NULL, &m_renderTargetView);
	if (!SUCCEEDED(hr))
	{
		ERROR_AND_DIE("Could not create the render target view for swap buffer chain.")
	}

	backBuffer->Release();

	m_defaultShader = CreateShader("Default", m_shaderSource);
	BindShader(m_defaultShader);

	m_immediateVBOForVertexPCU = CreateVertexBuffer(sizeof(Vertex_PCU), sizeof(Vertex_PCU));
	m_immediateVBOForVertexPCUTBN = CreateVertexBuffer(sizeof(Vertex_PCUTBN), sizeof(Vertex_PCUTBN));

	//Set Rasterizer State
	CreateRasterizerModes();
	SetRasterizerMode(RasterizerMode::SOLID_CULL_BACK);

	m_cameraCBO = CreateConstantBuffer(sizeof(CameraConstants));
	m_modelCBO = CreateConstantBuffer(sizeof(ModelConstants));
	m_lightCBO = CreateConstantBuffer(sizeof(LightConstants));

	//Blend Mode
	CreateBlendStates();
	SetBlendMode(BlendMode::ALPHA);

	//Binding texture
	Image whiteImage(IntVec2(2, 2), Rgba8(255, 255, 255, 255));
	m_defaultTexture = CreateTextureFromImage(whiteImage);
	BindTexture(m_defaultTexture);

	//SamplerState
	CreateSamplerStates();
	SetSamplerMode(SamplerMode::POINT_CLAMP);

	D3D11_TEXTURE2D_DESC depthTextureDesc = { 0 };
	depthTextureDesc.Width = m_config.m_window->GetClientDimensions().x;
	depthTextureDesc.Height = m_config.m_window->GetClientDimensions().y;
	depthTextureDesc.MipLevels = 1;
	depthTextureDesc.ArraySize = 1;
	depthTextureDesc.Usage = D3D11_USAGE_DEFAULT;
	depthTextureDesc.Format = DXGI_FORMAT_D24_UNORM_S8_UINT;
	depthTextureDesc.BindFlags = D3D11_BIND_DEPTH_STENCIL;
	depthTextureDesc.SampleDesc.Count = 1;

	hr = m_device->CreateTexture2D(&depthTextureDesc, nullptr, &m_depthStencilTexture);
	if (!SUCCEEDED(hr))
	{
		ERROR_AND_DIE("Could not create the depth texture.")
	}

	hr = m_device->CreateDepthStencilView(m_depthStencilTexture, nullptr, &m_depthStencilView);
	if (!SUCCEEDED(hr))
	{
		ERROR_AND_DIE("Could not create the depth stencil view.")
	}

	CreateDepthModes();
	SetDepthMode(DepthMode::ENABLED);

	//IMGUI
	if (m_config.m_window->GetConfig().m_isUsingIMGUI == true)
	{
		ImGui_ImplDX11_Init(m_device, m_deviceContext);
	}
}

//-----------------------------------------------------------------------------------------------
void Renderer::BeginFrame()
{
	m_deviceContext->OMSetRenderTargets(1, &m_renderTargetView, m_depthStencilView);
}

//-----------------------------------------------------------------------------------------------
void Renderer::EndFrame()
{
	HRESULT hr;
	hr = m_swapChain->Present(0, 0);
	if (hr == DXGI_ERROR_DEVICE_REMOVED || hr == DXGI_ERROR_DEVICE_RESET)
	{
		ERROR_AND_DIE("Device has been lost, applicaiton will now terminate.");
	}
}

//-----------------------------------------------------------------------------------------------
void Renderer::Shutdown()
{
	for (int shaderIndex = 0; shaderIndex < m_loadedShaders.size(); shaderIndex++)
	{
		if (m_loadedShaders[shaderIndex] != nullptr)
		{
			delete m_loadedShaders[shaderIndex];
			m_loadedShaders[shaderIndex] = nullptr;
		}
	}

	m_currentShader = nullptr;
	m_defaultShader = nullptr;

	delete m_immediateVBOForVertexPCU;
	m_immediateVBOForVertexPCU = nullptr;
	delete m_immediateVBOForVertexPCUTBN;
	m_immediateVBOForVertexPCUTBN = nullptr;
	
	delete m_cameraCBO;
	m_cameraCBO = nullptr;

	delete m_modelCBO;
	m_modelCBO = nullptr;

	delete m_lightCBO;
	m_lightCBO = nullptr;

	
	for (int textureIndex = 0; textureIndex < m_loadedTextures.size(); textureIndex++)
	{
		if (m_loadedTextures[textureIndex] != nullptr)
		{
			delete m_loadedTextures[textureIndex];
			m_loadedTextures[textureIndex] = nullptr;
		}
	}

	for (int rasterizerModeIndex = 0; rasterizerModeIndex < static_cast<int>(RasterizerMode::COUNT); rasterizerModeIndex++)
	{
		DX_SAFE_RELEASE(m_rasterizerStates[rasterizerModeIndex]);
	}

	for (int blendStateIndex = 0; blendStateIndex < static_cast<int>(BlendMode::COUNT); blendStateIndex++)
	{
		DX_SAFE_RELEASE(m_blendStates[blendStateIndex]);
	}

	for (int samplerStateIndex = 0; samplerStateIndex < static_cast<int>(SamplerMode::COUNT); samplerStateIndex++)
	{
		DX_SAFE_RELEASE(m_samplerStates[samplerStateIndex]);
	}

	for (int depthStateIndex = 0; depthStateIndex < static_cast<int>(DepthMode::COUNT); depthStateIndex++)
	{
		DX_SAFE_RELEASE(m_depthStencilStates[depthStateIndex]);
	}

	m_defaultTexture = nullptr;

	DX_SAFE_RELEASE(m_depthStencilTexture);
	DX_SAFE_RELEASE(m_depthStencilView);
	DX_SAFE_RELEASE(m_renderTargetView);
	DX_SAFE_RELEASE(m_swapChain);
	DX_SAFE_RELEASE(m_deviceContext);
	DX_SAFE_RELEASE(m_device);

	//Debug info after releasing
#if defined(ENGINE_DEBUG_RENDER)
	((IDXGIDebug*)m_dxgiDebug)->ReportLiveObjects(DXGI_DEBUG_ALL, (DXGI_DEBUG_RLO_FLAGS)(DXGI_DEBUG_RLO_DETAIL | DXGI_DEBUG_RLO_IGNORE_INTERNAL));

	((IDXGIDebug*)m_dxgiDebug)->Release();
	m_dxgiDebug = nullptr;

	::FreeLibrary((HMODULE)m_dxgiDebugModule);
	m_dxgiDebugModule = nullptr;
#endif
}

//-----------------------------------------------------------------------------------------------
void Renderer::ClearScreen(const Rgba8& clearColor)
{
	float colorAsFloats[4];
	clearColor.GetAsFloats(colorAsFloats);
	m_deviceContext->ClearRenderTargetView(m_renderTargetView, colorAsFloats);
	m_deviceContext->ClearDepthStencilView(m_depthStencilView, D3D11_CLEAR_DEPTH | D3D11_CLEAR_STENCIL, 1, 0);
}

//-----------------------------------------------------------------------------------------------
void Renderer::BeginCamera(const Camera& camera)
{
	camera;

	AABB2 fullScreenViewport;
	fullScreenViewport.m_mins = Vec2();
	fullScreenViewport.m_maxs = Vec2(static_cast<FLOAT>(m_config.m_window->GetClientDimensions().x), static_cast<FLOAT>(m_config.m_window->GetClientDimensions().y));

	D3D11_VIEWPORT viewport = { 0 };
	//DebuggerPrintf("%f, %f\n", camera.m_viewport.GetDimensions().GetLength(), fullScreenViewport.GetDimensions().GetLength());
	if (camera.m_viewport.GetDimensions().GetLength() == 0.0f && camera.m_viewport.m_mins == Vec2())
	{

		viewport.TopLeftX = 0.0f;
		viewport.TopLeftY = 0.0f;
		viewport.Width = static_cast<FLOAT>(m_config.m_window->GetClientDimensions().x);
		viewport.Height = static_cast<FLOAT>(m_config.m_window->GetClientDimensions().y);
		viewport.MinDepth = 0.0f;
		viewport.MaxDepth = 1.0f;
	}
	else
	{
		viewport.TopLeftX = camera.m_viewport.m_mins.x;
		viewport.TopLeftY = camera.m_viewport.m_mins.y;
		viewport.Width = camera.m_viewport.m_maxs.x;
		viewport.Height = camera.m_viewport.m_maxs.y;
		viewport.MinDepth = 0.0f;
		viewport.MaxDepth = 1.0f;
	}

	m_deviceContext->RSSetViewports(1, &viewport);

	//Constant Buffer
	CameraConstants cameraConstants;
	cameraConstants.ProjectionMatrix = camera.GetProjectionMatrix();
	cameraConstants.ViewMatrix = camera.GetViewMatrix();

	CopyCPUToGPU(&cameraConstants, sizeof(cameraConstants), m_cameraCBO);
	BindConstantBuffer(k_cameraConstantsSlot, m_cameraCBO);
}

//-----------------------------------------------------------------------------------------------
void Renderer::EndCamera(const Camera& camera) 
{
	camera;
}

//-----------------------------------------------------------------------------------------------
void Renderer::DrawVertexArray(int numVertexes, const Vertex_PCU* vertexes, PrimitiveTopology topology)
{
	CopyCPUToGPU(vertexes, sizeof(Vertex_PCU) * numVertexes, m_immediateVBOForVertexPCU);
	DrawVertexBuffer(m_immediateVBOForVertexPCU, numVertexes, topology, 0);
}

//-----------------------------------------------------------------------------------------------
void Renderer::DrawVertexArray(int numVertexes, const Vertex_PCUTBN* vertexes, Shader* shader, PrimitiveTopology topology)
{
	CopyCPUToGPU(vertexes, sizeof(Vertex_PCUTBN) * numVertexes, m_immediateVBOForVertexPCUTBN);
	DrawVertexBuffer(shader, m_immediateVBOForVertexPCUTBN, numVertexes, topology, 0);
}

//-----------------------------------------------------------------------------------------------
void Renderer::DrawIndexArray(int indexCount, VertexBuffer* vbo, IndexBuffer* ibo, Shader* shader, PrimitiveTopology topology)
{
	DrawIndexBuffer(shader, ibo, vbo, indexCount, topology);
}

//-----------------------------------------------------------------------------------------------
void Renderer::DrawGeometryShader(int numVerts, Shader* geometryShader, StructuredBuffer* structuredBuffer, int structuredBufferSlot, ConstantBuffer* constantBuffer, int constantBufferSlot, PrimitiveTopology topology)
{
	//Binding Shader
	BindShader(geometryShader);

	//Binding buffers to the shader
	if (structuredBuffer)
		BindStructuredBufferVSGSPS(structuredBufferSlot, structuredBuffer);
	if (constantBuffer)
		BindConstantBuffer(constantBufferSlot, constantBuffer);

	//Drawing
	m_deviceContext->IASetPrimitiveTopology(static_cast<D3D11_PRIMITIVE_TOPOLOGY>(topology));
	m_deviceContext->Draw(numVerts, 0);

	//Unbinding buffers
	UnbindStructuredBufferVSGSPS(structuredBufferSlot);
	UnbindConstantBuffer(constantBufferSlot);
}

//------------------------------------------------------------------------------------------------
void Renderer::DrawStructuredBuffer(int numVerts, Shader* shader, StructuredBuffer* structuredBuffer, int structuredBufferSlot, ConstantBuffer* constantBuffer, int constantBufferSlot, PrimitiveTopology topology)
{
	//Binding Shader
	BindShader(shader);

	//Binding buffers to the shader
	if (structuredBuffer)
		BindStructuredBufferVSGSPS(structuredBufferSlot, structuredBuffer);
	if(constantBuffer)
		BindConstantBuffer(constantBufferSlot, constantBuffer);

	//Drawing
	m_deviceContext->IASetPrimitiveTopology(static_cast<D3D11_PRIMITIVE_TOPOLOGY>(topology));
	m_deviceContext->Draw(numVerts, 0);

	//Unbinding buffers
	UnbindStructuredBufferVSGSPS(structuredBufferSlot);
	UnbindConstantBuffer(constantBufferSlot);
}

//------------------------------------------------------------------------------------------------
void Renderer::DispatchComputeShader(Shader* computeShader, int threadXCount, int threadYCount, int threadZCount)
{
	BindComputeShader(computeShader);
	m_deviceContext->Dispatch(threadXCount, threadYCount, threadZCount);
}

//-----------------------------------------------------------------------------------------------
RendererConfig const& Renderer::GetConfig() const
{
	return m_config;
}

//------------------------------------------------------------------------------------------------
Texture* Renderer::CreateTextureFromData(char const* name, IntVec2 dimensions, int bytesPerTexel, unsigned char* texelData)
{
	// Check if the load was successful
	GUARANTEE_OR_DIE(texelData, Stringf("CreateTextureFromData failed for \"%s\" - texelData was null!", name));
	GUARANTEE_OR_DIE(bytesPerTexel >= 3 && bytesPerTexel <= 4, Stringf("CreateTextureFromData failed for \"%s\" - unsupported BPP=%i (must be 3 or 4)", name, bytesPerTexel));
	GUARANTEE_OR_DIE(dimensions.x > 0 && dimensions.y > 0, Stringf("CreateTextureFromData failed for \"%s\" - illegal texture dimensions (%i x %i)", name, dimensions.x, dimensions.y));

	Texture* newTexture = new Texture();
	newTexture->m_name = name; // NOTE: m_name must be a std::string, otherwise it may point to temporary data!
	newTexture->m_dimensions = dimensions;

	m_loadedTextures.push_back(newTexture);
	return newTexture;
}

//-----------------------------------------------------------------------------------------------
void Renderer::BindTexture(Texture* texture)
{
	if (texture == nullptr)
	{
		m_deviceContext->PSSetShaderResources(0, 1, &m_defaultTexture->m_shaderResourceView);
	}
	else
	{
		m_deviceContext->PSSetShaderResources(0, 1, &texture->m_shaderResourceView);
	}
}

//------------------------------------------------------------------------------------------------
void Renderer::BindTextures(Texture* texture1, Texture* texture2, Texture* texture3)
{
	if (texture1 == nullptr && texture2 == nullptr && texture3 == nullptr)
	{
		m_deviceContext->PSSetShaderResources(0, 1, &m_defaultTexture->m_shaderResourceView);
	}
	else if(texture1 != nullptr && texture2 != nullptr && texture3 != nullptr)
	{
		m_deviceContext->PSSetShaderResources(0, 1, &texture1->m_shaderResourceView);
		m_deviceContext->PSSetShaderResources(1, 1, &texture2->m_shaderResourceView);
		m_deviceContext->PSSetShaderResources(2, 1, &texture3->m_shaderResourceView);
	}
	else if (texture1 != nullptr && texture2 != nullptr && texture3 == nullptr)
	{
		m_deviceContext->PSSetShaderResources(0, 1, &texture1->m_shaderResourceView);
		m_deviceContext->PSSetShaderResources(1, 1, &texture2->m_shaderResourceView);
	}
}

//------------------------------------------------------------------------------------------------
Texture* Renderer::CreateTextureFromFile(char const* imageFilePath)
{
	IntVec2 dimensions = IntVec2(0, 0);		// This will be filled in for us to indicate image width & height
	int bytesPerTexel = 0; // This will be filled in for us to indicate how many color components the image had (e.g. 3=RGB=24bit, 4=RGBA=32bit)
	int numComponentsRequested = 0; // don't care; we support 3 (24-bit RGB) or 4 (32-bit RGBA)

	// Load (and decompress) the image RGB(A) bytes from a file on disk into a memory buffer (array of bytes)
	stbi_set_flip_vertically_on_load(1); // We prefer uvTexCoords has origin (0,0) at BOTTOM LEFT
	unsigned char* texelData = stbi_load(imageFilePath, &dimensions.x, &dimensions.y, &bytesPerTexel, numComponentsRequested);

	// Check if the load was successful
	GUARANTEE_OR_DIE(texelData, Stringf("Failed to load image \"%s\"", imageFilePath));

	Texture* newTexture = CreateTextureFromData(imageFilePath, dimensions, bytesPerTexel, texelData);

	// Free the raw image texel data now that we've sent a copy of it down to the GPU to be stored in video memory
	stbi_image_free(texelData);

	return newTexture;
}

//------------------------------------------------------------------------------------------------
void Renderer::SetLightConstants(Vec3 sunDirection, float sunIntensity, float ambientIntensity,
	Rgba8 sunColor, Rgba8 ambientColor, std::vector<PointLight*> pointLights,
	int normalMode, int specularMode, float specularIntensity, float specularPower)
{
	LightConstants lightConstants;
	lightConstants.SunDirection = sunDirection;
	for (int lightIndex = 0; lightIndex < 100; lightIndex++)
	{
		if (lightIndex >= pointLights.size())
		{
			break;
		}
		lightConstants.PointLightPositions[lightIndex] = Vec4(pointLights[lightIndex]->m_position.x,
			pointLights[lightIndex]->m_position.y, pointLights[lightIndex]->m_position.z, 1.0f);
		lightConstants.PointLightRanges[lightIndex] = Vec4(pointLights[lightIndex]->m_range, 0.0f, 0.0f, 0.0f);
		lightConstants.PointLightIntensities[lightIndex] = Vec4(pointLights[lightIndex]->m_intensity, 0.0f, 0.0f, 0.0f);
	}
	lightConstants.SunIntensity = sunIntensity;
	lightConstants.AmbientIntensity = ambientIntensity;
	sunColor.GetAsFloats(lightConstants.SunColor);
	ambientColor.GetAsFloats(lightConstants.AmbientColor);

	if (pointLights.size() != 0)
	{
		pointLights[0]->m_color.GetAsFloats(lightConstants.PointLightColor);
	}
	else
	{
		Rgba8::WHITE.GetAsFloats(lightConstants.PointLightColor);
	}

	lightConstants.NormalMode = normalMode;
	lightConstants.SpecularMode = specularMode;
	lightConstants.SpecularIntensity = specularIntensity;
	lightConstants.SpecularPower = specularPower;

	CopyCPUToGPU(&lightConstants, sizeof(lightConstants), m_lightCBO);
	BindConstantBuffer(k_lightConstantsSlot, m_lightCBO);
}

//------------------------------------------------------------------------------------------------
Texture* Renderer::CreateOrGetTextureFromFile(char const* imageFilePath)
{
	// See if we already have this texture previously loaded
	Texture* existingTexture = GetTextureForFileName(imageFilePath);
	if (existingTexture)
	{
		return existingTexture;
	}

	// Never seen this texture before!  Let's load it.
	Image newImage(imageFilePath);
	Texture* newTexture = CreateTextureFromImage(newImage);
	return newTexture;
}

//------------------------------------------------------------------------------------------------
Texture* Renderer::GetTextureForFileName(char const* imageFilePath)
{
	for (int textureIndex = 0; textureIndex < m_loadedTextures.size(); textureIndex++)
	{
		if (m_loadedTextures[textureIndex]->GetImageFilePath() == imageFilePath)
		{
			return m_loadedTextures[textureIndex];
		}
	}

	return nullptr;
}

//------------------------------------------------------------------------------------------------
BitmapFont* Renderer::GetBitMapFont(char const* bitmapFontFilePathWithNoExtension)
{
	for (int bitmapFontIndex = 0; bitmapFontIndex < m_loadedFonts.size(); bitmapFontIndex++)
	{
		if (m_loadedFonts[bitmapFontIndex]->m_fontFilePathNameWithNoExtension == bitmapFontFilePathWithNoExtension)
		{
			return m_loadedFonts[bitmapFontIndex];
		}
	}

	return nullptr;
}

//------------------------------------------------------------------------------------------------
void Renderer::CreateBlendStates()
{
	for (int blendStateIndex = 0; blendStateIndex < static_cast<int>(BlendMode::COUNT); blendStateIndex++)
	{
		DX_SAFE_RELEASE(m_blendStates[blendStateIndex]);
		D3D11_BLEND_DESC blendDesc = { 0 };
		if (blendStateIndex == static_cast<int>(BlendMode::OPAQUE))
		{
			blendDesc.RenderTarget[0].SrcBlend = D3D11_BLEND_ONE;
			blendDesc.RenderTarget[0].DestBlend = D3D11_BLEND_ZERO;
		}
		else if (blendStateIndex == static_cast<int>(BlendMode::ALPHA))
		{
			blendDesc.RenderTarget[0].SrcBlend = D3D11_BLEND_SRC_ALPHA;
			blendDesc.RenderTarget[0].DestBlend = D3D11_BLEND_INV_SRC_ALPHA;
		}
		else
		{
			blendDesc.RenderTarget[0].SrcBlend = D3D11_BLEND_ONE;
			blendDesc.RenderTarget[0].DestBlend = D3D11_BLEND_ONE;
		}
		blendDesc.RenderTarget[0].BlendEnable = true;
		blendDesc.RenderTarget[0].BlendOp = D3D11_BLEND_OP_ADD;
		blendDesc.RenderTarget[0].SrcBlendAlpha = D3D11_BLEND_ONE;
		blendDesc.RenderTarget[0].DestBlendAlpha = D3D11_BLEND_ONE;
		blendDesc.RenderTarget[0].BlendOpAlpha = D3D11_BLEND_OP_ADD;
		blendDesc.RenderTarget[0].RenderTargetWriteMask = D3D11_COLOR_WRITE_ENABLE_ALL;

		HRESULT hr;
		hr = m_device->CreateBlendState(&blendDesc, &m_blendStates[blendStateIndex]);
		if (!SUCCEEDED(hr))
		{
			ERROR_AND_DIE("Could not create the blend state.")
		}
	}
}

//------------------------------------------------------------------------------------------------
void Renderer::CreateSamplerStates()
{
	for (int samplerStateIndex = 0; samplerStateIndex < static_cast<int>(SamplerMode::COUNT); samplerStateIndex++)
	{
		HRESULT hr;
		D3D11_SAMPLER_DESC samplerDesc = {};
		if (samplerStateIndex == static_cast<int>(SamplerMode::POINT_CLAMP))
		{
			samplerDesc.Filter = D3D11_FILTER_MIN_MAG_MIP_POINT;
			samplerDesc.AddressU = D3D11_TEXTURE_ADDRESS_CLAMP;
			samplerDesc.AddressV = D3D11_TEXTURE_ADDRESS_CLAMP;
			samplerDesc.AddressW = D3D11_TEXTURE_ADDRESS_CLAMP;
		}
		else if (samplerStateIndex == static_cast<int>(SamplerMode::BILINEAR_WRAP))
		{
			samplerDesc.Filter = D3D11_FILTER_MIN_MAG_MIP_LINEAR;
			samplerDesc.AddressU = D3D11_TEXTURE_ADDRESS_WRAP;
			samplerDesc.AddressV = D3D11_TEXTURE_ADDRESS_WRAP;
			samplerDesc.AddressW = D3D11_TEXTURE_ADDRESS_WRAP;
		}
		samplerDesc.ComparisonFunc = D3D11_COMPARISON_NEVER;
		samplerDesc.MaxLOD = D3D11_FLOAT32_MAX;

		hr = m_device->CreateSamplerState(&samplerDesc, &m_samplerStates[samplerStateIndex]);
		if (!SUCCEEDED(hr))
		{
			ERROR_AND_DIE("Could not create the sampler state.")
		}
	}
}

//------------------------------------------------------------------------------------------------
void Renderer::CreateRasterizerModes()
{
	for (int rasterizerModeIndex = 0; rasterizerModeIndex < static_cast<int>(RasterizerMode::COUNT); rasterizerModeIndex++)
	{
		HRESULT hr;
		D3D11_RASTERIZER_DESC rasterizerDesc = {};

		if (rasterizerModeIndex == static_cast<int>(RasterizerMode::SOLID_CULL_NONE))
		{
			rasterizerDesc.FillMode = D3D11_FILL_SOLID;
			rasterizerDesc.CullMode = D3D11_CULL_NONE;
		}
		else if (rasterizerModeIndex == static_cast<int>(RasterizerMode::SOLID_CULL_BACK))
		{
			rasterizerDesc.FillMode = D3D11_FILL_SOLID;
			rasterizerDesc.CullMode = D3D11_CULL_BACK;
		}
		else if (rasterizerModeIndex == static_cast<int>(RasterizerMode::WIREFRAME_CULL_NONE))
		{
			rasterizerDesc.FillMode = D3D11_FILL_WIREFRAME;
			rasterizerDesc.CullMode = D3D11_CULL_NONE;
		}
		else if (rasterizerModeIndex == static_cast<int>(RasterizerMode::WIREFRAME_CULL_BACK))
		{
			rasterizerDesc.FillMode = D3D11_FILL_WIREFRAME;
			rasterizerDesc.CullMode = D3D11_CULL_BACK;
		}
		rasterizerDesc.FrontCounterClockwise = true;
		rasterizerDesc.DepthClipEnable = true;
		rasterizerDesc.AntialiasedLineEnable = true;

		hr = m_device->CreateRasterizerState(&rasterizerDesc, &m_rasterizerStates[rasterizerModeIndex]);
		if (!SUCCEEDED(hr))
		{
			ERROR_AND_DIE("Could not create the rasterizer state.")
		}
	}
}

//------------------------------------------------------------------------------------------------
void Renderer::CreateDepthModes()
{
	for (int depthModeIndex = 0; depthModeIndex < static_cast<int>(DepthMode::COUNT); depthModeIndex++)
	{
		HRESULT hr;
		D3D11_DEPTH_STENCIL_DESC depthStencilDesc = { 0 };
		if (depthModeIndex == static_cast<int>(DepthMode::DISABLED))
		{
			depthStencilDesc.DepthWriteMask = D3D11_DEPTH_WRITE_MASK_ZERO;
			depthStencilDesc.DepthFunc = D3D11_COMPARISON_ALWAYS;
		}
		else if (depthModeIndex == static_cast<int>(DepthMode::ENABLED))
		{

			depthStencilDesc.DepthWriteMask = D3D11_DEPTH_WRITE_MASK_ALL;
			depthStencilDesc.DepthFunc = D3D11_COMPARISON_LESS_EQUAL;
		}
		depthStencilDesc.DepthEnable = true;

		hr = m_device->CreateDepthStencilState(&depthStencilDesc, &m_depthStencilStates[depthModeIndex]);
		if (!SUCCEEDED(hr))
		{
			ERROR_AND_DIE("Could not create the depth stencil state.")
		}
	}
}

//------------------------------------------------------------------------------------------------
void Renderer::SetStatesIfChanged()
{
	if (m_blendState != m_blendStates[static_cast<int>(m_desiredBlendMode)])
	{
		m_blendState = m_blendStates[static_cast<int>(m_desiredBlendMode)];
		float blendFactor[4] = { 0.0f, 0.0f, 0.0f, 0.0f };
		m_deviceContext->OMSetBlendState(m_blendState, blendFactor, 0xffffffff);
	}
	if (m_samplerState != m_samplerStates[static_cast<int>(m_desiredSamplerMode)])
	{
		m_samplerState = m_samplerStates[static_cast<int>(m_desiredSamplerMode)];
		m_deviceContext->PSSetSamplers(0, 1, &m_samplerState);
	}
	if (m_rasterizerState != m_rasterizerStates[static_cast<int>(m_desiredRasterizerMode)])
	{
		m_rasterizerState = m_rasterizerStates[static_cast<int>(m_desiredRasterizerMode)];
		m_deviceContext->RSSetState(m_rasterizerState);
	}
	if (m_depthStencilState != m_depthStencilStates[static_cast<int>(m_desiredDepthMode)])
	{
		m_depthStencilState = m_depthStencilStates[static_cast<int>(m_desiredDepthMode)];
		m_deviceContext->OMSetDepthStencilState(m_depthStencilState, 0);
	}
}

//------------------------------------------------------------------------------------------------
void Renderer::SetBlendMode(BlendMode blendMode)
{
	m_desiredBlendMode = blendMode;
}

//------------------------------------------------------------------------------------------------
void Renderer::SetSamplerMode(SamplerMode samplerMode)
{
	m_desiredSamplerMode = samplerMode;
}

//------------------------------------------------------------------------------------------------
void Renderer::SetRasterizerMode(RasterizerMode rasterizerMode)
{
	m_desiredRasterizerMode = rasterizerMode;
}

//------------------------------------------------------------------------------------------------
void Renderer::SetDepthMode(DepthMode depthMode)
{
	m_desiredDepthMode = depthMode;
}

//------------------------------------------------------------------------------------------------
void Renderer::SetLightConstants(Vec3 sunDirection, float sunIntensity, float ambientIntensity, Rgba8 sunColor, Rgba8 ambientColor, std::vector<PointLight*> pointLights)
{
	LightConstants lightConstants;
	lightConstants.SunDirection = sunDirection;
	for (int lightIndex = 0; lightIndex < 100; lightIndex++)
	{
		if (lightIndex >= pointLights.size())
		{
			break;
		}
		lightConstants.PointLightPositions[lightIndex] = Vec4(pointLights[lightIndex]->m_position.x,
			pointLights[lightIndex]->m_position.y, pointLights[lightIndex]->m_position.z, 1.0f);
		lightConstants.PointLightRanges[lightIndex] = Vec4(pointLights[lightIndex]->m_range, 0.0f, 0.0f, 0.0f);
		lightConstants.PointLightIntensities[lightIndex] = Vec4(pointLights[lightIndex]->m_intensity, 0.0f, 0.0f, 0.0f);
	}
	lightConstants.SunIntensity = sunIntensity;
	lightConstants.AmbientIntensity = ambientIntensity;
	sunColor.GetAsFloats(lightConstants.SunColor);
	ambientColor.GetAsFloats(lightConstants.AmbientColor);

	if (pointLights.size() != 0)
	{
		pointLights[0]->m_color.GetAsFloats(lightConstants.PointLightColor);
	}
	else
	{
		Rgba8::WHITE.GetAsFloats(lightConstants.PointLightColor);
	}

	CopyCPUToGPU(&lightConstants, sizeof(lightConstants), m_lightCBO);
	BindConstantBuffer(k_lightConstantsSlot, m_lightCBO);
}

//------------------------------------------------------------------------------------------------
void Renderer::SetLightConstants(Vec3 sunDirection, const std::vector<Vec3> pointLightPositions, float sunIntensity, float ambientIntensity, const std::vector<float> pointLightIntensities, const std::vector<float> pointLightRanges, Rgba8 sunColor, Rgba8 ambientColor, Rgba8 pointLightColor)
{
	LightConstants lightConstants;
	lightConstants.SunDirection = sunDirection;
	for (int lightIndex = 0; lightIndex < 100; lightIndex++)
	{
		if (lightIndex >= pointLightPositions.size())
		{
			break;
		}
		lightConstants.PointLightPositions[lightIndex] = Vec4(pointLightPositions[lightIndex].x, pointLightPositions[lightIndex].y, pointLightPositions[lightIndex].z, 1.0f);
		lightConstants.PointLightRanges[lightIndex] = Vec4(pointLightRanges[lightIndex], 0.0f, 0.0f, 0.0f);
		lightConstants.PointLightIntensities[lightIndex] = Vec4(pointLightIntensities[lightIndex], 0.0f, 0.0f, 0.0f);
	}
	lightConstants.SunIntensity = sunIntensity;
	lightConstants.AmbientIntensity = ambientIntensity;
	sunColor.GetAsFloats(lightConstants.SunColor);
	ambientColor.GetAsFloats(lightConstants.AmbientColor);
	pointLightColor.GetAsFloats(lightConstants.PointLightColor);

	CopyCPUToGPU(&lightConstants, sizeof(lightConstants), m_lightCBO);
	BindConstantBuffer(k_lightConstantsSlot, m_lightCBO);
}

//------------------------------------------------------------------------------------------------
void Renderer::BindShader(Shader* shader)
{
	if (shader == nullptr)
	{
		shader = m_defaultShader;
	}

	m_deviceContext->VSSetShader(shader->m_vertexShader, nullptr, 0);
	if (shader->m_geometryShader)
	{
		m_deviceContext->GSSetShader(shader->m_geometryShader, nullptr, 0);
	}
	else
	{
		m_deviceContext->GSSetShader(nullptr, nullptr, 0);
	}
	m_deviceContext->PSSetShader(shader->m_pixelShader, nullptr, 0);
	m_deviceContext->IASetInputLayout(shader->m_inputLayout);
}

//------------------------------------------------------------------------------------------------
void Renderer::BindComputeShader(Shader* shader)
{
	m_deviceContext->CSSetShader(shader->m_computeShader, nullptr, 0);
}

//------------------------------------------------------------------------------------------------
Shader* Renderer::CreateShader(char const* shaderName, VertexType vertexType, bool isGeometryShader, bool isStructuredBufferShader)
{
	std::string shaderSource = "";
	FileReadToString(shaderSource, shaderName);

	for (int shaderIndex = 0; shaderIndex < m_loadedShaders.size(); shaderIndex++)
	{
		if (m_loadedShaders[shaderIndex]->GetName() == shaderName)
		{
			return m_loadedShaders[shaderIndex];
		}
	}

	ShaderConfig shaderConfig;
	shaderConfig.m_name = shaderName;
	Shader* shader = new Shader(shaderConfig);

	//Vertex Shader
	HRESULT hr;
	std::vector<unsigned char> outVertexShaderByteCode;
	CompileShaderToByteCode(outVertexShaderByteCode, shaderName, shaderSource.c_str(), "VertexMain", "vs_5_0");
	hr = m_device->CreateVertexShader(outVertexShaderByteCode.data(), outVertexShaderByteCode.size(), nullptr, &shader->m_vertexShader);
	if (!SUCCEEDED(hr))
	{
		ERROR_AND_DIE("Could not create the vertex shader.")
	}

	//Pixel Shader
	std::vector<unsigned char> outPixelShaderByteCode;
	CompileShaderToByteCode(outPixelShaderByteCode, shaderName, shaderSource.c_str(), "PixelMain", "ps_5_0");
	hr = m_device->CreatePixelShader(outPixelShaderByteCode.data(), outPixelShaderByteCode.size(), nullptr, &shader->m_pixelShader);
	if (!SUCCEEDED(hr))
	{
		ERROR_AND_DIE("Could not create the pixel shader.")
	}

	//Geometry Shader
	if (isGeometryShader)
	{
		std::vector<unsigned char> outGeometryShaderByteCode;
		CompileShaderToByteCode(outGeometryShaderByteCode, shaderName, shaderSource.c_str(), "GSMain", "gs_5_0");
		hr = m_device->CreateGeometryShader(outGeometryShaderByteCode.data(), outGeometryShaderByteCode.size(), nullptr, &shader->m_geometryShader);
		if (!SUCCEEDED(hr))
		{
			ERROR_AND_DIE("Could not create the geometry shader.")
		}

		m_loadedShaders.push_back(shader);
		m_currentShader = shader;
		return shader;
	}

	if (isStructuredBufferShader)
	{
		m_loadedShaders.push_back(shader);
		m_currentShader = shader;
		return shader;
	}

	//Input Layouts
	if (vertexType == VertexType::VERTEX_PCUTBN)
	{
		D3D11_INPUT_ELEMENT_DESC inputElementDesc[] =
		{
			{"POSITION", 0, DXGI_FORMAT_R32G32B32_FLOAT, 0, 0, D3D11_INPUT_PER_VERTEX_DATA, 0},
			{"COLOR", 0, DXGI_FORMAT_R8G8B8A8_UNORM, 0, D3D11_APPEND_ALIGNED_ELEMENT, D3D11_INPUT_PER_VERTEX_DATA, 0},
			{"TEXCOORD", 0, DXGI_FORMAT_R32G32_FLOAT, 0, D3D11_APPEND_ALIGNED_ELEMENT, D3D11_INPUT_PER_VERTEX_DATA, 0},
			{"TANGENT", 0, DXGI_FORMAT_R32G32B32_FLOAT, 0, D3D11_APPEND_ALIGNED_ELEMENT, D3D11_INPUT_PER_VERTEX_DATA, 0},
			{"BINORMAL", 0, DXGI_FORMAT_R32G32B32_FLOAT, 0, D3D11_APPEND_ALIGNED_ELEMENT, D3D11_INPUT_PER_VERTEX_DATA, 0},
			{"NORMAL", 0, DXGI_FORMAT_R32G32B32_FLOAT, 0, D3D11_APPEND_ALIGNED_ELEMENT, D3D11_INPUT_PER_VERTEX_DATA, 0},
		};

		hr = m_device->CreateInputLayout(inputElementDesc, 6, outVertexShaderByteCode.data(), outVertexShaderByteCode.size(), &shader->m_inputLayout);
		if (!SUCCEEDED(hr))
		{
			ERROR_AND_DIE("Could not create the the input layout.")
		}
	}
	else
	{
		D3D11_INPUT_ELEMENT_DESC inputElementDesc[] =
		{
			{"POSITION", 0, DXGI_FORMAT_R32G32B32_FLOAT, 0, 0, D3D11_INPUT_PER_VERTEX_DATA, 0},
			{"COLOR", 0, DXGI_FORMAT_R8G8B8A8_UNORM, 0, D3D11_APPEND_ALIGNED_ELEMENT, D3D11_INPUT_PER_VERTEX_DATA, 0},
			{"TEXCOORD", 0, DXGI_FORMAT_R32G32_FLOAT, 0, D3D11_APPEND_ALIGNED_ELEMENT, D3D11_INPUT_PER_VERTEX_DATA, 0},
		};

		hr = m_device->CreateInputLayout(inputElementDesc, 3, outVertexShaderByteCode.data(), outVertexShaderByteCode.size(), &shader->m_inputLayout);
		if (!SUCCEEDED(hr))
		{
			ERROR_AND_DIE("Could not create the the input layout.")
		}
	}
	
	m_loadedShaders.push_back(shader);
	m_currentShader = shader;
	return shader;
}

//------------------------------------------------------------------------------------------------
Shader* Renderer::CreateShader(char const* shaderName, char const* shaderSource, VertexType vertexType, bool isGeometryShader, bool isStructuredBufferShader)
{
	for (int shaderIndex = 0; shaderIndex < m_loadedShaders.size(); shaderIndex++)
	{
		if (m_loadedShaders[shaderIndex]->GetName() == shaderName)
		{
			return m_loadedShaders[shaderIndex];
		}
	}

	ShaderConfig shaderConfig;
	shaderConfig.m_name = shaderName;
	Shader* shader = new Shader(shaderConfig);

	//Vertex Shader
	HRESULT hr;
	std::vector<unsigned char> outVertexShaderByteCode;
	CompileShaderToByteCode(outVertexShaderByteCode, shaderName, shaderSource, "VertexMain", "vs_5_0");
	hr = m_device->CreateVertexShader(outVertexShaderByteCode.data(), outVertexShaderByteCode.size(), nullptr, &shader->m_vertexShader);
	if (!SUCCEEDED(hr))
	{
		ERROR_AND_DIE("Could not create the vertex shader.")
	}

	//Pixel Shader
	std::vector<unsigned char> outPixelShaderByteCode;
	CompileShaderToByteCode(outPixelShaderByteCode, shaderName, shaderSource, "PixelMain", "ps_5_0");
	hr = m_device->CreatePixelShader(outPixelShaderByteCode.data(), outPixelShaderByteCode.size(), nullptr, &shader->m_pixelShader);
	if (!SUCCEEDED(hr))
	{
		ERROR_AND_DIE("Could not create the pixel shader.")
	}

	//Geometry Shader
	if (isGeometryShader)
	{
		std::vector<unsigned char> outGeometryShaderByteCode;
		CompileShaderToByteCode(outGeometryShaderByteCode, shaderName, shaderSource, "GSMain", "gs_5_0");
		hr = m_device->CreateGeometryShader(outGeometryShaderByteCode.data(), outGeometryShaderByteCode.size(), nullptr, &shader->m_geometryShader);
		if (!SUCCEEDED(hr))
		{
			ERROR_AND_DIE("Could not create the geometry shader.")
		}

		m_loadedShaders.push_back(shader);
		m_currentShader = shader;
		return shader;
	}

	if (isStructuredBufferShader)
	{
		m_loadedShaders.push_back(shader);
		m_currentShader = shader;
		return shader;
	}

	//Input Layouts
	if (vertexType == VertexType::VERTEX_PCUTBN)
	{
		D3D11_INPUT_ELEMENT_DESC inputElementDesc[] =
		{
			{"POSITION", 0, DXGI_FORMAT_R32G32B32_FLOAT, 0, 0, D3D11_INPUT_PER_VERTEX_DATA, 0},
			{"COLOR", 0, DXGI_FORMAT_R8G8B8A8_UNORM, 0, D3D11_APPEND_ALIGNED_ELEMENT, D3D11_INPUT_PER_VERTEX_DATA, 0},
			{"TEXCOORD", 0, DXGI_FORMAT_R32G32_FLOAT, 0, D3D11_APPEND_ALIGNED_ELEMENT, D3D11_INPUT_PER_VERTEX_DATA, 0},
			{"TANGENT", 0, DXGI_FORMAT_R32G32B32_FLOAT, 0, D3D11_APPEND_ALIGNED_ELEMENT, D3D11_INPUT_PER_VERTEX_DATA, 0},
			{"BINORMAL", 0, DXGI_FORMAT_R32G32B32_FLOAT, 0, D3D11_APPEND_ALIGNED_ELEMENT, D3D11_INPUT_PER_VERTEX_DATA, 0},
			{"NORMAL", 0, DXGI_FORMAT_R32G32B32_FLOAT, 0, D3D11_APPEND_ALIGNED_ELEMENT, D3D11_INPUT_PER_VERTEX_DATA, 0},
		};

		hr = m_device->CreateInputLayout(inputElementDesc, 6, outVertexShaderByteCode.data(), outVertexShaderByteCode.size(), &shader->m_inputLayout);
		if (!SUCCEEDED(hr))
		{
			ERROR_AND_DIE("Could not create the the input layout.")
		}
	}
	else
	{
		D3D11_INPUT_ELEMENT_DESC inputElementDesc[] =
		{
			{"POSITION", 0, DXGI_FORMAT_R32G32B32_FLOAT, 0, 0, D3D11_INPUT_PER_VERTEX_DATA, 0},
			{"COLOR", 0, DXGI_FORMAT_R8G8B8A8_UNORM, 0, D3D11_APPEND_ALIGNED_ELEMENT, D3D11_INPUT_PER_VERTEX_DATA, 0},
			{"TEXCOORD", 0, DXGI_FORMAT_R32G32_FLOAT, 0, D3D11_APPEND_ALIGNED_ELEMENT, D3D11_INPUT_PER_VERTEX_DATA, 0},
		};

		hr = m_device->CreateInputLayout(inputElementDesc, 3, outVertexShaderByteCode.data(), outVertexShaderByteCode.size(), &shader->m_inputLayout);
		if (!SUCCEEDED(hr))
		{
			ERROR_AND_DIE("Could not create the the input layout.")
		}
	}

	m_loadedShaders.push_back(shader);
	m_currentShader = shader;
	return shader;
}


//------------------------------------------------------------------------------------------------
Shader* Renderer::CreateComputeShader(char const* shaderName, char const* entryPointFuncitonName)
{
	std::string shaderSource = "";
	FileReadToString(shaderSource, shaderName);

	for (int shaderIndex = 0; shaderIndex < m_loadedShaders.size(); shaderIndex++)
	{
		if (m_loadedShaders[shaderIndex]->GetName() == shaderName)
		{
			return m_loadedShaders[shaderIndex];
		}
	}

	ShaderConfig shaderConfig;
	shaderConfig.m_name = shaderName;
	Shader* shader = new Shader(shaderConfig);

	HRESULT hr;
	std::vector<unsigned char> outComputeShaderByteCode;
	CompileShaderToByteCode(outComputeShaderByteCode, shaderName, shaderSource.c_str(), entryPointFuncitonName, "cs_5_0");
	hr = m_device->CreateComputeShader(outComputeShaderByteCode.data(), outComputeShaderByteCode.size(), nullptr, &shader->m_computeShader);
	if (!SUCCEEDED(hr)) 
	{
		ERROR_AND_DIE("Could not create the compute shader.")
	}

	m_loadedShaders.push_back(shader);
	m_currentShader = shader;
	return shader;
}

//------------------------------------------------------------------------------------------------
Shader* Renderer::CreateComputeShader(char const* shaderName, char const* shaderSource, char const* entryPointFuncitonName)
{
	for (int shaderIndex = 0; shaderIndex < m_loadedShaders.size(); shaderIndex++)
	{
		if (m_loadedShaders[shaderIndex]->GetName() == shaderName)
		{
			return m_loadedShaders[shaderIndex];
		}
	}

	ShaderConfig shaderConfig;
	shaderConfig.m_name = shaderName;
	Shader* shader = new Shader(shaderConfig);

	HRESULT hr;
	std::vector<unsigned char> outComputeShaderByteCode;
	CompileShaderToByteCode(outComputeShaderByteCode, shaderName, shaderSource, entryPointFuncitonName, "cs_5_0");
	hr = m_device->CreateComputeShader(outComputeShaderByteCode.data(), outComputeShaderByteCode.size(), nullptr, &shader->m_computeShader);
	if (!SUCCEEDED(hr))
	{
		ERROR_AND_DIE("Could not create the compute shader.")
	}

	m_loadedShaders.push_back(shader);
	m_currentShader = shader;
	return shader;
}

//------------------------------------------------------------------------------------------------
bool Renderer::CompileShaderToByteCode(std::vector<unsigned char>& outByteCode, char const* name, char const* source, char const* entryPoint, char const* target)
{
	DWORD flags = D3DCOMPILE_OPTIMIZATION_LEVEL3;
#if defined(ENGINE_DEBUG_RENDER)
	flags = D3DCOMPILE_DEBUG;
	flags |= D3DCOMPILE_SKIP_OPTIMIZATION;
	//flags |= D3DCOMPILE_WARNINGS_ARE_ERRORS;
#endif

	ID3DBlob* shaderBlob = nullptr;
	ID3DBlob* errorBlob = nullptr;
	HRESULT hr;

	hr = D3DCompile(source, strlen(source), name, nullptr, nullptr, entryPoint, target, flags, 0, &shaderBlob, &errorBlob);
	if (!SUCCEEDED(hr))
	{
		DebuggerPrintf(static_cast<const char*>(errorBlob->GetBufferPointer()));
		ERROR_AND_DIE("Could not compile the shader.");
	}

	outByteCode.insert(outByteCode.end(), static_cast<unsigned char*>(shaderBlob->GetBufferPointer()), static_cast<unsigned char*>(shaderBlob->GetBufferPointer()) + shaderBlob->GetBufferSize());
	return false;
}

//------------------------------------------------------------------------------------------------
VertexBuffer* Renderer::CreateVertexBuffer(const size_t size, unsigned int stride)
{
	HRESULT hr;
	//VertexBuffer
	D3D11_BUFFER_DESC bufferDesc = { 0 };
	bufferDesc.Usage = D3D11_USAGE_DYNAMIC;
	bufferDesc.ByteWidth = static_cast<UINT>(size);
	bufferDesc.BindFlags = D3D11_BIND_VERTEX_BUFFER;
	bufferDesc.CPUAccessFlags = D3D11_CPU_ACCESS_WRITE;

	VertexBuffer* vbo = new VertexBuffer(size, stride);

	hr = m_device->CreateBuffer(&bufferDesc, nullptr, &vbo->m_buffer);
	if (!SUCCEEDED(hr))
	{
		ERROR_AND_DIE("Could not create the vertex buffer.")
	}
	return vbo;
}

//------------------------------------------------------------------------------------------------
void Renderer::CopyCPUToGPU(const void* data, size_t size, VertexBuffer*& vbo)
{
	unsigned int stride = vbo->GetStride();
	if (vbo->m_size < size)
	{
		delete vbo;
		vbo = nullptr;

		vbo = CreateVertexBuffer(size, stride);
	}

	HRESULT hr;
	D3D11_MAPPED_SUBRESOURCE mappedSubresource = { 0 };
	hr = m_deviceContext->Map(vbo->m_buffer, 0, D3D11_MAP_WRITE_DISCARD, 0, &mappedSubresource);
	if (!SUCCEEDED(hr))
	{
		ERROR_AND_DIE("Could not map the the vertex buffer resource.")
	}

	memcpy(mappedSubresource.pData, data, size);
	m_deviceContext->Unmap(vbo->m_buffer, 0);
}

//------------------------------------------------------------------------------------------------
void Renderer::BindVertexBuffer(VertexBuffer* vbo, PrimitiveTopology topology)
{
	const UINT pStride = vbo->GetStride();
	const UINT pOffset = 0;
	m_deviceContext->IASetVertexBuffers(0, 1, &vbo->m_buffer, &pStride, &pOffset);
	m_deviceContext->IASetPrimitiveTopology(static_cast<D3D11_PRIMITIVE_TOPOLOGY>(topology));
}

//------------------------------------------------------------------------------------------------
IndexBuffer* Renderer::CreateIndexBuffer(const size_t size)
{
	HRESULT hr;
	//VertexBuffer
	D3D11_BUFFER_DESC bufferDesc = { 0 };
	bufferDesc.Usage = D3D11_USAGE_DYNAMIC;
	bufferDesc.ByteWidth = static_cast<UINT>(size);
	bufferDesc.BindFlags = D3D11_BIND_INDEX_BUFFER;
	bufferDesc.CPUAccessFlags = D3D11_CPU_ACCESS_WRITE;

	IndexBuffer* ibo = new IndexBuffer(size);

	hr = m_device->CreateBuffer(&bufferDesc, nullptr, &ibo->m_buffer);
	if (!SUCCEEDED(hr))
	{
		ERROR_AND_DIE("Could not create the index buffer.")
	}
	return ibo;
}

//------------------------------------------------------------------------------------------------
void Renderer::CopyCPUToGPU(const void* data, size_t size, IndexBuffer*& ibo)
{
	if (ibo->m_size < size)
	{
		delete ibo;
		ibo = nullptr;

		ibo = CreateIndexBuffer(size);
	}

	HRESULT hr;
	D3D11_MAPPED_SUBRESOURCE mappedSubresource = { 0 };
	hr = m_deviceContext->Map(ibo->m_buffer, 0, D3D11_MAP_WRITE_DISCARD, 0, &mappedSubresource);
	if (!SUCCEEDED(hr))
	{
		ERROR_AND_DIE("Could not map the the index buffer resource.")
	}

	memcpy(mappedSubresource.pData, data, size);
	m_deviceContext->Unmap(ibo->m_buffer, 0);
}

//------------------------------------------------------------------------------------------------
void Renderer::BindIndexBuffer(IndexBuffer* ibo)
{
	m_deviceContext->IASetIndexBuffer(ibo->m_buffer, DXGI_FORMAT_R32_UINT, 0);
}

//------------------------------------------------------------------------------------------------
ConstantBuffer* Renderer::CreateConstantBuffer(const size_t size)
{
	HRESULT hr;

	//Constant Buffer
	D3D11_BUFFER_DESC bufferDesc = { 0 };
	bufferDesc.Usage = D3D11_USAGE_DYNAMIC;
	bufferDesc.ByteWidth = static_cast<UINT>(size);
	bufferDesc.BindFlags = D3D11_BIND_CONSTANT_BUFFER;
	bufferDesc.CPUAccessFlags = D3D11_CPU_ACCESS_WRITE;

	ConstantBuffer* cbo = new ConstantBuffer(size);

	hr = m_device->CreateBuffer(&bufferDesc, nullptr, &cbo->m_buffer);
	if (!SUCCEEDED(hr))
	{
		ERROR_AND_DIE("Could not create the constant buffer.")
	}
	return cbo;
}

//------------------------------------------------------------------------------------------------
StructuredBuffer* Renderer::CreateStructuredBuffer(const size_t totalSize, const unsigned int stride, const void* data)
{
	HRESULT hr;
	//Structured Buffer
	D3D11_BUFFER_DESC bufferDesc = { 0 };
	bufferDesc.Usage = D3D11_USAGE_DEFAULT;
	bufferDesc.ByteWidth = static_cast<UINT>(totalSize);
	bufferDesc.BindFlags = D3D11_BIND_UNORDERED_ACCESS | D3D11_BIND_SHADER_RESOURCE;
	bufferDesc.CPUAccessFlags = D3D11_CPU_ACCESS_WRITE;
	bufferDesc.MiscFlags = D3D11_RESOURCE_MISC_BUFFER_STRUCTURED;
	bufferDesc.StructureByteStride = static_cast<UINT>(stride);
	StructuredBuffer* sbo = new StructuredBuffer(totalSize, stride);
	
	//Populate buffer data
	D3D11_SUBRESOURCE_DATA subresourceData = {};
	subresourceData.pSysMem = data;
	hr = m_device->CreateBuffer(&bufferDesc, &subresourceData, &sbo->m_buffer);
	if (!SUCCEEDED(hr))
	{
		ERROR_AND_DIE("Could not create the structured buffer.")
	}

	// Create SRV for the structured buffer
	D3D11_SHADER_RESOURCE_VIEW_DESC srvDesc = {};
	srvDesc.Format = DXGI_FORMAT_UNKNOWN;
	srvDesc.ViewDimension = D3D11_SRV_DIMENSION_BUFFER;
	srvDesc.Buffer.FirstElement = 0;
	srvDesc.Buffer.NumElements = static_cast<UINT>(totalSize / stride);
	hr = m_device->CreateShaderResourceView(sbo->m_buffer, &srvDesc, &sbo->m_SRV);
	if (FAILED(hr))
	{
		ERROR_AND_DIE("Could not create the SRV for the structure buffer.")
	}
	
	// Create UAV for the structured buffer
	D3D11_UNORDERED_ACCESS_VIEW_DESC uavDesc = {};
	uavDesc.Format = DXGI_FORMAT_UNKNOWN;
	uavDesc.ViewDimension = D3D11_UAV_DIMENSION_BUFFER;
	uavDesc.Buffer.FirstElement = 0;
	uavDesc.Buffer.NumElements = static_cast<UINT>(totalSize / stride);
	hr = m_device->CreateUnorderedAccessView(sbo->m_buffer, &uavDesc, &sbo->m_UAV);
	if (!SUCCEEDED(hr))
	{
		ERROR_AND_DIE("Could not create the UAV for the structure buffer.")
	}
	return sbo;
}

//------------------------------------------------------------------------------------------------
void Renderer::CopyCPUToGPU(const void* data, size_t size, ConstantBuffer*& cbo)
{
	HRESULT hr;
	D3D11_MAPPED_SUBRESOURCE mappedSubresource = { 0 };
 	hr = m_deviceContext->Map(cbo->m_buffer, 0, D3D11_MAP_WRITE_DISCARD, 0, &mappedSubresource);
	if (!SUCCEEDED(hr))
	{
		ERROR_AND_DIE("Could not map the the constant buffer resource.")
	}

	memcpy(mappedSubresource.pData, data, size);
	m_deviceContext->Unmap(cbo->m_buffer, 0);
}

//------------------------------------------------------------------------------------------------
void Renderer::CopyCPUToGPU(const void* data, size_t size, StructuredBuffer*& sbo)
{
	HRESULT hr;
	D3D11_MAPPED_SUBRESOURCE mappedSubresource = { 0 };
	hr = m_deviceContext->Map(sbo->m_buffer, 0, D3D11_MAP_WRITE_DISCARD, 0, &mappedSubresource);
	if (!SUCCEEDED(hr))
	{
		ERROR_AND_DIE("Could not map the the structured buffer resource.")
	}

	memcpy(mappedSubresource.pData, data, size);
	m_deviceContext->Unmap(sbo->m_buffer, 0);
}

//------------------------------------------------------------------------------------------------
void Renderer::CopyGPUToCPU(void* data, size_t size, StructuredBuffer*& sbo)
{
	HRESULT hr;
	D3D11_MAPPED_SUBRESOURCE mappedResource = {0};
	ID3D11Buffer* stagedBuffer = nullptr;
	D3D11_BUFFER_DESC bufferDesc = { 0 };
	bufferDesc.Usage = D3D11_USAGE_STAGING;
	bufferDesc.ByteWidth = static_cast<UINT>(sbo->m_size);
	bufferDesc.CPUAccessFlags = D3D11_CPU_ACCESS_READ;
	bufferDesc.MiscFlags = D3D11_RESOURCE_MISC_BUFFER_STRUCTURED;
	bufferDesc.StructureByteStride = static_cast<UINT>(sbo->GetStride());
	D3D11_SUBRESOURCE_DATA subresourceData = {};
	subresourceData.pSysMem = data;
	hr = m_device->CreateBuffer(&bufferDesc, &subresourceData, &stagedBuffer);
	if (!SUCCEEDED(hr))
	{
		ERROR_AND_DIE("Could not create the structured buffer.")
	}
	m_deviceContext->CopyResource(stagedBuffer, sbo->m_buffer);
	hr = m_deviceContext->Map(stagedBuffer, 0, D3D11_MAP_READ, 0, &mappedResource);
	if (!SUCCEEDED(hr))
	{
		ERROR_AND_DIE("Could not map the the structured buffer resource.")
	}
	memcpy(data, mappedResource.pData, size);
	m_deviceContext->Unmap(stagedBuffer, 0);
	DX_SAFE_RELEASE(stagedBuffer);
}

//------------------------------------------------------------------------------------------------
void Renderer::BindConstantBuffer(int slot, ConstantBuffer* cbo)
{
	m_deviceContext->CSSetConstantBuffers(slot, 1, &cbo->m_buffer);
	m_deviceContext->VSSetConstantBuffers(slot, 1, &cbo->m_buffer);
	m_deviceContext->GSSetConstantBuffers(slot, 1, &cbo->m_buffer);
	m_deviceContext->PSSetConstantBuffers(slot, 1, &cbo->m_buffer);
}

//------------------------------------------------------------------------------------------------
void Renderer::BindStructuredBufferVSGSPS(int slot, StructuredBuffer* sbo)
{
	m_deviceContext->VSSetShaderResources(slot, 1, &sbo->m_SRV);
	m_deviceContext->GSSetShaderResources(slot, 1, &sbo->m_SRV);
	m_deviceContext->PSSetShaderResources(slot, 1, &sbo->m_SRV);
}

//------------------------------------------------------------------------------------------------
void Renderer::BindStructuredBufferSRVCS(int slot, StructuredBuffer* sbo)
{
	m_deviceContext->CSSetShaderResources(slot, 1, &sbo->m_SRV);
}

//------------------------------------------------------------------------------------------------
void Renderer::BindStructuredBufferUAVCS(int slot, StructuredBuffer* sbo)
{
	m_deviceContext->CSSetUnorderedAccessViews(slot, 1, &sbo->m_UAV, nullptr);
}

//------------------------------------------------------------------------------------------------
void Renderer::UnbindConstantBuffer(int slot)
{
	ID3D11Buffer* buffer = nullptr;
	m_deviceContext->CSSetConstantBuffers(slot, 1, &buffer);
	m_deviceContext->VSSetConstantBuffers(slot, 1, &buffer);
	m_deviceContext->GSSetConstantBuffers(slot, 1, &buffer);
	m_deviceContext->PSSetConstantBuffers(slot, 1, &buffer);
}

//------------------------------------------------------------------------------------------------
void Renderer::UnbindStructuredBufferVSGSPS(int slot)
{
	ID3D11ShaderResourceView* srv = nullptr;
	m_deviceContext->VSSetShaderResources(slot, 1, &srv);
	m_deviceContext->GSSetShaderResources(slot, 1, &srv);
	m_deviceContext->PSSetShaderResources(slot, 1, &srv);
}

//------------------------------------------------------------------------------------------------
void Renderer::UnbindStructuredBufferSRVCS(int slot)
{
	ID3D11ShaderResourceView* srv = nullptr;
	m_deviceContext->CSSetShaderResources(slot, 1, &srv);
}

//------------------------------------------------------------------------------------------------
void Renderer::UnbindStructuredBufferUAVCS(int slot)
{
	ID3D11UnorderedAccessView* uav = nullptr;
	m_deviceContext->CSSetUnorderedAccessViews(slot, 1, &uav, nullptr);
}

//------------------------------------------------------------------------------------------------
void Renderer::DrawVertexBuffer(VertexBuffer* vbo, int vertexCount, PrimitiveTopology topology, int vertexOffset)
{
	SetStatesIfChanged();
	BindShader(m_defaultShader);
	BindVertexBuffer(vbo, topology);
	m_deviceContext->Draw(vertexCount, vertexOffset);
}

//------------------------------------------------------------------------------------------------
void Renderer::DrawVertexBuffer(Shader* shader, VertexBuffer* vbo, int vertexCount, PrimitiveTopology topology, int vertexOffset)
{
	SetStatesIfChanged();
	BindShader(shader);
	BindVertexBuffer(vbo, topology);
	m_deviceContext->Draw(vertexCount, vertexOffset);
}

//------------------------------------------------------------------------------------------------
void Renderer::DrawIndexBuffer(Shader* shader, IndexBuffer* ibo, VertexBuffer* vbo, int indexCount, PrimitiveTopology topology)
{
	SetStatesIfChanged();
	BindShader(shader);
	BindVertexBuffer(vbo, topology);
	BindIndexBuffer(ibo);
	m_deviceContext->DrawIndexed(indexCount, 0, 0);
}

//------------------------------------------------------------------------------------------------
Texture* Renderer::CreateTextureFromImage(const Image& image)
{
	Texture* newTexture = new Texture();
	newTexture->m_name = image.GetImageFilePath(); // NOTE: m_name must be a std::string, otherwise it may point to temporary data!
	newTexture->m_dimensions = image.GetDimensions();

	D3D11_TEXTURE2D_DESC textureDesc = { 0 };
	textureDesc.Width = image.GetDimensions().x;
	textureDesc.Height = image.GetDimensions().y;
	textureDesc.MipLevels = 1;
	textureDesc.ArraySize = 1;
	textureDesc.Format = DXGI_FORMAT_R8G8B8A8_UNORM;
	textureDesc.Usage = D3D11_USAGE_IMMUTABLE;
	textureDesc.BindFlags = D3D11_BIND_SHADER_RESOURCE;
	textureDesc.SampleDesc.Count = 1;

	D3D11_SUBRESOURCE_DATA subresourceData = { 0 };
	subresourceData.pSysMem = image.GetRawData();
	subresourceData.SysMemPitch = image.GetDimensions().x * 4;

	HRESULT hr;
	hr = m_device->CreateTexture2D(&textureDesc, &subresourceData, &newTexture->m_texture);
	if (!SUCCEEDED(hr))
	{
		ERROR_AND_DIE("Could not create the texture.")
	}

	hr = m_device->CreateShaderResourceView(newTexture->m_texture, NULL, &newTexture->m_shaderResourceView);
	if (!SUCCEEDED(hr))
	{
		ERROR_AND_DIE("Could not create the shader resource view.")
	}

	m_loadedTextures.push_back(newTexture);
	return newTexture;
}

//------------------------------------------------------------------------------------------------
void Renderer::SetModelConstants(const Mat44& modelMatrix, const Rgba8& modelColor)
{
	ModelConstants modelConstants;
	modelConstants.ModelMatrix = modelMatrix;
	modelColor.GetAsFloats(modelConstants.ModelColor);

	CopyCPUToGPU(&modelConstants, sizeof(modelConstants), m_modelCBO);
	BindConstantBuffer(k_modelConstantsSlot, m_modelCBO);
}

//------------------------------------------------------------------------------------------------
Query* Renderer::CreateQuery(bool isTimestep, bool isTimestepDisjoint)
{
	HRESULT hr;
	Query* query = new Query();
	D3D11_QUERY_DESC queryDesc = {};
	if (isTimestep)
	{
		queryDesc.Query = D3D11_QUERY_TIMESTAMP;
	}
	else if(isTimestepDisjoint)
	{
		queryDesc.Query = D3D11_QUERY_TIMESTAMP_DISJOINT;
	}
	queryDesc.MiscFlags = 0;
	hr = m_device->CreateQuery(&queryDesc, &query->m_query);
	if (!SUCCEEDED(hr))
	{
		ERROR_AND_DIE("Could not create the query.")
	}
	return query;
}


//------------------------------------------------------------------------------------------------
double Renderer::GetElapsedTimeGPU(Query* beginFrameTimestamp, Query* checkTimestamp, Query* disjointQuery)
{
	D3D11_QUERY_DATA_TIMESTAMP_DISJOINT disjointData;
	while (m_deviceContext->GetData(disjointQuery->m_query, &disjointData, sizeof(disjointData), 0) == S_FALSE){}
	UINT64 startTick = 0;
	while (m_deviceContext->GetData(beginFrameTimestamp->m_query, &startTick, sizeof(startTick), 0) == S_FALSE){}
	UINT64 endTick = 0;
	if (checkTimestamp)
	{
		while (m_deviceContext->GetData(checkTimestamp->m_query, &endTick, sizeof(endTick), 0) == S_FALSE){}
		UINT64 deltaTicks = endTick - startTick;
		double frequency = double(disjointData.Frequency);
		if (disjointData.Disjoint == FALSE)
		{
			double deltaTime = (deltaTicks / frequency) * 1000.0f;
			return deltaTime;
		}
		else
		{
			return -1.0f;
		}
	}
	
	return - 1.0f;
}

//------------------------------------------------------------------------------------------------
void Renderer::BeginQuery(Query* query)
{
	m_deviceContext->Begin(query->m_query);
}

//------------------------------------------------------------------------------------------------
void Renderer::EndQuery(Query* query)
{
	m_deviceContext->End(query->m_query);
}

//------------------------------------------------------------------------------------------------
BitmapFont* Renderer::CreateOrGetBitmapFont(const char* bitmapFontFilePathWithNoExtension)
{
	BitmapFont* existingFont = GetBitMapFont(bitmapFontFilePathWithNoExtension);
	if (existingFont)
	{
		return existingFont;
	}

	BitmapFont* newFont = CreateBitmapFont(bitmapFontFilePathWithNoExtension);
	m_loadedFonts.push_back(newFont);
	return newFont;
}

//------------------------------------------------------------------------------------------------
BitmapFont* Renderer::CreateBitmapFont(char const* fontFilePathNoExtenstion)
{
	std::string fileWithExtension = fontFilePathNoExtenstion;
	fileWithExtension += ".png";
	Texture* fontTexture = CreateOrGetTextureFromFile(fileWithExtension.c_str());
	BitmapFont* newFont = new BitmapFont(fontFilePathNoExtenstion, *fontTexture);
	return newFont;
}
