#pragma once
#include "Engine/Math/IntVec2.hpp"
#include "Engine/Math/Mat44.hpp"
#include "Engine/Core/Rgba8.hpp"
#include <vector>

class Camera;
class Window;
class Texture;
class Texture;
class BitmapFont;
class Shader;
class VertexBuffer;
class IndexBuffer;
class ConstantBuffer;
class Image;

struct Vertex_PCU;
struct Vertex_PNCU;
struct PointLight;
struct ID3D11Device;
struct ID3D11DeviceContext;
struct IDXGISwapChain;
struct ID3D11RenderTargetView;
struct ID3D11RasterizerState;
struct ID3D11BlendState;
struct ID3D11SamplerState;
struct ID3D11DepthStencilState;
struct ID3D11DepthStencilView;
struct ID3D11Texture2D;

#define DX_SAFE_RELEASE(dxObject)	\
{									\
	if ((dxObject) != nullptr)		\
	{								\
		(dxObject)->Release();		\
		(dxObject) = nullptr;		\
	}								\
}		

#if defined(OPAQUE)
#undef OPAQUE
#endif

//-----------------------------------------------------------------------------------------------
enum class BlendMode
{
	ALPHA,
	ADDITIVE,
	OPAQUE,
	COUNT
};

//-----------------------------------------------------------------------------------------------
enum class SamplerMode
{
	POINT_CLAMP,
	BILINEAR_WRAP,
	COUNT
};

//-----------------------------------------------------------------------------------------------
enum class RasterizerMode
{
	SOLID_CULL_NONE,
	SOLID_CULL_BACK,
	WIREFRAME_CULL_NONE,
	WIREFRAME_CULL_BACK,
	COUNT
};

//-----------------------------------------------------------------------------------------------
enum class DepthMode
{
	DISABLED,
	ENABLED,
	COUNT
};

//-----------------------------------------------------------------------------------------------
struct RendererConfig
{
	Window* m_window = nullptr;
};

//-----------------------------------------------------------------------------------------------
class Renderer
{
public:
	Renderer(RendererConfig const& config);
	~Renderer();
	void Startup();
	void BeginFrame();
	void EndFrame();
	void Shutdown();

	void ClearScreen(const Rgba8& clearColor);
	void BeginCamera(const Camera& camera);
	void EndCamera(const Camera& camera);
	void DrawVertexArray(int numVertexes, const Vertex_PCU* vertexes);
	void DrawVertexArray(int numVertexes, const Vertex_PNCU* vertexes, Shader* shader);
	void DrawIndexArray(int indexCount, VertexBuffer* vbo, IndexBuffer* ibo, Shader* shader);

	RendererConfig const& GetConfig() const;

	Texture* CreateTextureFromData(char const* name, IntVec2 dimensions, int bytesPerTexel, unsigned char* texelData);
	void BindTexture(Texture* texture);
	Texture* CreateOrGetTextureFromFile(char const* imageFilePath);
	Texture* GetTextureForFileName(char const* imageFilePath);
	BitmapFont* GetBitMapFont(char const* bitmapFontFilePathWithNoExtension);
	BitmapFont* CreateOrGetBitmapFont(const char* bitmapFontFilePathWithNoExtension);

	void BindShader(Shader* shader);
	Shader* CreateShader(char const* shaderName, char const* shaderSource);
	Shader* CreateShader(char const* shaderName);
	bool CompileShaderToByteCode(std::vector<unsigned char>& outByteCode, char const* name, char const* source, char const* entryPoint, char const* target);
	VertexBuffer* CreateVertexBuffer(const size_t size, unsigned int stride);
	void CopyCPUToGPU(const void* data, size_t size, VertexBuffer*& vbo);
	void BindVertexBuffer(VertexBuffer* vbo);
	IndexBuffer* CreateIndexBuffer(const size_t size);
	void CopyCPUToGPU(const void* data, size_t size, IndexBuffer*& ibo);
	void BindIndexBuffer(IndexBuffer* ibo);
	ConstantBuffer* CreateConstantBuffer(const size_t size);
	void CopyCPUToGPU(const void* data, size_t size, ConstantBuffer*& cbo);
	void BindConstantBuffer(int slot, ConstantBuffer* cbo);
	void DrawVertexBuffer(VertexBuffer* vbo, int vertexCount, int vertexOffset = 0);
	void DrawVertexBuffer(Shader* shader, VertexBuffer* vbo, int vertexCount, int vertexOffset = 0);
	void DrawIndexBuffer(Shader* shader, IndexBuffer* ibo, VertexBuffer* vbo, int indexCount);
	Texture* CreateTextureFromImage(const Image& image);
	void SetModelConstants(const Mat44& modelMatrix = Mat44(), const Rgba8& modelColor = Rgba8::WHITE);

	void CreateBlendStates();
	void CreateSamplerStates();
	void CreateRasterizerModes();
	void CreateDepthModes();
	void SetStatesIfChanged();
	void SetBlendMode(BlendMode blendMode);
	void SetSamplerMode(SamplerMode samplerMode);
	void SetRasterizerMode(RasterizerMode rasterizerMode);
	void SetDepthMode(DepthMode depthMode);

	void SetLightConstants(Vec3 sunDirection, float sunIntensity, float ambientIntensity, 
		Rgba8 sunColor, Rgba8 ambientColor, std::vector<PointLight*> pointLights);
	void SetLightConstants(Vec3 sunDirection, const std::vector<Vec3> pointLightPositions,
		float sunIntensity, float ambientIntensity, const std::vector<float> pointLightIntensities,
		const std::vector<float> pointLightRanges, Rgba8 sunColor, Rgba8 ambientColor, Rgba8 pointLightColor);

private:
	Texture* CreateTextureFromFile(char const* imageFilePath);
	BitmapFont* CreateBitmapFont(char const* fontFilePathNoExtenstion);

private:
	RendererConfig m_config;
	std::vector<Texture*> m_loadedTextures;
	std::vector<BitmapFont*> m_loadedFonts;

protected:
	ID3D11Device* m_device = nullptr;
	ID3D11DeviceContext* m_deviceContext = nullptr;
	IDXGISwapChain* m_swapChain = nullptr;
	ID3D11RenderTargetView* m_renderTargetView = nullptr;
	ID3D11RasterizerState* m_rasterizerState = nullptr;
	ID3D11BlendState* m_blendState = nullptr;
	ID3D11SamplerState* m_samplerState = nullptr;
	ID3D11DepthStencilState* m_depthStencilState = nullptr;
	ID3D11DepthStencilView* m_depthStencilView = nullptr;
	ID3D11Texture2D* m_depthStencilTexture = nullptr;

	BlendMode m_desiredBlendMode = BlendMode::ALPHA;
	SamplerMode m_desiredSamplerMode = SamplerMode::POINT_CLAMP;
	RasterizerMode m_desiredRasterizerMode = RasterizerMode::SOLID_CULL_BACK;
	DepthMode m_desiredDepthMode = DepthMode::ENABLED;

	ID3D11BlendState* m_blendStates[static_cast<int>(BlendMode::COUNT)] = {};
	ID3D11SamplerState* m_samplerStates[static_cast<int>(SamplerMode::COUNT)] = {};
	ID3D11RasterizerState* m_rasterizerStates[static_cast<int>(RasterizerMode::COUNT)] = {};
	ID3D11DepthStencilState* m_depthStencilStates[static_cast<int>(DepthMode::COUNT)] = {};

	void* m_dxgiDebugModule = nullptr;
	void* m_dxgiDebug = nullptr;

	std::vector<Shader*> m_loadedShaders;
	Shader const* m_currentShader = nullptr;
	VertexBuffer* m_immediateVBOForVertexPCU = nullptr;
	VertexBuffer* m_immediateVBOForVertexPNCU = nullptr;
	Shader* m_defaultShader = nullptr;
	ConstantBuffer* m_cameraCBO = nullptr;
	Texture* m_defaultTexture = nullptr;
	ConstantBuffer* m_modelCBO = nullptr;
	ConstantBuffer* m_lightCBO = nullptr;
};
