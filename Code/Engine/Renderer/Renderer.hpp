#pragma once
#include "Engine/Math/IntVec2.hpp"
#include "Engine/Math/Mat44.hpp"
#include "Engine/Core/Rgba8.hpp"
#include <vector>

//-----------------------------------------------------------------------------------------------
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
class StructuredBuffer;
class Query;
struct Vertex_PCU;
struct Vertex_PCUTBN;
struct DPVertex_PCU;
struct DPVertex_PCUTBN;
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

//-----------------------------------------------------------------------------------------------
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
enum class VertexType
{
	VERTEX_PCU,
	VERTEX_PCUTBN,
	COUNT
};

//-----------------------------------------------------------------------------------------------
enum class PrimitiveTopology
{
	D3D11_PRIMITIVE_TOPOLOGY_UNDEFINED = 0,
	D3D11_PRIMITIVE_TOPOLOGY_POINTLIST = 1,
	D3D11_PRIMITIVE_TOPOLOGY_LINELIST = 2,
	D3D11_PRIMITIVE_TOPOLOGY_LINESTRIP = 3,
	D3D11_PRIMITIVE_TOPOLOGY_TRIANGLELIST = 4,
	D3D11_PRIMITIVE_TOPOLOGY_TRIANGLESTRIP = 5,
	D3D11_PRIMITIVE_TOPOLOGY_LINELIST_ADJ = 10,
	D3D11_PRIMITIVE_TOPOLOGY_LINESTRIP_ADJ = 11,
	D3D11_PRIMITIVE_TOPOLOGY_TRIANGLELIST_ADJ = 12,
	D3D11_PRIMITIVE_TOPOLOGY_TRIANGLESTRIP_ADJ = 13,
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
	void						Startup();
	void						BeginFrame();
	void						EndFrame();
	void						Shutdown();

	void						ClearScreen(const Rgba8& clearColor);
	void						BeginCamera(const Camera& camera);
	void						EndCamera(const Camera& camera);
	void						DrawVertexArray(int numVertexes, const Vertex_PCU* vertexes, PrimitiveTopology topology = PrimitiveTopology::D3D11_PRIMITIVE_TOPOLOGY_TRIANGLELIST);
	void						DrawVertexArray(int numVertexes, const Vertex_PCUTBN* vertexes, Shader* shader, PrimitiveTopology topology = PrimitiveTopology::D3D11_PRIMITIVE_TOPOLOGY_TRIANGLELIST);
	void						DrawIndexArray(int indexCount, VertexBuffer* vbo, IndexBuffer* ibo, Shader* shader, PrimitiveTopology topology = PrimitiveTopology::D3D11_PRIMITIVE_TOPOLOGY_TRIANGLELIST);
	void						DrawGeometryShader(int numVerts, Shader* geometryShader, StructuredBuffer* structuredBuffer, int structuredBufferSlot, ConstantBuffer* constantBuffer, int constantBufferSlot, PrimitiveTopology topology = PrimitiveTopology::D3D11_PRIMITIVE_TOPOLOGY_TRIANGLELIST);
	void						DrawStructuredBuffer(int numVerts, Shader* shader, StructuredBuffer* structuredBuffer, int structuredBufferSlot, ConstantBuffer* constantBuffer, int constantBufferSlot, PrimitiveTopology topology = PrimitiveTopology::D3D11_PRIMITIVE_TOPOLOGY_TRIANGLELIST);
	void						DispatchComputeShader(Shader* computeShader, int threadXCount = 1, int threadYCount = 1, int threadZCount = 1);

	RendererConfig const&		GetConfig() const;

	Texture*					CreateTextureFromData(char const* name, IntVec2 dimensions, int bytesPerTexel, unsigned char* texelData);
	void						BindTexture(Texture* texture);
	void						BindTextures(Texture* texture1, Texture* texture2, Texture* texture3);
	Texture*					CreateOrGetTextureFromFile(char const* imageFilePath);
	Texture*					GetTextureForFileName(char const* imageFilePath);
	BitmapFont*					GetBitMapFont(char const* bitmapFontFilePathWithNoExtension);
	BitmapFont*					CreateOrGetBitmapFont(const char* bitmapFontFilePathWithNoExtension);

	void						BindShader(Shader* shader);
	void						BindComputeShader(Shader* shader);
	Shader*						CreateShader(char const* shaderName, VertexType vertexType = VertexType::VERTEX_PCU, bool isGeometryShader = false, bool isStructuredBufferShader = false);
	Shader*						CreateShader(char const* shaderName, char const* shaderSource, VertexType vertexType = VertexType::VERTEX_PCU, bool isGeometryShader = false, bool isStructuredBufferShader = false);
	Shader*						CreateComputeShader(char const* shaderName, char const* entryPointFuncitonName);
	Shader*						CreateComputeShader(char const* shaderName, char const* shaderSource, char const* entryPointFuncitonName);
	bool						CompileShaderToByteCode(std::vector<unsigned char>& outByteCode, char const* name, char const* source, char const* entryPoint, char const* target);
	VertexBuffer*				CreateVertexBuffer(const size_t size, unsigned int stride);
	void						CopyCPUToGPU(const void* data, size_t size, VertexBuffer*& vbo);
	void						BindVertexBuffer(VertexBuffer* vbo, PrimitiveTopology topology = PrimitiveTopology::D3D11_PRIMITIVE_TOPOLOGY_TRIANGLELIST);
	IndexBuffer*				CreateIndexBuffer(const size_t size);
	void						CopyCPUToGPU(const void* data, size_t size, IndexBuffer*& ibo);
	void						BindIndexBuffer(IndexBuffer* ibo);
	ConstantBuffer*				CreateConstantBuffer(const size_t size);
	StructuredBuffer*			CreateStructuredBuffer(const size_t totalSize, const unsigned int stride, const void* data);
	void						CopyCPUToGPU(const void* data, size_t size, ConstantBuffer*& cbo); 
	void						CopyCPUToGPU(const void* data, size_t size, StructuredBuffer*& sbo);
	void						CopyGPUToCPU(void* data, size_t size, StructuredBuffer*& sbo);
	void						BindConstantBuffer(int slot, ConstantBuffer* cbo);
	void						BindStructuredBufferVSGSPS(int slot, StructuredBuffer* sbo);
	void						BindStructuredBufferSRVCS(int slot, StructuredBuffer* sbo);
	void						BindStructuredBufferUAVCS(int slot, StructuredBuffer* sbo);
	void						UnbindConstantBuffer(int slot);
	void						UnbindStructuredBufferVSGSPS(int slot);
	void						UnbindStructuredBufferSRVCS(int slot);
	void						UnbindStructuredBufferUAVCS(int slot);
	void						DrawVertexBuffer(VertexBuffer* vbo, int vertexCount, PrimitiveTopology topology = PrimitiveTopology::D3D11_PRIMITIVE_TOPOLOGY_TRIANGLELIST, int vertexOffset = 0);
	void						DrawVertexBuffer(Shader* shader, VertexBuffer* vbo, int vertexCount, PrimitiveTopology topology = PrimitiveTopology::D3D11_PRIMITIVE_TOPOLOGY_TRIANGLELIST, int vertexOffset = 0);
	void						DrawIndexBuffer(Shader* shader, IndexBuffer* ibo, VertexBuffer* vbo, int indexCount, PrimitiveTopology topology = PrimitiveTopology::D3D11_PRIMITIVE_TOPOLOGY_TRIANGLELIST);
	Texture*					CreateTextureFromImage(const Image& image);
	void						SetModelConstants(const Mat44& modelMatrix = Mat44(), const Rgba8& modelColor = Rgba8::WHITE);
	Query*						CreateQuery(bool isTimestep = true, bool isTimestepDisjoint = false);
	double						GetElapsedTimeGPU(Query* beginFrameTimestamp, Query* checkTimestamp, Query* disjointQuery);
	void						BeginQuery(Query* query);
	void						EndQuery(Query* query);

	void						CreateBlendStates();
	void						CreateSamplerStates();
	void						CreateRasterizerModes();
	void						CreateDepthModes();
	void						SetStatesIfChanged();
	void						SetBlendMode(BlendMode blendMode);
	void						SetSamplerMode(SamplerMode samplerMode);
	void						SetRasterizerMode(RasterizerMode rasterizerMode);
	void						SetDepthMode(DepthMode depthMode);

	void						SetLightConstants(Vec3 sunDirection, float sunIntensity, float ambientIntensity, 
								Rgba8 sunColor, Rgba8 ambientColor, std::vector<PointLight*> pointLights);
	void						SetLightConstants(Vec3 sunDirection, const std::vector<Vec3> pointLightPositions,
								float sunIntensity, float ambientIntensity, const std::vector<float> pointLightIntensities,
								const std::vector<float> pointLightRanges, Rgba8 sunColor, Rgba8 ambientColor, Rgba8 pointLightColor);
	void						SetLightConstants(Vec3 sunDirection, float sunIntensity, float ambientIntensity,
								Rgba8 sunColor, Rgba8 ambientColor, std::vector<PointLight*> pointLights,
								int normalMode, int specularMode, float specularIntensity, float specularPower);

private:
	Texture*					CreateTextureFromFile(char const* imageFilePath);
	BitmapFont*					CreateBitmapFont(char const* fontFilePathNoExtenstion);

private:
	RendererConfig				m_config;
	std::vector<Texture*>		m_loadedTextures;
	std::vector<BitmapFont*>	m_loadedFonts;

protected:
	ID3D11Device*				m_device = nullptr;
	ID3D11DeviceContext*		m_deviceContext = nullptr;
	IDXGISwapChain*				m_swapChain = nullptr;
	ID3D11RenderTargetView*		m_renderTargetView = nullptr;
	ID3D11RasterizerState*		m_rasterizerState = nullptr;
	ID3D11BlendState*			m_blendState = nullptr;
	ID3D11SamplerState*			m_samplerState = nullptr;
	ID3D11DepthStencilState*	m_depthStencilState = nullptr;
	ID3D11DepthStencilView*		m_depthStencilView = nullptr;
	ID3D11Texture2D*			m_depthStencilTexture = nullptr;

	ID3D11BlendState*			m_blendStates[static_cast<int>(BlendMode::COUNT)] = {};
	ID3D11SamplerState*			m_samplerStates[static_cast<int>(SamplerMode::COUNT)] = {};
	ID3D11RasterizerState*		m_rasterizerStates[static_cast<int>(RasterizerMode::COUNT)] = {};
	ID3D11DepthStencilState*	m_depthStencilStates[static_cast<int>(DepthMode::COUNT)] = {};

	void*						m_dxgiDebugModule = nullptr;
	void*						m_dxgiDebug = nullptr;

	std::vector<Shader*>		m_loadedShaders;
	Shader const*				m_currentShader = nullptr;
	VertexBuffer*				m_immediateVBOForVertexPCU = nullptr;
	VertexBuffer*				m_immediateVBOForVertexPCUTBN = nullptr;
	Shader*						m_defaultShader = nullptr;
	ConstantBuffer*				m_cameraCBO = nullptr;
	Texture*					m_defaultTexture = nullptr;
	ConstantBuffer*				m_modelCBO = nullptr;
	ConstantBuffer*				m_lightCBO = nullptr;

	BlendMode					m_desiredBlendMode = BlendMode::ALPHA;
	SamplerMode					m_desiredSamplerMode = SamplerMode::POINT_CLAMP;
	RasterizerMode				m_desiredRasterizerMode = RasterizerMode::SOLID_CULL_BACK;
	DepthMode					m_desiredDepthMode = DepthMode::ENABLED;
};
