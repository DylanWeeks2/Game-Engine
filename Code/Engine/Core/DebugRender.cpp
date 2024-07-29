#include "DebugRender.hpp"
#include "Engine\Math\Vec3.hpp"
#include "Engine/Math/EulerAngles.hpp"
#include "Engine/Core/Rgba8.hpp"
#include "Engine/Math/Mat44.hpp"
#include "Engine/Core/Vertex_PCU.hpp"
#include "Engine/Core/VertexUtils.hpp"
#include "Engine/Renderer/Renderer.hpp"
#include "Engine/Core/Stopwatch.hpp"
#include "Engine/Math/MathUtils.hpp"
#include "Engine/Renderer/BitmapFont.hpp"
#include "Engine/Renderer/Camera.hpp"
#include "Engine/Core/Clock.hpp"
#include "Engine/Core/StringUtils.hpp"
#include "Engine/Core/ErrorWarningAssert.hpp"
#include <mutex>

class DebugRenderClass;
DebugRenderClass* s_debugRenderer = nullptr;

//-----------------------------------------------------------------------------------------------
class DebugRenderEntity
{
public:
	DebugRenderEntity(float duration, Rgba8 startColor, Rgba8 endColor, DebugRenderMode mode);
	~DebugRenderEntity();

	void Update();
	void Render() const;

	Mat44 GetModelMatrix() const;
	void XRayFirstRender() const;

public:
	Stopwatch* m_stopwatch = nullptr;
	Texture* m_texture = nullptr;
	std::string m_message;
	std::vector<Vertex_PCU> m_vertexes;
	Vec3 m_position;
	Vec3 m_velocity;
	EulerAngles m_orientation;
	EulerAngles m_angularVelocity;
	DebugRenderMode m_mode;
	float m_duraiton;
	Rgba8 m_startColor = Rgba8::WHITE;
	Rgba8 m_endColor = Rgba8::WHITE;
	Rgba8 m_currentColor = Rgba8::WHITE;
	bool m_isWireFrame = false;
	bool m_isLineList = false;
	bool m_isGarbage = false;
	bool m_isText = false;
	bool m_isBillboard = false;
};

//-----------------------------------------------------------------------------------------------
class DebugRenderClass
{
public:
	DebugRenderClass(DebugRenderConfig config);
	~DebugRenderClass();

	void BeginFrame();
	void Render() const;
	void EndFrame();

public:
	std::vector<DebugRenderEntity*> m_worldEntities;
	std::vector<DebugRenderEntity*> m_screenEntities;
	std::vector<DebugRenderEntity*> m_screenMessages;
	std::vector<std::string> m_messages;
	std::mutex m_debugRenderMutex;
	DebugRenderConfig m_config;
	Mat44 m_cameraMatrix;
	AABB2 m_screenCameraBounds;
	bool m_isVisible = true;
	bool m_isWorldCamera = true;
};

//-----------------------------------------------------------------------------------------------
DebugRenderEntity::DebugRenderEntity(float duration, Rgba8 startColor, Rgba8 endColor, DebugRenderMode mode)
{
	if (duration != -1.0f)
	{
		m_stopwatch = new Stopwatch(duration);
		m_stopwatch->Start();
	}

	m_duraiton = duration;
	m_mode = mode;
	m_startColor = startColor;
	m_endColor = endColor;
	m_currentColor = m_startColor;
}

//-----------------------------------------------------------------------------------------------
DebugRenderEntity::~DebugRenderEntity()
{
	delete m_stopwatch;
	m_stopwatch = nullptr;
}

//-----------------------------------------------------------------------------------------------
void DebugRenderEntity::Update()
{
	if (m_duraiton == -1.0f)
	{
		return;
	}

	if (m_stopwatch->HasDuraitonElapsed() == false)
	{
		//scale the color based on the time fraction 
		m_currentColor.r = static_cast<unsigned char>(RangeMap(m_stopwatch->GetElapsedFraction(), 0.0f, 1.0f, m_startColor.r, m_endColor.r));
		m_currentColor.g = static_cast<unsigned char>(RangeMap(m_stopwatch->GetElapsedFraction(), 0.0f, 1.0f, m_startColor.g, m_endColor.g));
		m_currentColor.b = static_cast<unsigned char>(RangeMap(m_stopwatch->GetElapsedFraction(), 0.0f, 1.0f, m_startColor.b, m_endColor.b));
		m_currentColor.a = static_cast<unsigned char>(RangeMap(m_stopwatch->GetElapsedFraction(), 0.0f, 1.0f, m_startColor.a, m_endColor.a));
	}
	else
	{
		m_isGarbage = true;
	}
}

//-----------------------------------------------------------------------------------------------
void DebugRenderEntity::Render() const
{
	if (m_mode == DebugRenderMode::X_RAY)
	{
		s_debugRenderer->m_config.m_renderer->SetBlendMode(BlendMode::ALPHA);
		s_debugRenderer->m_config.m_renderer->SetDepthMode(DepthMode::DISABLED);
		s_debugRenderer->m_config.m_renderer->SetRasterizerMode(RasterizerMode::SOLID_CULL_BACK);
		XRayFirstRender();		
		s_debugRenderer->m_config.m_renderer->SetBlendMode(BlendMode::OPAQUE);
		s_debugRenderer->m_config.m_renderer->SetDepthMode(DepthMode::ENABLED);
	}
	else if (m_mode == DebugRenderMode::ALWAYS)
	{
		s_debugRenderer->m_config.m_renderer->SetDepthMode(DepthMode::DISABLED);
	}
	else
	{
		s_debugRenderer->m_config.m_renderer->SetDepthMode(DepthMode::ENABLED);
	}

	if (m_isWireFrame == true)
	{
		s_debugRenderer->m_config.m_renderer->SetRasterizerMode(RasterizerMode::WIREFRAME_CULL_NONE);
	}
	else
	{
		s_debugRenderer->m_config.m_renderer->SetRasterizerMode(RasterizerMode::SOLID_CULL_BACK);
	}

	std::vector<Vertex_PCU> verts;
	verts.reserve(m_vertexes.size());
	for (int vertexIndex = 0; vertexIndex < m_vertexes.size(); vertexIndex++)
	{
		verts.push_back(m_vertexes[vertexIndex]);
	}

	if (m_isText == true)
	{
		s_debugRenderer->m_config.m_renderer->SetRasterizerMode(RasterizerMode::SOLID_CULL_NONE);
		if (m_isBillboard == true)
		{
			s_debugRenderer->m_config.m_renderer->BindTexture(m_texture);
			s_debugRenderer->m_config.m_renderer->SetModelConstants(GetBillboardMatrix(BillboardType::FULL_CAMERA_OPPOSING, s_debugRenderer->m_cameraMatrix, m_position), m_currentColor);
			s_debugRenderer->m_config.m_renderer->DrawVertexArray(static_cast<int>(verts.size()), verts.data());
		}
		else
		{
			s_debugRenderer->m_config.m_renderer->BindTexture(m_texture);
			s_debugRenderer->m_config.m_renderer->SetModelConstants(GetModelMatrix(), m_currentColor);
			s_debugRenderer->m_config.m_renderer->DrawVertexArray(static_cast<int>(verts.size()), verts.data());
		}
	}
	else
	{
		if (m_isLineList == true)
		{
			s_debugRenderer->m_config.m_renderer->BindTexture(nullptr);
			s_debugRenderer->m_config.m_renderer->SetModelConstants(GetModelMatrix(), m_currentColor);
			s_debugRenderer->m_config.m_renderer->DrawVertexArray(static_cast<int>(verts.size()), verts.data(), PrimitiveTopology::D3D11_PRIMITIVE_TOPOLOGY_LINELIST);
		}
		else
		{
			s_debugRenderer->m_config.m_renderer->BindTexture(nullptr);
			s_debugRenderer->m_config.m_renderer->SetModelConstants(GetModelMatrix(), m_currentColor);
			s_debugRenderer->m_config.m_renderer->DrawVertexArray(static_cast<int>(verts.size()), verts.data());
		}
	}
}

//-----------------------------------------------------------------------------------------------
Mat44 DebugRenderEntity::GetModelMatrix() const
{
	Mat44 modelMatrix;
	modelMatrix = m_orientation.GetAsMatrix_XFwd_YLeft_ZUp();
	modelMatrix.SetTranslation3D(m_position);
	return modelMatrix;
}

//-----------------------------------------------------------------------------------------------
void DebugRenderEntity::XRayFirstRender() const
{
	unsigned char r = m_currentColor.r;
	unsigned char g = m_currentColor.g;
	unsigned char b = m_currentColor.b;
	if (r + 40 > 255)
	{
		r = 255;
	}
	else
	{
		r += 40;
	}
	if (g + 40 > 255)
	{
		g = 255;
	}
	else
	{
		g += 40;
	}
	if (b + 40 > 255)
	{
		b = 255;
	}
	else
	{
		b += 40;
	}
	std::vector<Vertex_PCU> verts;
	verts.reserve(m_vertexes.size());
	for (int vertexIndex = 0; vertexIndex < m_vertexes.size(); vertexIndex++)
	{
		verts.push_back(m_vertexes[vertexIndex]);
	}

	s_debugRenderer->m_config.m_renderer->BindTexture(nullptr);
	s_debugRenderer->m_config.m_renderer->SetModelConstants(GetModelMatrix(), Rgba8(r, g, b, 128));
	s_debugRenderer->m_config.m_renderer->DrawVertexArray(static_cast<int>(verts.size()), verts.data());
}

//-----------------------------------------------------------------------------------------------
DebugRenderClass::DebugRenderClass(DebugRenderConfig config)
{
	m_config = config;
}

//-----------------------------------------------------------------------------------------------
DebugRenderClass::~DebugRenderClass()
{
	DebugRenderClear();
}

//-----------------------------------------------------------------------------------------------
void DebugRenderClass::BeginFrame()
{
	s_debugRenderer->m_debugRenderMutex.lock();
	for (int entityIndex = 0; entityIndex < m_worldEntities.size(); entityIndex++)
	{
		DebugRenderEntity*& entity = m_worldEntities[entityIndex];
		if (entity != nullptr)
		{
			if (entity->m_isGarbage == true)
			{
				delete entity;
				entity = nullptr;
			}
			else
			{
				entity->Update();
			}
		}
	}

	for (int entityIndex = 0; entityIndex < m_screenEntities.size(); entityIndex++)
	{
		DebugRenderEntity*& entity = m_screenEntities[entityIndex];
		if (entity != nullptr)
		{
			if (entity->m_isGarbage == true)
			{
				delete entity;
				entity = nullptr;
			}
			else
			{
				entity->Update();
			}
		}
	}

	for (int messageIndex = 0; messageIndex < m_screenMessages.size(); messageIndex++)
	{
		DebugRenderEntity*& message = m_screenMessages[messageIndex];
		if (message != nullptr)
		{
			if (message->m_isGarbage == true)
			{
				delete message;
				message = nullptr;
			}
			else
			{
				message->Update();
			}
		}
	}
	s_debugRenderer->m_debugRenderMutex.unlock();
}

//-----------------------------------------------------------------------------------------------
void DebugRenderClass::Render() const
{
	if (m_isVisible == false)
	{
		return;
	}

	if (m_isWorldCamera == true)
	{
		for (int entityIndex = 0; entityIndex < m_worldEntities.size(); entityIndex++)
		{
			DebugRenderEntity* const entity = m_worldEntities[entityIndex];
			if (entity != nullptr)
			{
				entity->Render();
			}
		}
	}
	else
	{
		m_config.m_renderer->SetBlendMode(BlendMode::ALPHA);
		m_config.m_renderer->SetDepthMode(DepthMode::DISABLED);
		m_config.m_renderer->SetRasterizerMode(RasterizerMode::SOLID_CULL_NONE);

		for (int entityIndex = 0; entityIndex < m_screenEntities.size(); entityIndex++)
		{
			DebugRenderEntity* const entity = m_screenEntities[entityIndex];
			if (entity != nullptr)
			{
				entity->Render();
			}
		}

		std::string orderedMessages = "";
		for (int messageIndex = 0; messageIndex < m_screenMessages.size(); messageIndex++)
		{
			DebugRenderEntity* const message = m_screenMessages[messageIndex];
			if (message != nullptr)
			{
				if (message->m_duraiton == -1.0f)
				{
					orderedMessages += message->m_message;
					orderedMessages += "\n";
				}
			}
		}

		for (int messageIndex = 0; messageIndex < m_screenMessages.size(); messageIndex++)
		{
			DebugRenderEntity* const message = m_screenMessages[messageIndex];
			if (message != nullptr)
			{
				if (message->m_duraiton == 0.0f)
				{
					orderedMessages += message->m_message;
					orderedMessages += "\n";
				}
			}
		}

		for (int messageIndex = 0; messageIndex < m_screenMessages.size(); messageIndex++)
		{
			DebugRenderEntity* const message = m_screenMessages[messageIndex];
			if (message != nullptr)
			{
				if (message->m_duraiton != 0.0f && message->m_duraiton != -1.0f)
				{
					orderedMessages += message->m_message;
					orderedMessages += "\n";
				}
			}
		}

		s_debugRenderer->m_debugRenderMutex.unlock();
		DebugAddScreenText(orderedMessages, Vec2(0.0f, 0.0f), 15.0f, Vec2(0.0f, 1.0f), 0.0f);
		s_debugRenderer->m_debugRenderMutex.lock();
	}
}

//-----------------------------------------------------------------------------------------------
void DebugRenderClass::EndFrame()
{
	s_debugRenderer->m_debugRenderMutex.lock();
	for (int entityIndex = 0; entityIndex < m_worldEntities.size(); entityIndex++)
	{
		DebugRenderEntity*& entity = m_worldEntities[entityIndex];
		if (entity != nullptr)
		{
			if (entity->m_isGarbage == true)
			{
				delete entity;
				entity = nullptr;
			}			 
		}
	}

	for (int entityIndex = 0; entityIndex < m_screenEntities.size(); entityIndex++)
	{
		DebugRenderEntity*& entity = m_screenEntities[entityIndex];
		if (entity != nullptr)
		{
			if (entity->m_isGarbage == true)
			{
				delete entity;
				entity = nullptr;
			}
		}
	}

	for (int messageIndex = 0; messageIndex < m_screenMessages.size(); messageIndex++)
	{
		DebugRenderEntity*& message = m_screenMessages[messageIndex];
		if (message != nullptr)
		{
			if (message->m_isGarbage == true)
			{
				delete message;
				message = nullptr;
			}
		}
	}
	s_debugRenderer->m_debugRenderMutex.unlock();
}

//-----------------------------------------------------------------------------------------------
void DebugRenderSystemStartup(const DebugRenderConfig& config)
{
	g_theEventSystem->SubscribeEventCallbackFunciton("debugrenderclear", Command_DebugRenderClear);
	g_theEventSystem->SubscribeEventCallbackFunciton("debugrendertoggle", Command_DebugRenderToggle);
	s_debugRenderer = new DebugRenderClass(config);
	DebugAddWorldArrow(Vec3(0.0f, 0.0f, 0.0f), Vec3(1.0f, 0.0f, 0.0f), 0.15f, -1.0f, Rgba8::RED, Rgba8::RED);
	DebugAddWorldArrow(Vec3(0.0f, 0.0f, 0.0f), Vec3(0.0f, 1.0f, 0.0f), 0.15f, -1.0f, Rgba8::GREEN, Rgba8::GREEN);
	DebugAddWorldArrow(Vec3(0.0f, 0.0f, 0.0f), Vec3(0.0f, 0.0f, 1.0f), 0.15f, -1.0f, Rgba8::BLUE, Rgba8::BLUE);

	Mat44 xAxisTransform;
	xAxisTransform.AppendXRotation(90.0f);
	xAxisTransform.SetTranslation3D(Vec3(0.4f, 0.0f, 0.4f));
	DebugAddWorldText("X - Forward", xAxisTransform, 0.1f, Vec2(0.5f, 0.5f), -1.0f, Rgba8::RED, Rgba8::RED);
	Mat44 yAxisTransform;
	yAxisTransform.AppendXRotation(90.0f);
	yAxisTransform.AppendYRotation(-90.0f);
	yAxisTransform.SetTranslation3D(Vec3(0.0f, 0.7f, -0.4f));
	DebugAddWorldText("Y - Left", yAxisTransform, 0.1f, Vec2(0.5f, 0.5f), -1.0f, Rgba8::GREEN, Rgba8::GREEN);
	Mat44 zAxisTransform;
	zAxisTransform.AppendYRotation(-90.0f);
	zAxisTransform.SetTranslation3D(Vec3(0.0f, -0.2f, 0.5f));
	DebugAddWorldText("Z - Up", zAxisTransform, 0.1f, Vec2(0.5f, 0.5f), -1.0f, Rgba8::BLUE, Rgba8::BLUE);
}

//-----------------------------------------------------------------------------------------------
void DebugRenderSystemShutdown()
{
	delete s_debugRenderer;
	s_debugRenderer = nullptr;
}

//-----------------------------------------------------------------------------------------------
void DebugRenderSetVisible()
{
	s_debugRenderer->m_debugRenderMutex.lock();
	s_debugRenderer->m_isVisible = !s_debugRenderer->m_isVisible;
	s_debugRenderer->m_debugRenderMutex.unlock();
}

//-----------------------------------------------------------------------------------------------
void DebugRenderSetHidden()
{
	s_debugRenderer->m_debugRenderMutex.lock();
	s_debugRenderer->m_config.m_startHidden = true;
	s_debugRenderer->m_debugRenderMutex.unlock();
}

//-----------------------------------------------------------------------------------------------
void DebugRenderClear()
{
	s_debugRenderer->m_debugRenderMutex.lock();
	for (int entityIndex = 0; entityIndex < s_debugRenderer->m_worldEntities.size(); entityIndex++)
	{
		DebugRenderEntity*& entity = s_debugRenderer->m_worldEntities[entityIndex];
		if (entity != nullptr)
		{
			delete entity;
			entity = nullptr;
		}
	}
	for (int entityIndex = 0; entityIndex < s_debugRenderer->m_screenEntities.size(); entityIndex++)
	{
		DebugRenderEntity*& entity = s_debugRenderer->m_screenEntities[entityIndex];
		if (entity != nullptr)
		{
			delete entity;
			entity = nullptr;
		}
	}
	for (int messageIndex = 0; messageIndex < s_debugRenderer->m_screenMessages.size(); messageIndex++)
	{
		DebugRenderEntity*& message = s_debugRenderer->m_screenMessages[messageIndex];
		if (message != nullptr)
		{
			delete message;
			message = nullptr;
		}
	}
	s_debugRenderer->m_debugRenderMutex.unlock();
}

//-----------------------------------------------------------------------------------------------
void DebugRenderBeginFrame()
{
	s_debugRenderer->BeginFrame();
}

//-----------------------------------------------------------------------------------------------
void DebugRenderWorld(const Camera& camera)
{
	s_debugRenderer->m_debugRenderMutex.lock();
	s_debugRenderer->m_config.m_renderer->BeginCamera(camera);
	s_debugRenderer->m_isWorldCamera = true;
	s_debugRenderer->m_cameraMatrix = camera.m_orientation.GetAsMatrix_XFwd_YLeft_ZUp();
	s_debugRenderer->m_cameraMatrix.SetTranslation3D(camera.m_position);
	s_debugRenderer->Render();
	s_debugRenderer->m_config.m_renderer->EndCamera(camera);
	s_debugRenderer->m_debugRenderMutex.unlock();
}

//-----------------------------------------------------------------------------------------------
void DebugRenderScreen(const Camera& camera)
{
	s_debugRenderer->m_debugRenderMutex.lock();
	s_debugRenderer->m_config.m_renderer->BeginCamera(camera);
	s_debugRenderer->m_isWorldCamera = false;
	s_debugRenderer->m_screenCameraBounds.m_mins = camera.GetOrthographicBottomLeft();
	s_debugRenderer->m_screenCameraBounds.m_maxs = camera.GetOrthographicTopRight();
	s_debugRenderer->Render();
	s_debugRenderer->m_config.m_renderer->EndCamera(camera);
	s_debugRenderer->m_debugRenderMutex.unlock();
}

//-----------------------------------------------------------------------------------------------
void DebugRenderEndFrame()
{
	s_debugRenderer->EndFrame();
}

//-----------------------------------------------------------------------------------------------
void DebugAddWorldConvexPoly3D(const ConvexPoly3D& poly, float duration, const Rgba8 startColor, const Rgba8 endColor, DebugRenderMode mode)
{
	DebugRenderEntity* entity = new DebugRenderEntity(duration, startColor, endColor, mode);
	AddVertsForConvexPoly3D(entity->m_vertexes, poly);
	s_debugRenderer->m_debugRenderMutex.lock();
	s_debugRenderer->m_worldEntities.push_back(entity);
	s_debugRenderer->m_debugRenderMutex.unlock();
}

//-----------------------------------------------------------------------------------------------
void DebugAddWorldWireConvexPoly3D(const ConvexPoly3D& poly, float duration, const Rgba8 startColor, const Rgba8 endColor, DebugRenderMode mode)
{
	DebugRenderEntity* entity = new DebugRenderEntity(duration, startColor, endColor, mode);
	entity->m_isLineList = true;
	AddVertsForWireConvexPoly3D(entity->m_vertexes, poly);
	s_debugRenderer->m_debugRenderMutex.lock();
	s_debugRenderer->m_worldEntities.push_back(entity);
	s_debugRenderer->m_debugRenderMutex.unlock();
}

//-----------------------------------------------------------------------------------------------
void DebugAddWorldPoint(const Vec3& pos, float radius, float duration, const Rgba8 startColor, const Rgba8 endColor, DebugRenderMode mode)
{
	DebugRenderEntity* point = new DebugRenderEntity(duration, startColor, endColor, mode);
	AddVertsForSphere3D(point->m_vertexes, pos, radius);
	s_debugRenderer->m_debugRenderMutex.lock();
	s_debugRenderer->m_worldEntities.push_back(point);
	s_debugRenderer->m_debugRenderMutex.unlock();
}

//-----------------------------------------------------------------------------------------------
void DebugAddWorldLine(const Vec3& start, const Vec3& end, float radius, float duration, const Rgba8 startColor, const Rgba8 endColor, DebugRenderMode mode)
{
	DebugRenderEntity* line = new DebugRenderEntity(duration, startColor, endColor, mode);
	AddVertsForLineSegment3D(line->m_vertexes, start, end, radius * 2.0f, startColor);
	s_debugRenderer->m_debugRenderMutex.lock();
	s_debugRenderer->m_worldEntities.push_back(line);
	s_debugRenderer->m_debugRenderMutex.unlock();
}

//-----------------------------------------------------------------------------------------------
void DebugAddWorldWireCylinder(const Vec3& base, const Vec3& top, float radius, float duration, const Rgba8 startColor, const Rgba8 endColor, DebugRenderMode mode)
{
	DebugRenderEntity* cylinder = new DebugRenderEntity(duration, startColor, endColor, mode);
	cylinder->m_isWireFrame = true;
	AddVertsForCylinder3D(cylinder->m_vertexes, base, top, radius);
	s_debugRenderer->m_debugRenderMutex.lock();
	s_debugRenderer->m_worldEntities.push_back(cylinder);
	s_debugRenderer->m_debugRenderMutex.unlock();
}

//-----------------------------------------------------------------------------------------------
void DebugAddWorldWireSphere(const Vec3& center, float radius, float duration, const Rgba8 startColor, const Rgba8 endColor, DebugRenderMode mode)
{
	DebugRenderEntity* sphere = new DebugRenderEntity(duration, startColor, endColor, mode);
	sphere->m_isWireFrame = true;
	AddVertsForSphere3D(sphere->m_vertexes, center, radius);
	s_debugRenderer->m_debugRenderMutex.lock();
	s_debugRenderer->m_worldEntities.push_back(sphere);
	s_debugRenderer->m_debugRenderMutex.unlock();
}

//-----------------------------------------------------------------------------------------------
void DebugAddWorldWireAABB3(const AABB3& aabb, float duration, const Rgba8 startColor, const Rgba8 endColor, DebugRenderMode mode)
{
	DebugRenderEntity* aabbEntity = new DebugRenderEntity(duration, startColor, endColor, mode);
	aabbEntity->m_isWireFrame = true;
	AddVertsForAABB3D(aabbEntity->m_vertexes, aabb);
	s_debugRenderer->m_debugRenderMutex.lock();
	s_debugRenderer->m_worldEntities.push_back(aabbEntity);
	s_debugRenderer->m_debugRenderMutex.unlock();
}

//-----------------------------------------------------------------------------------------------
void DebugAddWorldArrow(const Vec3& start, const Vec3& end, float radius, float duration, const Rgba8 startColor, const Rgba8 endColor, DebugRenderMode mode)
{
	DebugRenderEntity* arrow = new DebugRenderEntity(duration, startColor, endColor, mode);
	AddVertsForArrow3D(arrow->m_vertexes, start, end, radius * 2.0f, startColor);

	s_debugRenderer->m_debugRenderMutex.lock();
	s_debugRenderer->m_worldEntities.push_back(arrow);
	s_debugRenderer->m_debugRenderMutex.unlock();
}

//-----------------------------------------------------------------------------------------------
void DebugAddWorldText(const std::string& text, const Mat44& transform, float textHeight, const Vec2& alignment, float duration, const Rgba8 startColor, const Rgba8 endColor, DebugRenderMode mode)
{
	BitmapFont* theFont;
	std::string fileName = "Data/Fonts/SquirrelFixedFont";
	theFont = s_debugRenderer->m_config.m_renderer->CreateOrGetBitmapFont(fileName.c_str());
	DebugRenderEntity* textEntity = new DebugRenderEntity(duration, startColor, endColor, mode);
	theFont->AddVertsForText3D(textEntity->m_vertexes, transform.GetTranslation2D(), textHeight, text, startColor, 1.0f, alignment);
	textEntity->m_texture = &theFont->GetTexture();
	textEntity->m_isText = true;
	TransformVertexArray3D(textEntity->m_vertexes, transform);
	s_debugRenderer->m_worldEntities.push_back(textEntity);
}

//-----------------------------------------------------------------------------------------------
void DebugAddWorldBillboardText(const std::string& text, const Vec3& origin, float textHeight, const Vec2& alignment, float duration, const Rgba8 startColor, const Rgba8 endColor, DebugRenderMode mode)
{
	BitmapFont* theFont;
	std::string fileName = "Data/Fonts/SquirrelFixedFont";
	theFont = s_debugRenderer->m_config.m_renderer->CreateOrGetBitmapFont(fileName.c_str());
	DebugRenderEntity* textEntity = new DebugRenderEntity(duration, startColor, endColor, mode);
	theFont->AddVertsForText3D(textEntity->m_vertexes, Vec2(0.0f, 0.0f), textHeight, text, startColor, 1.0f, alignment);
	textEntity->m_texture = &theFont->GetTexture();
	textEntity->m_isText = true;
	textEntity->m_isBillboard = true;
	textEntity->m_position = origin;
	Mat44 transform;
	transform.AppendXRotation(90.0f);
	transform.AppendYRotation(90.0f);
	TransformVertexArray3D(textEntity->m_vertexes, transform);

	s_debugRenderer->m_debugRenderMutex.lock();
	s_debugRenderer->m_worldEntities.push_back(textEntity);
	s_debugRenderer->m_debugRenderMutex.unlock();
}

//-----------------------------------------------------------------------------------------------
void DebugAddScreenText(const std::string& text, const Vec2& position, float size, const Vec2& alignment, float duration, const Rgba8 startColor, const Rgba8 endColor, DebugRenderMode mode)
{
	BitmapFont* theFont;
	std::string fileName = "Data/Fonts/SquirrelFixedFont";
	theFont = s_debugRenderer->m_config.m_renderer->CreateOrGetBitmapFont(fileName.c_str());
	DebugRenderEntity* textEntity = new DebugRenderEntity(duration, startColor, endColor, mode);
	theFont->AddVertsForTextInBox2D(textEntity->m_vertexes, s_debugRenderer->m_screenCameraBounds, size, text, startColor, 1.0f, alignment);
	textEntity->m_texture = &theFont->GetTexture();
	textEntity->m_isText = true;
	position;
	s_debugRenderer->m_debugRenderMutex.lock();
	s_debugRenderer->m_screenEntities.push_back(textEntity);
	s_debugRenderer->m_debugRenderMutex.unlock();
}

//-----------------------------------------------------------------------------------------------
void DebugAddMessage(const std::string& text, float duration, const Rgba8 startColor, const Rgba8 endColor, DebugRenderMode mode)
{
	DebugRenderEntity* textEntity = new DebugRenderEntity(duration, startColor, endColor, mode);
	textEntity->m_message = text;
	s_debugRenderer->m_debugRenderMutex.lock();
	if (s_debugRenderer->m_messages.size() > 0)
	{
		if (s_debugRenderer->m_screenMessages[0] == nullptr)
		{
			s_debugRenderer->m_screenMessages[0] = textEntity;
		}
	}
	else
	{
		s_debugRenderer->m_screenMessages.push_back(textEntity);
	}
	s_debugRenderer->m_debugRenderMutex.unlock();
}

//-----------------------------------------------------------------------------------------------
bool Command_DebugRenderClear(EventArgs& args)
{
	DebugRenderClear();
	args;
	return true;
}

//-----------------------------------------------------------------------------------------------
bool Command_DebugRenderToggle(EventArgs& args)
{
	DebugRenderSetVisible();
	args;
	return true;
}
