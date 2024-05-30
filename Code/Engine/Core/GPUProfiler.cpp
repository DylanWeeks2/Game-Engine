#include "GPUProfiler.hpp"
#include "Engine/Renderer/Renderer.hpp"
#include "Engine/Renderer/Query.hpp"
#include "Engine/Core/ErrorWarningAssert.hpp"

//-----------------------------------------------------------------------------------------------
GPUProfiler::GPUProfiler(Renderer* renderer)
	:m_renderer(renderer)
{
	m_disjointQuery = m_renderer->CreateQuery(false, true);
	m_startFrameQuery = m_renderer->CreateQuery(true);
	m_endFrameQuery = m_renderer->CreateQuery(true);
}

//-----------------------------------------------------------------------------------------------
GPUProfiler::~GPUProfiler()
{
	delete m_disjointQuery;
	delete m_startFrameQuery;
	delete m_endFrameQuery;
}

//-----------------------------------------------------------------------------------------------
void GPUProfiler::BeginFrame()
{
	m_renderer->BeginQuery(m_disjointQuery);
	m_renderer->EndQuery(m_startFrameQuery);
}

//-----------------------------------------------------------------------------------------------
void GPUProfiler::EndFrame()
{
	m_renderer->EndQuery(m_endFrameQuery);
	m_renderer->EndQuery(m_disjointQuery);
}

//-----------------------------------------------------------------------------------------------
double GPUProfiler::GetDeltaTime(Query* currentQuery)
{
	double timeElapsed = m_renderer->GetElapsedTimeGPU(m_startFrameQuery, currentQuery, m_disjointQuery);
	return timeElapsed;
}
