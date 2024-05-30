#pragma once
#include <vector>

//-----------------------------------------------------------------------------------------------
class Renderer;
class Query;

//-----------------------------------------------------------------------------------------------
class GPUProfiler
{
public:
	GPUProfiler(Renderer* renderer);
	~GPUProfiler();

	void	BeginFrame();
	void	EndFrame();
	double	GetDeltaTime(Query* currentQuery);

public:
	Renderer*	m_renderer;
	Query*		m_disjointQuery;
	Query*		m_startFrameQuery;
	Query*		m_endFrameQuery;
};