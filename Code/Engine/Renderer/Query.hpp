#pragma once

//-----------------------------------------------------------------------------------------------
struct ID3D11Query;

//-----------------------------------------------------------------------------------------------
class Query
{
	friend class Renderer;

public:
	Query();
	Query(const Query& copy) = delete;
	virtual ~Query();

public:
	ID3D11Query* m_query = nullptr;
};