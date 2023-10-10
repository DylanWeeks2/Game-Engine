#pragma once
#include <string>
#include <vector>

//-----------------------------------------------------------------------------------------------
struct Vertex_PNCU;
struct Mat44;

//-----------------------------------------------------------------------------------------------
class OBJLoader;
extern OBJLoader* g_theOBJLoader;

//-----------------------------------------------------------------------------------------------
class OBJLoader
{
public:
	static void Load(std::string filename, std::vector<Vertex_PNCU>& vertices, 
		std::vector<int>& indices, Mat44& transform );
};