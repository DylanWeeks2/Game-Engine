#pragma once
#include <string>
#include <vector>

//-----------------------------------------------------------------------------------------------
struct Vertex_PCUTBN;
struct Mat44;
struct Vec3;
class  RigidBody3D;

//-----------------------------------------------------------------------------------------------
class OBJLoader;
extern OBJLoader* g_theOBJLoader;

//-----------------------------------------------------------------------------------------------
class OBJLoader
{
public:
	static void Load(std::string filename, std::vector<Vertex_PCUTBN>& vertices, std::vector<unsigned int>& indices, Mat44& transform );
	static void LoadIntoRigidBody(std::string filename, RigidBody3D* rigidBody, Mat44& transform);
};