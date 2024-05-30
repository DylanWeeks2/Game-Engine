#include "OBJLoader.hpp"
#include "ErrorWarningAssert.hpp"
#include "XmlUtils.hpp"
#include "FileUtils.hpp"
#include "Vertex_PCUTBN.hpp"
#include "Engine/Math/Vec3.hpp"
#include "Engine/Math/Mat44.hpp"
#include "Engine/Math/MathUtils.hpp"
#include "Engine/Core/Time.hpp"
#include "Engine/Core/VertexUtils.hpp"

//-----------------------------------------------------------------------------------------------
void OBJLoader::Load(std::string filename, std::vector<Vertex_PCUTBN>& vertices, std::vector<unsigned int>& indices, Mat44& transform)
{
	Strings verts;
	Strings vertTextureCoords;
	Strings vertNormals;
	Strings faces;
	std::vector<int> textureIndices;
	std::vector<int> normalIndices;
	std::vector<Vec3> parsedVerts;
	std::vector<Vec3> normals;
	std::vector<Vec2> textureCoords;
	int totalDataFaces = 0;
	int totalDataVerts = 0;
	int totalDataTriangles = 0;
	float parseStartTime = 0.0f;
	float parseEndTime = 0.0f;
	float createStartTime = 0.0f;
	float createEndTime = 0.0f;

	std::string buffer = "";
	FileReadToString(buffer, filename);

	parseStartTime = float(GetCurrentTimeSeconds());
	Strings stringsByLine = SplitStringOnDelimiter(buffer, '\r');
	if (stringsByLine.size() == 1)
	{
		stringsByLine = SplitStringOnDelimiter(buffer, '\n');
	}
	for (int stringIndex = 0; stringIndex < stringsByLine.size(); stringIndex++)
	{
		Strings splitLineStrings = SplitStringOnDelimiter(stringsByLine[stringIndex], ' ');
		if (splitLineStrings[0] == "v")
		{
			totalDataVerts++;
			int counter = 1;
			if (splitLineStrings[1] == "")
			{
				while (splitLineStrings[counter] == "")
				{
					counter++;
				}
			}
			verts.push_back(splitLineStrings[counter]);
			verts.push_back(splitLineStrings[counter + 1]);
			verts.push_back(splitLineStrings[counter + 2]);
		}
		else if (splitLineStrings[0] == "vt")
		{
			int counter = 1;
			if (splitLineStrings[1] == "")
			{
				while (splitLineStrings[counter] == "")
				{
					counter++;
				}
			}
			vertTextureCoords.push_back(splitLineStrings[counter]);
			vertTextureCoords.push_back(splitLineStrings[counter + 1]);
		}
		else if (splitLineStrings[0] == "vn")
		{
			int counter = 1;
			if (splitLineStrings[1] == "")
			{
				while (splitLineStrings[counter] == "")
				{
					counter++;
				}
			}
			vertNormals.push_back(splitLineStrings[counter]);
			vertNormals.push_back(splitLineStrings[counter + 1]);
			vertNormals.push_back(splitLineStrings[counter + 2]);
		}
		else if (splitLineStrings[0] == "f")
		{
			totalDataFaces++;
			int counter = 1;
			if (splitLineStrings[1] == "")
			{
				while (splitLineStrings[counter] == "")
				{
					counter++;
				}
			}

			Strings onlyFaceData;
			for (int faceIndex = counter; faceIndex < splitLineStrings.size(); faceIndex++)
			{
				if (splitLineStrings[faceIndex] != "")
				{
					onlyFaceData.push_back(splitLineStrings[faceIndex]);
				}
			}

			totalDataTriangles++;
			for (int faceIndex = 0; faceIndex < onlyFaceData.size(); faceIndex++)
			{ 
				if (onlyFaceData.size() == 3)
				{
					faces.push_back(onlyFaceData[faceIndex]);
				}
				else
				{
					if (faceIndex < 3)
					{
						faces.push_back(onlyFaceData[faceIndex]);
					}
					else
					{	
						faces.push_back(onlyFaceData[0]);
						faces.push_back(onlyFaceData[faceIndex - 1]);
						faces.push_back(onlyFaceData[faceIndex]);
						totalDataTriangles++;
					}
				}
			}
		}
	}

	//Faces Splitting
	if (faces.size() != 0)
	{
		//Split faces 
		for (int index = 0; index < faces.size(); index++)
		{
			Strings splitFaces = SplitStringOnDelimiter(faces[index], '/');
			indices.push_back(atoi(splitFaces[0].c_str()) - 1);

			if (splitFaces.size() <= 1)
			{
				continue;
			}

			if (splitFaces[1] != "/")
			{
				textureIndices.push_back(atoi(splitFaces[1].c_str()) - 1);
			}
			else
			{
				textureIndices.push_back(-1);
			}

			if (splitFaces[2] != "/")
			{
				normalIndices.push_back(atoi(splitFaces[2].c_str()) - 1);
			}
			else
			{
				normalIndices.push_back(-1);
			}
		}
	}
	parseEndTime = float(GetCurrentTimeSeconds());

	//Creation
	createStartTime = float(GetCurrentTimeSeconds());

	//Store Parsed Verts into Vec3s
	parsedVerts.reserve(verts.size() / 3);
	for (int vertIndex = 1; vertIndex <= verts.size(); vertIndex++)
	{
		if (vertIndex % 3 == 0)
		{
			float x = static_cast<float>(atof(verts[vertIndex - 3].c_str()));
			float y = static_cast<float>(atof(verts[vertIndex - 2].c_str()));
			float z = static_cast<float>(atof(verts[vertIndex - 1].c_str()));
			parsedVerts.push_back(Vec3(x, y, z));
			vertIndex += 2;
		}
	}

	//Transform all verts to the matrix
	for (int vertIndex = 0; vertIndex < parsedVerts.size(); vertIndex++)
	{
		parsedVerts[vertIndex] = transform.TransformPosition3D(parsedVerts[vertIndex]);
	}


	//Put normals into Vec3s
	if (vertNormals.size() != 0)
	{
		normals.reserve(vertNormals.size() / 3);
		for (int normalIndex = 1; normalIndex <= vertNormals.size(); normalIndex++)
		{
			if (normalIndex % 3 == 0)
			{
				float x = static_cast<float>(atof(vertNormals[normalIndex - 3].c_str()));
				float y = static_cast<float>(atof(vertNormals[normalIndex - 2].c_str()));
				float z = static_cast<float>(atof(vertNormals[normalIndex - 1].c_str()));
				normals.push_back(Vec3(x, y, z));
				normalIndex += 2;
			}
		}
	}

	//Put textures into Vec2s
	if (vertTextureCoords.size() != 0)
	{
		textureCoords.reserve(vertTextureCoords.size() / 2);
		for (int coordindex = 1; coordindex <= vertTextureCoords.size(); coordindex++)
		{
			if (coordindex % 2 == 0)
			{
				float u = static_cast<float>(atof(vertTextureCoords[coordindex - 2].c_str()));
				float v = static_cast<float>(atof(vertTextureCoords[coordindex - 1].c_str()));
				textureCoords.push_back(Vec2(u, v));
				coordindex += 1;
			}
		}
	}

	if (faces.size() > 0)
	{
		for (int faceIndex = 1; faceIndex <= indices.size(); faceIndex++)
		{
			if (faceIndex % 3 == 0)
			{
				//Normal Calculations and Assignments
				Vec3 normal1 = Vec3();
				Vec3 normal2 = Vec3();
				Vec3 normal3 = Vec3();
				if (vertNormals.size() == 0)
				{
					Vec3 u = (parsedVerts[indices[faceIndex - 2]] - parsedVerts[indices[faceIndex - 3]]).GetNormalized();
					Vec3 v = (parsedVerts[indices[faceIndex - 1]] - parsedVerts[indices[faceIndex - 2]]).GetNormalized();
					normal1 = CrossProduct3D(u, v);
					normal2 = CrossProduct3D(u, v);
					normal3 = CrossProduct3D(u, v);
				}
				else
				{
					normal1 = normals[normalIndices[faceIndex - 3]];
					normal2 = normals[normalIndices[faceIndex - 2]];
					normal3 = normals[normalIndices[faceIndex - 1]];
				}

				//Texture Coords Calculations and Assignments
				Vec2 textureCoords1 = Vec2();
				Vec2 textureCoords2 = Vec2();
				Vec2 textureCoords3 = Vec2();
				if (vertTextureCoords.size() != 0)
				{
					textureCoords1 = textureCoords[textureIndices[faceIndex - 3]];
					textureCoords2 = textureCoords[textureIndices[faceIndex - 2]];
					textureCoords3 = textureCoords[textureIndices[faceIndex - 1]];
				}

				vertices.push_back(Vertex_PCUTBN(parsedVerts[indices[faceIndex - 3]], Rgba8(), textureCoords1, Vec3(), Vec3(), normal1));
				vertices.push_back(Vertex_PCUTBN(parsedVerts[indices[faceIndex - 2]], Rgba8(), textureCoords2, Vec3(), Vec3(), normal2));
				vertices.push_back(Vertex_PCUTBN(parsedVerts[indices[faceIndex - 1]], Rgba8(), textureCoords3, Vec3(), Vec3(), normal3));
				indices[faceIndex - 3] = int(vertices.size()) - 3;
				indices[faceIndex - 2] = int(vertices.size()) - 2;
				indices[faceIndex - 1] = int(vertices.size()) - 1;
				faceIndex += 2;
			}
		}
	}
	else
	{
		for (int index = 1; index <= parsedVerts.size(); index++)
		{
			if (index % 3 == 0)
			{
				Vec3 u = (parsedVerts[index - 2] - parsedVerts[index - 3]).GetNormalized();
				Vec3 v = (parsedVerts[index - 1] - parsedVerts[index - 2]).GetNormalized();
				Vec3 normal = CrossProduct3D(u, v);
				vertices.push_back(Vertex_PCUTBN(parsedVerts[index - 3], Rgba8(), Vec2(), Vec3(), Vec3(), normal));
				vertices.push_back(Vertex_PCUTBN(parsedVerts[index - 2], Rgba8(), Vec2(), Vec3(), Vec3(), normal));
				vertices.push_back(Vertex_PCUTBN(parsedVerts[index - 1], Rgba8(), Vec2(), Vec3(), Vec3(), normal));
				index += 2;
			}
		}
	}
	
	//Transform all normals to the matrix
	if (vertNormals.size() != 0)
	{
		for (int vertIndex = 0; vertIndex < vertices.size(); vertIndex++)
		{
			vertices[vertIndex].m_normal = transform.TransformVectorQuantity3D(vertices[vertIndex].m_normal).GetNormalized();
		}
	}	

	CalculateTangentSpaceVectors(vertices, indices);

	createEndTime = float(GetCurrentTimeSeconds());

	//Debug Print Logic
	DebuggerPrintf("-----------------------------------------------------------------------------------------------\n");
	DebuggerPrintf("Loaded .obj file %s\n", filename.c_str());
	DebuggerPrintf("[file data] vertices: %d, texture coordinates: %d, normals: %d, faces: %d triangles: %d\n", 
		totalDataVerts, textureCoords.size(), normals.size(), totalDataFaces, totalDataTriangles);
	DebuggerPrintf("[loaded mesh] vertices: %d, indices: %d\n", vertices.size(), indices.size());
	DebuggerPrintf("[time] parse: %f seconds, create: %f seconds\n", parseEndTime - parseStartTime, createEndTime - createStartTime);
	DebuggerPrintf("-----------------------------------------------------------------------------------------------\n");
}
