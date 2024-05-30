#pragma once
#include "Engine/Math/IntVec2.hpp"
#include <string>
#include <vector>

//-----------------------------------------------------------------------------------------------
struct IntVec2;
struct Rgba8;

//-----------------------------------------------------------------------------------------------
class Image
{
	friend class Renderer;

public:
	Image();
	~Image();
	Image(const char* imageFilePath);
	Image(IntVec2 size, Rgba8 color);

	IntVec2				GetDimensions() const;
	const std::string&	GetImageFilePath() const;
	const void*			GetRawData() const;
	std::vector<Rgba8>	GetRgbaData();

private:
	std::string			m_imageFilePath;
	std::vector<Rgba8>	m_texelRgba8Data;
	IntVec2				m_dimensions;
};