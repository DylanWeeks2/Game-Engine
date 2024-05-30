#include "Image.hpp"
#include "Rgba8.hpp"
#include "StringUtils.hpp"
#include "Engine/Core/ErrorWarningAssert.hpp"
#define STB_IMAGE_IMPLEMENTATION // Exactly one .CPP (this Image.cpp) should #define this before #including stb_image.h
#include "ThirdParty/stb/stb_image.h"

//-----------------------------------------------------------------------------------------------
Image::Image()
{
}

//-----------------------------------------------------------------------------------------------
Image::~Image()
{
}

//-----------------------------------------------------------------------------------------------
Image::Image(const char* imageFilePath)
{
	m_imageFilePath = imageFilePath;

	IntVec2 dimensions = IntVec2(0, 0);		// This will be filled in for us to indicate image width & height
	int bytesPerTexel = 0; // This will be filled in for us to indicate how many color components the image had (e.g. 3=RGB=24bit, 4=RGBA=32bit)
	int numComponentsRequested = 0; // don't care; we support 3 (24-bit RGB) or 4 (32-bit RGBA)

	// Load (and decompress) the image RGB(A) bytes from a file on disk into a memory buffer (array of bytes)
	stbi_set_flip_vertically_on_load(1); // We prefer uvTexCoords has origin (0,0) at BOTTOM LEFT
	unsigned char* texelData = stbi_load(imageFilePath, &dimensions.x, &dimensions.y, &bytesPerTexel, numComponentsRequested);
	m_dimensions = dimensions;

	// Check if the load was successful
	GUARANTEE_OR_DIE(texelData, Stringf("Failed to load image \"%s\"", imageFilePath));

	int totalTexels = m_dimensions.x * m_dimensions.y;

	for (int texelIndex = 0; texelIndex < totalTexels; texelIndex++)
	{
		int byteOffset = texelIndex * bytesPerTexel;
		unsigned char r = texelData[byteOffset + 0];
		unsigned char g = texelData[byteOffset + 1];
		unsigned char b = texelData[byteOffset + 2];
		unsigned char a = 255;

		if (bytesPerTexel == 4)
		{
			a = texelData[byteOffset + 3];
		}

		Rgba8 currentRgba = Rgba8(r, g, b, a);
		m_texelRgba8Data.push_back(currentRgba);
	}

	// Free the raw image texel data now that we've sent a copy of it down to the GPU to be stored in video memory
	stbi_image_free(texelData);
	texelData = nullptr;
}

//-----------------------------------------------------------------------------------------------
Image::Image(IntVec2 size, Rgba8 color)
{
	m_dimensions = size;

	int totalTexels = m_dimensions.x * m_dimensions.y;

	for (int texelIndex = 0; texelIndex < totalTexels; texelIndex++)
	{
		unsigned char r = color.r;
		unsigned char g = color.g;
		unsigned char b = color.b;
		unsigned char a = color.a;
		Rgba8 currentRgba = Rgba8(r, g, b, a);
		m_texelRgba8Data.push_back(currentRgba);
	}
}

//-----------------------------------------------------------------------------------------------
IntVec2 Image::GetDimensions() const
{
	return m_dimensions;
}

//-----------------------------------------------------------------------------------------------
const std::string& Image::GetImageFilePath() const
{
	return m_imageFilePath;
}

//-----------------------------------------------------------------------------------------------
const void* Image::GetRawData() const
{
	return m_texelRgba8Data.data();
}

//-----------------------------------------------------------------------------------------------
std::vector<Rgba8> Image::GetRgbaData()
{
	return m_texelRgba8Data;
}
