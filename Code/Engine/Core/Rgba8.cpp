#include "Rgba8.hpp"
#include "StringUtils.hpp"
#include "Engine/Math/MathUtils.hpp"

Rgba8 Rgba8::WHITE = Rgba8(255, 255, 255, 255);
Rgba8 Rgba8::RED = Rgba8(255, 0, 0, 255);
Rgba8 Rgba8::ORANGE = Rgba8(255, 165, 0, 255);
Rgba8 Rgba8::GREEN = Rgba8(0, 255, 0, 255);
Rgba8 Rgba8::MEDIUM_GREEN = Rgba8(0, 128, 0, 255);
Rgba8 Rgba8::BLUE = Rgba8(0, 0, 255, 255);
Rgba8 Rgba8::BLACK = Rgba8(0, 0, 0, 255);
Rgba8 Rgba8::CYAN = Rgba8(0, 255, 255, 255);
Rgba8 Rgba8::MAGENTA = Rgba8(255, 0, 255, 255);
Rgba8 Rgba8::YELLOW = Rgba8(255, 255, 0, 255);
Rgba8 Rgba8::GREY = Rgba8(128, 128, 128, 255);
Rgba8 Rgba8::DARK_GREY = Rgba8(30, 30, 30, 255);

//-----------------------------------------------------------------------------------------------
Rgba8::Rgba8() 
{
}

//-----------------------------------------------------------------------------------------------
Rgba8::Rgba8(unsigned char initialR, unsigned char initialG, unsigned char initialB, unsigned char initialA)
	: r(initialR)
	, g(initialG)
	, b(initialB)
	, a(initialA)
{
}

//-----------------------------------------------------------------------------------------------
Rgba8::Rgba8(const Rgba8& copyFrom)
	: r(copyFrom.r)
	, g(copyFrom.g)
	, b(copyFrom.b)
	, a(copyFrom.a)
{
}

//-----------------------------------------------------------------------------------------------
void Rgba8::SetFromText(char const* text)
{
	Strings stringXY;
	stringXY = SplitStringOnDelimiter(text, ',');

	if (stringXY.size() == 3)
	{
		r = static_cast<unsigned char>(atoi(stringXY[0].c_str()));
		g = static_cast<unsigned char>(atoi(stringXY[1].c_str()));
		b = static_cast<unsigned char>(atoi(stringXY[2].c_str()));
		a = 255;
	}
	else
	{
		r = static_cast<unsigned char>(atoi(stringXY[0].c_str()));
		g = static_cast<unsigned char>(atoi(stringXY[1].c_str()));
		b = static_cast<unsigned char>(atoi(stringXY[2].c_str()));
		a = static_cast<unsigned char>(atoi(stringXY[3].c_str()));
	}
}

//-----------------------------------------------------------------------------------------------
void Rgba8::GetAsFloats(float* colorAsFloats) const
{
	colorAsFloats[0] = NormalizeByte(r);
	colorAsFloats[1] = NormalizeByte(g);
	colorAsFloats[2] = NormalizeByte(b);
	colorAsFloats[3] = NormalizeByte(a);
}

//-----------------------------------------------------------------------------------------------
bool Rgba8::operator==(const Rgba8& compare) const
{
	if (r == compare.r && g == compare.g && b == compare.b && a == compare.a)
	{
		return true;
	}

	return false;
}

//-----------------------------------------------------------------------------------------------
void Rgba8::operator*=(const unsigned char uniformScale)
{
	r *= uniformScale;
	g *= uniformScale;
	b *= uniformScale;
	a *= uniformScale;
}
