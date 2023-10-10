#pragma once

struct Rgba8
{
public:
	unsigned char r = 255;
	unsigned char g = 255;
	unsigned char b = 255;
	unsigned char a = 255;

	Rgba8();
	explicit Rgba8(unsigned char initialR, unsigned char initialG, unsigned char initialB, unsigned char initialA);
	Rgba8(const Rgba8& copyFrom);

	void SetFromText(char const* text);
	void GetAsFloats(float* colorAsFloats) const;

	bool operator==(const Rgba8& compare) const;
	void operator*=(const unsigned char uniformScale);

	static Rgba8 WHITE;
	static Rgba8 RED;
	static Rgba8 BLUE;
	static Rgba8 GREEN;
	static Rgba8 MEDIUM_GREEN;
	static Rgba8 BLACK;
	static Rgba8 CYAN;
	static Rgba8 MAGENTA;
	static Rgba8 YELLOW;
	static Rgba8 GREY;
	static Rgba8 DARK_GREY;
};