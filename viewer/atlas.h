#pragma once

#include <stdint.h>
#include <math.h>
#include <vector>
#include "shared/math.h"

struct Glyph {
	int m_codepoint;
	float m_advanceX;
	V2 xy0;
	V2 xy1;
	VI2 tex0;
	VI2 tex1;
	float m_baseline;
	bool m_visible;
};

struct Atlas {
	float m_sizePixels;					//Height in pixels. Height is measured as the distance from the highest ascender to the lowest descender
	float m_ascent;
	float m_descent;
	int m_width;
	int m_height;
	unsigned char* m_pixels;
	std::vector<Glyph> m_glyphs;
	bool FindGlyph(Glyph* glyph,int c)const;
};

void DestroyFontAtlas(Atlas* atlas);
bool CreateFontAtlas(Atlas* atlas);
