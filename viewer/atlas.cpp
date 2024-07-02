#include <stdio.h>
#include <stdarg.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <vector>
#include "atlas.h"
#include "shared/file.h"
#include "shared/output.h"

#define STBRP_STATIC
#define STB_RECT_PACK_IMPLEMENTATION
#include "stb_rectpack.h"

#define STB_TRUETYPE_IMPLEMENTATION
#include "stb_truetype.h"

#define FONTNAME "$(DATA)/fonts/inconsolata/InconsolataGo-Bold.ttf"

bool Atlas::FindGlyph(Glyph* glyph,int c)const {
	static bool first=true;
	static int lookup[512];		//Must be more than number glyph codepoints
	if(first) {
		memset(lookup,0xff,sizeof(lookup));
		for(int i=0;i!=(int)m_glyphs.size();i++) {
			lookup[m_glyphs[i].m_codepoint]=i;
		}
		first=false;
	}
	int ix=lookup[c];
	if(ix==-1)
		return false;
	*glyph=m_glyphs[ix];
	return true;
}

void* LoadFile(const char* filename) {
	std::string fn=GetFileNameRemap(filename);
	FILE*f=fopen(fn.c_str(),"rb");
	if(!f) {
		uprintf("ERROR: Could not open file %s!\n",filename);
		return 0;
	}
	fseek(f,0,SEEK_END);
	size_t sz=ftell(f);
	fseek(f,0,SEEK_SET);
	void*p=malloc(sz);
	if(fread(p,1,sz,f)!=sz) {
		uprintf("ERROR: Could not load file %s!\n",filename);
		free(p);
		return 0;
	}
	return p;
}

bool CreateFontAtlas(Atlas* atlas) {
	atlas->m_sizePixels=512.0f;
	int glyphPadding=32;
	void* fontData=LoadFile(FONTNAME);
	if(!fontData)
		return false;
	stbtt_fontinfo fontInfo;
	int font_offset=stbtt_GetFontOffsetForIndex((unsigned char*)fontData,0);
	assert(font_offset >= 0 && "fontData is incorrect,or FontNo 0 cannot be found.");
	if(!stbtt_InitFont(&fontInfo,(unsigned char*)fontData,font_offset)) {
		free(fontData);
		return false;
	}
	std::vector<int> glyphsList;
	for(uint32_t codepoint=0x20;codepoint<=0xa0;codepoint++) {
		if(!stbtt_FindGlyphIndex(&fontInfo,codepoint))
			continue;
		glyphsList.push_back(codepoint);
	}
	std::vector<stbrp_rect> buf_rects(glyphsList.size());
	std::vector<stbtt_packedchar> buf_packedchars(glyphsList.size());

	uint8_t oversampleH=1;
	uint8_t oversampleV=1;
	float scale=stbtt_ScaleForPixelHeight(&fontInfo,atlas->m_sizePixels);
	int padding=glyphPadding;
	for(int glyph_i=0;glyph_i<(int)glyphsList.size();glyph_i++) {
		int x0,y0,x1,y1;
		int glyph_index_in_font=stbtt_FindGlyphIndex(&fontInfo,glyphsList[glyph_i]);
		assert(glyph_index_in_font);
		stbtt_GetGlyphBitmapBoxSubpixel(&fontInfo,glyph_index_in_font,scale*oversampleH,scale*oversampleV,0,0,&x0,&y0,&x1,&y1);
		buf_rects[glyph_i].w=(stbrp_coord)(x1-x0+padding+oversampleH-1);
		buf_rects[glyph_i].h=(stbrp_coord)(y1-y0+padding+oversampleV-1);
	}

	atlas->m_width=4096;
	atlas->m_height=4096;

	int TEX_HEIGHT_MAX=1024*32;
	stbtt_pack_context spc={};
	stbtt_PackBegin(&spc,NULL,atlas->m_width,TEX_HEIGHT_MAX,0,glyphPadding,NULL);

	stbrp_pack_rects((stbrp_context*)spc.pack_info,buf_rects.data(),(int)glyphsList.size());

	atlas->m_pixels=new unsigned char[atlas->m_width*atlas->m_height];
	memset(atlas->m_pixels,0,atlas->m_width*atlas->m_height);
	spc.pixels=atlas->m_pixels;
	spc.height=atlas->m_height;

	stbtt_pack_range packRange;
	packRange.font_size=atlas->m_sizePixels;
	packRange.first_unicode_codepoint_in_range=0;
	packRange.array_of_unicode_codepoints=glyphsList.data();
	packRange.num_chars=(int)glyphsList.size();
	packRange.chardata_for_range=buf_packedchars.data();
	packRange.h_oversample=oversampleH;
	packRange.v_oversample=oversampleV;
	stbtt_PackFontRangesRenderIntoRects(&spc,&fontInfo,&packRange,1,buf_rects.data());

	stbtt_PackEnd(&spc);
	buf_rects.clear();

	int unscaled_ascent,unscaled_descent,unscaled_line_gap;
	stbtt_GetFontVMetrics(&fontInfo,&unscaled_ascent,&unscaled_descent,&unscaled_line_gap);
	atlas->m_ascent=unscaled_ascent*scale;
	atlas->m_descent=unscaled_descent*scale;
	for(int glyph_i=0;glyph_i<(int)glyphsList.size();glyph_i++) {
		int codepoint=glyphsList[glyph_i];
		stbtt_packedchar& pc=buf_packedchars[glyph_i];
		stbtt_aligned_quad q;
		float unused_x=0.0f,unused_y=0.0f;
		stbtt_GetPackedQuad(buf_packedchars.data(),atlas->m_width,atlas->m_height,glyph_i,&unused_x,&unused_y,&q,0);
		Glyph glyph;
		glyph.m_advanceX=pc.xadvance;
		glyph.m_codepoint=codepoint;

		glyph.xy0=V2(q.x0,q.y0);
		glyph.xy1=V2(q.x1,q.y1);
		glyph.tex0=VI2(pc.x0,pc.y0);
		glyph.tex1=VI2(pc.x1,pc.y1);

		if(q.y1==q.y0) {
			glyph.m_visible=false;
			glyph.m_baseline=0;
		}else{
			glyph.m_visible=true;
			glyph.m_baseline=(0-q.y0)/(q.y1-q.y0);
		}
		atlas->m_glyphs.push_back(glyph);
	}

	buf_rects.clear();
	buf_packedchars.clear();
	free(fontData);		//Free loaded font data

	return true;
}

void DestroyFontAtlas(Atlas* atlas) {
	atlas->m_glyphs.clear();
	delete [] atlas->m_pixels;
	atlas->m_pixels=0;
}
