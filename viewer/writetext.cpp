#include <stdio.h>
#include <stdarg.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <vector>
#include "shared/file.h"
#include "shared/output.h"
#include "writetext.h"
#include "atlas.h"

struct Vertex {
	V2 pos;
	V2 uv;
	V2 uvscale;
	V4 region;
	uint32_t col;
};

void AddQuadVertices(std::vector<Vertex>* vertices,std::vector<uint16_t>* indices,const V2& xy0,const V2& xy1,const V2& st0,const V2& st1,const V2& uvScale,const V4& region,uint32_t col=0xffffffff) {
	int ix=(int)indices->size();
	indices->resize(indices->size()+6);
	uint16_t* p=indices->data()+ix;
	ix=(int)vertices->size();
	p[0]=ix+0;
	p[1]=ix+1;
	p[2]=ix+2;
	p[3]=ix+2;
	p[4]=ix+3;
	p[5]=ix+0;
	vertices->resize(vertices->size()+4);
	Vertex* v=&vertices->at(vertices->size()-4);
	v->pos=xy0;
	v->uv=st0;
	v->uvscale=uvScale;
	v->region=region;
	v->col=col;
	v=&vertices->at(vertices->size()-3);
	v->pos=V2(xy1.x,xy0.y);
	v->uv=V2(st1.x,st0.y);
	v->uvscale=uvScale;
	v->region=region;
	v->col=col;
	v=&vertices->at(vertices->size()-2);
	v->pos=xy1;
	v->uv=st1;
	v->uvscale=uvScale;
	v->region=region;
	v->col=col;
	v=&vertices->at(vertices->size()-1);
	v->pos=V2(xy0.x,xy1.y);
	v->uv=V2(st0.x,st1.y);
	v->uvscale=uvScale;
	v->region=region;
	v->col=col;
}

void DrawTextLine(std::vector<Vertex>* vertices,std::vector<uint16_t>* indices,const char* string,const V2& pos,float sizePixels,const Atlas& atlas,uint32_t col) {
	float scale=sizePixels/atlas.m_sizePixels;
	const char* p=string;
	float linePosX=0;
	while(*p) {
		Glyph glyph;
		if(!atlas.FindGlyph(&glyph,*p)) {
			uprintf("ERROR: Glyph not found\n");
			continue;
		}
		if(!glyph.m_visible) {			//No visible character, could be space
			linePosX+=glyph.m_advanceX*scale;
			p++;
			continue;
		}
		V2 xy0=glyph.xy0;
		V2 xy1=glyph.xy1;
		V2 tex0=V2((float)glyph.tex0.x,(float)glyph.tex0.y);
		V2 tex1=V2((float)glyph.tex1.x,(float)glyph.tex1.y);
		xy0=xy0*scale;
		xy1=xy1*scale;
		V2 cell=(tex1-tex0)/(xy1-xy0);
#if 1
		//Center to get sharp edges around baseline
		V2 xypivot=V2(glyph.xy1.x,lerp(glyph.xy0.y,glyph.xy1.y,glyph.m_baseline));
		V2 texpivot=V2(tex1.x,lerp(tex0.y,tex1.y,glyph.m_baseline));
		xypivot=xypivot*scale;
		//Center pixel
		float frag=fmod(xypivot.y,1.0f)-0.49f;
		xy0.y-=frag;
		xy1.y-=frag;
		xypivot.y-=frag;
		//Snap to baseline
		V2 sz=V2(ceilf(xypivot.x-xy0.x),ceilf(xypivot.y-xy0.y));
		xy0=xypivot-sz;
		tex0=texpivot-sz*cell;
		//Extend to full size
		sz=V2(ceilf(xy1.x-xy0.x),ceilf(xy1.y-xy0.y));
		xy1=xy0+sz;
		tex1=tex0+sz*cell;
#endif
		//Extend with one texture cell to get edge anti-aliasing correct
		xy0=xy0-V2(1,1)*1;
		xy1=xy1+V2(1,1)*1;
		tex0=tex0-cell*1;
		tex1=tex1+cell*1;

		V2 p0=pos+xy0+V2(linePosX,0);
		V2 p1=pos+xy1+V2(linePosX,0);

		V4 region=V4((float)(glyph.tex0.x-1),(float)(glyph.tex0.y-1),(float)(glyph.tex1.x+1),(float)(glyph.tex1.y+1));
		AddQuadVertices(vertices,indices,p0,p1,tex0,tex1,cell,region,col);

		linePosX+=glyph.m_advanceX*scale;
		p++;
	}
}

inline unsigned char ReadBilinearPixel(float fx,float fy,const unsigned char* map,int width,int height) {
	if(fx<0)return 0;
	if(fy<0)return 0;
	int xCuri=(int)(fx*65536.0f);
	int yCuri=(int)(fy*65536.0f);
	if(xCuri>=(width-1)<<16)
		return 0;
	if(yCuri>=(height-1)<<16)
		return 0;
	int x=xCuri>>16;
	int xFragi=(xCuri&0xffff)>>4;
	int y=yCuri>>16;
	int yFragi=(yCuri&0xffff)>>8;
	int omxFragi=0x1000-xFragi;
	int omyFragi=0x100-yFragi;
	const unsigned char* p0=map+y*width+x;
	const unsigned char* p1=p0+width;
	int s0i=omxFragi*omyFragi;
	int s1i=xFragi*omyFragi;
	int s2i=yFragi*omxFragi;
	int s3i=xFragi*yFragi;
	return (p0[0]*s0i+p0[1]*s1i+p1[0]*s2i+p1[1]*s3i)>>20;
}

void WriteTextRGB(const V2& position,const char* text,float sizePixels,uint32_t color,uint8_t* pixelsRGB,int width,int height,const Atlas& atlas) {
	std::vector<Vertex> vertices;
	std::vector<uint16_t> indices;
	DrawTextLine(&vertices,&indices,text,position,sizePixels,atlas,0xffffff);
	int numQuads=(int)indices.size()/6;
	uint8_t a1=(color>>24)&0xff;
	uint8_t r1=(color>>16)&0xff;
	uint8_t g1=(color>>8)&0xff;
	uint8_t b1=(color)&0xff;
	
	for(int i=0;i!=numQuads;i++) {
		const Vertex& v0=vertices[indices[i*6+0]];
		const Vertex& v2=vertices[indices[i*6+3]];
		int stx=(int)v0.pos.x;
		int enx=(int)(v2.pos.x+0.999f);
		int sty=(int)v0.pos.y;
		int eny=(int)(v2.pos.y+0.999f);
		float sx=(float)(1+enx-stx);
		float sy=(float)(1+eny-sty);
		float fy=v0.uv.y;
		float fys=(v2.uv.y-v0.uv.y)/sy;
		for(int y=sty;y<=eny;y++) {
			float fx=v0.uv.x;
			float fxs=(v2.uv.x-v0.uv.x)/sx;
			for(int x=stx;x<=enx;x++) {
				if(x>=0 && y>=0 && x<width && y<height)
					if(fx>v0.region.x && fx<v2.region.z && fy>v0.region.y && fy<v2.region.w) {
						uint8_t v=ReadBilinearPixel(fx,fy,atlas.m_pixels,atlas.m_width,atlas.m_height);
						int o=((y*width)+x)*3;
						uint8_t r=pixelsRGB[o+0];
						uint8_t g=pixelsRGB[o+1];
						uint8_t b=pixelsRGB[o+2];
						v=(uint8_t)(((int)v*(int)a1)>>8);
						pixelsRGB[o+0]=(r*(0xff-v)+r1*v)>>8;
						pixelsRGB[o+1]=(g*(0xff-v)+g1*v)>>8;
						pixelsRGB[o+2]=(b*(0xff-v)+b1*v)>>8;
					}
				fx+=fxs;
			}
			fy+=fys;
		}
	}
}