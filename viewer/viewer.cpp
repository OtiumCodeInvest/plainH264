#ifdef _WIN32
#define WIN32_LEAN_AND_MEAN
#include <windows.h>
#endif

#include <stdio.h>
#include <stdarg.h>
#include <stdlib.h>
#include <string.h>
#include <string>
#include <mutex>
#include <thread>
#include <atomic>
#include <fstream>      // std::ofstream

#include"shared/file.h"
#include"shared/output.h"
#include"shared/math.h"
#include"shared/time.h"

#include "atlas.h"
#include "encodeimage.h"
#include "decodeimage.h"
#include "writetext.h"

#include <GL/glew.h>
#include <GLFW/glfw3.h>
#include "imgui.h"
#include "imgui_internal.h"
#include "backends/imgui_impl_opengl3.h"
#include "backends/imgui_impl_glfw.h"

#define STB_IMAGE_WRITE_STATIC
#define STBI_UNUSED_SYMBOLS
#define STB_IMAGE_IMPLEMENTATION
#define STB_IMAGE_WRITE_IMPLEMENTATION
#include "shared/stb_image.h"

uint8_t* DecodeImageFromMemory(const uint8_t* pBuffer,int lBufferByteSize,int* plWidth,int* pHeight,int* plChannels) {
	return stbi_load_from_memory(pBuffer,lBufferByteSize,plWidth,pHeight,plChannels,0);
}

void RGB2Yuv420Ch(uint8_t* chY,uint8_t* chU,uint8_t* chV,const uint8_t* rgb,uint32_t width,uint32_t height) {
	uint32_t i=0;
	uint32_t uvPos=0;
	for(uint32_t y=0;y!=height;y++) {
		if(y&1) {
			for(uint32_t x=0;x!=width;x+=2) {
				uint8_t r=rgb[3*i];
				uint8_t g=rgb[3*i+1];
				uint8_t b=rgb[3*i+2];
				chY[i++]=((66*r+129*g+25*b)>>8)+16;
				chU[uvPos]=((-38*r+-74*g+112*b)>>8)+128;
				chV[uvPos++]=((112*r+-94*g+-18*b)>>8)+128;
				r=rgb[3*i];
				g=rgb[3*i+1];
				b=rgb[3*i+2];
				chY[i++]=((66*r+129*g+25*b)>>8)+16;
			}
		}else{
			for(uint32_t x=0;x!=width;x++) {
				uint8_t r=rgb[3*i];
				uint8_t g=rgb[3*i+1];
				uint8_t b=rgb[3*i+2];
				chY[i++]=((66*r+129*g+25*b)>>8)+16;
			}
		}
	}
}

inline ImVec4 ImLoad(const V4& v){
	return ImVec4(v.x,v.y,v.z,v.w);
}
inline ImVec2 ImLoad(const V2& v){
	return ImVec2(v.x,v.y);
}
inline V2 VLoad(const ImVec2& v){
	return V2(v.x,v.y);
}

class GuiLog {
	public:
		enum PRIMITIVE_TYPE {
			NA=0,
			TEXT=1
		};
		struct Primitive {
			Primitive() : m_type(NA) {}
			Primitive(PRIMITIVE_TYPE type) : m_type(type){}
			PRIMITIVE_TYPE m_type;
			uint32_t m_color=0;
			std::string m_text;
		};
		void InitLog();
		void DrawLog();
		void AddText(const char* text);
		std::mutex m_lock;
		std::vector<Primitive> m_primitives;
		int m_maxSize=0;
		int m_tail = 0;
};

void GuiLog::AddText(const char* text) {
	const std::lock_guard<std::mutex> lock(m_lock);
	const int size = int(m_primitives.size());
	Primitive* primitive = nullptr;
	if(size < m_maxSize || m_maxSize < 1) {
		m_primitives.emplace_back();
		primitive = &m_primitives.back();
	}
	else {
		primitive = &m_primitives[m_tail];
	}
	if(m_maxSize > 0) {
		m_tail = (m_tail + 1) % m_maxSize;
	}
	primitive->m_type=TEXT;
	uint32_t color=0xc0c0c0ff;
	if(strstr(text,"ERROR")) {
		color=0xdc3545ff;
	}else
	if(strstr(text,"WARNING")) {
		color=0xffc107ff;
	}else
	if(strstr(text,"NOTIFY")) {
		color=0x007bffff;
	}
	primitive->m_color=color;
	primitive->m_text=text;
}

void GuiLog::InitLog() {
	m_primitives.reserve(m_maxSize);
}

void GuiLog::DrawLog() {
	const std::lock_guard<std::mutex> lock(m_lock);
	const int numPrimitives = (int)m_primitives.size();
	for(int i=0;i!=numPrimitives;i++) {
		const int index = (m_tail + i) % numPrimitives;
		const Primitive* primitive = &m_primitives[index];
		switch(primitive->m_type) {
			case TEXT: {
				int color=primitive->m_color;
				ImGui::TextColored(ImLoad(uint322V4(color)),"%s",primitive->m_text.c_str());
				if(primitive->m_text.rfind('\n')==std::string::npos)
					ImGui::SameLine(0,0);
				break;
			}
			default:
				FATAL("Viewer::DrawLog");
		}
	}
	if(ImGui::GetScrollY()==ImGui::GetScrollMaxY()) {
		ImGui::SetScrollHereY(1.0f);
	}
	static V2 popupPosition;
	if(ImGui::IsWindowHovered() && ImGui::IsMouseReleased(1)) {
		popupPosition=VLoad(ImGui::GetMousePos());
		ImGui::OpenPopup("LogPopup");
	}
	ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding,ImVec2(4,4));
	ImGui::SetNextWindowPos(ImLoad(popupPosition));
	if(ImGui::BeginPopup("LogPopup",ImGuiWindowFlags_NoTitleBar|ImGuiWindowFlags_NoResize|ImGuiWindowFlags_NoMove|ImGuiWindowFlags_NoScrollWithMouse|ImGuiWindowFlags_NoScrollbar)) {
		if(!ImGui::IsWindowHovered() && ImGui::IsMouseClicked(0)) {
			ImGui::CloseCurrentPopup();
		}
		if(ImGui::Button("Clear log")) {
			m_primitives.clear();
			m_tail = 0;
			ImGui::CloseCurrentPopup();
		}
		ImGui::EndPopup();
	}
	ImGui::PopStyleVar(1);
}

class Viewer {
	public:
		void End();
		void Begin();
		void Run();
		void DrawProfilerDisplay(const std::vector<DisplayTimer>& profilerDisplay);
		void Encode();
		void Decode();
		void SetFrame(int width,int height,const uint8_t* pixels);
		std::atomic<bool> m_close=false;
		std::atomic<bool> m_encode=false;
		std::atomic<bool> m_decode=false;
		Atlas m_atlas;
		std::mutex m_profilerDisplayLock;
		std::vector<DisplayTimer> m_profilerDisplay;


		GLuint m_textureId=0;

		uint64_t m_pixelsUpdateTime=0;
		uint64_t m_pixelsDrawTime=0;
		int m_pixelsWidth=0;
		int m_pixelsHeight=0;
		std::mutex m_pixelsLock;
		std::vector<uint8_t> m_pixelsRgb;
		GuiLog m_log;
};

static void glfw_error_callback(int error,const char* description){
	uprintf("Glfw Error %d: %s\n",error,description);
}

void LineBB(ImVec2& top_left,ImVec2& bottom_right){
	ImGuiWindow* window=ImGui::GetCurrentWindow();
	ImGuiContext& g=*GImGui;
	const ImVec2 size_arg(-1.0f,0);
	ImVec2 pos=window->DC.CursorPos;
	ImVec2 sz=ImGui::CalcItemSize(size_arg,ImGui::CalcItemWidth(),g.FontSize);
	ImRect bb(pos,ImVec2(pos.x+sz.x,pos.y+sz.y));
	top_left=bb.Min;
	bottom_right=bb.Max;
}
void LineMultiRect(float* positions,ImU32* colors,int count){
	ImGuiWindow* window=ImGui::GetCurrentWindow();
	if(window->SkipItems)
		return;
	ImGuiContext& g=*GImGui;
	const ImGuiStyle& style=g.Style;
	const ImVec2 size_arg(-1.0f,0);
	ImVec2 pos=window->DC.CursorPos;
	ImVec2 sz=ImGui::CalcItemSize(size_arg,ImGui::CalcItemWidth(),g.FontSize);
	ImRect bb(pos,ImVec2(pos.x+sz.x,pos.y+sz.y));
	ImGui::ItemSize(bb,style.FramePadding.y);
	if(!ImGui::ItemAdd(bb,0))
		return;
	ImGui::RenderFrame(bb.Min,bb.Max,ImGui::GetColorU32(ImGuiCol_FrameBg),false,0.0f);
	bb.Expand(ImVec2(-style.FrameBorderSize,-style.FrameBorderSize));
	for(int i=0;i!=count;i++){
		ImRect bb1=bb;
		bb1.Min.x=ImMin(bb.Min.x+positions[i*2+0],bb.Max.x);
		bb1.Max.x=ImMin(bb.Min.x+positions[i*2+1],bb.Max.x);
		ImGui::RenderFrame(bb1.Min,bb1.Max,colors[i],false,0.0f);
	}
}
void Viewer::Begin() {
	if(CreateFontAtlas(&m_atlas)) {
		uprintf("Atlas created %d,%d glyphs %d\n",m_atlas.m_width,m_atlas.m_height,(int)m_atlas.m_glyphs.size());
	}
	m_log.InitLog();
}
void Viewer::End() {
	DestroyFontAtlas(&m_atlas);
}

void Viewer::SetFrame(int width,int height,const uint8_t* pixels) {
	m_pixelsLock.lock();
	int byteSize=width*height*3;
	if(!m_pixelsRgb.size())
		m_pixelsRgb.resize(byteSize);
	
	if((int)m_pixelsRgb.size()!=byteSize)
		FATAL("WTF");
	m_pixelsUpdateTime=GetTimeEpochMicroseconds();
	m_pixelsWidth=width;
	m_pixelsHeight=height;
	memcpy(m_pixelsRgb.data(),pixels,byteSize);
	m_pixelsLock.unlock();
}

void Viewer::DrawProfilerDisplay(const std::vector<DisplayTimer>& profilerDisplay){
	for(int i=0;i!=(int)profilerDisplay.size();i++){
		ImGui::Text("%s","");
		ImGui::SameLine(20.0f+(float)profilerDisplay[i].m_depth*20.0f);
		ImGui::Text("%s",profilerDisplay[i].m_name.c_str());
		ImGui::SameLine(270);
		ImGui::Text("%d",profilerDisplay[i].m_count);
		ImGui::SameLine(290+40);
		ImGui::Text("%.2f",profilerDisplay[i].m_time);
		ImGui::SameLine(360+40);
		ImGui::SameLine(380+40);
		ImVec2 topLeft;
		ImVec2 bottomRight;
		LineBB(topLeft,bottomRight);
		float width=bottomRight.x-topLeft.x;
		int count=0;
		float positions[64*2];
		uint32_t colors[64];
		for(int j=0;j!=(int)profilerDisplay[i].m_bars.size();j++){
			positions[count*2+0]=profilerDisplay[i].m_bars[j].m_start*width;
			positions[count*2+1]=profilerDisplay[i].m_bars[j].m_end*width;
			if(positions[count*2+0]==positions[count*2+1]){
				positions[count*2+1]+=1;
			}
			colors[count++]=profilerDisplay[i].m_bars[j].m_color;
			if(count==countof(colors))
				break;
		}
		LineMultiRect(positions,colors,count);
	}
}

#define OUTPUT_YUV

void Viewer::Encode() {
	m_encode=true;

	ISVCEncoder *encoder_=CreateSVCEncoder();
	//std::string fileFormat="$(DATA)/frames/orange_720p/frame_%03d.png";
	//std::string fileFormat="$(DATA)/frames/dancers_720p/frame_%03d.png";
	std::string fileFormat="$(DATA)/frames/bunny_720p/frame_%03d.png";
	//std::string fileFormat="$(DATA)/frames/water_720p/frame_%03d.png";

	std::string fn=stdx::format_string(fileFormat.c_str(),(int)1).c_str();
	int width=0,height=0,channels=0;

	std::vector<char> texture;
	if(!LoadFile(&texture,fn.c_str())) {
		FATAL("Error unable load texture %s\n",fn.c_str());
	}else{
		uint8_t* imageRGB=DecodeImageFromMemory((uint8_t*)&texture[0],(int)texture.size(),&width,&height,&channels);
		delete [] imageRGB;
	}

	SEncParamBase param;
	memset(&param,0,sizeof(SEncParamBase));
	param.fMaxFrameRate=3;
	param.iPicWidth=width;
	param.iPicHeight=height;
	param.iTargetBitrate=4000000;
	//param.iTargetBitrate=1000000;
	encoder_->Initialize(param);

	std::string fileNameVideo=GetFileNameRemap("$(DATA)/frames/video.h264");
	std::remove(fileNameVideo.c_str());
#ifdef OUTPUT_YUV
	std::string fileNameVideoYUV=GetFileNameRemap("$(DATA)/frames/video.yuv");
	std::remove(fileNameVideoYUV.c_str());
#endif

	for(int num=0; num<1000; num++) {
		std::string fileName=GetFileNameRemap(stdx::format_string("$(DATA)/frames/frame_%08d.bin",num));
		std::remove(fileName.c_str());
	}

	uint8_t* chY=new uint8_t[width*height];
	uint8_t* chU=new uint8_t[(width>>1)*(height>>1)];
	uint8_t* chV=new uint8_t[(width>>1)*(height>>1)];
	std::vector<uint8_t> dataVideo;
	std::vector<uint8_t> dataVideoYUV;
	int ix=0;
	int rgbSize=0;
	for(int num=1; num<1000; num++) {
		if(m_close)
			break;
		std::string fn=stdx::format_string(fileFormat.c_str(),num+1).c_str();
		std::vector<char> texture;
		if(!LoadFile(&texture,fn.c_str())) {
			if(num==1) {
				FATAL("Error unable load texture %s\n");
			}else{
				break;
			}
		}
		int width1=0,height1=0,channels1=0;
		uint8_t* imageRGB=DecodeImageFromMemory((uint8_t*)&texture[0],(int)texture.size(),&width1,&height1,&channels1);
		if(width1!=width || height1!=height)
			FATAL("FRAME SIZE\n");
		rgbSize+=width1*height1*3;
		std::string text=stdx::format_string("%s %d",fn.c_str(),num);
		WriteTextRGB(V2(24,64),text.c_str(),24.0f,0x40101010,imageRGB,width,height,m_atlas);
		WriteTextRGB(V2(20,60),text.c_str(),24.0f,0xfff0f0f0,imageRGB,width,height,m_atlas);
		SetFrame(width,height,imageRGB);
		RGB2Yuv420Ch(chY,chU,chV,imageRGB,width,height);

#ifdef OUTPUT_YUV
		dataVideoYUV.insert(dataVideoYUV.end(),chY,chY+(width*height));
		dataVideoYUV.insert(dataVideoYUV.end(),chU,chU+((width>>1)*(height>>1)));
		dataVideoYUV.insert(dataVideoYUV.end(),chV,chV+((width>>1)*(height>>1)));
#endif//OUTPUT_YUV

		delete [] imageRGB;

		SSourcePicture pic;
		memset(&pic,0,sizeof(SSourcePicture));
		pic.iPicWidth=width;
		pic.iPicHeight=height;
		pic.iStride[0]=width;
		pic.iStride[1]=width>>1;
		pic.iStride[2]=width>>1;
		pic.pData[0]=chY;
		pic.pData[1]=chU;
		pic.pData[2]=chV;

		uint32_t byteSize=0;
		const uint8_t* outBuf1=encoder_->EncodeFrame(&byteSize,&pic);

		m_profilerDisplayLock.lock();
		encoder_->GetProfileDisplay(&m_profilerDisplay);
		m_profilerDisplayLock.unlock();

		std::string fileName=GetFileNameRemap(stdx::format_string("$(DATA)/frames/frame_%08d.bin",ix));
		ix++;
		std::ofstream outFrame;
		outFrame.open(fileName,std::ios::out|std::ios::binary);
		outFrame.write((char *)outBuf1,byteSize);
		dataVideo.insert(dataVideo.end(),outBuf1,outBuf1+byteSize);
		uprintf("frame %s byteSize %d\n",fileName.c_str(),byteSize);
		outFrame.close();
	}
	delete [] chY;
	delete [] chU;
	delete [] chV;
	std::ofstream outVideo;
	outVideo.open(fileNameVideo,std::ios::out|std::ios::binary);
	outVideo.write((char*)dataVideo.data(),dataVideo.size());
	outVideo.close();
	uprintf("NOTIFY: rgb=%d yuv=%d h264=%d ratio 1:%d\n",rgbSize,(int)dataVideoYUV.size(),(int)dataVideo.size(),rgbSize/(int)dataVideo.size());


#ifdef OUTPUT_YUV
	std::ofstream outVideoYUV;
	outVideoYUV.open(fileNameVideoYUV,std::ios::out|std::ios::binary);
	outVideoYUV.write((char*)dataVideoYUV.data(),dataVideoYUV.size());
	outVideoYUV.close();
#endif
	if (encoder_) {
		DestroySVCEncoder(encoder_);
	}
	m_encode=false;
}

#define YUV2B(Y,U,V) (int)(1.164f*(Y-16)+2.018f*(U-128))
#define YUV2G(Y,U,V) (int)(1.164f*(Y-16)-0.813f*(V-128)-0.391f*(U-128))
#define YUV2R(Y,U,V) (int)(1.164f*(Y-16)+1.596f*(V-128))

void ConvertYUV(uint8_t* rgb,const uint8_t* ybuf,const uint8_t* ubuf,const uint8_t* vbuf,int s0,int s1,int width,int height,int channels) {
	int p0=0,p1=0,pd=0;
	for(int y=0;y<(height>>1);y++) {
		for(int i=0;i<2;++i) {
			for(int x=0;x<(width>>1);x++) {
				int u=ubuf[p1+x];
				int v=vbuf[p1+x];
				int y0=ybuf[p0+(x<<1)];
				int y1=ybuf[p0+(x<<1)+1];
				int b0=YUV2B(y0,u,v);
				int g0=YUV2G(y0,u,v);
				int r0=YUV2R(y0,u,v);
				int b1=YUV2B(y1,u,v);
				int g1=YUV2G(y1,u,v);
				int r1=YUV2R(y1,u,v);
				if(r0<0) r0=0;
				if(g0<0) g0=0;
				if(b0<0) b0=0;
				if(r0>255) r0=255;
				if(g0>255) g0=255;
				if(b0>255) b0=255;
				if(r1<0) r1=0;
				if(g1<0) g1=0;
				if(b1<0) b1=0;
				if(r1>255) r1=255;
				if(g1>255) g1=255;
				if(b1>255) b1=255;
				rgb[pd+(x*6)+0]=r0;
				rgb[pd+(x*6)+1]=g0;
				rgb[pd+(x*6)+2]=b0;
				rgb[pd+(x*6)+3]=r1;
				rgb[pd+(x*6)+4]=g1;
				rgb[pd+(x*6)+5]=b1;
			}
			p0+=s0;
			pd+=width*3;
		}
		p1+=s1;
	}
}

void DestroyDecoderOpenH264(NewDec::ISVCDecoderBase* pSvcDecoder) {
	if(pSvcDecoder)
		delete pSvcDecoder;
	pSvcDecoder=0;
}
bool DecodeFrameOpenH264Tensor(NewDec::ISVCDecoderBase* pSvcDecoder,uint8_t* decoded,int* width,int* height,int* channels,const uint8_t* packetH264,int bytesizeH264,int* timerDecode,int* timerYuv2Rgb) {
	uint8_t* dst[3];
	if(!pSvcDecoder->DecodeFrame(packetH264,bytesizeH264,dst))
		FATAL("Decodeframe returned false");
	*width=pSvcDecoder->Width();
	*height=pSvcDecoder->Height();
	int s0=pSvcDecoder->StrideY();
	int s1=pSvcDecoder->StrideUV();
	uint8_t* rgb=(uint8_t*)decoded;
	*channels=3;
	ConvertYUV(rgb,dst[0],dst[1],dst[2],s0,s1,*width,*height,*channels);
	return true;
}

void Viewer::Decode() {
	m_decode=true;
	NewDec::ISVCDecoderBase* pSvcDecoder=0;
	CreateDecoder(&pSvcDecoder);
	NewDec::SDecodingParam sDecParam={0};
	sDecParam.uiTargetDqLayer=0xff;
	sDecParam.sVideoProperty.eVideoBsType=NewDec::VIDEO_BITSTREAM_DEFAULT;
	sDecParam.uiCpuLoad=100;
	pSvcDecoder->Initialize(&sDecParam);
	uint8_t* decoded=new uint8_t[1280*720*3];
	for(int ix=0;ix!=1000;ix++) {
		if(m_close)
			break;
		std::vector<char> data;
		std::string fileName=stdx::format_string("$(DATA)/frames/frame_%08d.bin",ix);
		if(!LoadFile(&data,fileName.c_str()))
			break;
		//uprintf("decode frame %s\n",fileName.c_str());
		memset(decoded,0,1280*720*3);
		int width,height,channels;
		int timers[2]={0,0};		//0=decodeh264 1=convertYUV
 		if(DecodeFrameOpenH264Tensor(pSvcDecoder,decoded,&width,&height,&channels,(uint8_t*)data.data(),(int)data.size(),&timers[0],&timers[1])) {
			//uprintf("frame decoded %d,%d,%d\n",width,height,channels);
			SetFrame(width,height,decoded);
			if(ix==50)
				uprintf("CRC32 frame %d 0x%08x\n",ix,CRC32(decoded,width*height*3));
		}else{
			FATAL("NOFRAME?");
		}
		m_profilerDisplayLock.lock();
		pSvcDecoder->GetProfileDisplay(&m_profilerDisplay);
		m_profilerDisplayLock.unlock();
	}
	delete [] decoded;
	DestroyDecoderOpenH264(pSvcDecoder);
	m_decode=false;
}

void Viewer::Run(){
	glfwSetErrorCallback(glfw_error_callback);
	if(!glfwInit()){
		uprintf("glfwInit failed\n");
		return;
	}

#ifdef __APPLE__
	const char* glsl_version="#version 330";
	glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR,3);
	glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR,3);
	glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT,GL_TRUE);
	glfwWindowHint(GLFW_OPENGL_PROFILE,GLFW_OPENGL_CORE_PROFILE);
#else
	glfwWindowHint(GLFW_CLIENT_API,GLFW_OPENGL_ES_API);
	const char* glsl_version="#version 300 es";//#version 130";
	glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR,3);
	glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR,0);
#endif

	GLFWwindow* window=glfwCreateWindow(1600,1200,"Plain H264",NULL,NULL);
	if(!window){
		uprintf("Unable to create window\n");
		return;
	}
	glfwMakeContextCurrent(window);
	glfwSwapInterval(1); // Enable vsync

	bool err=glewInit()!=GLEW_OK;
	if(err){
		FATAL("Failed to initialize OpenGL loader!");
		return;
	}
	uprintf("glsl_version %s\n",glsl_version);

	// Setup Dear ImGui context
	IMGUI_CHECKVERSION();
	ImGui::CreateContext();
	ImGuiIO& io=ImGui::GetIO(); (void)io;
	io.ConfigFlags|=ImGuiConfigFlags_NavEnableKeyboard;       // Enable Keyboard Controls
	io.ConfigFlags|=ImGuiConfigFlags_DockingEnable;           // Enable Docking
	io.ConfigFlags|=ImGuiConfigFlags_ViewportsEnable;         // Enable Multi-Viewport / Platform Windows

	ImGui::StyleColorsDark();

	ImGuiStyle& style=ImGui::GetStyle();
	if(io.ConfigFlags&ImGuiConfigFlags_ViewportsEnable){
		style.WindowRounding=0.0f;
		style.Colors[ImGuiCol_WindowBg].w=1.0f;
	}

	ImGui_ImplGlfw_InitForOpenGL(window,true);
	ImGui_ImplOpenGL3_Init(glsl_version);

	io.Fonts->AddFontFromFileTTF(GetFileNameRemap("$(DATA)/fonts/inconsolata/InconsolataGo-Regular.ttf").c_str(),20.0f);
	ImFont* fontRoboto=io.Fonts->AddFontFromFileTTF(GetFileNameRemap("$(DATA)/fonts/roboto/Roboto-Medium.ttf").c_str(),60.0f);

	std::mutex profilerDisplayLock;
	std::vector<DisplayTimer> profilerDisplay;

	int tick=0;
	while(!glfwWindowShouldClose(window)){
		if(glfwGetKey(window,GLFW_KEY_ESCAPE)==GLFW_PRESS)
			glfwSetWindowShouldClose(window,true);
		glfwPollEvents();
		auto time=std::chrono::system_clock::now();

		Profiler profiler;
		START_TIMER(enctimer,&profiler,"main",0xffff20);

		ImGui_ImplOpenGL3_NewFrame();
		ImGui_ImplGlfw_NewFrame();
		ImGui::NewFrame();

		ImGuiID dockspace_id=ImGui::GetID("EditDockSpace");
		if(!tick||ImGui::DockBuilderGetNode(dockspace_id)==NULL){
			ImGui::DockBuilderRemoveNode(dockspace_id);
			ImVec2 dockspace_size=ImGui::GetMainViewport()->Size;
			ImGui::DockBuilderAddNode(dockspace_id,ImGuiDockNodeFlags_DockSpace);
			ImGui::DockBuilderSetNodeSize(dockspace_id,dockspace_size);
			ImGuiID dock_video_id=dockspace_id;
			ImGuiID dock_prop_id=ImGui::DockBuilderSplitNode(dock_video_id,ImGuiDir_Left,0.50f,NULL,&dock_video_id);
			ImGuiID dock_status_id=ImGui::DockBuilderSplitNode(dock_video_id,ImGuiDir_Down,0.35f,NULL,&dock_video_id);
			ImGui::DockBuilderDockWindow("Frame",dock_prop_id);
			ImGui::DockBuilderDockWindow("Profiler",dock_video_id);
			ImGui::DockBuilderDockWindow("Log",dock_status_id);
			ImGui::DockBuilderFinish(dockspace_id);
		}
		ImGuiViewport* viewport=ImGui::GetMainViewport();
		const ImGuiWindowClass* window_class=0;
		ImVec2 p=viewport->Pos;
		ImVec2 s=viewport->Size;

		ImGui::SetNextWindowPos(p);
		ImGui::SetNextWindowSize(s);
		ImGui::SetNextWindowViewport(viewport->ID);
		ImGui::SetNextWindowBgAlpha(0);

		ImGuiDockNodeFlags dockspace_flags=0;
		ImGuiWindowFlags host_window_flags=0;
		host_window_flags|=ImGuiWindowFlags_NoTitleBar|ImGuiWindowFlags_NoCollapse|ImGuiWindowFlags_NoResize|ImGuiWindowFlags_NoMove|ImGuiWindowFlags_NoDocking|ImGuiWindowFlags_NoBackground;
		host_window_flags|=ImGuiWindowFlags_NoBringToFrontOnFocus|ImGuiWindowFlags_NoNavFocus|ImGuiWindowFlags_MenuBar;

		ImGui::PushStyleVar(ImGuiStyleVar_WindowRounding,0.0f);
		ImGui::PushStyleVar(ImGuiStyleVar_WindowBorderSize,0.0f);
		ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding,ImVec2(0.0f,0.0f));
		ImGui::Begin("MainWindow",NULL,host_window_flags);
		ImGui::DockSpace(dockspace_id,ImVec2(0.0f,0.0f),dockspace_flags,window_class);
		ImGui::PopStyleVar(3);

		ImGui::Begin("Frame");
		ImGui::PushFont(fontRoboto);
		if(m_encode)
			ImGui::Text("Encode");
		if(m_decode)
			ImGui::Text("Decode");
		ImGui::PopFont();
		if(m_pixelsWidth && m_pixelsHeight) {
			if(m_pixelsUpdateTime>m_pixelsDrawTime) {
				m_pixelsDrawTime=GetTimeEpochMicroseconds();
				int width=m_pixelsWidth;
				int height=m_pixelsHeight;
				m_pixelsLock.lock();
				if(!m_textureId) {
					glGenTextures(1,&m_textureId);
					glBindTexture(GL_TEXTURE_2D,m_textureId);
					glTexParameteri(GL_TEXTURE_2D,GL_TEXTURE_MIN_FILTER,GL_LINEAR);
					glTexParameteri(GL_TEXTURE_2D,GL_TEXTURE_MAG_FILTER,GL_LINEAR);
					glTexParameteri(GL_TEXTURE_2D,GL_TEXTURE_WRAP_S,GL_CLAMP); 
					glTexParameteri(GL_TEXTURE_2D,GL_TEXTURE_WRAP_T,GL_CLAMP); 
					glPixelStorei(GL_UNPACK_ALIGNMENT,1);
					glPixelStorei(GL_UNPACK_ROW_LENGTH,0);
					glTexImage2D(GL_TEXTURE_2D,0,GL_RGB,width,height,0,GL_RGB,GL_UNSIGNED_BYTE,m_pixelsRgb.data());
				}else{
					glBindTexture(GL_TEXTURE_2D,m_textureId);
					glTexImage2D(GL_TEXTURE_2D,0,GL_RGB,width,height,0,GL_RGB,GL_UNSIGNED_BYTE,m_pixelsRgb.data());
				}
				m_pixelsLock.unlock();
			}
			ImGui::Image((ImTextureID)(uint64_t)m_textureId,ImVec2((float)m_pixelsWidth,(float)m_pixelsHeight));
		}
		ImGui::End();
		ImGui::Begin("Profiler");
		ImGui::Text("Render thread");
		DrawProfilerDisplay(profilerDisplay);

		ImGui::Text("Encode thread");
		m_profilerDisplayLock.lock();
		std::vector<DisplayTimer> profilerEncodeDisplay=m_profilerDisplay;
		m_profilerDisplayLock.unlock();
		DrawProfilerDisplay(profilerEncodeDisplay);

		ImGui::End();
		ImGui::Begin("Log");
		m_log.DrawLog();
		ImGui::End();

		ImGui::End();

		ImGui::Render();

		int windowWidth,windowHeight;
		glfwGetWindowSize(window,&windowWidth,&windowHeight);

		glViewport(0,0,windowWidth,windowHeight);
		glClearColor(.1,.1,.1,1);
		glClear(GL_COLOR_BUFFER_BIT);

		END_TIMER(enctimer,&profiler);
		profilerDisplay.clear();
		profilerDisplayLock.lock();
		profiler.GetDisplayTimers(profilerDisplay);
		profilerDisplayLock.unlock();

		ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());

		if(io.ConfigFlags&ImGuiConfigFlags_ViewportsEnable){
			GLFWwindow* backup_current_context=glfwGetCurrentContext();
			ImGui::UpdatePlatformWindows();
			ImGui::RenderPlatformWindowsDefault();
			glfwMakeContextCurrent(backup_current_context);
		}

		std::this_thread::sleep_until(time+std::chrono::microseconds(1000000/60));		//limit frame rate
		glfwSwapBuffers(window);

		tick++;
	}
	if(m_textureId)
		glDeleteTextures(1,&m_textureId);
	m_textureId=0;

	ImGui_ImplOpenGL3_Shutdown();
	ImGui_ImplGlfw_Shutdown();
	ImGui::DestroyContext();

	glfwDestroyWindow(window);
	glfwTerminate();
}

Viewer* g_viewer=0;

void PrintCallback(const char* str) {
#ifdef _WIN32
	OutputDebugString(str);
#else
	::printf("%s",str);
#endif
	if(g_viewer)
		g_viewer->m_log.AddText(str);
}

int main(){
	SetPrintCallback(PrintCallback);
#ifdef CMAKE_SOURCE_DIR
	AddFilePathRemap("$(DATA)",std::string(CMAKE_SOURCE_DIR)+"/data");
#else
	AddFilePathRemap("$(DATA)",GetExecutablePath()+"/data");
#endif
	Viewer d;
	d.Begin();
	g_viewer=&d;
	std::thread tr=std::thread([&](){
		d.Encode();
		d.Decode();
	});
	d.Run();
	d.m_close=true;
	tr.join();
	d.End();
	g_viewer=0;
	return 0;
}

#ifdef _WIN32
#undef APIENTRY
#include<windows.h>
#include"debugapi.h"
#include<crtdbg.h>
int WINAPI WinMain(HINSTANCE hInstance,HINSTANCE hPrevInstance,PSTR lpCmdLine,INT nCmdShow){
	_CrtSetDbgFlag(_CRTDBG_ALLOC_MEM_DF|_CRTDBG_LEAK_CHECK_DF);
	//_CrtSetBreakAlloc(167);
	return main();
}
#endif
