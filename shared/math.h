#pragma once

#include <float.h>
#include <math.h>
#include <memory.h>
#include <limits.h>
#include <stdint.h>

#define PI 3.14159265358979323846f
#define HALFPI (3.14159265358979323846f/2.0f)
#define PI2 (3.14159265358979323846f*2.0f)

#define countof(Array) ((int)(sizeof(Array)/sizeof(*Array)))

#ifdef __cplusplus

#ifdef _MSC_VER
#define ALIGN( x ) __declspec( align( x ) )
#define MALLOC64( x ) ( ( x ) == 0 ? 0 : _aligned_malloc( ( x ), 64 ) )
#define FREE64( x ) _aligned_free( x )
#else
#define ALIGN( x ) __attribute__( ( aligned( x ) ) )
#define MALLOC64( x ) ( ( x ) == 0 ? 0 : aligned_alloc( 64, ( x ) ) )
#define FREE64( x ) free( x )
#endif

void SeedRandom(uint32_t seed);
float RandomUnitFloat();
uint32_t mRandomU32();

static constexpr float FEPSILON=(1.0f/4096.0f);
static constexpr float HEPSILON=(1.0f/256.0f);

#define FHUGE 1e38f
#ifndef MIN
#define MIN(x, y) (((x) < (y))? (x) : (y))
#endif

#ifndef MAX
#define MAX(x, y) (((x) > (y))? (x) : (y))
#endif

#ifndef BOUND
#define BOUND(min,x,max) ((x)<(min)?(min):(x)>(max)?(max):(x))
#endif

#ifndef CLAMP
#define CLAMP(min,x,max) ((x)<(min)?(min):(x)>(max)?(max):(x))
#endif

#ifndef SATURATE
#define SATURATE(x) ((x)<0?0:(x)>1?1:(x))
#endif

#define SATURATE_ADD_UCHAR(a, b) (((b)>UCHAR_MAX-(a)) ? UCHAR_MAX : ((a)+(b)))
#define SATURATE_ADD_UINT(x, y) (((y)>UINT_MAX-(x)) ? UINT_MAX : ((x)+(y)))
#define SATURATE_ADD_USHORT(x, y) (((y)>SHRT_MAX-(x)) ? USHRT_MAX : ((x)+(y)))

#define FPI 3.1415926535897f
#define DEGREES_TO_RADIANS(__ANGLE__) ( (__ANGLE__)/180.0f*3.14159265358979323846f )
#define RADIANS_TO_DEGREES(__ANGLE__) ( (__ANGLE__)*(180.0f/3.14159265358979323846f) )

template <typename T> inline T round_up(T k,T alignment){
	return (k+alignment-1)&~(alignment-1);
}

template<typename _T> inline _T Clamp(const _T& Value,const _T& Min,const _T& Max){
	if(Value<Min){
		return Min;
	} else if(Value>Max){
		return Max;
	} else{
		return Value;
	}
}

inline int Abs(int a){
	if(a>=0)return a;
	return -a;
}

inline float FMax(float x,float y){
	if(x>y)return x;
	return y;
}
inline float FClamp(float fValue,float fMin,float fMax){
	return Clamp<float>(fValue,fMin,fMax);
}

inline float sign(float v){
	return v>=0.0f ? 1.0f : -1.0f;
}

inline bool RoughlyEqual(float a,float b,float fLimit=0.001f){
	return fabsf(a-b)<fLimit;
}

inline int Min(int a,int b){
	return a<b ? a : b;
}
inline int Max(int a,int b){
	return a>b ? a : b;
}

inline float Min(float a,float b){
	return a<b ? a : b;
}
inline float Max(float a,float b){
	return a>b ? a : b;
}

inline float VLerp(float a,float b,float s){
	return a*(1-s)+b*s;
}

inline float GetAngle(const float x,const float y){
	float v;
	if(x==0){
		if(0<y){
			v=DEGREES_TO_RADIANS(90.0f);
		} else{
			v=DEGREES_TO_RADIANS(270.0f);
		}
	} else{
		v=(float)atan(y/x);
		if(x<0){
			v+=DEGREES_TO_RADIANS(180.0f);
		} else
			if(y<0){
				v+=DEGREES_TO_RADIANS(360.0f);
			}
	}
	return v;
}

#define COL32A(r,g,b,a) ( ((int)a<<24)|(BOUND(0,r,255)<<16) | (BOUND(0,g,255)<<8) | (BOUND(0,b,255)) )

template <class T> inline bool ArrayEquals(const T* a,const T* b,int size){
	for(int i=0; i<size; i++){
		if(a[i]!=b[i]){
			return false;
		}
	}
	return true;
}

struct VI2{
	int x,y;
	VI2(){
		x=y=0;
	}
	VI2(int _x,int _y){
		x=_x;y=_y;
	}
};
struct VI4{
	int x,y,z,w;
	VI4(){
		x=y=z=w=0;
	}
	VI4(int _x,int _y,int _z,int _w){
		x=_x;y=_y;z=_z;w=_w;
	}
};

class V2I{
	public:
		inline V2I(){};
		inline V2I(int in_x,int in_y){
			x=in_x;y=in_y;
		}
		bool operator==(const V2I& v) const{
			return(x==v.x&&y==v.y);
		}
		bool operator!=(const V2I& v) const{
			return(x!=v.x||y!=v.y);
		}
		int x;
		int y;
};

class V2ISize{
	public:
		inline V2ISize(){};
		inline V2ISize(int in_width,int in_height){
			width=in_width;height=in_height;
		}
		inline V2ISize(const V2I& v){
			width=v.x;height=v.y;
		}
		bool operator==(const V2ISize& v) const{
			return(width==v.width&&height==v.height);
		}
		bool operator!=(const V2ISize& v) const{
			return(width!=v.width||height!=v.height);
		}
		int width;
		int height;
};

class V3;

class SV2{
	public:
		float x;
		float y;
};

class V2: public SV2{
	public:
		inline V2(){};
		inline explicit V2(const float* v){
			x=v[0];y=v[1];
		}
		inline V2(float in_x,float in_y){
			x=in_x;y=in_y;
		}
		inline V2(float in){
			x=in;y=in;
		}
		inline V2& operator=(const SV2& v){
			x=v.x;y=v.y;return *this;
		}
		inline V2(const SV2& v){
			x=v.x;y=v.y;
		}
		inline V2& operator=(const class V4& v);
		V2& operator=(const V2& v){
			x=v.x; y=v.y; return *this;
		}
		V2& operator +=(const V2& v){
			(*this)=(*this)+v; return *this;
		}
		V2& operator -=(const V2& v){
			(*this)=(*this)-v; return *this;
		}
		V2& operator *=(const V2& v){
			(*this)=(*this)*v; return *this;
		}
		V2& operator /=(const V2& v){
			(*this)=(*this)/v; return *this;
		}
		V2& operator *=(float s){
			(*this)=(*this)*s; return *this;
		}
		V2& operator /=(float s){
			(*this)=(*this)/s; return *this;
		}
		V2& operator +=(float s){
			(*this)=(*this)+s; return *this;
		}
		V2& operator -=(float s){
			(*this)=(*this)-s; return *this;
		}
		V2 operator+() const{
			return *this;
		}
		V2 operator-() const{
			return V2(-x,-y);
		}
		V2 operator+(const V2& v) const{
			return V2(x+v.x,y+v.y);
		}
		V2 operator-(const V2& v) const{
			return V2(x-v.x,y-v.y);
		}
		V2 operator*(const V2& v) const{
			return V2(x*v.x,y*v.y);
		}
		V2 operator/(const V2& v) const{
			return V2(x/v.x,y/v.y);
		}
		V2 operator*(float s) const{
			return V2(x*s,y*s);
		}
		V2 operator/(float s) const{
			return V2(x/s,y/s);
		}
		V2 operator + (float s) const{
			return V2(x+s,y+s);
		}
		V2 operator - (float s) const{
			return V2(x-s,y-s);
		}
		bool operator==(const V2& v) const{
			return(x==v.x&&y==v.y);
		}
		bool operator!=(const V2& v) const{
			return(x!=v.x||y!=v.y);
		}
		operator float* (){
			return &x;
		}
		operator const float* () const{
			return &x;
		}
		V2 Floor(){
			return V2(floorf(x),floorf(y));
		}
		V3 VX0Y()const;
		V2I ConvertToV2I()const{
			return V2I(int(x),int(y));
		}
};

static inline V2 operator-(const float a,const V2& b){
	return V2(a,a)-b;
}
static inline V2 operator/(const float a,const V2& b){
	return V2(a,a)/b;
}

inline V2 operator*(float s,const V2& v){
	return v*s;
}

inline V2 VMul(const V2& v0,const V2& v1){
	return v0*v1;
}
inline V2 VMul(const V2& v0,float fValue){
	return v0*V2(fValue,fValue);
}

inline V2 VMax(const V2& v0,const V2& v1){
	V2 vres;
	if(v0.x>v1.x){
		vres.x=v0.x;
	} else{
		vres.x=v1.x;
	}
	if(v0.y>v1.y){
		vres.y=v0.y;
	} else{
		vres.y=v1.y;
	}
	return vres;
}

inline V2 VMin(const V2& v0,const V2& v1){
	V2 vres;
	if(v0.x<v1.x){
		vres.x=v0.x;
	} else{
		vres.x=v1.x;
	}
	if(v0.y<v1.y){
		vres.y=v0.y;
	} else{
		vres.y=v1.y;
	}
	return vres;
}

inline float VDot(const V2& a,const V2& b){
	return(a.x*b.x+a.y*b.y);
}

inline bool RoughlyEqual(const V2& a,const V2& b,const float fLimit=0.001f){
	return (fabsf(a.x-b.x)<fLimit&&fabsf(a.y-b.y)<fLimit);
}

inline V2 RadiansToDegrees(const V2& v){
	return V2(RADIANS_TO_DEGREES(v.x),RADIANS_TO_DEGREES(v.y));
}

inline V2 DegreesToRadians(const V2& v){
	return V2(DEGREES_TO_RADIANS(v.x),DEGREES_TO_RADIANS(v.y));
}

inline V2 VAbs(const V2& v){
	return V2(fabsf(v.x),fabsf(v.y));
}

inline float VLength(const V2& v){
	return sqrtf(VDot(v,v));
}

inline float VLengthSquared(const V2& v){
	return VDot(v,v);
}

inline V2 VNeg(const V2& v){
	return V2(-v.x,-v.y);
}

inline V2 VNormalize(const V2& a){
	float len=VLength(a);
	return len>0.0f ? a/len : a;
}

inline float VCross(const V2& a,const	V2& b){
	return a.x*b.y-a.y*b.x;
}

inline V2 Min(const V2& a,const V2& b){
	return V2(Min(a.x,b.x),Min(a.y,b.y));
}

inline V2 Max(const V2& a,const V2& b){
	return V2(Max(a.x,b.x),Max(a.y,b.y));
}

inline V2 VLerp(const V2& a,const V2& b,float s){
	return a*(1-s)+b*s;
}

inline V2 Rotate(const V2& v,float a){
	float c=cosf(a);
	float s=sinf(a);
	return V2(c*v.x+s*v.y,-s*v.x+c*v.y);
}

inline float lerp(float a,float b,float t){
	return a+(b-a)*t;
}
inline V2 lerp(const V2& a,const V2& b,float t){
	return a+(b-a)*V2(t,t);
}

class SV3{
	public:
		float x;
		float y;
		float z;
};

class V3: public SV3{
	public:
		inline V3(){};
		inline explicit V3(const float* v){
			x=v[0];y=v[1];z=v[2];
		}
		inline V3(float in_x,float in_y,float in_z){
			x=in_x;y=in_y;z=in_z;
		}
		inline V3(const V2& v,float in_z){
			x=v.x;y=v.y;z=in_z;
		}
		inline V3& operator=(const SV3& v){
			x=v.x;y=v.y;z=v.z;return *this;
		}
		inline V3(const SV3& v){
			x=v.x;y=v.y;z=v.z;
		}
		inline V3& operator=(const class V4& v);
		V3& operator=(const V3& v){
			x=v.x; y=v.y; z=v.z; return *this;
		}
		V3& operator+=(const V3& v){
			(*this)=(*this)+v; return *this;
		}
		V3& operator-=(const V3& v){
			(*this)=(*this)-v; return *this;
		}
		V3& operator*=(const V3& v){
			(*this)=(*this)*v; return *this;
		}
		V3& operator/=(const V3& v){
			(*this)=(*this)/v; return *this;
		}
		V3& operator *=(float s){
			(*this)=(*this)*s; return *this;
		}
		V3& operator/=(float s){
			(*this)=(*this)/s; return *this;
		}
		V3 operator+() const{
			return *this;
		}
		V3 operator-() const{
			return V3(-x,-y,-z);
		}
		V3 operator+(const V3& v) const{
			return V3(x+v.x,y+v.y,z+v.z);
		}
		V3 operator-(const V3& v) const{
			return V3(x-v.x,y-v.y,z-v.z);
		}
		V3 operator*(const V3& v) const{
			return V3(x*v.x,y*v.y,z*v.z);
		}
		V3 operator/(const V3& v) const{
			return V3(x/v.x,y/v.y,z/v.z);
		}
		V3 operator*(float s) const{
			return V3(x*s,y*s,z*s);
		}
		V3 operator/(float s) const{
			return V3(x/s,y/s,z/s);
		}
		bool operator==(const V3& v) const{
			return(x==v.x&&y==v.y&&z==v.z);
		}
		bool operator!=(const V3& v) const{
			return(x!=v.x||y!=v.y||z!=v.z);
		}
		operator float*(){
			return &x;
		}
		operator const float*() const{
			return &x;
		}
		V2 VXY()const{
			return V2(x,y);
		}
		V2 VXZ()const{
			return V2(x,z);
		}
		V3 VX0Z()const{
			return V3(x,0,z);
		}
		V3 V0YZ()const{
			return V3(0,y,z);
		}
		bool IsZero() const{
			return (x==0.f&&y==0.f&&z==0.f);
		}
};

inline V3 V2::VX0Y() const{
	return V3(x,0,y);
}

inline V3 operator*(float s,const V3& v){
	return v*s;
}

inline V3 VMul(const V3& v0,const V3& v1){
	return v0*v1;
}

inline V3 VMul(const V3& v0,float fValue){
	return v0*V3(fValue,fValue,fValue);
}

inline V3 VSplat(float fValue){
	return V3(fValue,fValue,fValue);
}

inline V3 VMax(const V3& v0,const V3& v1){
	V3 vres;
	if(v0.x>v1.x){
		vres.x=v0.x;
	} else{
		vres.x=v1.x;
	}
	if(v0.y>v1.y){
		vres.y=v0.y;
	} else{
		vres.y=v1.y;
	}
	if(v0.z>v1.z){
		vres.z=v0.z;
	} else{
		vres.z=v1.z;
	}
	return vres;
}

inline V3 VMin(const V3& v0,const V3& v1){
	V3 vres;
	if(v0.x<v1.x){
		vres.x=v0.x;
	} else{
		vres.x=v1.x;
	}
	if(v0.y<v1.y){
		vres.y=v0.y;
	} else{
		vres.y=v1.y;
	}
	if(v0.z<v1.z){
		vres.z=v0.z;
	} else{
		vres.z=v1.z;
	}
	return vres;
}

inline V3 VAbs(const V3& v){
	return V3(fabsf(v.x),fabsf(v.y),fabsf(v.z));
}

inline float VDot(const V3& a,const V3& b){
	return(a.x*b.x+a.y*b.y+a.z*b.z);
}

inline V3 MulComponents(const V3& a,const V3& b){
	return a*b;
}

inline bool RoughlyEqual(const V3& a,const V3& b,const float fLimit=0.001f){
	return (fabsf(a.x-b.x)<fLimit&&fabsf(a.y-b.y)<fLimit&&fabsf(a.z-b.z)<fLimit);
}

inline V3 RadiansToDegrees(const V3& v){
	return V3(RADIANS_TO_DEGREES(v.x),RADIANS_TO_DEGREES(v.y),RADIANS_TO_DEGREES(v.z));
}

inline V3 DegreesToRadians(const V3& v){
	return V3(DEGREES_TO_RADIANS(v.x),DEGREES_TO_RADIANS(v.y),DEGREES_TO_RADIANS(v.z));
}

inline float VLength(const V3& v){
	return sqrtf(VDot(v,v));
}

inline float VLengthXZ(const V3& v){
	return VLength(V2(v.x,v.z));
}

inline float VLengthSquared(const V3& v){
	return VDot(v,v);
}

inline float VLengthSquaredXZ(const V3& v){
	return VLengthSquared(V2(v.x,v.z));
}

inline V3 VNeg(const V3& v){
	return V3(-v.x,-v.y,-v.z);
}

inline V3 VNormalize(const V3& a){
	float len=VLength(a);
	return len>0.0f ? a/len : a;
}

inline V3 VNormalizeXZ(const V3& a){
	return VNormalize(V3(a.x,0,a.z));
}

inline V3 VCross(const V3& a,const V3& b){
	V3 c;
	c.x=a.y*b.z-a.z*b.y;
	c.y=a.z*b.x-a.x*b.z;
	c.z=a.x*b.y-a.y*b.x;
	return c;
}

inline V3 Min(const V3& a,const V3& b){
	return V3(Min(a.x,b.x),Min(a.y,b.y),Min(a.z,b.z));
}

inline V3 Max(const V3& a,const V3& b){
	return V3(Max(a.x,b.x),Max(a.y,b.y),Max(a.z,b.z));
}

inline float MinXYZ(const V3& v){
	return Min(Min(v.x,v.y),v.z);
}

inline float MaxXYZ(const V3& v){
	return Max(Max(v.x,v.y),v.z);
}

inline V3 VLerp(const V3& a,const V3& b,float s){
	return a*(1-s)+b*s;
}

inline void UpdateBounds(const V3& p,V3& min,V3& max){
	if(p.x<min.x) min.x=p.x;
	if(p.x>max.x) max.x=p.x;
	if(p.y<min.y) min.y=p.y;
	if(p.y>max.y) max.y=p.y;
	if(p.z<min.z) min.z=p.z;
	if(p.z>max.z) max.z=p.z;
}

//  | x cos a-y sin a|   |x'|
//  | x sin a+y cos a|=|y'|
//  |         z        |   |z'|
inline V3 RotateZ(const V3& v,float a){
	float c=cosf(a);
	float s=sinf(a);
	return V3(c*v.x+s*v.y,c*v.y-s*v.x,v.z);
}

//  | x cos a+z sin a|   |x'|
//  |         y        |=|y'|
//  |-x sin a+z cos a|   |z'|
inline V3 RotateY(const V3& v,float a){
	float c=cosf(a);
	float s=sinf(a);
	return V3(c*v.x+s*v.z,v.y,c*v.z-s*v.x);
}
//  |         x        |   |x'|
//  | y cos a-z sin a|=|y'|
//  | y sin a+z cos a|   |z'|

inline V3 RotateX(const V3& v,float a){
	float c=cosf(a);
	float s=sinf(a);
	return V3(v.x,c*v.y-s*v.z,s*v.y+c*v.z);
}

inline V2 VectorToEulerPY(const V3& v){
	V3 n=VNormalize(v);
	float yaw=GetAngle(n.x,-n.z);
	float pitch=acosf(n.y)-DEGREES_TO_RADIANS(90.0f);
	return V2(pitch,yaw);
}

inline V3 EulerPYToVector(const V2& e){
	V2 v=Rotate(V2(1,0),e.x);
	return RotateY(V3(v.x,v.y,0),e.y);
}

class SV4{
	public:
		float x;
		float y;
		float z;
		float w;
};

class V4: public SV4{
	public:
		inline V4(){};
		inline explicit V4(const float* v){
			x=v[0];y=v[1];z=v[2];w=v[3];
		}
		inline V4(float in_x,float in_y,float in_z,float in_w){
			x=in_x;y=in_y;z=in_z;w=in_w;
		}
		inline V4(const V3& v,float in_w){
			x=v.x;y=v.y;z=v.z;w=in_w;
		}
		inline V4(float xyzw){
			x=xyzw;y=xyzw;z=xyzw;w=xyzw;
		}
		inline V4& operator=(const SV4& v){
			x=v.x;y=v.y;z=v.z;w=v.w;return *this;
		}
		inline V4(const SV4& v){
			x=v.x;y=v.y;z=v.z;w=v.w;
		}
		inline V3 XYZ()const{
			return V3(x,y,z);
		}
		V4& operator=(const V4& v){
			x=v.x; y=v.y; z=v.z; w=v.w; return *this;
		}
		V4& operator +=(const V4& v){
			(*this)=(*this)+v; return *this;
		}
		V4& operator -=(const V4& v){
			(*this)=(*this)-v; return *this;
		}
		V4& operator *=(const V4& v){
			(*this)=(*this)*v; return *this;
		}
		V4& operator /=(const V4& v){
			(*this)=(*this)/v; return *this;
		}
		V4& operator *=(float s){
			(*this)=(*this)*s; return *this;
		}
		V4& operator /=(float s){
			(*this)=(*this)/s; return *this;
		}
		V4 operator+() const{
			return *this;
		}
		V4 operator-() const{
			return V4(-x,-y,-z,-w);
		}
		V4 operator+(const V4& v) const{
			return V4(x+v.x,y+v.y,z+v.z,w+v.w);
		}
		V4 operator-(const V4& v) const{
			return V4(x-v.x,y-v.y,z-v.z,w-v.w);
		}
		V4 operator*(const V4& v) const{
			return V4(x*v.x,y*v.y,z*v.z,w*v.w);
		}
		V4 operator/(const V4& v) const{
			return V4(x/v.x,y/v.y,z/v.z,w/v.w);
		}
		V4 operator*(float s) const{
			return V4(x*s,y*s,z*s,w*s);
		}
		V4 operator/(float s) const{
			return V4(x/s,y/s,z/s,w/s);
		}
		bool operator==(const V4& v) const{
			return(x==v.x&&y==v.y&&z==v.z&&w==v.w);
		}
		bool operator!=(const V4& v) const{
			return(x!=v.x||y!=v.y||z!=v.z||w!=v.w);
		}
		operator float* (){
			return &x;
		}
		operator const float* () const{
			return &x;
		}
		V3 VXYZ()const{
			return V3(x,y,z);
		}
};

inline V4 VSet(float x,float y,float z=0.0f,float w=1.0f){
	return V4(x,y,z,w);
}

inline V3& V3::operator=(const V4& v){
	x=v.x;y=v.y;z=v.z;return *this;
}

inline V2& V2::operator=(const V4& v){
	x=v.x;y=v.y;return *this;
}

inline V4 operator*(float s,const V4& v){
	return v*s;
}

inline V4 VAbs(const V4& v){
	return V4(fabsf(v.x),fabsf(v.y),fabsf(v.z),fabsf(v.w));
}

inline float VDot(const V4& a,const V4& b){
	return(a.x*b.x+a.y*b.y+a.z*b.z+a.w*b.w);
}

inline bool RoughlyEqual(const V4& a,const V4& b,const float fLimit=0.001f){
	return (fabsf(a.x-b.x)<fLimit&&fabsf(a.y-b.y)<fLimit&&fabsf(a.z-b.z)<fLimit&&fabsf(a.w-b.w)<fLimit);
}

inline float VLength(const V4& v){
	return sqrtf(VDot(v,v));
}

inline float VLengthSquared(const V4& v){
	return VDot(v,v);
}

inline V4 VNeg(const V4& v){
	return V4(-v.x,-v.y,-v.z,-v.w);
}

inline V4 VNormalize(const V4& a){
	float len=VLength(a);
	return len>0.0f ? a/len : a;
}

inline V4 Homogenize(const V4& v){
	return V4(v.x/v.w,v.y/v.w,v.z/v.w,1.0f);
}

inline V4 VLerp(const V4& a,const V4& b,float s){
	return a*(1-s)+b*s;
}

inline const V3& V4ToV3(const V4& v){
	return *(const V3*)&v;
}

inline V3 HomogenizeToV3(const V4& v){
	return V3(v.x/v.w,v.y/v.w,v.z/v.w);
}

inline float MinXYZW(const V4& v){
	return Min(Min(v.x,v.y),Min(v.z,v.w));
}

inline float MaxXYZW(const V4& v){
	return Max(Max(v.x,v.y),Max(v.z,v.w));
}

class M33{
	public:
		inline M33(){}
		inline explicit M33(const float* data);
		inline M33(float e11,float e12,float e13,float e21,float e22,float e23,float e31,float e32,float e33);
		inline M33(const V3& xa,const V3& ya,const V3& za);
		static M33 Identity(){
			return M33(1,0,0,0,1,0,0,0,1);
		}
		inline operator float* (){
			return m[0];
		}
		inline operator const float* () const{
			return m[0];
		}
		inline V3& XAxis(){
			return *(V3*)&m[0][0];
		}
		inline V3& YAxis(){
			return *(V3*)&m[1][0];
		}
		inline V3& ZAxis(){
			return *(V3*)&m[2][0];
		}
		void SetAxis(V3 v,const int i){
			m[i][0]=v.x;
			m[i][1]=v.y;
			m[i][2]=v.z;
		}
		inline const V3& XAxis() const{
			return *(V3*)&m[0][0];
		}
		inline const V3& YAxis() const{
			return *(V3*)&m[1][0];
		}
		inline const V3& ZAxis() const{
			return *(V3*)&m[2][0];
		}
		inline const V3& Axis(const int i) const{
			return *(V3*)&m[i][0];
		}
		struct e33{
			float _11,_12,_13;
			float _21,_22,_23;
			float _31,_32,_33;
		};
		union{
			e33 e;
			float m[3][3];
		};
		inline M33& operator *=(float s){
			for(int row=0; row<3; row++)
				for(int col=0; col<3; col++)
					m[row][col]*=s;
			return *this;
		}
};

inline M33 RotateY(const M33& m,float angle){
	float cosA,sinA,tmp;
	cosA=(float)cosf(angle);
	sinA=(float)sinf(angle);
	M33 res=m;
	tmp=res.m[0][0]*sinA;
	res.m[0][0]*=cosA;
	res.m[0][0]+=res.m[2][0]*sinA;
	res.m[2][0]*=cosA;
	res.m[2][0]-=tmp;
	tmp=res.m[0][1]*sinA;
	res.m[0][1]*=cosA;
	res.m[0][1]+=res.m[2][1]*sinA;
	res.m[2][1]*=cosA;
	res.m[2][1]-=tmp;
	tmp=res.m[0][2]*sinA;
	res.m[0][2]*=cosA;
	res.m[0][2]+=res.m[2][2]*sinA;
	res.m[2][2]*=cosA;
	res.m[2][2]-=tmp;
	return res;
}

inline M33 MTranspose(const M33& m){
	M33 r;
	r.m[0][0]=m.m[0][0];
	r.m[1][0]=m.m[0][1];
	r.m[2][0]=m.m[0][2];
	r.m[0][1]=m.m[1][0];
	r.m[1][1]=m.m[1][1];
	r.m[2][1]=m.m[1][2];
	r.m[0][2]=m.m[2][0];
	r.m[1][2]=m.m[2][1];
	r.m[2][2]=m.m[2][2];
	return r;
}

inline M33 Multiply(const M33& m1,const M33& m2){
	M33 r;
	for(int iCol=0; iCol<3; iCol++){
		r.m[0][iCol]=m1.m[0][0]*m2.m[0][iCol]+m1.m[0][1]*m2.m[1][iCol]+m1.m[0][2]*m2.m[2][iCol];
		r.m[1][iCol]=m1.m[1][0]*m2.m[0][iCol]+m1.m[1][1]*m2.m[1][iCol]+m1.m[1][2]*m2.m[2][iCol];
		r.m[2][iCol]=m1.m[2][0]*m2.m[0][iCol]+m1.m[2][1]*m2.m[1][iCol]+m1.m[2][2]*m2.m[2][iCol];
	}
	return r;
}

inline M33::M33(const V3& xa,const V3& ya,const V3& za){
	m[0][0]=xa.x;
	m[0][1]=xa.y;
	m[0][2]=xa.z;
	m[1][0]=ya.x;
	m[1][1]=ya.y;
	m[1][2]=ya.z;
	m[2][0]=za.x;
	m[2][1]=za.y;
	m[2][2]=za.z;
}

inline M33::M33(float e11,float e12,float e13,float e21,float e22,float e23,float e31,float e32,float e33){
	m[0][0]=e11;
	m[0][1]=e12;
	m[0][2]=e13;
	m[1][0]=e21;
	m[1][1]=e22;
	m[1][2]=e23;
	m[2][0]=e31;
	m[2][1]=e32;
	m[2][2]=e33;
}

inline M33 CreateMat33FromXAxisUserY(const V3& xa,const V3& ya){
	V3 xan=VNormalize(xa);
	V3 zan=VNormalize(VCross(xan,ya));
	V3 yan=VNormalize(VCross(zan,xan));
	return M33(xan,yan,zan);
}

inline M33 CreateMat33FromYAxisUserZ(const V3& ya,const V3& za){
	V3 yan=VNormalize(ya);
	V3 xan=VNormalize(VCross(yan,za));
	V3 zan=VNormalize(VCross(xan,yan));
	return M33(xan,yan,zan);
}

inline M33 CreateMat33FromZAxisUserX(const V3& za,const V3& xa){
	V3 zan=VNormalize(za);
	V3 yan=VNormalize(VCross(zan,xa));
	V3 xan=VNormalize(VCross(yan,zan));
	return M33(xan,yan,zan);
}

inline M33 Rx(float theta){
	M33 res;
	float s=sinf(theta);
	float c=cosf(theta);
	res.m[0][0]=1;res.m[0][1]=0;res.m[0][2]=0;
	res.m[1][0]=0;res.m[1][1]=c;res.m[1][2]=s;
	res.m[2][0]=0;res.m[2][1]=-s;res.m[2][2]=c;
	return res;
}

inline M33 Ry(float theta){
	M33 res;
	float s=sinf(theta);
	float c=cosf(theta);
	res.m[0][0]=c;res.m[0][1]=0;res.m[0][2]=-s;
	res.m[1][0]=0;res.m[1][1]=1;res.m[1][2]=0;
	res.m[2][0]=s;res.m[2][1]=0;res.m[2][2]=c;
	return res;
}

inline M33 Rz(float theta){
	M33 res;
	float s=sinf(theta);
	float c=cosf(theta);
	res.m[0][0]=c;res.m[0][1]=s;res.m[0][2]=0;
	res.m[1][0]=-s;res.m[1][1]=c;res.m[1][2]=0;
	res.m[2][0]=0;res.m[2][1]=0;res.m[2][2]=1;
	return res;
}

inline V3 RotationToEulerRPY(const M33& m){
	float m00=m.m[0][0];
	float m10=m.m[1][0];
	float m20=m.m[2][0];
	float m21=m.m[2][1];
	float m22=m.m[2][2];
	V3 rpy;
	rpy.x=atan2f(-m21,m22);
	rpy.y=atan2f(m20,sqrtf(m21*m21+m22*m22));
	rpy.z=atan2f(-m10,m00);
	return rpy;
}

inline M33 EulerRPYToRotation(const V3& rpy){
	return Multiply(Multiply(Rz(rpy.z),Ry(rpy.y)),Rx(rpy.x));
}

inline V3 Multiply(const V3& in,const M33& m){
	V3 out;
	out.x=in.x*m.m[0][0]+in.y*m.m[1][0]+in.z*m.m[2][0];
	out.y=in.x*m.m[0][1]+in.y*m.m[1][1]+in.z*m.m[2][1];
	out.z=in.x*m.m[0][2]+in.y*m.m[1][2]+in.z*m.m[2][2];
	return out;
}
inline V3 Multiply(const M33& m,const V3& in){
	V3 out;
	out.x=in.x*m.m[0][0]+in.y*m.m[0][1]+in.z*m.m[0][2];
	out.y=in.x*m.m[1][0]+in.y*m.m[1][1]+in.z*m.m[1][2];
	out.z=in.x*m.m[2][0]+in.y*m.m[2][1]+in.z*m.m[2][2];
	return out;
}

inline void MEulerRPY(float* roll,float* pitch,float* yaw,const M33& m){
	float m00=m.m[0][0];
	float m10=m.m[1][0];
	float m20=m.m[2][0];
	float m21=m.m[2][1];
	float m22=m.m[2][2];
	*roll=atan2f(-m21,m22);
	*pitch=atan2f(m20,sqrtf(m21*m21+m22*m22));
	*yaw=atan2f(-m10,m00);
}

inline void Renormalize(M33& mat){
	V3 z=mat.ZAxis();
	V3 x=mat.XAxis();
	V3 y=VNormalize(VCross(z,x));
	mat.XAxis()=VNormalize(VCross(y,z));
	mat.YAxis()=y;
	mat.ZAxis()=VNormalize(z);
}

inline M33 CreateMat33FromZAxisUserY(V3 zaxis,const V3& vUp){
	zaxis=VNormalize(zaxis);
	V3 xa=VNormalize(VCross(vUp,zaxis));
	M33 m(xa,VCross(zaxis,xa),zaxis);
	return m;
}

class M44{
	public:
		inline M44(){}
		inline explicit M44(const float* data);
		inline M44(float e11,float e12,float e13,float e14,float e21,float e22,float e23,float e24,float e31,float e32,float e33,float e34,float e41,float e42,float e43,float e44);
		inline M44(const V4& xa,const V4& ya,const V4& za,const V4& wa);
		inline operator float* (){
			return m[0];
		}
		inline operator const float* () const{
			return m[0];
		}
		M44& operator=(const M44& mat);
		M44& operator+=(const M44& mat);
		M44& operator-=(const M44& mat);
		M44& operator*=(float s);
		M44& operator/=(float s);
		M44 operator+(const M44& mat) const{
			M44 a=(*this); a+=mat; return a;
		}
		M44 operator-(const M44& mat) const{
			M44 a=(*this); a-=mat; return a;
		}
		M44 operator*(float s) const{
			M44 a=(*this); a*=s; return a;
		}
		M44 operator/(float s) const{
			M44 a=(*this); a/=s; return a;
		}
		bool operator==(const M44& mat) const{
			return ArrayEquals(m[0],mat.m[0],16);
		}
		bool operator!=(const M44& mat) const{
			return!ArrayEquals(m[0],mat.m[0],16);
		}
		inline const V4& XAxis()const{
			return *(V4*)&m[0][0];
		}
		inline const V4& YAxis()const{
			return *(V4*)&m[1][0];
		}
		inline const V4& ZAxis()const{
			return *(V4*)&m[2][0];
		}
		inline const V4& WAxis()const{
			return *(V4*)&m[3][0];
		}
		inline const V4& Axis(const int axis) const{
			return *(V4*)&m[axis][0];
		}
		inline V4& XAxis(){
			return *(V4*)&m[0][0];
		}
		inline V4& YAxis(){
			return *(V4*)&m[1][0];
		}
		inline V4& ZAxis(){
			return *(V4*)&m[2][0];
		}
		inline V4& WAxis(){
			return *(V4*)&m[3][0];
		}
		inline void SetXAxis(const V4& vXAxis)const{
			*(V4*)&m[0][0]=vXAxis;
		}
		inline void SetYAxis(const V4& vYAxis)const{
			*(V4*)&m[1][0]=vYAxis;
		}
		inline void SetZAxis(const V4& vZAxis)const{
			*(V4*)&m[2][0]=vZAxis;
		}
		inline void SetWAxis(const V4& vWAxis)const{
			*(V4*)&m[3][0]=vWAxis;
		}
		union{
			float m[4][4];
		};
};

inline M44::M44(const float* data){
	memcpy(*this,data,sizeof(M44));
}

inline M44 MTranspose(const M44& m){
	M44 r;
	r.m[0][0]=m.m[0][0];
	r.m[1][0]=m.m[0][1];
	r.m[2][0]=m.m[0][2];
	r.m[3][0]=m.m[0][3];
	r.m[0][1]=m.m[1][0];
	r.m[1][1]=m.m[1][1];
	r.m[2][1]=m.m[1][2];
	r.m[3][1]=m.m[1][3];
	r.m[0][2]=m.m[2][0];
	r.m[1][2]=m.m[2][1];
	r.m[2][2]=m.m[2][2];
	r.m[3][2]=m.m[2][3];
	r.m[0][3]=m.m[3][0];
	r.m[1][3]=m.m[3][1];
	r.m[2][3]=m.m[3][2];
	r.m[3][3]=m.m[3][3];
	return r;
}

inline M44::M44(const V4& xa,const V4& ya,const V4& za,const V4& wa){
	SetXAxis(xa);
	SetYAxis(ya);
	SetZAxis(za);
	SetWAxis(wa);
}

inline M44::M44(float e11,float e12,float e13,float e14,float e21,float e22,float e23,float e24,float e31,float e32,float e33,float e34,float e41,float e42,float e43,float e44){
	m[0][0]=e11;
	m[0][1]=e12;
	m[0][2]=e13;
	m[0][3]=e14;
	m[1][0]=e21;
	m[1][1]=e22;
	m[1][2]=e23;
	m[1][3]=e24;
	m[2][0]=e31;
	m[2][1]=e32;
	m[2][2]=e33;
	m[2][3]=e34;
	m[3][0]=e41;
	m[3][1]=e42;
	m[3][2]=e43;
	m[3][3]=e44;
}

inline M44& M44::operator=(const M44& mat){
	memcpy(*this,&mat,sizeof(M44));
	return *this;
}

inline M44& M44::operator+=(const M44& mat){
	for(int row=0; row<4; row++)
		for(int col=0; col<4; col++)
			m[row][col]+=mat.m[row][col];
	return *this;
}

inline M44& M44::operator-=(const M44& mat){
	for(int row=0; row<4; row++)
		for(int col=0; col<4; col++)
			m[row][col]-=mat.m[row][col];
	return *this;
}

inline M44& M44::operator*=(float s){
	for(int row=0; row<4; row++)
		for(int col=0; col<4; col++)
			m[row][col]*=s;
	return *this;
}

inline M44& M44::operator/=(float s){
	for(int row=0; row<4; row++)
		for(int col=0; col<4; col++)
			m[row][col]/=s;
	return *this;
}

inline M44 operator*(float s,const M44& mat){
	return mat*s;
}

inline V4 TransformVec4(const V4& in,const M44& m){
	V4 out;
	out.x=in.x*m.m[0][0]+in.y*m.m[1][0]+in.z*m.m[2][0]+in.w*m.m[3][0];
	out.y=in.x*m.m[0][1]+in.y*m.m[1][1]+in.z*m.m[2][1]+in.w*m.m[3][1];
	out.z=in.x*m.m[0][2]+in.y*m.m[1][2]+in.z*m.m[2][2]+in.w*m.m[3][2];
	out.w=in.x*m.m[0][3]+in.y*m.m[1][3]+in.z*m.m[2][3]+in.w*m.m[3][3];
	return out;
}

inline V4 TransformVec4(const M44& m,const V4& in){
	V4 out;
	out.x=in.x*m.m[0][0]+in.y*m.m[0][1]+in.z*m.m[0][2]+in.w*m.m[0][3];
	out.y=in.x*m.m[1][0]+in.y*m.m[1][1]+in.z*m.m[1][2]+in.w*m.m[1][3];
	out.z=in.x*m.m[2][0]+in.y*m.m[2][1]+in.z*m.m[2][2]+in.w*m.m[2][3];
	out.w=in.x*m.m[3][0]+in.y*m.m[3][1]+in.z*m.m[3][2]+in.w*m.m[2][3];
	return out;
}

inline V4 operator*(const M44& m,const V4& in){
	return TransformVec4(in,m);
}

inline V4 operator*(const V4& in,const M44& m){
	return TransformVec4(in,m);
}

inline M44 MInverse(const M44& m){
	M44 r;
	float f22Det1=m.m[2][2]*m.m[3][3]-m.m[2][3]*m.m[3][2];
	float f22Det2=m.m[2][1]*m.m[3][3]-m.m[2][3]*m.m[3][1];
	float f22Det3=m.m[2][1]*m.m[3][2]-m.m[2][2]*m.m[3][1];
	float f22Det4=m.m[2][0]*m.m[3][3]-m.m[2][3]*m.m[3][0];
	float f22Det5=m.m[2][0]*m.m[3][2]-m.m[2][2]*m.m[3][0];
	float f22Det6=m.m[2][0]*m.m[3][1]-m.m[2][1]*m.m[3][0];
	float f22Det7=m.m[1][2]*m.m[3][3]-m.m[1][3]*m.m[3][2];
	float f22Det8=m.m[1][1]*m.m[3][3]-m.m[1][3]*m.m[3][1];
	float f22Det9=m.m[1][1]*m.m[3][2]-m.m[1][2]*m.m[3][1];
	float f22Det10=m.m[1][2]*m.m[2][3]-m.m[1][3]*m.m[2][2];
	float f22Det11=m.m[1][1]*m.m[2][3]-m.m[1][3]*m.m[2][1];
	float f22Det12=m.m[1][1]*m.m[2][2]-m.m[1][2]*m.m[2][1];
	float f22Det13=m.m[1][0]*m.m[3][3]-m.m[1][3]*m.m[3][0];
	float f22Det14=m.m[1][0]*m.m[3][2]-m.m[1][2]*m.m[3][0];
	float f22Det15=m.m[1][0]*m.m[2][3]-m.m[1][3]*m.m[2][0];
	float f22Det16=m.m[1][0]*m.m[2][2]-m.m[1][2]*m.m[2][0];
	float f22Det17=m.m[1][0]*m.m[3][1]-m.m[1][1]*m.m[3][0];
	float f22Det18=m.m[1][0]*m.m[2][1]-m.m[1][1]*m.m[2][0];
	float fFirst33Det=m.m[1][1]*f22Det1-m.m[1][2]*f22Det2+m.m[1][3]*f22Det3;
	float fSec33Det=m.m[1][0]*f22Det1-m.m[1][2]*f22Det4+m.m[1][3]*f22Det5;
	float fThird33Det=m.m[1][0]*f22Det2-m.m[1][1]*f22Det4+m.m[1][3]*f22Det6;
	float fFourth33Det=m.m[1][0]*f22Det3-m.m[1][1]*f22Det5+m.m[1][2]*f22Det6;
	float fDet44=m.m[0][0]*fFirst33Det-m.m[0][1]*fSec33Det+m.m[0][2]*fThird33Det-m.m[0][3]*fFourth33Det;
	float s=1.0f/fDet44;
	r.m[0][0]=s*fFirst33Det;
	r.m[0][1]=-s*(m.m[0][1]*f22Det1-m.m[0][2]*f22Det2+m.m[0][3]*f22Det3);
	r.m[0][2]=s*(m.m[0][1]*f22Det7-m.m[0][2]*f22Det8+m.m[0][3]*f22Det9);
	r.m[0][3]=-s*(m.m[0][1]*f22Det10-m.m[0][2]*f22Det11+m.m[0][3]*f22Det12);

	r.m[1][0]=-s*fSec33Det;
	r.m[1][1]=s*(m.m[0][0]*f22Det1-m.m[0][2]*f22Det4+m.m[0][3]*f22Det5);
	r.m[1][2]=-s*(m.m[0][0]*f22Det7-m.m[0][2]*f22Det13+m.m[0][3]*f22Det14);
	r.m[1][3]=s*(m.m[0][0]*f22Det10-m.m[0][2]*f22Det15+m.m[0][3]*f22Det16);

	r.m[2][0]=s*fThird33Det;
	r.m[2][1]=-s*(m.m[0][0]*f22Det2-m.m[0][1]*f22Det4+m.m[0][3]*f22Det6);
	r.m[2][2]=s*(m.m[0][0]*f22Det8-m.m[0][1]*f22Det13+m.m[0][3]*f22Det17);
	r.m[2][3]=-s*(m.m[0][0]*f22Det11-m.m[0][1]*f22Det15+m.m[0][3]*f22Det18);

	r.m[3][0]=-s*fFourth33Det;
	r.m[3][1]=s*(m.m[0][0]*f22Det3-m.m[0][1]*f22Det5+m.m[0][2]*f22Det6);
	r.m[3][2]=-s*(m.m[0][0]*f22Det9-m.m[0][1]*f22Det14+m.m[0][2]*f22Det17);
	r.m[3][3]=s*(m.m[0][0]*f22Det12-m.m[0][1]*f22Det16+m.m[0][2]*f22Det18);
	return r;
}

inline M44 MAffineInverse(const M44& m1){
	M44 r;
	r.m[0][0]=m1.m[0][0];
	r.m[1][0]=m1.m[0][1];
	r.m[2][0]=m1.m[0][2];

	r.m[0][1]=m1.m[1][0];
	r.m[1][1]=m1.m[1][1];
	r.m[2][1]=m1.m[1][2];

	r.m[0][2]=m1.m[2][0];
	r.m[1][2]=m1.m[2][1];
	r.m[2][2]=m1.m[2][2];

	r.m[0][3]=0;
	r.m[1][3]=0;
	r.m[2][3]=0;
	r.m[3][3]=1;

	float fTx=m1.m[3][0];
	float fTy=m1.m[3][1];
	float fTz=m1.m[3][2];

	r.m[3][0]=-(m1.m[0][0]*fTx+m1.m[0][1]*fTy+m1.m[0][2]*fTz);
	r.m[3][1]=-(m1.m[1][0]*fTx+m1.m[1][1]*fTy+m1.m[1][2]*fTz);
	r.m[3][2]=-(m1.m[2][0]*fTx+m1.m[2][1]*fTy+m1.m[2][2]*fTz);
	return r;
}

inline M44 Multiply(const M44& m1,const M44& m2){
	M44 r;
	for(int iCol=0; iCol<4; iCol++){
		r.m[0][iCol]=m1.m[0][0]*m2.m[0][iCol]+m1.m[0][1]*m2.m[1][iCol]+m1.m[0][2]*m2.m[2][iCol]+m1.m[0][3]*m2.m[3][iCol];
		r.m[1][iCol]=m1.m[1][0]*m2.m[0][iCol]+m1.m[1][1]*m2.m[1][iCol]+m1.m[1][2]*m2.m[2][iCol]+m1.m[1][3]*m2.m[3][iCol];
		r.m[2][iCol]=m1.m[2][0]*m2.m[0][iCol]+m1.m[2][1]*m2.m[1][iCol]+m1.m[2][2]*m2.m[2][iCol]+m1.m[2][3]*m2.m[3][iCol];
		r.m[3][iCol]=m1.m[3][0]*m2.m[0][iCol]+m1.m[3][1]*m2.m[1][iCol]+m1.m[3][2]*m2.m[2][iCol]+m1.m[3][3]*m2.m[3][iCol];
	}
	return r;
}

inline M44 PreRotationMatrix(const V3& vAngle){
	float sx=sinf(vAngle.x);
	float cx=cosf(vAngle.x);
	float sy=sinf(vAngle.y);
	float cy=cosf(vAngle.y);
	float sz=sinf(vAngle.z);
	float cz=cosf(vAngle.z);
	return {
		{cy*cz,cz*sx*sy-cx*sz,cx*cz*sy+sx*sz,0},
		{cy*sz,sx*sy*sz+cx*cz,cx*sy*sz-cz*sx,0},
		{-sy,cy*sx,cx*cy,0},
		{0,0,0,1}
	};
}

inline void SetTranslation4(M44& mat,const V4& vector){
	mat.m[3][0]=vector.x;
	mat.m[3][1]=vector.y;
	mat.m[3][2]=vector.z;
	mat.m[3][3]=vector.w;
}

inline void SetTranslation(M44& mat,float x,float y,float z){
	mat.m[3][0]=x;
	mat.m[3][1]=y;
	mat.m[3][2]=z;
}

inline const V4& GetTranslation4(const M44& mat){
	return *(const V4*)&mat.m[3][0];
}

inline void Lerp(M44& out,const M44& in0,const M44& in1,float i) {
	float	oi=1.0f-i;

	out.m[0][0]=in0.m[0][0]*oi+in1.m[0][0]*i;
	out.m[0][1]=in0.m[0][1]*oi+in1.m[0][1]*i;
	out.m[0][2]=in0.m[0][2]*oi+in1.m[0][2]*i;
	out.m[0][3]=in0.m[0][3]*oi+in1.m[0][3]*i;

	out.m[1][0]=in0.m[1][0]*oi+in1.m[1][0]*i;
	out.m[1][1]=in0.m[1][1]*oi+in1.m[1][1]*i;
	out.m[1][2]=in0.m[1][2]*oi+in1.m[1][2]*i;
	out.m[1][3]=in0.m[1][3]*oi+in1.m[1][3]*i;

	out.m[2][0]=in0.m[2][0]*oi+in1.m[2][0]*i;
	out.m[2][1]=in0.m[2][1]*oi+in1.m[2][1]*i;
	out.m[2][2]=in0.m[2][2]*oi+in1.m[2][2]*i;
	out.m[2][3]=in0.m[2][3]*oi+in1.m[2][3]*i;

	out.m[3][0]=in0.m[3][0]*oi+in1.m[3][0]*i;
	out.m[3][1]=in0.m[3][1]*oi+in1.m[3][1]*i;
	out.m[3][2]=in0.m[3][2]*oi+in1.m[3][2]*i;
	out.m[3][3]=in0.m[3][3]*oi+in1.m[3][3]*i;
}

inline M44 MTranspose33(const M44& m){
	M44 m1;
	m1.m[0][0]=m.m[0][0];
	m1.m[0][1]=m.m[1][0];
	m1.m[0][2]=m.m[2][0];
	m1.m[0][3]=m.m[0][3];
	m1.m[1][0]=m.m[0][1];
	m1.m[1][1]=m.m[1][1];
	m1.m[1][2]=m.m[2][1];
	m1.m[1][3]=m.m[1][3];
	m1.m[2][0]=m.m[0][2];
	m1.m[2][1]=m.m[1][2];
	m1.m[2][2]=m.m[2][2];
	m1.m[2][3]=m.m[2][3];
	m1.m[3][0]=m.m[3][0];
	m1.m[3][1]=m.m[3][1];
	m1.m[3][2]=m.m[3][2];
	m1.m[3][3]=m.m[3][3];
	return m1;
}

inline void SetTranslation(M44& mat,const V3& vector){
	mat.m[3][0]=vector.x;
	mat.m[3][1]=vector.y;
	mat.m[3][2]=vector.z;
}

inline V3 MulNoTrans(const M44& m,const V3& in) {
	V3 out;
	out.x=in.x*m.m[0][0]+in.y*m.m[1][0]+in.z*m.m[2][0];
	out.y=in.x*m.m[0][1]+in.y*m.m[1][1]+in.z*m.m[2][1];
	out.z=in.x*m.m[0][2]+in.y*m.m[1][2]+in.z*m.m[2][2];
	return out;
}

inline V3 MulTrans(const M44& m,const V3& in){
	V3 out;
	out.x=in.x*m.m[0][0]+in.y*m.m[1][0]+in.z*m.m[2][0]+m.m[3][0];
	out.y=in.x*m.m[0][1]+in.y*m.m[1][1]+in.z*m.m[2][1]+m.m[3][1];
	out.z=in.x*m.m[0][2]+in.y*m.m[1][2]+in.z*m.m[2][2]+m.m[3][2];
	return out;
}

inline V3 TransformNormal(const V3& in,const M44& m){
	return MulNoTrans(m,in);
}

inline V3 TransformCoord(const V3& in,const M44& m){
	return MulTrans(m,in);
}

inline V3 TransformVec3(const V3& in,const M44& m){
	V3 out;
	out.x=in.x*m.m[0][0]+in.y*m.m[1][0]+in.z*m.m[2][0]+m.m[3][0];
	out.y=in.x*m.m[0][1]+in.y*m.m[1][1]+in.z*m.m[2][1]+m.m[3][1];
	out.z=in.x*m.m[0][2]+in.y*m.m[1][2]+in.z*m.m[2][2]+m.m[3][2];
	return out;
}
inline V2 TransformCoord(const V2& in,const M44& m){
	V2 out;
	out.x=in.x*m.m[0][0]+in.y*m.m[1][0]+m.m[3][0];
	out.y=in.x*m.m[0][1]+in.y*m.m[1][1]+m.m[3][1];
	return out;
}

inline V3 operator*(const M44& m,const V3& in){
	return MulTrans(m,in);
}

inline V3 operator*(const V3& in,const M44& m){
	return MulTrans(m,in);
}

inline const V3& GetX(const M44& m){
	return *(const V3*)&m.m[0][0];
}

inline const V3& GetY(const M44& m){
	return *(const V3*)&m.m[1][0];
}

inline const V3& GetZ(const M44& m){
	return *(const V3*)&m.m[2][0];
}

inline const V3& GetT(const M44& m){
	return *(const V3*)&m.m[3][0];
}

inline V3& GetX(M44& m){
	return *(V3*)&m.m[0][0];
}

inline V3& GetY(M44& m){
	return *(V3*)&m.m[1][0];
}

inline V3& GetZ(M44& m){
	return *(V3*)&m.m[2][0];
}

inline V3& GetT(M44& m){
	return *(V3*)&m.m[3][0];
}

inline const V3& GetTranslation(const M44& mat){
	return *(const V3*)&mat.m[3][0];
}

inline M33 MRotation(const M44& m){
	M33 m1(m.m[0][0],m.m[0][1],m.m[0][2],m.m[1][0],m.m[1][1],m.m[1][2],m.m[2][0],m.m[2][1],m.m[2][2]);
	return m1;
}

inline M44 MLoadIdentity(){
	return {
		1,0,0,0,
		0,1,0,0,
		0,0,1,0,
		0,0,0,1
	};
}

inline void MLoad(M44& m44,const M33& m){
	m44.m[0][0]=m.m[0][0];
	m44.m[0][1]=m.m[0][1];
	m44.m[0][2]=m.m[0][2];
	m44.m[1][0]=m.m[1][0];
	m44.m[1][1]=m.m[1][1];
	m44.m[1][2]=m.m[1][2];
	m44.m[2][0]=m.m[2][0];
	m44.m[2][1]=m.m[2][1];
	m44.m[2][2]=m.m[2][2];
}

inline M44 MLoad(const M33& m){
	M44 m1(m.m[0][0],m.m[0][1],m.m[0][2],0,m.m[1][0],m.m[1][1],m.m[1][2],0,m.m[2][0],m.m[2][1],m.m[2][2],0,0,0,0,1);
	return m1;
}

inline M44 MLoad(const M33& m,const V3& v){
	M44 m1(m.m[0][0],m.m[0][1],m.m[0][2],0,m.m[1][0],m.m[1][1],m.m[1][2],0,m.m[2][0],m.m[2][1],m.m[2][2],0,v.x,v.y,v.z,1);
	return m1;
}

inline void CreateScalingAndTranslation(M44& mat,const V3& scale,const V3& offset){
	mat.m[0][0]=scale.x;mat.m[0][1]=0;mat.m[0][2]=0;mat.m[0][3]=0;
	mat.m[1][0]=0;mat.m[1][1]=scale.y;mat.m[1][2]=0;mat.m[1][3]=0;
	mat.m[2][0]=0;mat.m[2][1]=0;mat.m[2][2]=scale.z;mat.m[2][3]=0;
	mat.m[3][0]=offset.x;mat.m[3][1]=offset.y;mat.m[3][2]=offset.z;mat.m[3][3]=1.0f;
}

inline void CreateTranslation(M44& mat,const V3& offset){
	CreateScalingAndTranslation(mat,V3(1,1,1),offset);
}

inline void CreateScaling(M44& mat,const V3& scale){
	CreateScalingAndTranslation(mat,scale,V3(0,0,0));
}

inline void CreateScaling(M44& mat,float scale){
	CreateScaling(mat,V3(scale,scale,scale));
}

inline M44 CreateMatFromZAxisUserY(V3 zaxis,const V3& vUp,const V3& translation){
	M44 m=MLoad(CreateMat33FromZAxisUserY(zaxis,vUp),translation);
	return m;
}

inline M44 CreateMatFromXAxis(V3 xaxis){
	xaxis=VNormalize(xaxis);
	V3 za=VNormalize(VCross(xaxis,V3(0,1,0)));
	M44 m=MLoadIdentity();
	GetX(m)=xaxis;
	GetY(m)=VCross(za,xaxis);
	GetZ(m)=za;
	return m;
}

inline M44 CreateMatFromYAxis(V3 yaxis){
	yaxis=VNormalize(yaxis);
	V3 xa=VNormalize(VCross(yaxis,V3(0,0,1)));
	M44 m=MLoadIdentity();
	GetY(m)=yaxis;
	GetZ(m)=VCross(xa,yaxis);
	GetX(m)=xa;
	return m;
}

inline M44 CreateMatFromZAxis(V3 zaxis,bool bLockY=true){
	zaxis=VNormalize(zaxis);
	M44 m=MLoadIdentity();
	if(bLockY){
		V3 xa=VNormalize(VCross(V3(0,1,0),zaxis));
		GetZ(m)=zaxis;
		GetY(m)=VCross(zaxis,xa);
		GetX(m)=xa;
	} else{
		V3 ya=VNormalize(VCross(zaxis,V3(1,0,0)));
		GetZ(m)=zaxis;
		GetX(m)=VCross(ya,zaxis);
		GetY(m)=ya;
	}
	return m;

}

inline void Renormalize(M44& mat){
	V3 z=GetZ(mat);
	V3 x=GetX(mat);
	V3 y=VNormalize(VCross(z,x));
	GetX(mat)=VNormalize(VCross(y,z));
	GetY(mat)=y;
	GetZ(mat)=VNormalize(z);
}

class RectV2{
	public:
		RectV2(float xa,float ya,float widtha,float heighta){
			x=xa;y=ya;width=widtha;height=heighta;
		}
		RectV2(const V2& tl,float w,float h){
			x=tl.x;y=tl.y;width=w;height=h;
		}
		RectV2(){
			x=0;y=0;width=0;height=0;
		}
		RectV2(const V2& tl,const V2& br){
			x=tl.x;y=tl.y;width=br.x-tl.x;height=br.y-tl.y;
		}
		V2 Center()const{
			return V2(x+(width*0.5f),y+(height*0.5f));
		}
		V2 tl()const{
			return V2(x,y);
		}
		V2 br()const{
			return V2(x+width,y+height);
		}
		V2 TopLeft()const{
			return V2(x,y);
		}
		V2 BottomRight()const{
			return V2(x+width,y+height);
		}
		V2 Size()const{
			return V2(width,height);
		}
		bool Inside(const V2& a)const{
			if(x<a.x&&y<a.y&&x+width>=a.x&&y+height>=a.y)
				return true;
			return false;
		}
		RectV2 Intersection(const RectV2& b)const{
			RectV2 a=*this;
			float x1=MAX(a.x,b.x),y1=MAX(a.y,b.y);
			a.width=MIN(a.x+a.width,b.x+b.width)-x1;
			a.height=MIN(a.y+a.height,b.y+b.height)-y1;
			a.x=x1; a.y=y1;
			if(a.width<=0||a.height<=0)
				a=RectV2();
			return a;
		}
		RectV2 Union(const RectV2& b)const{
			RectV2 a=*this;
			float x1=MIN(a.x,b.x),y1=MIN(a.y,b.y);
			a.width=MAX(a.x+a.width,b.x+b.width)-x1;
			a.height=MAX(a.y+a.height,b.y+b.height)-y1;
			a.x=x1; a.y=y1;
			return a;
		}
		RectV2 operator & (const RectV2& a)const{
			return Intersection(a);
		}
		RectV2 operator | (const RectV2& a)const{
			return Union(a);
		}
		float area()const{
			return height*width;
		}
		float height;
		float width;
		float x;
		float y;
};

inline V3 uint322V3(uint32_t d){
	return V3(((d>>16)&255)/255.f,((d>>8)&255)/255.f,((d>>0)&255)/255.f);
}
inline V4 uint322V4(uint32_t d){
	return V4(((d>>24)&255)/255.f,((d>>16)&255)/255.f,((d>>8)&255)/255.f,((d>>0)&255)/255.f);
}
inline V4 VLoadABGR(uint32_t d){
	return V4(((d>>24)&255)/255.f,((d>>16)&255)/255.f,((d>>8)&255)/255.f,((d>>0)&255)/255.f);
}
inline V4 VLoadRGBA(uint32_t d){
	return V4(((d>>0)&255)/255.f,((d>>8)&255)/255.f,((d>>16)&255)/255.f,((d>>24)&255)/255.f);
}

inline uint32_t V42uint32(const V4& v,float scalar=255){
	int r=int(v.x*scalar);
	int g=int(v.y*scalar);
	int b=int(v.z*scalar);
	int a=int(v.w*scalar);
	return (BOUND(0,a,255)<<24)|(BOUND(0,r,255)<<16)|(BOUND(0,g,255)<<8)|(BOUND(0,b,255));
}

inline V3 SnapPosition(const V3& pos,const V3& snap_resolution){
	V3 p=pos*V3(1.0f/snap_resolution.x,1.0f/snap_resolution.y,1.0f/snap_resolution.z);
	p.x=floorf(p.x+0.5f);
	p.y=floorf(p.y+0.5f);
	p.z=floorf(p.z+0.5f);
	return p*snap_resolution;
}

inline V3 NearClipPlaneToWorld(M44& invViewProj,const V2& clip_pos){
	V4	p(clip_pos.x,clip_pos.y,0.0f,1.0f);
	V4	tp=p*invViewProj;
	return V3(tp.x/tp.w,tp.y/tp.w,tp.z/tp.w);
}

float AABBROI(const V3& minA,const V3& maxA,const V3& minB,const V3& maxB);

class M23{
	public:
	M23(){
		Identity();
	}
	void Identity(){
		m[0][0]=1.0f;
		m[0][1]=0.0f;
		m[1][0]=0.0f;
		m[1][1]=1.0f;
		m[2][0]=0.0f;
		m[2][1]=0.0f;
	}
	static M23 Mul(M23& m1,M23& m2){
		M23 r;
		r.m[0][0]=m1.m[0][0]*m2.m[0][0]+m1.m[0][1]*m2.m[1][0];
		r.m[0][1]=m1.m[0][0]*m2.m[0][1]+m1.m[0][1]*m2.m[1][1];
		r.m[1][0]=m1.m[1][0]*m2.m[0][0]+m1.m[1][1]*m2.m[1][0];
		r.m[1][1]=m1.m[1][0]*m2.m[0][1]+m1.m[1][1]*m2.m[1][1];
		r.m[2][0]=m1.m[2][0]*m2.m[0][0]+m1.m[2][1]*m2.m[1][0]+m2.m[2][0];
		r.m[2][1]=m1.m[2][0]*m2.m[0][1]+m1.m[2][1]*m2.m[1][1]+m2.m[2][1];
		return r;
	}
	V2 TransformCoord(float x,float y)const{
		return V2(x*m[0][0]+y*m[1][0]+m[2][0],x*m[0][1]+y*m[1][1]+m[2][1]);
	}
	V2 TransformCoord(const V2& v)const{
		return V2(v.x*m[0][0]+v.y*m[1][0]+m[2][0],v.x*m[0][1]+v.y*m[1][1]+m[2][1]);
	}
	V3 TransformCoordV3(float x,float y)const{
		return V3(x*m[0][0]+y*m[1][0]+m[2][0],x*m[0][1]+y*m[1][1]+m[2][1],0.0f);
	}
	void PostTranslate(float x,float y){
		M23 trans;
		trans.m[2][0]=x;
		trans.m[2][1]=y;
		*this=Mul(*this,trans);
	}
	void PreTranslate(float x,float y){
		M23 trans;
		trans.m[2][0]=x;
		trans.m[2][1]=y;
		*this=Mul(trans,*this);
	}
	void PostScale(float x,float y){
		M23 scl;
		scl.m[0][0]=x;
		scl.m[1][1]=y;
		*this=Mul(*this,scl);
	}
	void PreScale(float x,float y){
		M23 scl;
		scl.m[0][0]=x;
		scl.m[1][1]=y;
		*this=Mul(scl,*this);
	}
	float m[3][2];
};

inline void PreRotate(M23& mat,float angle){
	float cosA=(float)cosf(angle);
	float sinA=(float)sinf(angle);
	float tmp=mat.m[0][0]*sinA;
	mat.m[0][0]*=cosA;
	mat.m[0][0]-=mat.m[1][0]*sinA;
	mat.m[1][0]*=cosA;
	mat.m[1][0]+=tmp;
	tmp=mat.m[0][1]*sinA;
	mat.m[0][1]*=cosA;
	mat.m[0][1]-=mat.m[1][1]*sinA;
	mat.m[1][1]*=cosA;
	mat.m[1][1]+=tmp;
}

inline void PreXRotate(M44& mat,float angle){
	float cosA=(float)cosf(angle);
	float sinA=(float)sinf(angle);
	float tmp=mat.m[1][0]*sinA;
	mat.m[1][0]*=cosA;
	mat.m[1][0]-=mat.m[2][0]*sinA;
	mat.m[2][0]*=cosA;
	mat.m[2][0]+=tmp;
	tmp=mat.m[1][1]*sinA;
	mat.m[1][1]*=cosA;
	mat.m[1][1]-=mat.m[2][1]*sinA;
	mat.m[2][1]*=cosA;
	mat.m[2][1]+=tmp;
	tmp=mat.m[1][2]*sinA;
	mat.m[1][2]*=cosA;
	mat.m[1][2]-=mat.m[2][2]*sinA;
	mat.m[2][2]*=cosA;
	mat.m[2][2]+=tmp;
}

inline void PreYRotate(M44& mat,float angle){
	float cosA=(float)cosf(angle);
	float sinA=(float)sinf(angle);
	float tmp=mat.m[0][0]*sinA;
	mat.m[0][0]*=cosA;
	mat.m[0][0]+=mat.m[2][0]*sinA;
	mat.m[2][0]*=cosA;
	mat.m[2][0]-=tmp;
	tmp=mat.m[0][1]*sinA;
	mat.m[0][1]*=cosA;
	mat.m[0][1]+=mat.m[2][1]*sinA;
	mat.m[2][1]*=cosA;
	mat.m[2][1]-=tmp;
	tmp=mat.m[0][2]*sinA;
	mat.m[0][2]*=cosA;
	mat.m[0][2]+=mat.m[2][2]*sinA;
	mat.m[2][2]*=cosA;
	mat.m[2][2]-=tmp;
}

inline void PreZRotate(M44& mat,float angle){
	float cosA=(float)cosf(angle);
	float sinA=(float)sinf(angle);
	float tmp=mat.m[0][0]*sinA;
	mat.m[0][0]*=cosA;
	mat.m[0][0]-=mat.m[1][0]*sinA;
	mat.m[1][0]*=cosA;
	mat.m[1][0]+=tmp;
	tmp=mat.m[0][1]*sinA;
	mat.m[0][1]*=cosA;
	mat.m[0][1]-=mat.m[1][1]*sinA;
	mat.m[1][1]*=cosA;
	mat.m[1][1]+=tmp;
	tmp=mat.m[0][2]*sinA;
	mat.m[0][2]*=cosA;
	mat.m[0][2]-=mat.m[1][2]*sinA;
	mat.m[1][2]*=cosA;
	mat.m[1][2]+=tmp;
}

inline float GetAngle(const V3& v0,const V3& v1){
	const float l0=VLength(v0);
	const float l1=VLength(v1);
	const V3 v01=v0*l1;
	const V3 v10=v1*l0;
	return 2.f*atan2f(VLength(v01-v10),VLength(v01+v10));
}

inline float GetAngle(const V3& v0,const V3& v1,const V3& ref){
	const float angle=GetAngle(v0,v1);
	const V3 cross=VCross(v0,v1);
	const bool posHalfSpace=VDot(ref,cross)>=0.f;
	return posHalfSpace ? angle : -angle;
}

#endif//__cplusplus
