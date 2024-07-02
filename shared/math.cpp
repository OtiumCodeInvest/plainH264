
#include "shared/math.h"

#include <stdlib.h>
#include <time.h>
#include <math.h>

static unsigned long rand_x=123456789,rand_y=362436069,rand_z=521288629;

void SeedRandom(uint32_t seed){
	srand((uint32_t)seed);
}

void InitRandom(){
	time_t rawtime;
	time(&rawtime);
	srand((uint32_t)rawtime);
}

unsigned long xorshf96(void){          //period 2^96-1
	rand_x^=rand_x<<16;
	rand_x^=rand_x>>5;
	rand_x^=rand_x<<1;
	unsigned long t=rand_x;
	rand_x=rand_y;
	rand_y=rand_z;
	rand_z=t^rand_x^rand_y;
	return rand_z;
}

float RandomUnitFloat(){
	float r=(float)(xorshf96()&0xffff);
	return(r/65536.0f);
}
uint32_t mRandomU32(){
	return (uint32_t)xorshf96();
}

inline float Overlap(float minA,float maxA,float minB,float maxB){
	if(maxA<minB)
		return 0;
	if(minA>maxB)
		return 0;
	if(minA>=minB&&maxA<=maxB){		//A included 100% in B
		return maxA-minA;
	}
	if(minB>=minA&&maxB<=maxA){		//B included 100% in A
		return maxB-minB;
	}
	if(minA<minB){
		return maxA-minB;
	}
	return maxB-minA;
}

float AABBROI(const V3& minA,const V3& maxA,const V3& minB,const V3& maxB){
	V3 o;
	o.x=Overlap(minA.x,maxA.x,minB.x,maxB.x);
	o.y=Overlap(minA.y,maxA.y,minB.y,maxB.y);
	o.z=Overlap(minA.z,maxA.z,minB.z,maxB.z);
	float arealA=(maxA.x-minA.x)*(maxA.y-minA.y)*(maxA.z-minA.z);
	float arealB=(maxB.x-minB.x)*(maxB.y-minB.y)*(maxB.z-minB.z);
	float arealO=o.x*o.y*o.z;
	if(arealO==0)
		return 0;
	return arealO/(arealA+arealB-arealO);
}
