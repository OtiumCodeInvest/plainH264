#pragma once

#include <stdint.h>
#include <math.h>
#include <vector>
#include "shared/math.h"

#include "atlas.h"

void WriteTextRGB(const V2& position,const char* text,float sizePixels,uint32_t color,uint8_t* pixelsRGB,int width,int height,const Atlas& atlas);
