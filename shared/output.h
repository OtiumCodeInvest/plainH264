#pragma once

#include <stdint.h>

#ifdef __cplusplus
#include <string>
#include <functional>
extern "C" {
#endif
void Print(const char* format,...);
void Fatal(const char* file,int line,const char* func,const char* format,...);
#ifdef __cplusplus
}
#endif

#define uprintf(...)Print(__VA_ARGS__)

#ifdef _WIN32
#define DEBUGBREAK __debugbreak
#else
#define DEBUGBREAK __builtin_trap
#endif

#define FATAL(...)do{Fatal(__FILE__,__LINE__,__func__,__VA_ARGS__);DEBUGBREAK();}while(0)
#define ASSERT(x,...)do{if(!(x)){Fatal(__FILE__,__LINE__,__func__,__VA_ARGS__);DEBUGBREAK();}}while(0)

typedef void (*TPrintCallbackFunc)(const char* str);
void SetPrintCallback(TPrintCallbackFunc pcb);
