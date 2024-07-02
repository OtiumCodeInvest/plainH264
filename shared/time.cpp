#include "time.h"
#include <string>
#include <cstring>
#include <chrono>
#include "output.h"

uint64_t GetTimer() {
	auto t=std::chrono::steady_clock::now().time_since_epoch();
	return std::chrono::duration_cast<std::chrono::microseconds>(t).count();
}

double Time::GetTime() {
	static Time s_time;
	if(!s_time.m_frequency) {
		s_time.m_frequency=1000000;
		s_time.m_start=GetTimer();
	}
	uint64_t lHDTicks=GetTimer();
	return ((double)(lHDTicks-s_time.m_start)/(double)(s_time.m_frequency));
}

uint64_t GetTimerU64() {
	uint64_t microsecondsUTC=std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
	return microsecondsUTC;
}
uint64_t GetTimeEpochMicroseconds() {
	return GetTimerU64();
}
uint64_t GetTimeEpochMilliseconds(){
	return GetTimerU64()/1000L;
}
uint64_t ElapsedMilliseconds(uint64_t t) {
	return ElapsedMicroseconds(t)/1000;
}

uint64_t ElapsedMicroseconds(uint64_t t) {
	return GetTimer()-t;
}

uint64_t TimersGetHDTicks() {
	uint64_t ticks=GetTimerU64();
	return ticks;
}

uint64_t TimersGetHDTicksSecond() {
	uint64_t frequency=1000000;
	return frequency;
}

//Profiler
Profiler::Profiler() {
	m_tempTimers=new TempTimer[MAX_NUMBER_DRAW_TIMERS];
	m_tempTimersPrev=new TempTimer[MAX_NUMBER_DRAW_TIMERS];
	memset((char*)m_tempTimers,0,sizeof(*m_tempTimers)*MAX_NUMBER_DRAW_TIMERS);
	memset((char*)m_tempTimersPrev,0,sizeof(*m_tempTimersPrev)*MAX_NUMBER_DRAW_TIMERS);
	m_numTempTimers=0;
	m_numTempTimersPrev=0;
	m_rangeMilliseconds=1000;
	memset(m_timers,0,sizeof(m_timers));
	m_depth=0;
	m_writePosition=0;
	m_readPosition=0;
}
Profiler::~Profiler() {
	delete m_tempTimers;
	delete m_tempTimersPrev;
}

void Profiler::PrintDisplayTimers(std::vector<DisplayTimer>& display) {
	for(int i=0;i!=(int)display.size();i++) {
		std::string spaces0(display[i].m_depth,' ');
		uprintf("timer %d %s%s %7.2fms",i,spaces0.c_str(),display[i].m_name.c_str(),display[i].m_time);
	}
}
void Profiler::Reset() {
	m_depth=0;
	m_numTempTimers=0;
	m_readPosition=m_writePosition;
	memset((char*)m_tempTimers,0,sizeof(m_tempTimers));
}
Timer* Profiler::StartTimer(const char* name,uint32_t lColor) {
	Timer* timer=m_timers+(m_writePosition&(MAX_NUMBER_TIMERS-1));
	strncpy(timer->m_name,name,sizeof(timer->m_name)-1);
	timer->m_name[sizeof(timer->m_name)-1]=0;
  	timer->m_color=lColor;
	timer->m_depth=m_depth;
	m_writePosition++;
	m_depth++;
	timer->m_usedTime=0;
	timer->m_core=0;//GetCurrentProcessorNumber();
	timer->m_startTime=TimersGetHDTicks();
	return timer;
}
void Profiler::EndTimer(Timer* timer) {
	if((m_writePosition-m_readPosition)>MAX_NUMBER_TIMERS)return;
	if(timer-m_timers>=MAX_NUMBER_TIMERS)return;
	if(timer->m_startTime!=-1) {
		timer->m_usedTime+=TimersGetHDTicks()-timer->m_startTime;
	}
	m_depth--;
}
void Profiler::StartSubTime(Timer* timer) {
	timer->m_startTime=TimersGetHDTicks();
}
void Profiler::StopSubTime(Timer* timer) {
	timer->m_usedTime+=TimersGetHDTicks()-timer->m_startTime;
	timer->m_startTime=-1;
}
void Profiler::GetDisplayTimers(std::vector<DisplayTimer>& display) {
	//Backup draw timers
	TempTimer tempTimersPrev[MAX_NUMBER_DRAW_TIMERS];
	int numTempTimersPrev=m_numTempTimers;
	memcpy(tempTimersPrev,m_tempTimers,m_numTempTimers*sizeof(TempTimer));
	m_depth=0;
	//Collect drawtimers
	int numTempTimers=0;
	uint8_t displayDepth=0;
	Timer* timer=m_timers+(m_readPosition&(MAX_NUMBER_TIMERS-1));
	if(m_writePosition-m_readPosition<MAX_NUMBER_TIMERS) {
		while(m_readPosition!=m_writePosition) {
			timer=m_timers+(m_readPosition&(MAX_NUMBER_TIMERS-1));
			displayDepth=timer->m_depth;
			TempTimer* temp_timer=0;
			if(numTempTimers) {
				for(int a=numTempTimers-1;a>0;a--) {
					if(m_tempTimers[a].m_depth<displayDepth)break;		
					if(m_tempTimers[a].m_depth==displayDepth && !strcmp(m_tempTimers[a].m_name,timer->m_name)) {
						temp_timer=m_tempTimers+a;
						temp_timer->m_count++;
						break;
					}
				}
			}
			if(!temp_timer) {
				temp_timer=m_tempTimers+numTempTimers++;
				temp_timer->m_time=0;
				temp_timer->m_count=1;
				temp_timer->m_core=0;
			}
			temp_timer->m_time+=timer->m_usedTime;
			temp_timer->m_color=timer->m_color;
			temp_timer->m_depth=displayDepth;
			temp_timer->m_core|=1<<timer->m_core;
			temp_timer->m_name=timer->m_name;
			m_readPosition++;
			if(numTempTimers==MAX_NUMBER_DRAW_TIMERS)break;
		}
	}
	m_numTempTimers=numTempTimers;
	m_readPosition=m_writePosition;
	//Copy values from previus drawtimers
	uint8_t HashLookup[0x100];
	memset(HashLookup,0xff,sizeof(HashLookup));
	for(int a=0;a!=numTempTimersPrev;a++) {
		uint8_t lHash=tempTimersPrev[a].m_name[0];
		tempTimersPrev[a].m_lNextSameHash=HashLookup[lHash];
		HashLookup[lHash]=a;
	}
	for(int a=0;a!=m_numTempTimers;a++) {
		TempTimer* temp_timer=m_tempTimers+a;
		uint8_t lHash=temp_timer->m_name[0];
		uint8_t lIndex=HashLookup[lHash];
		while(lIndex!=0xff) {
			TempTimer* temp_timerOld=tempTimersPrev+lIndex;
			if(temp_timerOld->m_depth==temp_timer->m_depth && !strcmp(temp_timer->m_name,temp_timerOld->m_name)) {
				temp_timer->m_Times=temp_timerOld->m_Times;
				break;
			}
			lIndex=temp_timerOld->m_lNextSameHash;
		}		
		temp_timer->m_Times.Set(temp_timer->m_time);
	}

	int64_t cyclesSecond=TimersGetHDTicksSecond();
	int64_t cyclesMillisecond=cyclesSecond/1000;
	TempTimer tempTimers[MAX_NUMBER_DRAW_TIMERS];
	float rangeMilliseconds=m_rangeMilliseconds;
	if(m_rangeMilliseconds)rangeMilliseconds=m_rangeMilliseconds;
	float range=(float)((double)cyclesSecond/(1000.0f/rangeMilliseconds));
	TempTimer* ptempTimers=m_tempTimersPrev;
	numTempTimers=m_numTempTimers;
	m_numTempTimersPrev=numTempTimers;
	memcpy(ptempTimers,m_tempTimers,numTempTimers*sizeof(TempTimer));
	memcpy(tempTimers,ptempTimers,numTempTimers*sizeof(TempTimer));
	m_NumberTimers.Set(numTempTimers);
	if(numTempTimers) {
		float lBarSizeX=1;
		for(int b=0;b!=numTempTimers;b++) {
			float barEnd=lBarSizeX*(float)((double)tempTimers[b].m_time/(double)range);
			float bar_max=(float)tempTimers[b].m_Times.Max();
			float bar_maxX=lBarSizeX*bar_max/range;
			float time=(float)((double)tempTimers[b].m_time/(double)cyclesMillisecond);
			int64_t sum_children_time=0;
			for(int c=b+1;c<numTempTimers;c++) {
				if(tempTimers[b].m_depth>=tempTimers[c].m_depth)break;
				if(tempTimers[b].m_depth+1==tempTimers[c].m_depth) {
					sum_children_time+=tempTimers[c].m_time;
				}
			}
			DisplayTimer display_timer;
			uint32_t color=tempTimers[b].m_color&0xffffff;
			if(sum_children_time>tempTimers[b].m_time) {
				display_timer.m_bars.push_back({0,barEnd,0x80000000|color});
			}else{
				float lUnaccountedBarSize=lBarSizeX*(float)(((double)(tempTimers[b].m_time-sum_children_time))/(double)range);
				display_timer.m_bars.push_back({0,barEnd-lUnaccountedBarSize,0x80000000|color});
				display_timer.m_bars.push_back({barEnd-lUnaccountedBarSize,barEnd,0x40000000|color});
			}
			display_timer.m_bars.push_back({bar_maxX,bar_maxX,0xff000000|color});
			display_timer.m_depth=tempTimers[b].m_depth;
			display_timer.m_time=time;
			display_timer.m_count=tempTimers[b].m_count;
			display_timer.m_name=tempTimers[b].m_name;
			display_timer.m_core=tempTimers[b].m_core;
			display.push_back(display_timer);
			tempTimers[b].m_time-=sum_children_time;
		}
	}
}

extern "C" {
	Timer* CStartTimer(Profiler* profiler,const char* name,uint32_t color){
		return StartTimer(profiler,name,color);
	}
	void CEndTimer(Timer* timer,Profiler* profiler){
		EndTimer(timer,profiler);
	}
	void CStartSubTimer(Timer* timer,Profiler* profiler){
		StartSubTimer(timer,profiler);
	}
	void CEndSubTimer(Timer* timer,Profiler* profiler){
		EndSubTimer(timer,profiler);
	}
};
