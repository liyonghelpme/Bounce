#include "b3Time.h"
#include <Windows.h>

u64 b3Time::GetRealTime() {
	u64 ui64Ret;
	::QueryPerformanceCounter(reinterpret_cast<LARGE_INTEGER*>(&ui64Ret));
	return ui64Ret;
}

b3Time::b3Time() :
m_ui64Resolution(B3_ONE_SECOND_MICROSECONDS),
m_ui64CurTime(0ULL),
m_ui64LastTime(0ULL),
m_ui64LastRealTime(0ULL),
m_ui64CurMicros(0ULL),
m_ui64DeltaMicros(0ULL) {
	::QueryPerformanceFrequency(reinterpret_cast<LARGE_INTEGER*>(&m_ui64Resolution));
	m_ui64LastRealTime = GetRealTime();
}

void b3Time::UpdateBy(u64 _ui64Ticks) {
	m_ui64LastTime = m_ui64CurTime;
	m_ui64CurTime += _ui64Ticks;

	u64 ui64LastMicros = m_ui64CurMicros;
	m_ui64CurMicros = m_ui64CurTime * B3_ONE_SECOND_MICROSECONDS / m_ui64Resolution;
	m_ui64DeltaMicros = m_ui64CurMicros - ui64LastMicros;
}

void b3Time::Update() {
	u64 ui64TimeNow = GetRealTime();
	u64 ui64DeltaTime = ui64TimeNow - m_ui64LastRealTime;
	m_ui64LastRealTime = ui64TimeNow;

	UpdateBy(ui64DeltaTime);
}