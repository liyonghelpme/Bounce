/*
* Copyright (c) 2015-2015 Irlan Robson http://www.irlans.wordpress.com
*
* This software is provided 'as-is', without any express or implied
* warranty.  In no event will the authors be held liable for any damages
* arising from the use of this software.
* Permission is granted to anyone to use this software for any purpose,
* including commercial applications, and to alter it and redistribute it
* freely, subject to the following restrictions:
* 1. The origin of this software must not be misrepresented; you must not
* claim that you wrote the original software. If you use this software
* in a product, an acknowledgment in the product documentation would be
* appreciated but is not required.
* 2. Altered source versions must be plainly marked as such, and must not be
* misrepresented as being the original software.
* 3. This notice may not be removed or altered from any source distribution.
*/

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
