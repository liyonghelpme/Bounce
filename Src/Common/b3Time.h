#ifndef __B3_TIME_H__
#define __B3_TIME_H__

#include "b3Settings.h"

// This is used to define the simulation configuration.
struct b3TimeStep {
	r32 dt;
	u32 velocityIterations;
	bool sleeping;
};

// Call b3Scene::GetProfile() to inspect the time
// spend for running a single simulation module.
struct b3StepProfile {
	r32 broadPhaseTime;
	r32 narrowPhaseTime;
	r32 solverTime;
	r32 totalTime;
};

// Convert microseconds to seconds.
inline r32 MicrosToSecs(r32 us) {
	return (B3_ONE / r32(B3_ONE_SECOND_MICROSECONDS)) * us;
}

// A (very) precise time class.
class b3Time {
public :
	b3Time();
	
	static u64 GetRealTime();

	// Set B3_ONE_SECOND_MICROSECONDS to use as a normal timer.
	void SetResolution(u64 _ui64Resolution);
	
	u64 GetCurMicros() const;
	u64 GetDeltaMicros() const;
	r32 GetCurSecs() const;
	r32 GetDeltaSecs() const;

	void Update();
	void UpdateBy(u64 _ui64Delta);
protected :
	u64 m_ui64Resolution;
	u64 m_ui64LastRealTime;
	u64 m_ui64CurTime;
	u64 m_ui64LastTime;
	u64 m_ui64CurMicros;
	u64 m_ui64DeltaMicros;
};

inline void b3Time::SetResolution(u64 _ui64Resolution) {
	m_ui64Resolution = _ui64Resolution;
}

inline u64 b3Time::GetCurMicros() const { return m_ui64CurMicros; }

inline u64 b3Time::GetDeltaMicros() const { return m_ui64DeltaMicros; }

inline r32 b3Time::GetCurSecs() const { return MicrosToSecs(r32(m_ui64CurMicros)); }

inline r32 b3Time::GetDeltaSecs() const { return MicrosToSecs(r32(m_ui64DeltaMicros)); }

#endif