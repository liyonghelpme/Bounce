#include "b3Shape.h"
#include "..\..\Dynamics\b3Body.h"

void b3Shape::SetSensor(bool flag) {
	if (flag != m_isSensor) {
		if (m_body) {
			m_body->SetAwake(true);
		}
		m_isSensor = flag;
	}
}