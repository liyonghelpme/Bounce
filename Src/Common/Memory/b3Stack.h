#ifndef __B3_STACK_H__
#define __B3_STACK_H__

#include "..\b3Settings.h"

const u32 b3_stackCapacity = 256;

template <class T> 
class b3Stack {
public :
	b3Stack() {
		// Initially point to the pre-allocated memory. 
		m_elements = m_stackElements;
		m_elementCount = 0;
		m_elementCapacity = b3_stackCapacity;
	}
	
	~b3Stack() {
		if (m_elements != m_stackElements) {
			// Then free the general purpose allocated memory.
			b3Free(m_elements);
			m_elements = nullptr;
		}
	}

	void Push(const T& ele) {
		if (m_elementCount == m_elementCapacity) {
			// Duplicate capacity.
			T* oldElements = m_elements;
			m_elementCapacity *= 2;
			m_elements = (T*)b3Alloc(m_elementCapacity * sizeof(T));
			::memcpy(m_elements, oldElements, m_elementCount * sizeof(T));
			
			if (oldElements != m_stackElements) {
				b3Free(oldElements);
			}
		}

		m_elements[m_elementCount] = ele;
		++m_elementCount;
	}

	T Pop() {
		b3Assert(m_elementCount > 0);
		--m_elementCount;
		return m_elements[m_elementCount];
	}

	const T& Top() const {
		return m_elements[m_elementCount - 1];
	}

	bool Empty() {
		return m_elementCount == 0;
	}
protected :
	T* m_elements;
	T m_stackElements[b3_stackCapacity];
	u32 m_elementCount;
	u32 m_elementCapacity;
};

#endif