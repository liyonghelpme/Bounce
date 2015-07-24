/*
******************************************************************************
   Copyright (c) 2015 Irlan Robson http://www.irlanengine.wordpress.com

   This software is provided 'as-is', without any express or implied
   warranty. In no event will the authors be held liable for any damages
   arising from the use of this software.

   Permission is granted to anyone to use this software for any purpose,
   including commercial applications, and to alter it and redistribute it
   freely, subject to the following restrictions:

   1. The origin of this software must not be misrepresented; you must not
	 claim that you wrote the original software. If you use this software
	 in a product, an acknowledgment in the product documentation would be
	 appreciated but is not required.
   2. Altered source versions must be plainly marked as such, and must not
	 be misrepresented as being the original software.
   3. This notice may not be removed or altered from any source distribution.
*******************************************************************************
*/

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
