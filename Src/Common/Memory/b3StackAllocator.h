#ifndef __B3_STACK_ALLOCATOR_H__
#define __B3_STACK_ALLOCATOR_H__

#include "..\b3Settings.h"

// Allocate 5MB. 
// Increase as you want.
const u32 b3_maxStackSize = 5 * 1024 * 1024;

class b3StackAllocator {
public :
	// This is a general purpose stack allocator.
	// We can create how many entries if we want
	// as long the memory size stays below b3_maxStackSize.
	// If there is no stack memory left then we 
	// allocate using the general purpose allocator.
	b3StackAllocator();
	~b3StackAllocator();

	// Push.
	void* Allocate(u32 size);
	/// Pop.
	void Free(void* p);
protected :
	struct b3Entry {
		u32 size;
		u8* data;
		bool general;
	};
	
	b3Entry* m_entries;
	u32 m_entryCount;
	u32 m_entryCapacity;

	u8 m_memory[b3_maxStackSize];
	u32 m_allocatedSize;
};

#endif