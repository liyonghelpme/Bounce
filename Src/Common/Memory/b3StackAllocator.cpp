#include "b3StackAllocator.h"

b3StackAllocator::b3StackAllocator() {
	// Initially allocate 32 entries.
	m_entryCapacity = 32;
	m_entries = (b3Entry*)b3Alloc(m_entryCapacity * sizeof(b3Entry));
	m_entryCount = 0;
	m_allocatedSize = 0;
}

b3StackAllocator::~b3StackAllocator() {
	b3Assert(m_allocatedSize == 0);
	b3Assert(m_entryCount == 0);
}

void* b3StackAllocator::Allocate(u32 size) {
	if (m_entryCount == m_entryCapacity) {
		// Then duplicate capacity if needed.
		b3Entry* oldEntries = m_entries;
		m_entryCapacity *= 2;
		m_entries = (b3Entry*)b3Alloc(m_entryCapacity * sizeof(b3Entry));
		::memcpy(m_entries, oldEntries, m_entryCount * sizeof(b3Entry));
		b3Free(oldEntries);
	}

	b3Entry* entry = m_entries + m_entryCount;
	entry->size = size;
	if (m_allocatedSize + size > b3_maxStackSize) {
		// Pass to the general-purpose allocator.
		entry->data = (u8*) b3Alloc(size);
		entry->general = true;
	}
	else {
		// Use the this stack memory.
		entry->data = m_memory + m_allocatedSize;
		entry->general = false;
		m_allocatedSize += size;
	}
	
	++m_entryCount;

	return entry->data;
}

void b3StackAllocator::Free(void* p) {
	b3Assert(m_entryCount > 0);
	b3Entry* entry = m_entries + m_entryCount - 1;
	b3Assert(entry->data == p);
	if (entry->general) {
		b3Free(p);
	}
	else {
		m_allocatedSize -= entry->size;
	}
	--m_entryCount;
}