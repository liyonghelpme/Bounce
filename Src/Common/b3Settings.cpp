#include "b3Settings.h"

void* b3Alloc(i32 size) {
	return ::malloc(size);
}

void b3Free(void* memory) {
	return ::free(memory);
}
