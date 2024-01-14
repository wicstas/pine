#pragma once

#if 1

#include <new>

#else

#ifndef _NEW
#define _NEW
#include <psl/stdint.h>
inline void* operator new(psl::size_t, void* p) {
    return p;
}
inline void* operator new[](psl::size_t, void* p) {
    return p;
}
inline void operator delete(void*, void*) {
}
inline void operator delete[](void*, void*) {
}
#endif

#endif