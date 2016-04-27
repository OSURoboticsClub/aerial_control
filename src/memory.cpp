#include <cstdio>
#include <cstdlib>
#include <ch.h>

void *operator new(std::size_t size) {
  return chHeapAlloc(NULL, size);
}

void operator delete(void *p) {
  chHeapFree(p);
}
