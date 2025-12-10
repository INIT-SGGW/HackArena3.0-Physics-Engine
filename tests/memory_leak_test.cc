#ifdef _WIN32
#define _CRTDBG_MAP_ALLOC
#include <crtdbg.h>
#include <stdlib.h>
#endif
void test() { int* p = new int[10]; }
int main() {
  test();

#ifdef _WIN32
  _CrtSetReportMode(_CRT_WARN, _CRTDBG_MODE_DEBUG);
  _CrtDumpMemoryLeaks();
#endif
}