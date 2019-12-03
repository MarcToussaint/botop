#ifndef RAI_TIMER_MACROS
#define RAI_TIMER_MACROS

#include <chrono>

#define RAI_NOW std::chrono::high_resolution_clock::now()
#define RAI_TIME(start) std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::high_resolution_clock::now() - start).count()
#define RAI_TIME_POINT std::chrono::time_point<std::chrono::high_resolution_clock>

#endif
