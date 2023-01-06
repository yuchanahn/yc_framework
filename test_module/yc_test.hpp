#pragma once
#include <iostream>
#include <chrono>
#define CPU_Time(filed, cnt, name){\
std::chrono::system_clock::time_point s = std::chrono::system_clock::now();\
for(int i__ = 0; i__ < cnt; ++i__){\
filed\
}\
std::cout << "[" << name <<"] elapsed time: " << std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::system_clock::now() - s).count() << "mc\n";}\

namespace yc::test {
    
    
}