#include <cassert>
#include <iostream>
#include "safety/output_interlock.h"
int main()
{
    safety::OutputInterlock lock;
    assert(!lock.Update(true, false, true)); // 上电拨杆非急停也不能自行运动
    for (int i = 0; i < 10; ++i) assert(!lock.Update(true, true, false));
    assert(!lock.Armed()); // 摇杆不居中，不能解锁
    for (int i = 0; i < 3; ++i) assert(!lock.Update(true, true, true));
    assert(lock.Armed() && lock.Update(true, false, true));
    assert(!lock.Update(true, true, true)); // 急停即时生效
    assert(!lock.Update(false, false, true));
    assert(!lock.Update(true, false, true)); // 失联恢复后仍需手势
    for (int i = 0; i < 3; ++i) lock.Update(true, true, true);
    assert(lock.Update(true, false, true));
    std::cout << "Output interlock: boot, neutral sticks, emergency stop, offline rearm: PASS\n";
}
