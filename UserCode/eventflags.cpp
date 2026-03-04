#include "eventflags.hpp"

// 信号量低24位有效，其中低9位表示按键有没有被按下过
osEventFlagsId_t flags_id;
void  flags_create(void)
{
    flags_id = osEventFlagsNew(NULL);
}