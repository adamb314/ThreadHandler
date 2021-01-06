#define SET_THREAD_HANDLER_TICK(InterruptTimerTick) \
uint32_t getInterruptTimerTick() \
{ \
    return InterruptTimerTick; \
}

class ThreadHandler;

ThreadHandler* createAndConfigureThreadHandler();
