#define THREAD_HANDLER(InterruptTimer) \
ThreadHandler::InterruptTimerInterface* getInterruptTimerInstance() \
{ \
    return InterruptTimer; \
}

#define THREAD_HANDLER_WITH_EXECUTION_ORDER_OPTIMIZED(InterruptTimer) THREAD_HANDLER(InterruptTimer)

#define SET_THREAD_HANDLER_TICK(InterruptTimerTick) \
uint32_t getInterruptTimerTick() \
{ \
    return InterruptTimerTick; \
}

class ThreadHandler;

ThreadHandler* createAndConfigureThreadHandler();
