extern uint32_t getInterruptTimerTick();

#if defined(_SAMD21_)
extern "C"
{
void tc5InterruptRunCaller();
}

class InterruptTimer
{
public:
    static void initialize();

    static bool isInitialized();

    static void enableNewInterrupt();

    static void blockInterrupts();
    static void unblockInterrupts();

private:
    static void interruptRun();

    static void configure(uint16_t period);

    static bool isSyncing();

    static void startCounter();

    static void reset();

    static void disable();

    static bool initialized;

    friend void tc5InterruptRunCaller();
};
#elif defined(__AVR__)
#include <TimerOne.h>

void interruptHandler();

class InterruptTimer
{
public:
    static void initialize();

    static bool isInitialized();

    static void enableNewInterrupt();

    static void blockInterrupts();
    static void unblockInterrupts();

private:
    static void interruptRun();

    static bool initialized;

    friend void interruptHandler();
};
#endif
