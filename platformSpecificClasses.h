#if !defined(__AVR__)
#undef min
#undef max
#include <algorithm>
class ThreadHandlerExecutionOrderOptimized : public ThreadHandler
{
public:
    virtual ~ThreadHandlerExecutionOrderOptimized();

    virtual void add(Thread* t);

    virtual void remove(const Thread* t);

    virtual void updated(const Thread* t);

private:
    ThreadHandlerExecutionOrderOptimized();

    class ThreadPriorityGroup
    {
    public:
        ThreadPriorityGroup(Thread* t);

        void add(Thread* t);

        void remove(const Thread* th);

        int8_t getPriority();

        void generateExecutionOrder();

        Thread* handleUnoptimizableGroupGetNextToRun(uint32_t currentTimestamp);

        Thread* getNextThreadToRun(uint32_t currentTimestamp);

    private:
        int8_t priority;
        std::vector<Thread*> threads;
        std::vector<Thread*> executeRingBuffer;
        std::vector<Thread*>::iterator itToNextThreadToRun;
    };

    void generateExecutionOrder();

    virtual Thread* getNextThreadToRun(uint32_t currentTimestamp);

    bool executionOrderGenerated;
    std::vector<ThreadPriorityGroup> priorityGroups;

    friend ThreadHandler* createAndConfigureThreadHandler();
};
#endif

#if defined(_SAMD21_)
extern "C"
{
void tc5InterruptRunCaller();
}

class InterruptTimer : public ThreadHandler::InterruptTimerInterface
{
public:
    static InterruptTimer* getInstance();
    virtual ~InterruptTimer();

    static void enableNewInterrupt();

    static void blockInterrupts();
    static void unblockInterrupts();

private:
    InterruptTimer(uint16_t interruptTick);

    void configure(uint16_t period);

    bool isSyncing();

    void startCounter();

    void reset();

    void disable();

    friend void tc5InterruptRunCaller();
};
#elif defined(__AVR__)
#include <TimerOne.h>

void interruptHandler();

class InterruptTimer : public ThreadHandler::InterruptTimerInterface
{
public:
    static InterruptTimer* getInstance();
    virtual ~InterruptTimer();

    static void enableNewInterrupt();

    static void blockInterrupts();
    static void unblockInterrupts();

private:
    InterruptTimer(uint16_t interruptTick);

    friend void interruptHandler();
};
#endif
