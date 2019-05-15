#if !defined(__AVR__)
#undef min
#undef max
#include <algorithm>
class ThreadHandlerExecutionOrderOptimized : public ThreadHandler
{
public:
    virtual ~ThreadHandlerExecutionOrderOptimized();

    virtual void add(int8_t priority, int32_t period, uint32_t startOffset, Thread* t);

    virtual void remove(const Thread* t);

    virtual void updated(const Thread* t);

private:
    ThreadHandlerExecutionOrderOptimized();

    class ThreadHolderPriorityGroup
    {
    public:
        ThreadHolderPriorityGroup(InternalThreadHolder* t);

        void add(InternalThreadHolder* t);

        void remove(const InternalThreadHolder* th);

        int8_t getPriority();

        void generateExecutionOrder();

        InternalThreadHolder* handleUnoptimizableGroupGetNextToRun(uint32_t currentTimestamp);

        InternalThreadHolder* getNextThreadToRun(uint32_t currentTimestamp);

    private:
        int8_t priority;
        std::vector<InternalThreadHolder*> threadHolders;
        std::vector<InternalThreadHolder*> executeRingBuffer;
        std::vector<InternalThreadHolder*>::iterator itToNextThreadToRun;
    };

    void generateExecutionOrder();

    virtual InternalThreadHolder* getNextThreadToRun(uint32_t currentTimestamp);

    bool executionOrderGenerated;
    std::vector<ThreadHolderPriorityGroup> priorityGroups;

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

    virtual void enableNewInterrupt();

    virtual void blockInterrupts();
    virtual void unblockInterrupts();

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

    virtual void enableNewInterrupt();

    virtual void blockInterrupts();
    virtual void unblockInterrupts();

private:
    InterruptTimer(uint16_t interruptTick);

    friend void interruptHandler();
};
#endif
