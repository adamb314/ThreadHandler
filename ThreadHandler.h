#ifndef THREAD_HANDLER_H
#define THREAD_HANDLER_H

#include <Arduino.h>
#include <LinkedList.h>

#if !defined(__AVR__)
#undef min
#undef max
#include <vector>
#endif

#include "configuringMacros.h"

class Thread
{
public:
    Thread(int8_t priority, uint32_t period, uint32_t startOffset);

    virtual ~Thread();

    virtual void run() = 0;
};

class ThreadInterruptBlocker
{
public:
    ThreadInterruptBlocker();

    ~ThreadInterruptBlocker();

    void lock();

    void unlock();

private:
    static uint32_t blockerCount;

    bool iAmLocked;
};

class ThreadHandler
{
public:
    static ThreadHandler* getInstance();

    virtual ~ThreadHandler();

    void enableThreadExecution(bool enable = true);

    uint16_t getCpuLoad();

    class InterruptTimerInterface
    {
    public:
        InterruptTimerInterface(){};

        virtual ~InterruptTimerInterface(){};

        virtual void enableNewInterrupt() = 0;

        virtual void blockInterrupts() = 0;
        virtual void unblockInterrupts() = 0;

    protected:
        void interruptRun();
    };

protected:
    ThreadHandler();

    virtual void add(int8_t priority, uint32_t period, uint32_t startOffset, Thread* t);

    virtual void remove(const Thread* t);

    class InternalThreadHolder
    {
    public:
        InternalThreadHolder(int8_t priority, uint32_t period, uint32_t startOffset, Thread* t);

        ~InternalThreadHolder();

        void updateCurrentTime(uint32_t time);

        bool pendingRun();

        bool higherPriorityThan(const InternalThreadHolder* other);
        bool higherPriorityThan(const InternalThreadHolder& other);

        void runThread();

        bool isHolderFor(const Thread* t) const;

        int8_t getPriority() const;

#if !defined(__AVR__)
        static std::vector<InternalThreadHolder*> generateExecutionOrderVector(const std::vector<InternalThreadHolder*>& threadHolders);
#endif

    private:
        Thread* thread;

        bool initiated;
        uint32_t runAtTimestamp;
        int32_t timeUntillRun;
        uint32_t period;
        uint32_t startOffset;
        int8_t priority;
    };

    virtual InternalThreadHolder* getNextThreadToRun(uint32_t currentTimestamp);

    void interruptRun(InterruptTimerInterface* caller);

    InterruptTimerInterface* interruptTimer;

    bool threadExecutionEnabled;
    int8_t priorityOfRunningThread;
    uint32_t cpuLoadTime;
    uint32_t totalTime;
    LinkedList<InternalThreadHolder*> threadHolders;

    friend Thread;
    friend ThreadInterruptBlocker;
    friend ThreadHandler* createAndConfigureThreadHandler();
};

#include "platformSpecificClasses.h"

#endif
