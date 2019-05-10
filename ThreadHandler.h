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

class FunctionalWrapper
{
public:
    FunctionalWrapper(){};
    ~FunctionalWrapper(){};

    virtual void operator()() = 0;
};

template <typename F>
class FunctionalWrapperTemplate : public FunctionalWrapper
{
public:
    FunctionalWrapperTemplate(F fun) :
        fun(fun)
    {
    }

    ~FunctionalWrapperTemplate(){};

    virtual void operator()()
    {
        fun();
    }

private:
    F fun;
};

class Thread
{
public:
    Thread(int8_t priority, int32_t period, uint32_t startOffset);

    virtual ~Thread();

    virtual void run() = 0;

    virtual bool firstCodeBlock()
    {
        return true;
    }

    virtual bool splitIntoCodeBlocks()
    {
        return false;
    }

    static void delayNextCodeBlock(int32_t delay);

private:
    Thread(const Thread&) = delete;
    Thread& operator=(const Thread&) = delete;
};

template <typename F>
class FunctionThreadTemplate : public Thread
{
public:
    FunctionThreadTemplate(int8_t priority, int32_t period, uint32_t startOffset, F fun) :
        Thread(priority, period, startOffset),
        fun(fun)
    {
    }

    virtual ~FunctionThreadTemplate()
    {
    }

    virtual void run()
    {
        fun();
    }

private:
    F fun;
};

template <typename F>
FunctionThreadTemplate<F>* createThread(int8_t priority, int32_t period, uint32_t startOffset, F fun)
{
    return new FunctionThreadTemplate<F>(priority, period, startOffset, fun);
}

class CodeBlocksThread : public Thread
{
public:
    template <typename F>
    CodeBlocksThread(int8_t priority, int32_t period, uint32_t startOffset, F fun);

    virtual ~CodeBlocksThread();

    template <typename F>
    void addCodeBlock(F fun);

    virtual void run();

    virtual bool firstCodeBlock();

    virtual bool splitIntoCodeBlocks();

private:
    size_t nextFunBlockIndex;
    LinkedList<void*> funList;
};

template <typename F>
CodeBlocksThread* createThreadWithCodeBlocks(int8_t priority, int32_t period, uint32_t startOffset, F fun)
{
    return new CodeBlocksThread(priority, period, startOffset, fun);
}

class ThreadInterruptBlocker
{
public:
    ThreadInterruptBlocker();

    ~ThreadInterruptBlocker();

    void lock();

    void unlock();

private:
    static unsigned int blockerCount;

    bool iAmLocked;
};

class ThreadHandler
{
public:
    static ThreadHandler* getInstance();

    virtual ~ThreadHandler();

    void enableThreadExecution(bool enable = true);

    uint16_t getCpuLoad();

    void delayNextCodeBlock(int32_t delay);

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

private:
    ThreadHandler(const ThreadHandler&) = delete;
    ThreadHandler& operator=(const ThreadHandler&) = delete;

protected:
    ThreadHandler();

    virtual void add(int8_t priority, int32_t period, uint32_t startOffset, Thread* t);

    virtual void remove(const Thread* t);

    virtual void updated(const Thread* t);

    class InternalThreadHolder
    {
    public:
        InternalThreadHolder(int8_t priority, int32_t period, uint32_t startOffset, Thread* t);

        ~InternalThreadHolder();

        void updateCurrentTime(uint32_t time);

        bool pendingRun();

        bool higherPriorityThan(const InternalThreadHolder* other);
        bool higherPriorityThan(const InternalThreadHolder& other);

        void runThread();

        bool isHolderFor(const Thread* t) const;

        int8_t getPriority() const;

        void delayNextCodeBlock(int32_t delay);

        bool firstCodeBlock();

        bool splitIntoCodeBlocks();

#if !defined(__AVR__)
        static std::vector<InternalThreadHolder*> generateExecutionOrderVector(const std::vector<InternalThreadHolder*>& threadHolders);
#endif

    private:
        Thread* thread;

        bool initiated;
        uint32_t runAtTimestamp;
        int32_t timeUntillRun;
        int32_t period;
        uint32_t startOffset;
        int8_t priority;
    };

    virtual InternalThreadHolder* getNextThreadToRun(uint32_t currentTimestamp);

    void interruptRun(InterruptTimerInterface* caller);

    InterruptTimerInterface* interruptTimer;

    bool threadExecutionEnabled;
    InternalThreadHolder* currentInternalThreadHolder;
    int8_t priorityOfRunningThread;
    unsigned int cpuLoadTime;
    unsigned int totalTime;
    LinkedList<void*> threadHolders;

    friend Thread;
    friend CodeBlocksThread;
    friend ThreadInterruptBlocker;
    friend ThreadHandler* createAndConfigureThreadHandler();
};

template <typename F>
CodeBlocksThread::CodeBlocksThread(int8_t priority, int32_t period, uint32_t startOffset, F fun) :
    Thread(priority, period, startOffset)
{
    addCodeBlock<F>(fun);
    nextFunBlockIndex = 0;
}

template <typename F>
void CodeBlocksThread::addCodeBlock(F fun)
{
    funList.add(new FunctionalWrapperTemplate<F>(fun));
    ThreadHandler::getInstance()->updated(this);
}

#include "platformSpecificClasses.h"

#endif
