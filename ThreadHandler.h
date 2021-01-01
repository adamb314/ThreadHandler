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

template <typename returnType = void>
class FunctionalWrapper
{
public:
    FunctionalWrapper(){};
    ~FunctionalWrapper(){};

    virtual returnType operator()() = 0;
};

template <typename F, typename returnType = void>
class FunctionalWrapperTemplate : public FunctionalWrapper<returnType>
{
public:
    FunctionalWrapperTemplate(F fun) :
        fun(fun)
    {
    }

    ~FunctionalWrapperTemplate(){};

    virtual returnType operator()()
    {
        return fun();
    }

private:
    F fun;
};

template <typename returnType = void, typename F>
FunctionalWrapper<returnType>* createFunctionalWrapper(F fun)
{
    return new FunctionalWrapperTemplate<F, returnType>(fun);
}

class ThreadHandler;
class ThreadHandlerExecutionOrderOptimized;

class Thread
{
public:
    Thread(int8_t priority, int32_t period, uint32_t startOffset);

    virtual ~Thread();

    virtual void run() = 0;

    virtual bool firstCodeBlock();

    virtual bool splitIntoCodeBlocks();

    static void delayNextCodeBlock(int32_t delay);

    static void delayNextCodeBlockUntil(FunctionalWrapper<bool>* fun);

private:
    Thread(const Thread&) = delete;
    Thread& operator=(const Thread&) = delete;

    void internalDelayNextCodeBlock(int32_t delay);

    void internalDelayNextCodeBlockUntil(FunctionalWrapper<bool>* fun);

    void updateCurrentTime(uint32_t time);

    bool pendingRun();

    bool higherPriorityThan(const Thread* other);
    bool higherPriorityThan(const Thread& other);

    void runThread();

    int8_t getPriority() const;

#if !defined(__AVR__)
    static std::vector<Thread*> generateExecutionOrderVector(const std::vector<Thread*>& threads);
#endif

private:
    bool initiated{false};
    int8_t priority;
    uint32_t runAtTimestamp{0};
    int32_t timeUntillRun{0};
    int32_t period;
    uint32_t startOffset;
    FunctionalWrapper<bool>* delayCodeBlockUntilFun{nullptr};

    Thread* previous{nullptr};
    Thread* next{nullptr};

    friend class ThreadHandler;
    friend class ThreadHandlerExecutionOrderOptimized;
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

    void delayNextCodeBlockUntil(FunctionalWrapper<bool>* fun);

    class InterruptTimerInterface
    {
    public:
        InterruptTimerInterface(){};

        virtual ~InterruptTimerInterface(){};

    protected:
        void interruptRun();
    };

private:
    ThreadHandler(const ThreadHandler&) = delete;
    ThreadHandler& operator=(const ThreadHandler&) = delete;

protected:
    ThreadHandler();

    virtual void add(Thread* t);

    virtual void remove(const Thread* t);

    virtual void updated(const Thread* t);

    virtual Thread* getNextThreadToRun(uint32_t currentTimestamp);

    void interruptRun(InterruptTimerInterface* caller);

    InterruptTimerInterface* interruptTimer{nullptr};

    bool threadExecutionEnabled{false};
    Thread* currentThread{nullptr};
    int8_t priorityOfRunningThread{-128};
    unsigned int cpuLoadTime{0};
    unsigned int totalTime{0};
    Thread* firstThread{nullptr};
    Thread* lastThread{nullptr};

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
