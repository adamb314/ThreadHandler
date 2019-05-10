#include "ThreadHandler.h"

extern ThreadHandler::InterruptTimerInterface* getInterruptTimerInstance();
extern bool executionOrderOptimizedThreadHandler();
ThreadHandler* createAndConfigureThreadHandler()
{
    if (executionOrderOptimizedThreadHandler())
    {
#if !defined(__AVR__)
        static ThreadHandlerExecutionOrderOptimized threadHandler;
        return &threadHandler;
#else
        static ThreadHandler threadHandler;
        return &threadHandler;
#endif
    }
    else
    {
        static ThreadHandler threadHandler;
        return &threadHandler;
    }
}

Thread::Thread(int8_t priority, int32_t period, uint32_t startOffset)
{
    ThreadHandler::getInstance()->add(priority, period, startOffset, this);
}

Thread::~Thread()
{
    ThreadHandler::getInstance()->remove(this);
}

void Thread::delayNextCodeBlock(int32_t delay)
{
    ThreadHandler::getInstance()->delayNextCodeBlock(delay);
}

CodeBlocksThread::~CodeBlocksThread()
{
    while (funList.size() != 0)
    {
        delete funList.get(0);
        funList.remove(0);
    }
}

void CodeBlocksThread::run()
{
    FunctionalWrapper& f = *reinterpret_cast<FunctionalWrapper*>(funList.get(nextFunBlockIndex));
    f();
    ++nextFunBlockIndex;
    if (nextFunBlockIndex == funList.size())
    {
        nextFunBlockIndex = 0;
    }
}

bool CodeBlocksThread::firstCodeBlock()
{
    return nextFunBlockIndex == 0;
}

bool CodeBlocksThread::splitIntoCodeBlocks()
{
    return funList.size() > 1;
}

unsigned int ThreadInterruptBlocker::blockerCount = 0;

ThreadInterruptBlocker::ThreadInterruptBlocker() :
    iAmLocked(false)
{
    lock();
}

ThreadInterruptBlocker::~ThreadInterruptBlocker()
{
    unlock();
}

void ThreadInterruptBlocker::lock()
{
    if (ThreadHandler::getInstance()->interruptTimer == nullptr)
    {
        return;
    }

    ThreadHandler::getInstance()->interruptTimer->blockInterrupts();
    if (!iAmLocked)
    {
        iAmLocked = true;
        blockerCount++;
    }
}

void ThreadInterruptBlocker::unlock()
{
     if (ThreadHandler::getInstance()->interruptTimer == nullptr)
    {
        return;
    }

    if (iAmLocked)
    {
        iAmLocked = false;
        blockerCount--;
    }
    if (blockerCount == 0)
    {
        ThreadHandler::getInstance()->interruptTimer->unblockInterrupts();
    }
}

ThreadHandler::InternalThreadHolder::InternalThreadHolder(int8_t priority, int32_t period, uint32_t startOffset, Thread* t) :
    thread(t),
    initiated(false),
    runAtTimestamp(0),
    timeUntillRun(0),
    period(period),
    startOffset(startOffset),
    priority(priority)
{
}

ThreadHandler::InternalThreadHolder::~InternalThreadHolder()
{
}

void ThreadHandler::InternalThreadHolder::updateCurrentTime(uint32_t currnetTime)
{
    if (!initiated)
    {
        runAtTimestamp = currnetTime + startOffset;
        startOffset = runAtTimestamp;
        initiated = true;
    }

    timeUntillRun = static_cast<int32_t>(runAtTimestamp - currnetTime);

    if (timeUntillRun < -(static_cast<int32_t>(1) << 30))
    {
        runAtTimestamp += static_cast<uint32_t>(1) << 29;
    }
}

bool ThreadHandler::InternalThreadHolder::pendingRun()
{
    return timeUntillRun <= 0;
}

bool ThreadHandler::InternalThreadHolder::higherPriorityThan(const InternalThreadHolder* other)
{
    if (other == nullptr)
    {
        return true;
    }

    return higherPriorityThan(*other);
}

bool ThreadHandler::InternalThreadHolder::higherPriorityThan(const InternalThreadHolder& other)
{
    if (other.priority < priority)
    {
        return true;
    }
    else if (other.priority == priority)
    {
        if (other.timeUntillRun > timeUntillRun)
        {
            return true;
        }
    }

    return false;
}

void ThreadHandler::InternalThreadHolder::runThread()
{
    thread->run();
    if (thread->firstCodeBlock())
    {
        runAtTimestamp = startOffset + period;
        startOffset = runAtTimestamp;
    }

}

bool ThreadHandler::InternalThreadHolder::isHolderFor(const Thread* t) const
{
    return thread == t;
}

int8_t ThreadHandler::InternalThreadHolder::getPriority() const
{
    return priority;
}

void ThreadHandler::InternalThreadHolder::delayNextCodeBlock(int32_t delay)
{
    runAtTimestamp = micros() + delay;
}

bool ThreadHandler::InternalThreadHolder::firstCodeBlock()
{
    return thread->firstCodeBlock();
}

bool ThreadHandler::InternalThreadHolder::splitIntoCodeBlocks()
{
    return thread->splitIntoCodeBlocks();
}

#if !defined(__AVR__)
std::vector<ThreadHandler::InternalThreadHolder*> ThreadHandler::InternalThreadHolder::generateExecutionOrderVector(const std::vector<InternalThreadHolder*>& threadHolders)
{
    std::vector<InternalThreadHolder*> executeRingBuffer;

    if (threadHolders.size() == 1)
    {
        executeRingBuffer.push_back(threadHolders[0]);
    }
    else
    {
        std::vector<uint32_t> executeTimeRingBuffer;

        bool threadSplitIntoCodeBlocksExists = false;

        for (auto& th : threadHolders)
        {
            th->runAtTimestamp = th->startOffset;

            if (th->splitIntoCodeBlocks())
            {
                threadSplitIntoCodeBlocksExists = true;
            }
        }

        if (threadSplitIntoCodeBlocksExists)
        {
            executeRingBuffer.clear();
        }
        else
        {
            while (true)
            {
                InternalThreadHolder* nextToExecute = threadHolders[0];
                for (auto& th : threadHolders)
                {
                    if (nextToExecute->runAtTimestamp > th->runAtTimestamp)
                    {
                        nextToExecute = th;
                    }
                }

                executeRingBuffer.push_back(nextToExecute);
                executeTimeRingBuffer.push_back(nextToExecute->runAtTimestamp);
                nextToExecute->runAtTimestamp += nextToExecute->period;

                auto startPatternIterator = executeRingBuffer.begin();
                auto temp = std::find(executeRingBuffer.rbegin(), executeRingBuffer.rend(), executeRingBuffer.front());
                auto endPatternIterator = executeRingBuffer.begin() + std::distance(temp, executeRingBuffer.rend() - 1);

                size_t startPatternIndex = startPatternIterator - executeRingBuffer.begin();
                size_t endPatternIndex = endPatternIterator - executeRingBuffer.begin();

                if (startPatternIndex == endPatternIndex)
                {
                    continue;
                }

                uint32_t startPatternTimeOffset = executeTimeRingBuffer[startPatternIndex];
                uint32_t endPatternTimeOffset = executeTimeRingBuffer[endPatternIndex];

                bool patternMach = true;
                for (auto& th : threadHolders)
                {
                    auto itInStartPattern = std::find(startPatternIterator, executeRingBuffer.end(), th);
                    auto itInEndPattern = std::find(endPatternIterator, executeRingBuffer.end(), th);

                    if (itInStartPattern == executeRingBuffer.end() ||
                        itInEndPattern == executeRingBuffer.end())
                    {
                        patternMach = false;
                        break;
                    }

                    size_t indexInStartPattern = itInStartPattern - startPatternIterator;
                    size_t indexInEndPattern = itInEndPattern - startPatternIterator;

                    if (executeTimeRingBuffer[indexInStartPattern] - startPatternTimeOffset !=
                        executeTimeRingBuffer[indexInEndPattern] - endPatternTimeOffset)
                    {
                        patternMach = false;
                        break;
                    }
                }

                if (patternMach)
                {
                    executeRingBuffer.erase(endPatternIterator, executeRingBuffer.end());
                    break;
                }
                else if (executeRingBuffer.size() > 5 * threadHolders.size())
                {
                    executeRingBuffer.clear();
                    break;
                }
            }
        }
    }

    for (auto& th : threadHolders)
    {
        th->initiated = false;
    }

    return executeRingBuffer;
}
#endif

ThreadHandler* ThreadHandler::getInstance()
{
    return createAndConfigureThreadHandler();
}

ThreadHandler::ThreadHandler() :
    interruptTimer(nullptr),
    threadExecutionEnabled(false),
    currentInternalThreadHolder(nullptr),
    priorityOfRunningThread(-128),
    cpuLoadTime(0),
    totalTime(0)
{
}

ThreadHandler::~ThreadHandler()
{
}

void ThreadHandler::add(int8_t priority, int32_t period, uint32_t startOffset, Thread* t)
{
    threadHolders.add(new InternalThreadHolder(priority, period, startOffset, t));
}

void ThreadHandler::remove(const Thread* t)
{
    for (int i = 0; i < threadHolders.size();)
    {
        if ((reinterpret_cast<InternalThreadHolder*>(threadHolders.get(i)))->isHolderFor(t))
        {
            InternalThreadHolder* th = threadHolders.remove(i);
            delete th;
        }
        else
        {
            i++;
        }
    }
}

void ThreadHandler::updated(const Thread* t)
{
}

ThreadHandler::InternalThreadHolder* ThreadHandler::getNextThreadToRun(uint32_t currentTimestamp)
{
    InternalThreadHolder* threadToRunHolder = nullptr;

    for (int i = 0; i < threadHolders.size(); i++)
    {
        InternalThreadHolder& th = *(reinterpret_cast<InternalThreadHolder*>(threadHolders.get(i)));
        if (th.getPriority() <= priorityOfRunningThread)
        {
            continue;
        }

        th.updateCurrentTime(currentTimestamp);

        if (th.pendingRun())
        {
            if (th.higherPriorityThan(threadToRunHolder))
            {
                threadToRunHolder = &th;
            }
        }
    }

    return threadToRunHolder;
}

void ThreadHandler::enableThreadExecution(bool enable)
{
    if (interruptTimer == nullptr)
    {
        interruptTimer = getInterruptTimerInstance();
    }

    ThreadInterruptBlocker blocker;

    threadExecutionEnabled = enable;
}

uint16_t ThreadHandler::getCpuLoad()
{
    ThreadInterruptBlocker blocker;

    uint16_t out = static_cast<uint16_t>((cpuLoadTime * 1000) / totalTime);
    return out;
}

void ThreadHandler::delayNextCodeBlock(int32_t delay)
{
    currentInternalThreadHolder->delayNextCodeBlock(delay);
}

#if !defined(__AVR__)
ThreadHandlerExecutionOrderOptimized::ThreadHolderPriorityGroup::ThreadHolderPriorityGroup(InternalThreadHolder* t) :
    priority(t->getPriority())
{
    threadHolders.push_back(t);
}

void ThreadHandlerExecutionOrderOptimized::ThreadHolderPriorityGroup::add(InternalThreadHolder* t)
{
    threadHolders.push_back(t);
}

void ThreadHandlerExecutionOrderOptimized::ThreadHolderPriorityGroup::remove(const InternalThreadHolder* th)
{
    threadHolders.erase(std::remove(threadHolders.begin(), threadHolders.end(), th),
        threadHolders.end());
}

int8_t ThreadHandlerExecutionOrderOptimized::ThreadHolderPriorityGroup::getPriority()
{
    return priority;
}

void ThreadHandlerExecutionOrderOptimized::ThreadHolderPriorityGroup::generateExecutionOrder()
{
    executeRingBuffer = InternalThreadHolder::generateExecutionOrderVector(threadHolders);

    itToNextThreadToRun = executeRingBuffer.begin();
}

ThreadHandler::InternalThreadHolder* ThreadHandlerExecutionOrderOptimized::ThreadHolderPriorityGroup::handleUnoptimizableGroupGetNextToRun(uint32_t currentTimestamp)
{
    InternalThreadHolder* threadToRunHolder = nullptr;

    for (auto th : threadHolders)
    {
        th->updateCurrentTime(currentTimestamp);

        if (th->pendingRun())
        {
            if (th->higherPriorityThan(threadToRunHolder))
            {
                threadToRunHolder = th;
            }
        }
    }

    return threadToRunHolder;
}

ThreadHandler::InternalThreadHolder* ThreadHandlerExecutionOrderOptimized::ThreadHolderPriorityGroup::getNextThreadToRun(uint32_t currentTimestamp)
{
    if (executeRingBuffer.size() == 0)
    {
        return handleUnoptimizableGroupGetNextToRun(currentTimestamp);
    }

    auto th = (*itToNextThreadToRun);

    th->updateCurrentTime(currentTimestamp);
    if (th->pendingRun())
    {
        ++itToNextThreadToRun;
        if (itToNextThreadToRun == executeRingBuffer.end())
        {
            itToNextThreadToRun = executeRingBuffer.begin();
        }

        return th;
    }

    return nullptr;
}

ThreadHandlerExecutionOrderOptimized::ThreadHandlerExecutionOrderOptimized() :
    ThreadHandler(),
    executionOrderGenerated(false)
{
}


void ThreadHandlerExecutionOrderOptimized::add(int8_t priority, int32_t period, uint32_t startOffset, Thread* t)
{
    executionOrderGenerated = false;
    ThreadHandler::add(priority, period, startOffset, t);
}

void ThreadHandlerExecutionOrderOptimized::remove(const Thread* t)
{
    executionOrderGenerated = false;
    ThreadHandler::remove(t);
}

void ThreadHandlerExecutionOrderOptimized::updated(const Thread* t)
{
    executionOrderGenerated = false;
    ThreadHandler::updated(t);
}

ThreadHandlerExecutionOrderOptimized::~ThreadHandlerExecutionOrderOptimized()
{
}

void ThreadHandlerExecutionOrderOptimized::generateExecutionOrder()
{
    priorityGroups.clear();

    for (int i = 0; i < threadHolders.size(); i++)
    {
        InternalThreadHolder& th = *(reinterpret_cast<InternalThreadHolder*>(threadHolders.get(i)));

        auto it = priorityGroups.begin();
        for (; it != priorityGroups.end(); ++it)
        {
            if (th.getPriority() == it->getPriority())
            {
                it->add(&th);
                break;
            }
            else if (th.getPriority() > it->getPriority())
            {
                priorityGroups.insert(it, ThreadHolderPriorityGroup(&th));
                break;
            }
        }
        if (it == priorityGroups.end())
        {
            priorityGroups.insert(it, ThreadHolderPriorityGroup(&th));
        }
    }

    for (auto& pg : priorityGroups)
    {
        pg.generateExecutionOrder();
    }
}

ThreadHandler::InternalThreadHolder* ThreadHandlerExecutionOrderOptimized::getNextThreadToRun(uint32_t currentTimestamp)
{
    if (!executionOrderGenerated)
    {
        generateExecutionOrder();
        executionOrderGenerated = true;
    }

    for (auto& pg : priorityGroups)
    {
        if (pg.getPriority() <= priorityOfRunningThread)
        {
            return nullptr;
        }

        InternalThreadHolder* th = pg.getNextThreadToRun(currentTimestamp);

        if (th != nullptr)
        {
            return th;
        }
    }

    return nullptr;
}
#endif

void ThreadHandler::interruptRun(InterruptTimerInterface* caller)
{
    //check so that the InterruptTimerInterface calling
    //is the one configured. This is to protect against
    //the user creating multiple InterruptTimerInterfaces
    //and activating them. Only the InterruptTimerInterface
    //configured is allowed to execute Thread::interruptRun
    if (caller != interruptTimer)
    {
        return;
    }

    ThreadInterruptBlocker blocker;
    interruptTimer->enableNewInterrupt();

    if (!threadExecutionEnabled)
    {
        return;
    }

    static uint32_t startTime = micros();
    uint32_t loadStartTime = startTime;
    uint32_t endTime;

    if (priorityOfRunningThread == -128)
    {
        loadStartTime = micros();
    }

    bool threadExecuted = false;
    
    uint32_t currentTimestamp = micros();

    while (true)
    {

        InternalThreadHolder* threadToRunHolder = getNextThreadToRun(currentTimestamp);

        if (threadToRunHolder != nullptr)
        {
            threadExecuted = true;
            int8_t temp = priorityOfRunningThread;
            auto temp2 = currentInternalThreadHolder;
            priorityOfRunningThread = threadToRunHolder->getPriority();
            currentInternalThreadHolder = threadToRunHolder;
            blocker.unlock();

            threadToRunHolder->runThread();

            blocker.lock();
            currentInternalThreadHolder = temp2;
            priorityOfRunningThread = temp;
        }
        else
        {
            if (priorityOfRunningThread == -128)
            {
                endTime = micros();
                int timeDiff = static_cast<int>(endTime - startTime);
                int loadTimeDiff = static_cast<int>(endTime - loadStartTime);
                startTime = endTime;

                totalTime += timeDiff;
                if (threadExecuted)
                {
                    cpuLoadTime += loadTimeDiff;
                }

                if (totalTime > 1000)
                {
                    totalTime = (totalTime >> 1);
                    cpuLoadTime = (cpuLoadTime >> 1);
                }
            }
            break;
        }
    }
}

void ThreadHandler::InterruptTimerInterface::interruptRun()
{
    ThreadHandler::getInstance()->interruptRun(this);
}

extern uint32_t getInterruptTimerTick();

#if defined(_SAMD21_)
InterruptTimer* InterruptTimer::getInstance()
{
    static InterruptTimer interruptTimer(getInterruptTimerTick());
    return &interruptTimer;
}

InterruptTimer::InterruptTimer(uint16_t interruptTick)
{
    if (interruptTick == 0)
    {
        interruptTick = 1000;
    }
    configure(interruptTick);
    startCounter();
}

InterruptTimer::~InterruptTimer()
{
}

void InterruptTimer::enableNewInterrupt()
{
    TC5->COUNT16.INTFLAG.reg = TC_INTFLAG_OVF;
}

void InterruptTimer::blockInterrupts()
{
    NVIC_DisableIRQ(TC5_IRQn);
}

void InterruptTimer::unblockInterrupts()
{
    NVIC_EnableIRQ(TC5_IRQn);
}

void InterruptTimer::configure(uint16_t period)
{
    if (period != 0)
    {
        // Enable GCLK for TCC2 and TC5 (timer counter input clock)
        GCLK->CLKCTRL.reg = (uint16_t) (GCLK_CLKCTRL_CLKEN | GCLK_CLKCTRL_GEN_GCLK0 | GCLK_CLKCTRL_ID(GCM_TC4_TC5)) ;
        while (GCLK->STATUS.bit.SYNCBUSY);

        disable();

        // Set Timer counter Mode to 16 bits
        TC5->COUNT16.CTRLA.reg |= TC_CTRLA_MODE_COUNT16;
        // Set TC5 mode as match frequency
        TC5->COUNT16.CTRLA.reg |= TC_CTRLA_WAVEGEN_MFRQ;
        //set prescaler and enable TC5
        TC5->COUNT16.CTRLA.reg |= TC_CTRLA_PRESCALER_DIV1;
        //set TC5 timer counter based off of the system clock and the user defined sample rate or waveform
        TC5->COUNT16.CC[0].reg = static_cast<uint16_t>(period * (F_CPU / 1000000ul));
        while (isSyncing());
    }

    // Interrupts 
    TC5->COUNT16.INTENSET.reg = TC_INTENSET_OVF;          // enable overfollow

    // Enable InterruptVector
    NVIC_DisableIRQ(TC5_IRQn);
    NVIC_ClearPendingIRQ(TC5_IRQn);
    NVIC_SetPriority(TC5_IRQn, 3);
    NVIC_EnableIRQ(TC5_IRQn);

    while (isSyncing()); //wait until TC5 is done syncing
}

//Function that is used to check if TC5 is done syncing
//returns true when it is done syncing
bool InterruptTimer::isSyncing()
{
    return TC5->COUNT16.STATUS.reg & TC_STATUS_SYNCBUSY;
}

//This function enables TC5 and waits for it to be ready
void InterruptTimer::startCounter()
{
    TC5->COUNT16.CTRLA.reg |= TC_CTRLA_ENABLE; //set the CTRLA register
    while (isSyncing()); //wait until snyc'd
}

//Reset TC5 
void InterruptTimer::reset()
{
    TC5->COUNT16.CTRLA.reg = TC_CTRLA_SWRST;
    while (isSyncing());
    while (TC5->COUNT16.CTRLA.bit.SWRST);
}

//disable TC5
void InterruptTimer::disable()
{
    TC5->COUNT16.CTRLA.reg &= ~TC_CTRLA_ENABLE;
    while (isSyncing());
}

extern "C"
{
//this function gets called by the TC5 interrupt
void TC5_Handler();
}

void TC5_Handler() {
    InterruptTimer::getInstance()->interruptRun();
}

#elif defined(__AVR__)

InterruptTimer* InterruptTimer::getInstance()
{
    static InterruptTimer interruptTimer(getInterruptTimerTick());
    return &interruptTimer;
}

InterruptTimer::InterruptTimer(uint16_t interruptTick)
{
    if (interruptTick != 0)
    {
        Timer1.initialize(interruptTick);
    }
    Timer1.attachInterrupt(interruptHandler);
}

InterruptTimer::~InterruptTimer()
{
}

void InterruptTimer::enableNewInterrupt()
{
    TIFR1 = _BV(TOV1);
}

void InterruptTimer::blockInterrupts()
{
    TIMSK1 = 0;
}

void InterruptTimer::unblockInterrupts()
{
    TIMSK1 = _BV(TOIE1);
}

void interruptHandler() {
    interrupts();
    InterruptTimer::getInstance()->interruptRun();
}

#endif
