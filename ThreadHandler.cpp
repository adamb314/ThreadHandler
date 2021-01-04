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

CodeBlocksThread::~CodeBlocksThread()
{
    for (Node* it = funListStart; it != nullptr;)
    {
        Node* temp = it;
        it = temp->next;

        delete temp->fun;
        delete temp;
    }
}

void CodeBlocksThread::run()
{
    FunctionalWrapper<>& f = *(nextFunBlockNode->fun);
    f();
    nextFunBlockNode = nextFunBlockNode->next;
    if (nextFunBlockNode == nullptr)
    {
        nextFunBlockNode = funListStart;
    }
}

bool CodeBlocksThread::firstCodeBlock()
{
    return nextFunBlockNode == funListStart;
}

bool CodeBlocksThread::splitIntoCodeBlocks()
{
    return funListStart != funListLast;
}

void CodeBlocksThread::internalDelayNextCodeBlock(int32_t delay)
{
    runAtTimestamp = micros() + delay;
}

void CodeBlocksThread::internalDelayNextCodeBlockUntil(FunctionalWrapper<bool>* fun)
{
    delayCodeBlockUntilFun = fun;
}

void CodeBlocksThread::updateCurrentTime(uint32_t currentTime)
{
    Thread::initiate(currentTime);

    if (delayCodeBlockUntilFun != nullptr &&
        splitIntoCodeBlocks())
    {
        runAtTimestamp = currentTime;
        if ((*delayCodeBlockUntilFun)())
        {
            delayCodeBlockUntilFun = nullptr;
            timeUntillRun = 0;
        }
        else
        {
            runAtTimestamp += 1;
            timeUntillRun = 1;
        }
        return;
    }

    Thread::updateCurrentTime(currentTime);
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
    static InterruptTimer* interruptTimer = nullptr;
    if (interruptTimer == nullptr)
    {
        interruptTimer = InterruptTimer::getInstance();
        if (interruptTimer == nullptr)
        {
            return;
        }
    }

    InterruptTimer::blockInterrupts();
    if (!iAmLocked)
    {
        iAmLocked = true;
        blockerCount++;
    }
}

void ThreadInterruptBlocker::unlock()
{
    static InterruptTimer* interruptTimer = nullptr;
    if (interruptTimer == nullptr)
    {
        interruptTimer = InterruptTimer::getInstance();
        if (interruptTimer == nullptr)
        {
            return;
        }
    }

    if (iAmLocked)
    {
        iAmLocked = false;
        blockerCount--;
    }
    if (blockerCount == 0)
    {
        InterruptTimer::unblockInterrupts();
    }
}

Thread::Thread(int8_t priority, int32_t period, uint32_t startOffset) :
    priority(priority),
    period(period),
    startOffset(startOffset)
{
    ThreadHandler::getInstance()->add(this);
}

Thread::~Thread()
{
    ThreadHandler::getInstance()->remove(this);
}

void Thread::initiate(uint32_t currentTime)
{
    if (!initiated)
    {
        runAtTimestamp = currentTime + startOffset;
        startOffset = runAtTimestamp;
        initiated = true;
    }
}

void Thread::updateCurrentTime(uint32_t currentTime)
{
    initiate(currentTime);

    timeUntillRun = static_cast<int32_t>(runAtTimestamp - currentTime);

    if (timeUntillRun < -(static_cast<int32_t>(1) << 30))
    {
        runAtTimestamp += static_cast<uint32_t>(1) << 29;
    }
}

bool Thread::pendingRun()
{
    return timeUntillRun <= 0;
}

bool Thread::higherPriorityThan(const Thread* other)
{
    if (other == nullptr)
    {
        return true;
    }

    return higherPriorityThan(*other);
}

bool Thread::higherPriorityThan(const Thread& other)
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

void Thread::runThread()
{
    run();
    if (firstCodeBlock())
    {
        runAtTimestamp = startOffset + period;
        startOffset = runAtTimestamp;
    }

}

int8_t Thread::getPriority() const
{
    return priority;
}

void Thread::delayNextCodeBlock(int32_t delay)
{
    ThreadHandler::getInstance()->delayNextCodeBlock(delay);
}

void Thread::delayNextCodeBlockUntil(FunctionalWrapper<bool>* fun)
{
    ThreadHandler::getInstance()->delayNextCodeBlockUntil(fun);
}

uint32_t Thread::getTimingError()
{
    return ThreadHandler::getInstance()->getTimingError();
}

void Thread::internalDelayNextCodeBlock(int32_t delay)
{
}

void Thread::internalDelayNextCodeBlockUntil(FunctionalWrapper<bool>* fun)
{
}

uint32_t Thread::internalGetTimingError()
{
    return micros() - runAtTimestamp;
}

bool Thread::firstCodeBlock()
{
    return true;
}

bool Thread::splitIntoCodeBlocks()
{
    return false;
}

#if !defined(__AVR__)
std::vector<Thread*> Thread::generateExecutionOrderVector(const std::vector<Thread*>& threads)
{
    std::vector<Thread*> executeRingBuffer;

    if (threads.size() == 1)
    {
        executeRingBuffer.push_back(threads[0]);
    }
    else
    {
        std::vector<uint32_t> executeTimeRingBuffer;

        bool threadSplitIntoCodeBlocksExists = false;

        for (auto& th : threads)
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
                Thread* nextToExecute = threads[0];
                for (auto& th : threads)
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
                for (auto& th : threads)
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
                else if (executeRingBuffer.size() > 5 * threads.size())
                {
                    executeRingBuffer.clear();
                    break;
                }
            }
        }
    }

    for (auto& th : threads)
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

ThreadHandler::ThreadHandler()
{
}

ThreadHandler::~ThreadHandler()
{
}

void ThreadHandler::add(Thread* t)
{
    if (firstThread == nullptr)
    {
        firstThread = t;
    }
    else
    {
        Thread* insertPoint = nullptr;
        for (Thread* it = firstThread; it != nullptr; it = it->next)
        {
            if (t->getPriority() > it->getPriority())
            {
                break;
            }
            insertPoint = it;
        }

        if (insertPoint != nullptr)
        {
            t->previous = insertPoint;
            t->next = insertPoint->next;

            insertPoint->next = t;
            if (t->next != nullptr)
            {
                t->next->previous = t;
            }
        }
        else
        {
            t->previous = nullptr;
            t->next = firstThread;

            t->next->previous = t;

            firstThread = t;
        }
    }
}

void ThreadHandler::remove(const Thread* t)
{
    if (firstThread == t)
    {
        firstThread = t->next;

        t->next->previous = t->previous;
    }
    else if (lastThread == t)
    {
        lastThread = t->previous;

        t->previous->next = t->next;
    }
    else
    {
        t->previous->next = t->next;
        t->next->previous = t->previous;
    }
}

void ThreadHandler::updated(const Thread* t)
{
}

Thread* ThreadHandler::getNextThreadToRun(uint32_t currentTimestamp)
{
    Thread* threadToRun = nullptr;

    for (Thread* it = firstThread; it != nullptr; it = it->next)
    {
        Thread& th = *it;

        if (th.getPriority() <= priorityOfRunningThread)
        {
            continue;
        }

        th.updateCurrentTime(currentTimestamp);

        if (th.pendingRun())
        {
            if (th.higherPriorityThan(threadToRun))
            {
                threadToRun = &th;
            }
        }
    }

    return threadToRun;
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
    currentThread->internalDelayNextCodeBlock(delay);
}

void ThreadHandler::delayNextCodeBlockUntil(FunctionalWrapper<bool>* fun)
{
    currentThread->internalDelayNextCodeBlockUntil(fun);
}

uint32_t ThreadHandler::getTimingError()
{
    return currentThread->internalGetTimingError();
}

#if !defined(__AVR__)
ThreadHandlerExecutionOrderOptimized::ThreadPriorityGroup::ThreadPriorityGroup(Thread* t) :
    priority(t->getPriority())
{
    threads.push_back(t);
}

void ThreadHandlerExecutionOrderOptimized::ThreadPriorityGroup::add(Thread* t)
{
    threads.push_back(t);
}

void ThreadHandlerExecutionOrderOptimized::ThreadPriorityGroup::remove(const Thread* th)
{
    threads.erase(std::remove(threads.begin(), threads.end(), th),
        threads.end());
}

int8_t ThreadHandlerExecutionOrderOptimized::ThreadPriorityGroup::getPriority()
{
    return priority;
}

void ThreadHandlerExecutionOrderOptimized::ThreadPriorityGroup::generateExecutionOrder()
{
    executeRingBuffer = Thread::generateExecutionOrderVector(threads);

    itToNextThreadToRun = executeRingBuffer.begin();
}

Thread* ThreadHandlerExecutionOrderOptimized::ThreadPriorityGroup::handleUnoptimizableGroupGetNextToRun(uint32_t currentTimestamp)
{
    Thread* threadToRun = nullptr;

    for (auto th : threads)
    {
        th->updateCurrentTime(currentTimestamp);

        if (th->pendingRun())
        {
            if (th->higherPriorityThan(threadToRun))
            {
                threadToRun = th;
            }
        }
    }

    return threadToRun;
}

Thread* ThreadHandlerExecutionOrderOptimized::ThreadPriorityGroup::getNextThreadToRun(uint32_t currentTimestamp)
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


void ThreadHandlerExecutionOrderOptimized::add(Thread* t)
{
    executionOrderGenerated = false;
    ThreadHandler::add(t);
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

    for (int i = 0; i < threads.size(); i++)
    {
        Thread& th = *(reinterpret_cast<Thread*>(threads.get(i)));

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
                priorityGroups.insert(it, ThreadPriorityGroup(&th));
                break;
            }
        }
        if (it == priorityGroups.end())
        {
            priorityGroups.insert(it, ThreadPriorityGroup(&th));
        }
    }

    for (auto& pg : priorityGroups)
    {
        pg.generateExecutionOrder();
    }
}

Thread* ThreadHandlerExecutionOrderOptimized::getNextThreadToRun(uint32_t currentTimestamp)
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

        Thread* th = pg.getNextThreadToRun(currentTimestamp);

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
    InterruptTimer::enableNewInterrupt();

    if (!threadExecutionEnabled)
    {
        return;
    }

    static uint32_t startTime = millis();
    uint32_t loadStartTime = startTime;
    uint32_t endTime;

    if (priorityOfRunningThread == -128)
    {
        loadStartTime = millis();
    }

    bool threadExecuted = false;
    
    uint32_t currentTimestamp = micros();

    while (true)
    {
        Thread* threadToRun = getNextThreadToRun(currentTimestamp);

        if (threadToRun != nullptr)
        {
            threadExecuted = true;
            int8_t temp = priorityOfRunningThread;
            auto temp2 = currentThread;
            priorityOfRunningThread = threadToRun->getPriority();
            currentThread = threadToRun;
            blocker.unlock();

            threadToRun->runThread();

            blocker.lock();
            currentThread = temp2;
            priorityOfRunningThread = temp;
        }
        if (threadToRun == nullptr)
        {
            if (priorityOfRunningThread == -128)
            {
                endTime = millis();
                int timeDiff = static_cast<int>(endTime - startTime);
                int loadTimeDiff = static_cast<int>(endTime - loadStartTime);
                startTime = endTime;

                totalTime += timeDiff;
                if (threadExecuted)
                {
                    cpuLoadTime += loadTimeDiff;
                }

                if (totalTime > 2000)
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
void tc5InterruptRunCaller()
{
    InterruptTimer::getInstance()->interruptRun();
}

__attribute__((naked))
void TC5_Handler()
{
    asm volatile(
    // Now we are in Handler mode, using Main Stack, and
    // SP should be Double word aligned
    "  PUSH {R4, LR}       \n"// Need to save LR in stack, keep double word alignment
    );

    TC5->COUNT16.INTFLAG.reg = TC_INTFLAG_OVF;
    NVIC_DisableIRQ(TC5_IRQn);

    asm volatile(
    "  SUB  SP, SP , #0x20 \n"// Reserve 8 words for dummy stack frame for return
    "  MOV  R2, SP\n"
    "  LDR  R3,=SysTick_Handler_thread_pt\n"
    "  STR  R3,[R2, #24]   \n"// Set return address as SysTick_Handler_thread_pt
    "  LDR  R3,=0x01000000 \n"// Initial xPSR when running Reentrant_SysTick_Handler
    "  STR  R3,[R2, #28]   \n"// Put in new created stack frame
    "  LDR  R3,=0xFFFFFFF9 \n"// Return to Thread with Main Stack
    "  BX   R3             \n"// Exception return with new created stack frame
    "SysTick_Handler_thread_pt:\n"
    );

    NVIC_EnableIRQ(TC5_IRQn);

    asm volatile(
    "  BL   tc5InterruptRunCaller \n"// Call real ISR in thread mode
    "  SVC  0              \n"// Use SVC to return to original Thread
    "  B    .              \n"// Should not return here
    "  .align 4"
    );
}

// SVC handler - restore stack
//__attribute__((naked))
void SVC_Handler(void)
{
    asm volatile(
    "  MOVS   r0, #4\n"
    "  MOV    r1, LR\n"
    "  TST    r0, r1\n"
    "  BEQ    stacking_used_MSP\n"
    "  MRS    R0, PSP \n"// first parameter - stacking was using PSP
    "  B      get_SVC_num\n"
    "stacking_used_MSP:\n"
    "  MRS    R0, MSP \n"// first parameter - stacking was using MSP
    "get_SVC_num:\n"
    "  LDR     R1, [R0, #24]  \n"// Get stacked PC
    "  SUB    R1, R1, #2\n"
    "  LDRB    R0, [R1, #0]   \n"// Get SVC parameter at stacked PC minus 2
    "  CMP     R0, #0\n"
    "  BEQ     svc_service_0\n"
    "  BL      Unknown_SVC_Request\n"
    "  BX      LR \n"// return
    "svc_service_0:\n"
    // SVC service 0
    // Reentrant code finished, we can discard the current stack frame
    // and restore the original stack frame.
    "  ADD     SP, SP, #0x20\n"
    "  POP     {R4, PC} \n"// Return
    "  .align 4\n"
    );
}
//--------------------------------------
void Unknown_SVC_Request(unsigned int svc_num)
{  //Display Error Message when SVC service is not known
    printf("Error: Unknown SVC service request %d\n", svc_num);
    while(1);
}

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
