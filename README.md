# ThreadHandler Library

#### Interrupting scheduling library for cyclic threads

Installing
----------

Download this repository as a .ZIP file and use "Add .ZIP Library" in the Arduino IDE to install it. This library depends on the library TimerOne. It can be installed from "Manage Libraries..." by searching for TimerOne.

Execution demonstration example
------------------------------

This library comes with an example which can be found under "Examples/ThreadHandler/ExecutionDemo" in the Arduino IDE. In this example three threads are created and the serial printout show how the execution of the different threads is split up by the library.

Example output:
```
___
|LP|
| #|
     ___
     |T1|
     | #|
     | #|
          ___
          |T3|
          | #|
           ^^
     | #|
      ^^
| #|
| #|
| #|
 ^^
___
|LP|
| #|
| #|
```
where
LP = main loop function (priority -128), 
T1 = Thread1 (priority 1) and
T3 = Thread3 (priority 2)

Scheduling rules
----------------

The scheduling scheme of the ThreadHandler library is as follows:

1. Highest priority first.
2. If the priority is the same then the thread with the earliest deadline is executed first.
3. If two threads have the same deadline then the first created thread will execute first.
4. A thread can only be interrupted by threads with higher priority.
5. Once a thread is executing it will block execution for all threads with lower priority until the run function returns.
6. The loop function has priority -128 compared to ThreadHandler threads.

How to use
----------

A thread type is created by inheriting the Thread class:

```c++
class MyThread : public Thread
{
public:
    MyThread() : Thread(priority, period, offset){}

    virtual ~MyThread(){}

    virtual void run()
    {
        //code to run
    }
};
```

An instance of a thread type is added to the ThreadHandler automatically when the instance is created:

```c++
MyThread* threadObj = new MyThread();
```

And removed from the ThreadHandler when the thread object is destroyed:

```c++
delete threadObj;
```

This should only be done from loop function or threads with lower priority to not delete a thread that is executing.

Thread objects can also be created directly from lambda functions by using the createThread function:

```c++
Thread* myThread = createThread(priority, period, offset,
    []()
    {
        //code to run
    });
```

Execution of threads is initiated by calling:

```c++
ThreadHandler::getInstance()->enableThreadExecution();
```
And dissabled by calling:

```c++
ThreadHandler::getInstance()->enableThreadExecution(false);
```

See the "ExecutionDemo" example for more info on how to use this library.

Avoiding race conditions, Thread safety
---------------------------------------

To avoid race conditions use the ThreadInterruptBlocker class. When creating a ThreadInterruptBlocker object the interrupt timer interrupt vector is disabled and when it is destroyed it enables the interrupt again. It does not affect global interrupts, only the interrupt timer. The blocker object also has a lock and an unlock function to be able to disable/enable interrupts without creating/destroying blocker objects.

### Example use
```c++
int copyOfCriticalVariable = 0;

{
    ThreadInterruptBlocker blocker;
    copyOfCriticalVariable = criticalVariable;
}

//use copyOfCriticalVariable
```

Avoiding blocking delays
------------------------
This library has no function or system to pause the execution of a thread and allow others to run. The only way for a thread to handover execution is by returning. This means that if a high priority thread calls delay then that delay time will be added to all threads with lower priority as well, since they have to wait for the high priority thread to finish.

Let's say we want a thread to generate the following LED blinking pattern:

```
on
wait 300ms
off
wait 100ms
on
wait 200ms
off
wait 400ms
on
wait 300ms
off
wait 100ms
on
wait 200ms
off
wait 400ms
.
.
.
```

This could be accomplished with a thread like this:

```c++
class MyThread : public Thread
{
public:
    MyThread() : Thread(priority, 1000000, 0){}

    virtual ~MyThread(){}

    virtual void run()
    {
        digitalWrite(13, HIGH);
        delay(300);
        digitalWrite(13, LOW);
        delay(100);
        digitalWrite(13, HIGH);
        delay(200);
        digitalWrite(13, LOW);
    }
};
```
But doing it like this will block all lower priority threads for 600ms every second, since it takes 600ms to execute the thread. This is not desirable. Instead, the recommended way of doing this is by creating a state machine:

```c++
class MyThread : public Thread
{
public:
    MyThread() : Thread(priority, 100000, 0), state(0){}

    virtual ~MyThread(){}

    virtual void run()
    {
        switch (state)
        {
            case 0:
                digitalWrite(13, HIGH);
                break;

            case 3:
                digitalWrite(13, LOW);
                break;

            case 4:
                digitalWrite(13, HIGH);
                break;

            case 6:
                digitalWrite(13, LOW);
                break;

            default:
                break;
        }
        state++;
        if (state >= 10)
        {
            state = 0;
        }
    }

private:
    int state;
};
```

This thread will be expected ten times each second instead of once, but only take a couple of CPU cycles to complete and therefore blocking lower priority threads as little as possible.

Configuring
-----------

To configures the interrupt timer ticks the following macro is used:
```c++
SET_THREAD_HANDLER_TICK(tickTime);
```
where tickTime is in us (1000 gives 1ms). This tick time will be the lowest time resolution for thread periods and offsets. Setting the value to zero leaves the Interrupt timers default tick, this is useful since this setting will not be in conflict with other libraries and analogWrites.

To setup the ThreadHandler the following macro has to be used somewhere in the code.
```c++
THREAD_HANDLER(InterruptTimer::getInstance());
```
This macro configures which timer should be used for generating the interrupts driving the ThreadHandler. The user can implement there own InterruptTimer by inheriting from the ThreadHandler::InterruptTimerInterface. This is useful for adding support for new boards that this library does not support.

Extra optimization for ARM based boards
----------------------------------------

For boards with c++ STL support there is an alternative configuration option.
```c++
THREAD_HANDLER_WITH_EXECUTION_ORDER_OPTIMIZED(InterruptTimer::getInstance());
```
This configures the ThreadHandler to precalculate the execution order of all threads with the same priority and stores it. This changes how the CPU load from the thread scheduling scales with number of threads. The default configuration will search through all threads before calling run on one. This means that the overhead before calling the right thread scales by O(n) where n is the number of threads. With optimization active the scheduler only need to check the unique number of thread priority groups, which means that it instead scales by O(p) where p is the number of unique priorities.

The downside to the optimization is that it requires more memory since the precalculated execution order has to be stored. When creating and destroying threads after enabling thread execution by calling
```c++
ThreadHandler::getInstance()->enableThreadExecution();
```
an extra computational load will be added since the execution order has to be regenerated. It is therefore recommended to dissable thread execution, before creating/destroying threads, by calling
```c++
ThreadHandler::getInstance()->enableThreadExecution(false);
```

## Supported boards
Currently the library supports SAMD21 boards and all AVR based boards supported by the TimerOne library. 

However, the user can implement there own InterruptTimer by inheriting from the ThreadHandler::InterruptTimerInterface to add support for more boards or to gain better control over the timer. To see how to do this lock in the "platformSpecificClasses.h".

### InterruptTimer example implementation

```c++
void interruptHandler();

class InterruptTimer : public ThreadHandler::InterruptTimerInterface
{
public:
    static InterruptTimer* getInstance()
    {
        //the getInterruptTimerTick function returns the tickTime the
        //user configured with the macro SET_THREAD_HANDLER_TICK(tickTime);
        static InterruptTimer interruptTimer(getInterruptTimerTick());
        return &interruptTimer;
    }

private:
    InterruptTimer(uint16_t interruptTick)
    {
        if (interruptTick != 0)
        {
            Timer1.initialize(interruptTick);
        }
        Timer1.attachInterrupt(interruptHandler);
    }

public:
    virtual ~InterruptTimer()
    {
    }

    virtual void enableNewInterrupt()
    {
        //this resets the timer overflow flag
        //and enables a new interrupt to be
        //triggered
        TIFR1 = _BV(TOV1);
    }

    virtual void blockInterrupts()
    {
        //this dissables all interrupts from the timer
        TIMSK1 = 0;
    }

    virtual void unblockInterrupts()
    {
        //this enables the timer overflow interrupt
        TIMSK1 = _BV(TOIE1);
    }

private:
    InterruptTimer(uint16_t interruptTick)
    {
        if (interruptTick != 0)
        {
            Timer1.initialize(interruptTick);
        }
        Timer1.attachInterrupt(interruptHandler);
    }

    friend void interruptHandler();
};

void interruptHandler() {
    //the TimerOne implementation dissables all
    //interrupts before calling the intrrupt handler.
    //We do not want that so calling intterrupts() to
    //enable global interrupts again
    interrupts();

    //calling the protected InterruptTimerInterface::interruptRun
    //function which in turn will call
    //ThreadHandler::interruptRun
    InterruptTimer::getInstance()->interruptRun();
}
```

## License
Open Source License

ThreadHandler is free software. You can redistribute it and/or modify it under the terms of Creative Commons Attribution 3.0 United States License.

To view a copy of this license, visit
http://creativecommons.org/licenses/by/3.0/us/
