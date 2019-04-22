#include "ThreadHandler.h"

void simulateWork()
{
    delay(1000);
}

//Configures the interrupt timer ticks in us (1000 gives 1ms).
//This tick time will be the lowest time resolution for thread periods and offsets.
//Setting the value to 0 leaves the Interrupt timers defautl tick, this is usefull since
//this setting will not be in conflict with other libraries and analogWrite.
SET_THREAD_HANDLER_TICK(0);

//This macro configures which timer should be used for generating
//the interrupts driving the ThreadHandler.
//The user can implement there own InterruptTimer by inhereting
//from the ThreadHandler::InterruptTimerInterface. This is usefull
//for adding support for new boards that this library does not support yet.
//Currently the library supports SAMD21 and all version supported by the
//TimerOne library. To see how to do this lock in the "platformSpecificClasses.h".
THREAD_HANDLER(InterruptTimer::getInstance());

//This is the first thread class with its run function
//configured to run every 20s with an offset of 0s and priority 1.
//The scheduling scheme of the ThreadHandler library is as follows:
//    1) Highest priority first.
//    2) If the priority is the same then
//       the thread with the earliest dedline 
//       is executed first.
//    3) If two threads have the same dedline then
//       the first created thread will execute first.
//    4) A thread can only be intrrupted by threads with
//       higher priority.
//    5) Once a thread is executing it will block execution
//       for all threads with lower priority untill the run
//       function returns.
//    6) The loop function has priority -128 compared to
//       ThreadHandler threads.
class TestThread1 : public Thread
{
public:
    TestThread1() : Thread(1, 20000000, 0)
    {
    }

    virtual ~TestThread1()
    {
    }

    virtual void run()
    {
        {
            //The ThreadInterruptBlocker class is used
            //to avoid race conditions between threads.
            //When creating a blocker the interrupt timers
            //interrupt vector is dissabled and when it is
            //destroyd it enables the interrupt again.
            //It does not affect global interrupts, only
            //the interrupt timer. The blocker object also
            //has a lock and an unlock function to be able
            //to dissable/enable interrupts without
            //creating/destroying blocker objects.
            ThreadInterruptBlocker block;
            Serial.println("     ___");
            Serial.println("     |T1|");
        }
        for (int i = 0; i < 3; i++)
        {
            simulateWork();
            ThreadInterruptBlocker block;
            Serial.println("     | #|");
        }
        {
            ThreadInterruptBlocker block;
            Serial.println("      ^^");
        }
    }
};

//This is the second thread class with its run function
//configured to run every 20s with an offset of 6s and priority 1.
//If the offset of TestThread2 would be set to 0s, in this case, TestThread1
//and TestThread2 objects whould be qued up to run directly after each
//other, since they both have the same period time. Now TestThread2 object
//run 6s after TestThread1 objects.
class TestThread2 : public Thread
{
public:
    TestThread2() : Thread(1, 20000000, 6000000)
    {
    }

    virtual ~TestThread2()
    {
    }

    virtual void run()
    {
        {
            ThreadInterruptBlocker block;
            Serial.println("     ___");
            Serial.println("     |T2|");
        }
        for (int i = 0; i < 3; i++)
        {
            simulateWork();
            ThreadInterruptBlocker block;
            Serial.println("     | #|");
        }
        {
            ThreadInterruptBlocker block;
            Serial.println("      ^^");
        }
    }
};

//create the thread objects. When a thread object
//is created it will add it self to the ThreadHandler
//automatically.
TestThread1* testThread1 = new TestThread1();
TestThread2* testThread2 = new TestThread2();

//This creates the third thread object directly without first defining
//a thread type. This thread will run its lambda function
//every 4s with an offset of 0s and priority 2.
Thread* testThread3 = createThread(2, 4000000, 0,
    []()
    {
        {
            ThreadInterruptBlocker block;
            Serial.println("          ___");
            Serial.println("          |T3|");
        }
        for (int i = 0; i < 1; i++)
        {
            simulateWork();
            ThreadInterruptBlocker block;
            Serial.println("          | #|");
        }
        {
            ThreadInterruptBlocker block;
            Serial.println("           ^^");
        }
    });

void setup()
{
    Serial.begin(115200);

    //start executing threads
    ThreadHandler::getInstance()->enableThreadExecution();
}

int loopCount = 0;

//The loop function has priority -128 and will be interrupted
//by all the thread objects
void loop()
{
    {
        ThreadInterruptBlocker block;
        Serial.println("___");
        Serial.println("|LP|");
    }
    for (int i = 0; i < 6; i++)
    {
        simulateWork();
        ThreadInterruptBlocker block;
        Serial.println("| #|");
    }
    {
        ThreadInterruptBlocker block;
        Serial.println(" ^^");
    }
    if (loopCount == 10)
    {
        loopCount = 11;

        //It is also possible to delete threads
        //to stop executing them. However if a thread
        //is supposed to be started and stopped multiple
        //times it is more efficient to just add an if
        //statement first in the run function checking
        //if the function should do any work.
        ThreadInterruptBlocker block;
        delete testThread3;
        Serial.println("T3 deleted");
    }
    else
    {
        loopCount++;
    }
}
