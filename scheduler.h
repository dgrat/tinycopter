#ifndef SCHEDULER_h
#define SCHEDULER_h

#include <stdint.h>
#include <stddef.h>

#include <AP_HAL.h>
#include <AP_HAL_AVR.h>

#include "config.h"


///////////////////////////////////////////////////////////
// Container for tasks
///////////////////////////////////////////////////////////
class Task {
private:
  bool m_bSend;
  uint_fast32_t m_iTimer;                   // Timer variable
  uint_fast16_t m_iDelay;                   // Certain delay which is added to the tick rate
  void (*pfTask)(int);                         // function pointer
  int m_iArg;

public:
  Task(void (*pf_foo)(int), uint_fast16_t delay = 0);

  bool start();
  void reset();
  void set_argument(int);

  uint_fast16_t get_delay() const;
  void set_delay(const uint_fast32_t iDelay);

  uint_fast32_t get_timer() const;
  void set_timer(const uint_fast32_t iTimer);
};

///////////////////////////////////////////////////////////
// Simple task managemant
///////////////////////////////////////////////////////////
class Scheduler {
private:
  const AP_HAL::HAL *m_pHAL;

  uint_fast8_t   m_iItems;                        // Current number of items in the arrays below
  Task*          m_functionList[NO_PRC_SCHED];    // function list
  uint_fast32_t  m_tickrateList[NO_PRC_SCHED];    // tick rates are intervals e.g.: Call rate is 100 ms + delay[ms]*multiplier
  bool           m_bSuspend;

protected:
  bool is_started(const uint_fast8_t iInd);

public:
  Scheduler(const AP_HAL::HAL *);

  void add_task(Task *pTask, uint_fast32_t iTickRate);
  void reset_all();
  void set_arguments(int);

  void run();     // run all tasks in the list
  void stop();    // stop running tasks
  void resume();  // resume running tasks
};

#endif
