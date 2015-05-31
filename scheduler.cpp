#include "scheduler.h"


Task::Task(void (*pf_foo)(int), uint_fast16_t delay) {
  m_bSend           = false;
  m_iDelay          = delay;
  pfTask            = pf_foo;
  m_iTimer          = 0;
  m_iArg            = 0;
}

bool Task::start() {
  if(!m_bSend && pfTask != NULL) {
    pfTask(m_iArg);
    m_bSend = true;
    return true;
  }
  return false;
}

void Task::reset() {
  m_bSend = false;
}

void Task::set_argument(int iArg) {
  m_iArg = iArg;
}

uint_fast32_t Task::get_timer() const {
  return m_iTimer;
}

void Task::set_delay(const uint_fast32_t iDelay) {
  m_iDelay = iDelay;
}

void Task::set_timer(const uint_fast32_t iTimer) {
  m_iTimer = iTimer;
}

uint_fast16_t Task::get_delay() const {
  return m_iDelay;
}
///////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////
Scheduler::Scheduler(const AP_HAL::HAL *p) {
  m_pHAL = p;

  memset(m_functionList, 0, sizeof(m_functionList) );
  memset(m_tickrateList, 0, sizeof(m_tickrateList) );

  m_iItems = 0;
  m_bSuspend = false;
}

void Scheduler::add_task(Task *p, uint_fast32_t iTickRate) {
  if(m_iItems < NO_PRC_SCHED && p != NULL) {
    m_functionList[m_iItems] = p;
    m_tickrateList[m_iItems] = iTickRate;
    m_iItems++;
  }
}

bool Scheduler::is_started(const uint_fast8_t i) {
  Task *pCurTask = m_functionList[i];
  uint_fast32_t time = m_pHAL->scheduler->millis() - pCurTask->get_timer();

  // Time yet to start the current emitter?
  if(time <= m_tickrateList[i] + pCurTask->get_delay() ) {
    return false;
  } else {
    // Release the block for the transmitter
    pCurTask->reset();
  }

  if(pCurTask->start() ) {
    // Set timer to the current time
    pCurTask->set_timer(m_pHAL->scheduler->millis() );
  } else {
    return false;
  }

  return true;
}

void Scheduler::reset_all() {
  // Reset everything if last emitter successfully emitted
  for(uint_fast16_t i = 0; i < m_iItems; i++) {
    m_functionList[i]->reset();
  }
}

void Scheduler::set_arguments(int iArg) {
  // Reset everything if last emitter successfully emitted
  for(uint_fast16_t i = 0; i < m_iItems; i++) {
    m_functionList[i]->set_argument(iArg);
  }
}

void Scheduler::run() {
  if(m_pHAL == NULL || m_bSuspend == true)
    return;

  for(uint_fast8_t i = 0; i < m_iItems; i++) {
    // Run all tasks
    if(!is_started(i) ) {
      continue;
    }
  }
}

void Scheduler::stop() {
  m_bSuspend = true;
}

void Scheduler::resume() {
  m_bSuspend = false;
}
