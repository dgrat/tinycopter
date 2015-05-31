#ifndef NAVIG_h
#define NAVIG_h

#include "containers.h"
#include "config.h"

class Device;
class Receiver;


class ExeptionDevice {
private:
  bool         m_bRcvrLock;

protected /*variables*/:
  Receiver*    m_pReceiver;
  Device*      m_pHalBoard;

  int_fast32_t m_rgChannelsRC[APM_IOCHAN_CNT];                // Array:  Override remote control if certain exceptions happen

protected /*functions*/:
  /*
   * Saves the current remote control command one time.
   * write_recvr() must be called before, this function is usable again.
   * The parameter is a reference to a lock which will set to true if it was false and then the function executes.
   * If the parameter is already true, the function will do nothing.
   */
  bool read_recvr();  // Saves the current remote control command one time to m_rgChannelsRC to have a changeable copy.
  bool write_recvr(); // Writes the current remote control command back to the receiver.

  void dsbl_althld_recvr(); // Disables altitude hold in receiver
  void dsbl_gpsnav_recvr(); // Disables GPS navigation in receiver

public:
  ExeptionDevice(Device *, Receiver *);
};

class Exception : public ExeptionDevice {
private:
  bool m_bPauseTD;
  uint_fast32_t m_t32Pause;
  uint_fast32_t m_iPauseTDTime;

  uint_fast32_t m_t32Device;                                   // Timer for calculating the reduction of the throttle
  uint_fast32_t m_t32Altitude;                                 // Timer for reading the current altitude

  /*
   * This reduces the throttle.
   * The amount of reduction dependent on the speed and the height.
   */
  void reduce_thr(float fTime);

protected:
  /*
   * !Critical: Section!
   * This function only stops if the motors of the model stopped spinning!
   * There is no possibility to get control back before.
   * Only use this function, if there are severe hardware problems.
   */
  void dev_take_down();   // Calls reduce_thr()

  /*
   * !Critical: Section!
   * This function stops if the receiver fetches a signal
   * So this function has less severe consequences if the remote control works again
   */
  void rcvr_take_down();  // Calls reduce_thr()

public:
  Exception(Device *, Receiver *);
  bool handle();

  void pause_take_down();
  void cont_take_down();
};

#endif /*UAV_h*/
