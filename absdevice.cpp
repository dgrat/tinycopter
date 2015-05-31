#include "absdevice.h"


AbsErrorDevice::AbsErrorDevice() {
  m_eErrors = NOTHING_F;
}

AbsErrorDevice::DEVICE_ERROR_FLAGS AbsErrorDevice::get_errors() const {
  return m_eErrors;
}

void AbsErrorDevice::set_errors(AbsErrorDevice::DEVICE_ERROR_FLAGS flags) {
  m_eErrors = flags;
}
