#ifndef _SHIFT_FRAME_H_
#define _SHIFT_FRAME_H_

#include <string>

#include "ksl/matrix.h"

#include "shift-core/base.h"
#include "shift-core/serialize.h"

namespace Shift {

/*!
@brief Shift Frame class
*/
class Frame
  : public Base
  , public Serialize {

public:
  Frame(std::string name)
    : Base(name) {
    addJsonKeys();
  }
  Frame()
    : Base("") {
    addJsonKeys();
  }
  ~Frame() {}

private:
  void addJsonKeys();

  ksl_SE3_t pose_;
};

} // namespace Shift

#endif
