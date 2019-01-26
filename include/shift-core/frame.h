#ifndef SHIFT_FRAME_H_
#define SHIFT_FRAME_H_

#include <string>

#include "ksl/SE3.h"
#include "ksl/mat4x4.h"
#include "ksl/quaternion.h"

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
    addKeys();
  }
  Frame()
    : Base("") {
    addKeys();
  }
  ~Frame() {}
  void print();

private:
  void addKeys();
  std::vector<std::string> fullyQualifiedName_;
  ksl_SE3_t pose_ =
    ksl_SE3(1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0);
};

} // namespace Shift

#endif
