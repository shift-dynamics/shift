#ifndef SHIFT_BODY_MODAL_H_
#define SHIFT_BODY_MODAL_H_

#include <string>

#include "shift/core/body/flex.h"

namespace shift {
namespace core {
namespace body {

/*!
@brief A flexible body whose elastic coordinates are modal coordinates: the
shape matrix holds mode shapes (fixed-interface normal modes and the static
modes of the attachments), with general modal mass, stiffness and damping
matrices, so that imported modes of any normalization can be used (Roger's
and the Craig–Bampton modes of the JPL report are stiffness-normalized,
\f$K^{\eta} = I\f$).

The equations are those of body::Flex. The modes are given with SetModel;
making them from a finite element model of the body's geometry, or reading
them from CalculiX, is still to be added, and until then the type is not
registered for model files.
*/
class Modal : public Flex {
public:
  Modal()
    : Flex() {}
  Modal(std::string name)
    : Flex(name) {}
  virtual ~Modal() override {}
};

} // namespace body
} // namespace core
} // namespace shift

#endif
