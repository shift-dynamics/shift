#ifndef SHIFT_BODY_MODAL_H_
#define SHIFT_BODY_MODAL_H_

#include <memory>
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

The equations are those of body::Flex. The modes are given with SetModel, or
made from fe_geometry (model type "modal"): the solid is meshed with 10-node
tetrahedra, each frame a joint or chord uses (and each listed in
attachments) is tied to the mesh, and the mesh is reduced by Craig and
Bampton to the static modes of the attachments other than the reference
frame and the fixed-interface modes chosen by mode_count (the lowest n,
10 by default), maximum_frequency or modes_kept (fea::Reduce).
*/
class Modal : public Flex {
public:
  Modal()
    : Flex() {
    DeclareParameters();
  }
  Modal(std::string name)
    : Flex(name) {
    DeclareParameters();
  }
  virtual ~Modal() override {}
  static std::unique_ptr<Modal> Create();
  static bool is_registered() {
    return registered_;
  }
  void DeclareParameters() override;

protected:
  bool HasFixedInterfaceModes() const override {
    return true;
  }

private:
  static bool registered_;
};

} // namespace body
} // namespace core
} // namespace shift

#endif
