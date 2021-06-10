#ifndef SHIFT_JOINT_RIGID_H_
#define SHIFT_JOINT_RIGID_H_

#include "shift/core/Frame.h"
#include "shift/core/joint/Base.h"

namespace shift {
namespace core {
namespace joint {

struct DHVA;

void rigid_dh(const shift::core::Context&, topology::DHVA&, const topology::Offsets&,
              const double*);

void rigidinv_dh(const shift::core::Context&, topology::DHVA&,
                 const topology::Offsets&, const double*);

void rigid_dhva(const shift::core::Context&, topology::DHVA&,
                const topology::Offsets&, const double*, const double*);

void rigidinv_dhva(const shift::core::Context&, topology::DHVA&,
                   const topology::Offsets&, const double*, const double*);

class Rigid : public Base {
public:
  Rigid()
    : Base() {
    DeclareParameters();
  }

  Rigid(std::string name)
    : Base(name) {
    DeclareParameters();
  }

  virtual ~Rigid() override {}

  static std::unique_ptr<Rigid> Create();
  static bool is_registered() {
    return registered_;
  }

  topology::dh_fptr Dfptr() override;
  topology::dh_fptr DHfptr() override;
  topology::dhva_fptr DHVAfptr() override;

  virtual ksl_SE3_t get_transform() override {
    ksl_SE3_t out;
    const ksl_SE3_t* temp = frame_.get_d_wrt_origin();
    if(sense_ >= 0) {
      ksl_SE3_copy(temp, &out);
    } else {
      ksl_SE3_inverted(temp, &out);
    }
    return out;
  }

private:
  virtual void DeclareParameters() override;
  static bool registered_;
  shift::core::Frame frame_;
};

} // namespace joint
}
} // namespace shift

#endif
