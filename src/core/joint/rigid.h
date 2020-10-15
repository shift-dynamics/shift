#ifndef SHIFT_JOINT_RIGID_H_
#define SHIFT_JOINT_RIGID_H_

#include "shift/core/frame.h"
#include "shift/core/joint/base.h"

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
    addParameters();
  }

  Rigid(std::string name)
    : Base(name) {
    addParameters();
  }

  virtual ~Rigid() override {}

  static std::unique_ptr<Rigid> create();
  static bool isRegistered() {
    return registered_;
  }

  topology::dh_fptr Dfptr() override;
  topology::dh_fptr DHfptr() override;
  topology::dhva_fptr DHVAfptr() override;

  virtual ksl_SE3_t transform() override {
    ksl_SE3_t out;
    const ksl_SE3_t* temp = frame_.dWRTOrigin();
    if(sense_ >= 0) {
      ksl_SE3_copy(temp, &out);
    } else {
      ksl_SE3_inverted(temp, &out);
    }
    return out;
  }

private:
  virtual void addParameters() override;
  static bool registered_;
  shift::core::Frame frame_;
};

} // namespace joint
}
} // namespace shift

#endif
