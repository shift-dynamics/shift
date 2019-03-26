#ifndef SHIFT_JOINT_RIGID_H_
#define SHIFT_JOINT_RIGID_H_

#include "shift-core/frame.h"
#include "shift-core/joint/base.h"

namespace Shift {

namespace Joint {

struct DHVA;

void rigid_dh(const Shift::Context&, Topology::DHVA&, const Topology::Offsets&,
              const double*);

void rigidinv_dh(const Shift::Context&, Topology::DHVA&,
                 const Topology::Offsets&, const double*);

void rigid_dhva(const Shift::Context&, Topology::DHVA&,
                const Topology::Offsets&, const double*, const double*);

void rigidinv_dhva(const Shift::Context&, Topology::DHVA&,
                   const Topology::Offsets&, const double*, const double*);

class Rigid : public Base {
public:
  Rigid()
    : Base() {
    addKeys();
  }

  Rigid(std::string name)
    : Base(name) {
    addKeys();
  }

  virtual ~Rigid() override {}

  static std::unique_ptr<Rigid> create();
  static bool isRegistered() {
    return registered_;
  }

  virtual void addKeys() override;
  virtual void readJSON(Json::Value&) override;

  Topology::dh_fptr getDfptr() override;
  Topology::dh_fptr getDHfptr() override;
  Topology::dhva_fptr getDHVAfptr() override;

  virtual ksl_SE3_t getTransform() override {
    ksl_SE3_t out;
    const ksl_SE3_t* temp = frame_.getDWRTOrigin();
    if(sense_ >= 0) {
      ksl_SE3_copy(temp, &out);
    } else {
      ksl_SE3_inverted(temp, &out);
    }
    return out;
  }

private:
  static bool registered_;
  Shift::Frame frame_;
};

} // namespace Joint

} // namespace Shift

#endif
