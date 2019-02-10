#ifndef SHIFT_JOINT_RIGID_H_
#define SHIFT_JOINT_RIGID_H_

#include "shift-core/frame.h"
#include "shift-core/joint/base.h"

namespace Shift {

namespace Joint {

struct DHVA;

void rigid_dh(const Shift::Context&, DHVA&, const Offsets&, const double*);

void rigidinv_dh(const Shift::Context&, DHVA&, const Offsets&, const double*);

void rigid_dhva(const Shift::Context&, DHVA&, const Offsets&, const double*,
                const double*);

void rigidinv_dhva(const Shift::Context&, DHVA&, const Offsets&, const double*,
                   const double*);

class Rigid : public Base {
public:
  Rigid()
    : Base() {}

  Rigid(std::string name)
    : Base(name) {}

  ~Rigid() {}

  static std::unique_ptr<Rigid> create();
  static bool isRegistered() {
    return registered_;
  }

  void addKeys();
  virtual void readJSON(Json::Value&);

  dh_fptr getDH();
  dhva_fptr getDHVA();

private:
  static bool registered_;
  Shift::Frame frame_;
};

} // namespace Joint

} // namespace Shift

#endif
