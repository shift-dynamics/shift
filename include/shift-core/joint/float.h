#ifndef SHIFT_JOINT_FLOAT_H_
#define SHIFT_JOINT_FLOAT_H_

#include "shift-core/frame.h"
#include "shift-core/joint/base.h"

namespace Shift {

namespace Joint {

void float_d(const Shift::Context&, Topology::DHVA&, const Topology::Offsets&,
             const double*);

void floatinv_d(const Shift::Context&, Topology::DHVA&,
                const Topology::Offsets&, const double*);

void float_dh(const Shift::Context&, Topology::DHVA&, const Topology::Offsets&,
              const double*);

void floatinv_dh(const Shift::Context&, Topology::DHVA&,
                 const Topology::Offsets&, const double*);

void float_dhva(const Shift::Context&, Topology::DHVA&,
                const Topology::Offsets&, const double*, const double*);

void floatinv_dhva(const Shift::Context&, Topology::DHVA&,
                   const Topology::Offsets&, const double*, const double*);

class Float : public Base {
public:
  Float()
    : Base() {
    dof_ = 6;
    qCount_ = 7;
    dqCount_ = 6;
    q_0_ = new double[qCount_]();
    dq_0_ = new double[dqCount_]();
  }
  Float(std::string name)
    : Base(name) {
    dof_ = 6;
    qCount_ = 7;
    dqCount_ = 6;
    q_0_ = new double[qCount_]();
    dq_0_ = new double[dqCount_]();
  }
  ~Float() {
    delete[] q_0_;
    delete[] dq_0_;
  }

  static std::unique_ptr<Float> create();
  static bool isRegistered() {
    return registered_;
  }

  virtual void addKeys() override;
  virtual void readJSON(Json::Value&) override;

  Topology::dh_fptr getDfptr() override;
  Topology::dh_fptr getDHfptr() override;
  Topology::dhva_fptr getDHVAfptr() override;

private:
  static bool registered_;
  Shift::Frame frame_;
};

} // namespace Joint

} // namespace Shift

#endif
