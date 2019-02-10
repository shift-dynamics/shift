#include "shift-core/frame.h"

#include "ksl/print.h"

void Shift::Frame::addKeys() {

  SerialEntry name("name", SerialType::STRING, &name_);
  name.setRequired(true);
  addSerialEntry(name);

  SerialEntry active("active", SerialType::STRING, &active_);
  active.setRequired(false);
  addSerialEntry(active);

  SerialEntry R("R", SerialType::KSL_MAT3X3, &(transform_.R));
  R.setRequired(false);
  addSerialEntry(R);

  SerialEntry rotation("rotation", SerialType::KSL_MAT3X3, &(transform_.R));
  rotation.setRequired(false);
  addSerialEntry(rotation);

  SerialEntry T("T", SerialType::KSL_VEC3, &(transform_.t));
  T.setRequired(false);
  addSerialEntry(T);

  SerialEntry t("t", SerialType::KSL_VEC3, &(transform_.t));
  t.setRequired(false);
  addSerialEntry(t);

  SerialEntry translation("translation", SerialType::KSL_VEC3, &(transform_.t));
  translation.setRequired(false);
  addSerialEntry(translation);

  SerialEntry Q("Q", SerialType::KSL_QUATERNION, &(transform_.R));
  Q.setRequired(false);
  Q.setQuaternionMapsToMat3x3(true);
  addSerialEntry(Q);

  SerialEntry q("q", SerialType::KSL_QUATERNION, &(transform_.R));
  q.setRequired(false);
  q.setQuaternionMapsToMat3x3(true);
  addSerialEntry(q);

  SerialEntry quaternion("quaternion", SerialType::KSL_QUATERNION,
                         &(transform_.R));
  quaternion.setRequired(false);
  quaternion.setQuaternionMapsToMat3x3(true);
  addSerialEntry(quaternion);

  SerialEntry euler_angles("euler_angles", SerialType::EULERANGLES,
                           &(transform_.R));
  euler_angles.setRequired(false);
  euler_angles.setEulerAnglesMapToMat3x3(true);
  addSerialEntry(euler_angles);
}

void Shift::Frame::print() {
  ksl_SE3_print(stdout, &transform_, name_.c_str());
}

const ksl_SE3_t* Shift::Frame::getTransform() {
  return &transform_;
}

void Shift::Frame::setNameRequired(bool required) {
  for(auto& entry : serialEntries_) {
    if(entry.getKey() == "name") {
      entry.setRequired(required);
      break;
    }
  }
}
