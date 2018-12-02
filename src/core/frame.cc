#include "shift-core/frame.h"

void Shift::Frame::addKeys() {

  SerialEntry name("name", SerialType::STRING, &name_);
  name.setRequired(true);
  addSerialEntry(name);

  SerialEntry active("active", SerialType::STRING, &active_);
  active.setRequired(false);
  addSerialEntry(active);

  SerialEntry R("R", SerialType::KSL_MAT3X3, &(pose_.R));
  R.setRequired(false);
  addSerialEntry(R);

  SerialEntry rotation("rotation", SerialType::KSL_MAT3X3, &(pose_.R));
  rotation.setRequired(false);
  addSerialEntry(rotation);

  SerialEntry T("T", SerialType::KSL_VEC3, &(pose_.t));
  T.setRequired(false);
  addSerialEntry(T);

  SerialEntry t("t", SerialType::KSL_VEC3, &(pose_.t));
  t.setRequired(false);
  addSerialEntry(t);

  SerialEntry translation("translation", SerialType::KSL_VEC3, &(pose_.t));
  translation.setRequired(false);
  addSerialEntry(translation);

  SerialEntry Q("Q", SerialType::KSL_QUATERNION, &(pose_.R));
  Q.setRequired(false);
  Q.setQuaternionMapsToMat3x3(true);
  addSerialEntry(Q);

  SerialEntry q("q", SerialType::KSL_QUATERNION, &(pose_.R));
  q.setRequired(false);
  q.setQuaternionMapsToMat3x3(true);
  addSerialEntry(q);

  SerialEntry quaternion("quaternion", SerialType::KSL_QUATERNION, &(pose_.R));
  quaternion.setRequired(false);
  quaternion.setQuaternionMapsToMat3x3(true);
  addSerialEntry(quaternion);

  SerialEntry euler_angles("euler_angles", SerialType::EULERANGLES, &(pose_.R));
  euler_angles.setRequired(false);
  euler_angles.setEulerAnglesMapToMat3x3(true);
  addSerialEntry(euler_angles);
}
