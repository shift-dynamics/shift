#include "shift/core/frame.h"

#include "ksl/print.h"

void shift::core::Frame::addParameters() {

  addParameter("name", &name_, "descriptive frame name", true);

  addParameter(
    "active", &active_,
    "indicates whether frame is active, defaults to true if not specified",
    false);

  addParameter(
    "R", &d_wrt_origin_.R,
    "specify orientation as a 3x3 rotation matrix in row-major "
    "order, defaults to identity if not specified; the keys 'rotation', 'R', "
    "'euler_angles', 'quaternion', 'q' or 'Q' can be used to designate an "
    "orientation. If multiple key-value pairs are specified for orientation, "
    "only one of the entries will be loaded.",
    false);

  addParameter(
    "rotation", &d_wrt_origin_.R,
    "specify orientation as a 3x3 rotation matrix in row-major "
    "order, defaults to identity if not specified; the keys 'rotation', 'R', "
    "'euler_angles', 'quaternion', 'q' or 'Q' can be used to designate an "
    "orientation. If multiple key-value pairs are specified for orientation, "
    "only one of the entries will be loaded.",
    false);

  addParameter(
    "quaternion", &d_wrt_origin_.R,
    "specify orientation as a 3x3 rotation matrix in row-major "
    "order, defaults to identity if not specified; the keys 'rotation', 'R', "
    "'euler_angles', 'quaternion', 'q' or 'Q' can be used to designate an "
    "orientation. If multiple key-value pairs are specified for orientation, "
    "only one of the entries will be loaded.",
    false)
    .setQuaternionMapsToMat3x3(true);

  addParameter(
    "Q", &d_wrt_origin_.R,
    "specify orientation as a quaternion, "
    "defaults to identity if not specified; the keys 'rotation', 'R', "
    "'euler_angles', 'quaternion', 'q' or 'Q' can be used to designate an "
    "orientation. If multiple key-value pairs are specified for orientation, "
    "only one of the entries will be loaded.",
    false)
    .setQuaternionMapsToMat3x3(true);

  addParameter(
    "q", &d_wrt_origin_.R,
    "specify orientation as a quaternion, "
    "defaults to identity if not specified; the keys 'rotation', 'R', "
    "'euler_angles', 'quaternion', 'q' or 'Q' can be used to designate an "
    "orientation. If multiple key-value pairs are specified for orientation, "
    "only one of the entries will be loaded.",
    false)
    .setQuaternionMapsToMat3x3(true);

  addParameter(
    "euler_angles", &d_wrt_origin_.R,
    "specify orientation as a 3x3 rotation matrix in row-major "
    "order, defaults to identity if not specified; the keys 'rotation', 'R', "
    "'euler_angles', 'quaternion', 'q' or 'Q' can be used to designate an "
    "orientation. If multiple key-value pairs are specified for orientation, "
    "only one of the entries will be loaded.",
    false)
    .setEulerAnglesMapToMat3x3(true);

  addParameter(
    "translation", &d_wrt_origin_.t,
    "specify translation vector; the keys 'translation', 'T' and 't' "
    "may be used to designate translation. If multiple key-value pairs are "
    "specified for translation, only one of the entries will be loaded.",
    false);

  addParameter(
    "T", &d_wrt_origin_.t,
    "specify translation vector; the keys 'translation', 'T' and 't' "
    "may be used to designate translation. If multiple key-value pairs are "
    "specified for translation, only one of the entries will be loaded.",
    false);

  addParameter(
    "t", &d_wrt_origin_.t,
    "specify translation vector; the keys 'translation', 'T' and 't' "
    "may be used to designate translation. If multiple key-value pairs are "
    "specified for translation, only one of the entries will be loaded.",
    false);
}

const ksl_SE3_t* shift::core::Frame::dWRTOrigin() const {
  return &d_wrt_origin_;
}

const ksl_SE3_t* shift::core::Frame::dWRTReference() const {
  return &d_wrt_reference_;
}

const ksl_SE3_t* shift::core::Frame::systemDWRTReference() const {
  if(use_system_reference_frame_) {
    return &system_d_wrt_reference_;
  } else {
    return &d_wrt_reference_;
  }
}
