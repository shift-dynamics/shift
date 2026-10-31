#include "shift/core/frame.h"

#include "ksl/print.h"

void shift::core::Frame::DeclareParameters() {

  DeclareParameter("name", &name_, "descriptive frame name", true);

  DeclareParameter(
    "active", &active_,
    "indicates whether frame is active, defaults to true if not specified",
    false);

  DeclareParameter(
    "R", &d_wrt_origin_.R,
    "specify orientation as a 3x3 rotation matrix in row-major "
    "order, defaults to identity if not specified; the keys 'rotation', 'R', "
    "'euler_angles', 'quaternion', 'q' or 'Q' can be used to designate an "
    "orientation. If multiple key-value pairs are specified for orientation, "
    "only one of the entries will be loaded.",
    false);

  DeclareParameter(
    "rotation", &d_wrt_origin_.R,
    "specify orientation as a 3x3 rotation matrix in row-major "
    "order, defaults to identity if not specified; the keys 'rotation', 'R', "
    "'euler_angles', 'quaternion', 'q' or 'Q' can be used to designate an "
    "orientation. If multiple key-value pairs are specified for orientation, "
    "only one of the entries will be loaded.",
    false);

  DeclareParameter(
    "quaternion", &d_wrt_origin_.R,
    "specify orientation as a 3x3 rotation matrix in row-major "
    "order, defaults to identity if not specified; the keys 'rotation', 'R', "
    "'euler_angles', 'quaternion', 'q' or 'Q' can be used to designate an "
    "orientation. If multiple key-value pairs are specified for orientation, "
    "only one of the entries will be loaded.",
    false)
    .set_quaternion_maps_to_mat3x3(true);

  DeclareParameter(
    "Q", &d_wrt_origin_.R,
    "specify orientation as a quaternion, "
    "defaults to identity if not specified; the keys 'rotation', 'R', "
    "'euler_angles', 'quaternion', 'q' or 'Q' can be used to designate an "
    "orientation. If multiple key-value pairs are specified for orientation, "
    "only one of the entries will be loaded.",
    false)
    .set_quaternion_maps_to_mat3x3(true);

  DeclareParameter(
    "q", &d_wrt_origin_.R,
    "specify orientation as a quaternion, "
    "defaults to identity if not specified; the keys 'rotation', 'R', "
    "'euler_angles', 'quaternion', 'q' or 'Q' can be used to designate an "
    "orientation. If multiple key-value pairs are specified for orientation, "
    "only one of the entries will be loaded.",
    false)
    .set_quaternion_maps_to_mat3x3(true);

  DeclareParameter(
    "euler_angles", &d_wrt_origin_.R,
    "specify orientation as a 3x3 rotation matrix in row-major "
    "order, defaults to identity if not specified; the keys 'rotation', 'R', "
    "'euler_angles', 'quaternion', 'q' or 'Q' can be used to designate an "
    "orientation. If multiple key-value pairs are specified for orientation, "
    "only one of the entries will be loaded.",
    false)
    .set_euler_angles_map_to_mat3x3(true);

  DeclareParameter(
    "translation", &d_wrt_origin_.t,
    "specify translation vector; the keys 'translation', 'T' and 't' "
    "may be used to designate translation. If multiple key-value pairs are "
    "specified for translation, only one of the entries will be loaded.",
    false);

  DeclareParameter(
    "T", &d_wrt_origin_.t,
    "specify translation vector; the keys 'translation', 'T' and 't' "
    "may be used to designate translation. If multiple key-value pairs are "
    "specified for translation, only one of the entries will be loaded.",
    false);

  DeclareParameter(
    "t", &d_wrt_origin_.t,
    "specify translation vector; the keys 'translation', 'T' and 't' "
    "may be used to designate translation. If multiple key-value pairs are "
    "specified for translation, only one of the entries will be loaded.",
    false);
}

const ksl_SE3_t* shift::core::Frame::get_d_wrt_origin() const {
  return &d_wrt_origin_;
}

const ksl_SE3_t* shift::core::Frame::get_d_wrt_reference() const {
  return &d_wrt_reference_;
}

const ksl_SE3_t* shift::core::Frame::get_system_d_wrt_reference() const {
  if(use_system_reference_frame_) {
    return &system_d_wrt_reference_;
  } else {
    return &d_wrt_reference_;
  }
}
