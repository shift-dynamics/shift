#include "shift-core/frame.h"

#include "ksl/print.h"

void Shift::Frame::addParameters() {

  addParameter("name", &name_, "descriptive frame name", true);

  addParameter(
    "active", &active_,
    "indicates whether frame is active, defaults to true if not specified",
    false);

  addParameter(
    "R", &dWRTOrigin_.R,
    "specify orientation as a 3x3 rotation matrix in row-major "
    "order, defaults to identity if not specified; the keys 'rotation', 'R', "
    "'euler_angles', 'quaternion', 'q' or 'Q' can be used to designate an "
    "orientation. If multiple key-value pairs are specified for orientation, "
    "only one of the entries will be loaded.",
    false);

  addParameter(
    "rotation", &dWRTOrigin_.R,
    "specify orientation as a 3x3 rotation matrix in row-major "
    "order, defaults to identity if not specified; the keys 'rotation', 'R', "
    "'euler_angles', 'quaternion', 'q' or 'Q' can be used to designate an "
    "orientation. If multiple key-value pairs are specified for orientation, "
    "only one of the entries will be loaded.",
    false);

  addParameter(
    "quaternion", &dWRTOrigin_.R,
    "specify orientation as a 3x3 rotation matrix in row-major "
    "order, defaults to identity if not specified; the keys 'rotation', 'R', "
    "'euler_angles', 'quaternion', 'q' or 'Q' can be used to designate an "
    "orientation. If multiple key-value pairs are specified for orientation, "
    "only one of the entries will be loaded.",
    false)
    .setQuaternionMapsToMat3x3(true);

  addParameter(
    "Q", &dWRTOrigin_.R,
    "specify orientation as a 3x3 rotation matrix in row-major "
    "order, defaults to identity if not specified; the keys 'rotation', 'R', "
    "'euler_angles', 'quaternion', 'q' or 'Q' can be used to designate an "
    "orientation. If multiple key-value pairs are specified for orientation, "
    "only one of the entries will be loaded.",
    false)
    .setQuaternionMapsToMat3x3(true);

  addParameter(
    "q", &dWRTOrigin_.R,
    "specify orientation as a 3x3 rotation matrix in row-major "
    "order, defaults to identity if not specified; the keys 'rotation', 'R', "
    "'euler_angles', 'quaternion', 'q' or 'Q' can be used to designate an "
    "orientation. If multiple key-value pairs are specified for orientation, "
    "only one of the entries will be loaded.",
    false)
    .setQuaternionMapsToMat3x3(true);

  addParameter(
    "euler_angles", &dWRTOrigin_.R,
    "specify orientation as a 3x3 rotation matrix in row-major "
    "order, defaults to identity if not specified; the keys 'rotation', 'R', "
    "'euler_angles', 'quaternion', 'q' or 'Q' can be used to designate an "
    "orientation. If multiple key-value pairs are specified for orientation, "
    "only one of the entries will be loaded.",
    false)
    .setEulerAnglesMapToMat3x3(true);

  addParameter(
    "translation", &dWRTOrigin_.t,
    "specify translation vector; the keys 'translation', 'T' and 't' "
    "may be used to designate translation. If multiple key-value pairs are "
    "specified for translation, only one of the entries will be loaded.",
    false);

  addParameter(
    "T", &dWRTOrigin_.t,
    "specify translation vector; the keys 'translation', 'T' and 't' "
    "may be used to designate translation. If multiple key-value pairs are "
    "specified for translation, only one of the entries will be loaded.",
    false);

  addParameter(
    "t", &dWRTOrigin_.t,
    "specify translation vector; the keys 'translation', 'T' and 't' "
    "may be used to designate translation. If multiple key-value pairs are "
    "specified for translation, only one of the entries will be loaded.",
    false);
}

const ksl_SE3_t* Shift::Frame::dWRTOrigin() const {
  return &dWRTOrigin_;
}

const ksl_SE3_t* Shift::Frame::dWRTReference() const {
  return &dWRTReference_;
}

const ksl_SE3_t* Shift::Frame::systemDWRTReference() const {
  if(useSystemReferenceFrame_) {
    return &systemDWRTReference_;
  } else {
    return &dWRTReference_;
  }
}

void Shift::Frame::print() {}
