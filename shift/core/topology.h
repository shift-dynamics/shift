#ifndef SHIFT_TOPOLOGY_H_
#define SHIFT_TOPOLOGY_H_

#include <assert.h>

#include <iostream>
#include <memory>
#include <sstream>
#include <string.h>
#include <string>
#include <vector>

#include "shift/core/exception.h"
#include "shift/core/log.h"

#include "ksl/SE3.h"
#include "ksl/screw.h"

namespace shift {
namespace core {
class Context;

namespace topology {

/*!
@brief Displacements (d), influence coefficient matrices (h), velocities (v) and
quadratic accelerations (a).
*/
struct DHVA {
  DHVA() {}
  DHVA(size_t frame_count, size_t dc_count, size_t dq_count,
       size_t non_rigid_joint_count)
    : frame_count_(frame_count)
    , dc_count_(dc_count)
    , dq_count_(dq_count)
    , non_rigid_joint_count_(non_rigid_joint_count) {

    Initialize();
  }

  ~DHVA() {}

  void Initialize() {

#ifdef SHIFT_WITH_CONTIGUOUS_DHVA
    dhva_size_ = 2 * dc_count_ * sizeof(double) +                     /* dc */
                 frame_count_ * sizeof(ksl_SE3_t) +                   /* d */
                 dq_count_ * sizeof(ksl_screw_t) +                    /* h */
                 (non_rigid_joint_count_ + 1) * sizeof(ksl_screw_t) + /* v */
                 (non_rigid_joint_count_ + 1) * sizeof(ksl_screw_t);  /* aq */

    data_ = std::unique_ptr<double[]>(new double[dhva_size_]);

    dc = data_.get();
    if(dc == nullptr) {
      SHIFT_ERROR("Failed to allocate memory for DHVA");
      throw shift::core::Exception("Failed to allocate memory for DHVA");
    }
    memset(dc, 0, dhva_size_);
    d = (ksl_SE3_t*)(dc + 2 * dc_count_);
    h = (ksl_screw_t*)(d + frame_count_);
    v = h + dq_count_;
    aq = v + (non_rigid_joint_count_ + 1);
    for(size_t i = 0; i < frame_count_; i++) {
      ksl_SE3_setIdentity(d + i);
    }

    // assert memory layout
    assert((double*)(v + 1) == ((double*)v) + 6);

    // increment pointers to v and aq. This allows using the index -1 for
    // the fixed inertial reference frame
    v++;
    aq++;

#else

    dc.resize(2 * dc_count_);
    std::fill(dc.begin(), dc.end(), 0.0);

    d.resize(frame_count_);
    std::fill(d.begin(), d.end(), ksl_SE3_identity());

    h.resize(dq_count_);
    std::fill(h.begin(), h.end(), ksl_screw(0.0, 0.0, 0.0, 0.0, 0.0, 0.0));

    v_.resize(non_rigid_joint_count_ + 1);
    std::fill(v_.begin(), v_.end(), ksl_screw(0.0, 0.0, 0.0, 0.0, 0.0, 0.0));
    v = v_.data() + 1;

    aq_.resize(non_rigid_joint_count_ + 1);
    std::fill(aq_.begin(), aq_.end(), ksl_screw(0.0, 0.0, 0.0, 0.0, 0.0, 0.0));
    aq = aq_.data() + 1;

#endif
  }

  inline size_t get_frame_count() const {
    return frame_count_;
  }

  inline size_t get_dc_count() const {
    return dc_count_;
  }

  inline size_t get_dq_count() const {
    return dq_count_;
  }

  inline size_t get_non_rigid_joint_count() const {
    return non_rigid_joint_count_;
  }

  DHVA& operator=(const DHVA& other) {

    // check for self assignment
    if(&other == this) {
      return *this;
    }
    frame_count_ = other.get_frame_count();
    dc_count_ = other.get_dc_count();
    dq_count_ = other.get_dq_count();
    non_rigid_joint_count_ = other.get_non_rigid_joint_count();
    Initialize();

#ifdef SHIFT_WITH_CONTIGUOUS_DHVA
    memcpy(dc, other.dc, dhva_size_);
#else
    d = other.d;
    h = other.h;
    v_ = other.v_;
    v = v_.data() + 1;
    aq_ = other.aq_;
    aq = aq_.data() + 1;
#endif
    return *this;
  }

#ifdef SHIFT_WITH_CONTIGUOUS_DHVA
  double* dc = nullptr;     /*!< direction cosine arrays */
  ksl_SE3_t* d = nullptr;   /*!< array of displacements equal
                              in size to number of frames */
  ksl_screw_t* h = nullptr; /*!< an array of influence coefficient matrices */
  ksl_screw_t* v =
    nullptr; /*!< an array of body velocities equal to the
        number of non rigid joints + 1 for the fixed inertial reference frame */
  ksl_screw_t* aq =
    nullptr; /*!< an array of body accelerations equal to the
        number of non rigid joints + 1 for the fixed inertial reference frame */
#else
  std::vector<double> dc;
  std::vector<ksl_SE3_t> d;
  std::vector<ksl_screw_t> h;
  ksl_screw_t* v;
  ksl_screw_t* aq;
#endif

  inline void Print(std::stringstream& ss) const {

    ss << "  dc: ";
    for(size_t i = 0; i < 2 * dc_count_; i++) {
      ss << dc[i] << " ";
    }
    ss << "\n";

    ss << "  d: ";
    for(size_t i = 0; i < frame_count_; i++) {
      for(size_t j = 0; j < 12; j++) {
        ss << d[i].as_array[j] << " ";
      }
    }
    ss << "\n";

    ss << "  h: ";
    for(size_t i = 0; i < dq_count_; i++) {
      for(size_t j = 0; j < 6; j++) {
        ss << h[i].at[j] << " ";
      }
    }
    ss << "\n";

    ss << "  v: ";
    for(ssize_t i = -1; i < non_rigid_joint_count_; i++) {
      for(size_t j = 0; j < 6; j++) {
        ss << v[i].at[j] << " ";
      }
    }
    ss << "\n";

    ss << "  aq: ";
    for(ssize_t i = -1; i < non_rigid_joint_count_; i++) {
      for(size_t j = 0; j < 6; j++) {
        ss << aq[i].at[j] << " ";
      }
    }
    ss << "\n";
  }

private:
#ifdef SHIFT_WITH_CONTIGUOUS_DHVA
  std::unique_ptr<double[]> data_;
#else
  std::vector<ksl_screw_t> v_;
  std::vector<ksl_screw_t> aq_;
#endif
size_t frame_count_ = 0;
size_t dc_count_ = 0;
size_t dq_count_ = 0;
ssize_t non_rigid_joint_count_ = 0;
size_t dhva_size_ = 0;
};

// ostream& operator<<(ostream& os, const DHVA& dhva)
// {
//     os << dt.mo << '/' << dt.da << '/' << dt.yr;
//     return os;
// }

/*!
@brief Struct that defines offsets into DHVA array.
*/
struct Offsets {
  union {
    size_t dc;    /*!< offset into direction cosine array */
    size_t q = 0; /*!< offset into array of generalized coordinates */
  };
  size_t dq = 0; /*!< offset into array of generalized velocities */
  struct {
    int d = 0; /*!< offset for parent body's displacement */
    int v = 0; /*!< offset for parent body's velocity */
  } parent;
  struct {
    int d = 0; /*!< offset for child body's displacement */
    int v = 0; /*!< offset for child body's velocity */
  } child;
  size_t data = 0; /*!< offset into data array */
};

typedef void (*dh_fptr)(const shift::core::Context&, DHVA&, const Offsets&,
                        const double*);

typedef void (*dhva_fptr)(const shift::core::Context&, DHVA&, const Offsets&,
                          const double*, const double*);

bool MatchFullyQualifiedName(const std::vector<std::string>&,
                             const std::vector<std::string>&);

bool InPath(std::vector<int>& pcl, int j, int k);

bool IsSubstructureHead(int, std::vector<std::vector<int>>&);

bool IsSubstructureMember(int, std::vector<std::vector<int>>&);

/*!
\brief set inverse permutation
\param p forward permutation array
\param pInverse inverse permutation array
*/
void SetInversePermutation(const std::vector<int>& p,
                           std::vector<int>& pInverse);

std::string GetJointType(dh_fptr f);

std::string GetBasename(const std::string& s);

std::string GetExtension(std::string& filename);

std::string RemoveExtension(std::string& filename);

std::string GetFilename(const std::string& filename);

std::string GetValidGraphvizExtension(std::string& extension);

} // namespace topology
} // namespace core
} // namespace shift
#endif
