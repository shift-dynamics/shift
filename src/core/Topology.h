#ifndef SHIFT_TOPOLOGY_H_
#define SHIFT_TOPOLOGY_H_

#include <assert.h>
#include <iostream>
#include <string.h>
#include <string>
#include <vector>

#include "shift/core/Exception.h"
#include "shift/core/log.h"
#include "shift/core/Exception.h"

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
  DHVA(unsigned int frame_count, unsigned int dc_count, unsigned int dq_count,
       unsigned int non_rigid_joint_count) {

    frame_count_ = frame_count;
    dc_count_ = dc_count;
    dq_count_ = dq_count;
    non_rigid_joint_count_ = non_rigid_joint_count;

    size_t dhva_size_ =
      2 * dc_count * sizeof(double) +                     /* dc */
      frame_count * sizeof(ksl_SE3_t) +                   /* d */
      (dq_count + 1) * sizeof(ksl_screw_t) +              /* h */
      (non_rigid_joint_count + 1) * sizeof(ksl_screw_t) + /* v */
      (non_rigid_joint_count + 1) * sizeof(ksl_screw_t);  /* aq */
    // @TODO convert this to a smart pointer
    
    dc = new double[dhva_size_];
    if(dc == nullptr) {
      SHIFT_ERROR("Failed to allocate memory for DHVA");
      throw shift::core::Exception("Failed to allocate memory for DHVA");
    }
    memset(dc, 0, dhva_size_);
    d = (ksl_SE3_t*)(dc + 2 * dc_count);
    h = (ksl_screw_t*)(d + frame_count);
    v = h + (non_rigid_joint_count + 1);
    aq = v + (non_rigid_joint_count + 1);
    for(unsigned int i = 0; i < frame_count; i++) {
      ksl_SE3_setIdentity(d + i);
    }

    // increment pointers to h, v, and aq. This allows using the index -1 for
    // the fixed inertial reference frame
    h++;
    v++;
    aq++;
  }

  ~DHVA() {
    if(dc != nullptr) {
      delete[] dc;
      dc = nullptr;
      d = nullptr;
      h = nullptr;
      v = nullptr;
      aq = nullptr;
    }
  }

  DHVA& operator=(const DHVA& other) {

    // check for self assignment
    if(&other == this) {
      return *this;
    }

    frame_count_ = other.frame_count_;
    dc_count_ = other.dc_count_;
    dq_count_ = other.dq_count_;
    non_rigid_joint_count_ = other.non_rigid_joint_count_;

    size_t dhva_size_ =
      2 * dc_count_ * sizeof(double) +                     /* dc */
      frame_count_ * sizeof(ksl_SE3_t) +                   /* d */
      (dq_count_) * sizeof(ksl_screw_t) +              /* h */
      (non_rigid_joint_count_ + 1) * sizeof(ksl_screw_t) + /* v */
      (non_rigid_joint_count_ + 1) * sizeof(ksl_screw_t);  /* aq */
    dc = new double[dhva_size_];
    if(dc == nullptr) {
      throw shift::core::Exception(
        "Error: failed to allocate memory for DHVA");
      assert(0);
    }
    memset(dc, 0, dhva_size_);
    d = (ksl_SE3_t*)(dc + 2 * dc_count_);
    h = (ksl_screw_t*)(d + frame_count_);
    v = h + (non_rigid_joint_count_ + 1);
    aq = v + (non_rigid_joint_count_ + 1);

    // increment pointers to h, v, and aq. This allows using the index -1 for
    // the fixed inertial reference frame
    h++;
    v++;
    aq++;

    memcpy(dc, other.dc, dhva_size_);
    return *this;
  }

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
  std::shared_ptr<double> data;
  unsigned int frame_count_ = 0;
  unsigned int dc_count_ = 0;
  unsigned int dq_count_ = 0;
  unsigned int non_rigid_joint_count_ = 0;
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
    unsigned int dc;    /*!< offset into direction cosine array */
    unsigned int q = 0; /*!< offset into array of generalized coordinates */
  };
  unsigned int dq = 0; /*!< offset into array of generalized velocities */
  struct {
    int d = 0; /*!< offset for parent body's displacement */
    int v = 0; /*!< offset for parent body's velocity */
  } parent;
  struct {
    int d = 0; /*!< offset for child body's displacement */
    int v = 0; /*!< offset for child body's velocity */
  } child;
  unsigned int data = 0; /*!< offset into data array */
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
