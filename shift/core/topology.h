#ifndef SHIFT_TOPOLOGY_H_
#define SHIFT_TOPOLOGY_H_

#include <assert.h>

#include <array>
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
@brief Displacements (d), velocities (v) and accelerations (a) of the free
bodies (body::Free), in the order of Context::get_free_bodies()

A free body has no joint, so no influence coefficients. The displacement is
that of the body origin in the fixed frame; v and a are spatial, in fixed
coordinates and referred to the fixed origin.
*/
struct DVA {
  std::vector<ksl_SE3_t> d;
  std::vector<ksl_screw_t> v;
  std::vector<ksl_screw_t> a;

  void Resize(size_t count) {
    d.assign(count, ksl_SE3_identity());
    v.assign(count, ksl_screw(0.0, 0.0, 0.0, 0.0, 0.0, 0.0));
    a.assign(count, ksl_screw(0.0, 0.0, 0.0, 0.0, 0.0, 0.0));
  }
  size_t get_count() const {
    return d.size();
  }
};

/*!
@brief Displacements (d), influence coefficient matrices (h), velocities (v) and
quadratic accelerations (a), and the displacements, velocities and
accelerations of the free bodies (free).
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
    free = other.free;
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
  DVA free; /*!< the free bodies, outside the kinematic tree */

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
@brief One tree of the kinematic forest: the bodies that reach the fixed body
through the same joint

Removing the fixed body from the spanning tree of a model leaves a forest. Each
tree in it has one root body, attached to the fixed body by its joint, and is
dynamically independent of the other trees unless a chord constraint closes a
loop through it (forces that couple trees, such as contact, are evaluated
before the equations of motion are solved). Bodies attached to their parents by
rigid joints are merged into their parents, so a tree lists the compressed
(non rigid) joints, see Context::get_pcl_compressed().

The kind of a tree determines the algorithms that can solve its equations of
motion:

- `kSerialChain`: every body has at most one child body.
- `kBranched`: some body has two or more child bodies, and there are no
  closed loops.
- `kClosedLoop`: a chord constraint closes a loop through a body of the
  tree, either within the tree or through another tree or the fixed body;
  solved with the constraint Jacobian by solver::RNCA.

Serial chains and branched trees are solved by the articulated-body algorithm
(solver::ABA) or the divide-and-conquer algorithm (solver::DCA), or by
solver::RNCA.
*/
struct Tree {
  enum class Kind { kSerialChain, kBranched, kClosedLoop };

  Kind kind = Kind::kSerialChain;

  /*! compressed joint (and body) indices of the tree in level-increasing
   * order (permutation order 1); bodies[0] is the root */
  std::vector<int> bodies;

  /*! index of the root joint in Context::get_active_joints() */
  int root_joint = -1;

  /*! number of generalized velocities of the tree */
  size_t dq_count = 0;

  static std::string KindName(Kind kind) {
    switch(kind) {
    case Kind::kSerialChain:
      return "serial chain";
    case Kind::kBranched:
      return "branched tree";
    case Kind::kClosedLoop:
      return "closed loop";
    }
    return "";
  }
};

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

/*!
\brief split the compressed spanning tree into the trees of the kinematic
forest and classify each one, see Tree
\param pcl_compressed compressed parent child list (permutation order 1, -1
for the fixed body)
\param joint_dof_compressed degrees of freedom of each compressed joint
\param joint_lookup compressed index of each active joint, -1 for a rigid
joint
\param loop_bodies compressed indices of the bodies joined by chord
constraints; their trees are closed loops
*/
std::vector<Tree>
DetectTrees(const std::vector<int>& pcl_compressed,
            const std::vector<unsigned int>& joint_dof_compressed,
            const std::vector<int>& joint_lookup,
            const std::vector<int>& loop_bodies);

bool IsSubstructureMember(int, std::vector<std::vector<int>>&);

/*!
\brief set inverse permutation
\param p forward permutation array
\param pInverse inverse permutation array
*/
void SetInversePermutation(const std::vector<int>& p,
                           std::vector<int>& pInverse);

/*!
\brief Find the floating parts of a model: the groups of bodies that no path
of joints joins to the fixed body
\param body_count number of bodies, indexed 0, ..., body_count - 1
\param joint_bodies the two bodies of each joint; -1 is the fixed body
\return for each floating part, in the order of its lowest body index, the
index of its body with the most joints (the lowest index among equals); a
body with no joints at all is a floating part of its own
*/
std::vector<int>
FindFloatingRoots(int body_count,
                  const std::vector<std::array<int, 2>>& joint_bodies);

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
