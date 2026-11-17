#ifndef SHIFT_CORE_FEA_CACHE_H_
#define SHIFT_CORE_FEA_CACHE_H_

#include <array>
#include <cstdint>
#include <string>
#include <vector>

#include "shift/core/fea/reduction.h"

namespace shift {
namespace core {
namespace fea {

/*!
@brief what a flexible body keeps of its finite element analysis: the
reduction with every fixed-interface mode computed kept (KeepModes chooses
among them), and the surface of the mesh for drawing
*/
struct FlexCache {
  Reduction full;
  unsigned int node_count = 0; /*!< the mesh's nodes, the first points */
  std::vector<std::array<int, 6>> boundary;
  std::vector<int> boundary_face;
  double volume = 0.0;
  int tet_count = 0;
  /*! the mesh nodes each attachment ties to its frame, in the order of the
   * attachments (the reference frame's first) */
  std::vector<std::vector<int>> attachment_nodes;
  /*! the number of stations of each sliding attachment, in the order of
   * the sliding frames; their attachments follow the others' */
  std::vector<int> sliding_stations;
  /*! the nodes of the boundary, ascending, and their stresses in each of
   * full's shapes (NodalStresses: 6 (i full.elastic_count + j) + c) */
  std::vector<int> surface_nodes;
  std::vector<double> surface_stresses;
};

/*!
@brief the reduction with only the fixed-interface modes kept (indices
into those of full, all of which are kept in full), the static modes
first as in full: the shapes are columns of full's and \f$K^{\eta}\f$,
\f$M^{\eta}\f$ its principal submatrices
*/
Reduction KeepModes(const Reduction& full,
                    const std::vector<unsigned int>& kept);

/*!
@brief a 64-bit FNV-1a hash of bytes, continuing from hash; for the
fingerprint of what a cache was computed from
*/
uint64_t Fnv1a(const void* data, size_t size,
               uint64_t hash = 0xcbf29ce484222325ull);

/*!
@brief read the cache file at path into cache if it was written with the
same fingerprint (and this format); false if there is none, it is of
other data, or it cannot be read
*/
bool ReadFlexCache(const std::string& path, const std::string& fingerprint,
                   FlexCache& cache);

/*!
@brief write the cache file at path (to a file beside it, then renamed, so
that a reader never sees it half written); false if it cannot be written
*/
bool WriteFlexCache(const std::string& path, const std::string& fingerprint,
                    const FlexCache& cache);

} // namespace fea
} // namespace core
} // namespace shift

#endif
