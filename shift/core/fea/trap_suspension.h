#ifndef SHIFT_CORE_FEA_TRAP_SUSPENSION_H_
#define SHIFT_CORE_FEA_TRAP_SUSPENSION_H_

#include <cfenv>

namespace shift {
namespace core {
namespace fea {

/*!
@brief the divide-by-zero trap (SHIFT_FP_TRAP) suspended for the lifetime
of the object, and restored as it was

For library code whose vectorized operations divide by zero in the unused
lanes of a vector register and discard those results:
  - Netgen's vectorized atan2 divides by zero when x is 0, e.g. for a point
    of a surface on an axis, and returns the right angle;
  - Eigen (built for AVX, -Dcpu=native) solves the last rows of a
    SimplicialLDLT solve, those left over from whole registers, with masked
    loads that fill the unused lanes with zeros, and divides by the whole
    register (the inverse of the diagonal D).
The caller checks its results for values that are not finite instead.
*/
class DivideByZeroTrapSuspension {
public:
  DivideByZeroTrapSuspension() {
#if defined(SHIFT_FP_TRAP) && defined(__linux__)
    enabled_ = fedisableexcept(FE_DIVBYZERO);
#endif
  }
  ~DivideByZeroTrapSuspension() {
#if defined(SHIFT_FP_TRAP) && defined(__linux__)
    if(enabled_ != -1) {
      /* the flag the discarded divisions raised, which a trap enabled again
      does not report */
      feclearexcept(FE_DIVBYZERO);
      feenableexcept(enabled_ & FE_DIVBYZERO);
    }
#endif
  }
  DivideByZeroTrapSuspension(const DivideByZeroTrapSuspension&) = delete;
  DivideByZeroTrapSuspension&
  operator=(const DivideByZeroTrapSuspension&) = delete;

private:
  int enabled_ = -1;
};

} // namespace fea
} // namespace core
} // namespace shift

#endif
