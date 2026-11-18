#include "shift/core/body/modal.h"
#include "shift/core/factory.h"

bool shift::core::body::Modal::registered_ =
  shift::core::Factory<shift::core::body::Base>::get()->RegisterModule(
    "modal", shift::core::body::Modal::Create);

std::unique_ptr<shift::core::body::Modal> shift::core::body::Modal::Create() {
  return std::make_unique<shift::core::body::Modal>();
}

void shift::core::body::Modal::DeclareParameters() {
  shift::core::body::Base::DeclareParameters();
  DeclareFiniteElementParameters();
  DeclareParameter("mode_count", &mode_count_,
                   "the number of lowest fixed-interface modes kept "
                   "(default 10)",
                   false);
  DeclareParameter("maximum_frequency", &maximum_frequency_,
                   "keep the fixed-interface modes up to this frequency (Hz)",
                   false);
  DeclareParameter("modes_kept", &modes_kept_,
                   "the fixed-interface modes kept, as indices in ascending "
                   "frequency from 0 (the lowest)",
                   false)
    .set_row_dim(-1);
}
