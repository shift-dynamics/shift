#include "shift/core/body/nodal.h"

#include <cmath>

#include "ksl/linalg.h"

#include "shift/core/exception.h"
#include "shift/core/factory.h"

bool shift::core::body::Nodal::registered_ =
  shift::core::Factory<shift::core::body::Base>::get()->RegisterModule(
    "nodal", shift::core::body::Nodal::Create);

std::unique_ptr<shift::core::body::Nodal> shift::core::body::Nodal::Create() {
  return std::make_unique<shift::core::body::Nodal>();
}

void shift::core::body::Nodal::DeclareParameters() {
  shift::core::body::Base::DeclareParameters();
  DeclareFiniteElementParameters();
  DeclareParameter("retained_points", &retained_points_,
                   "[[x, y, z], ...], points (body origin coordinates) "
                   "whose nearest mesh nodes are kept, three translations "
                   "each, after the attachments",
                   false)
    .set_row_dim(-1)
    .set_column_dim(3);
}

void shift::core::body::Nodal::SetFiniteElementModel(
  const std::vector<ksl_vec3_t>& positions, const std::vector<double>& masses,
  const std::vector<ksl_mat3x3_t>& rotary_inertias, unsigned int dofs,
  const std::vector<double>& stiffness, const std::vector<double>& damping,
  const std::vector<unsigned int>& fixed_points,
  const std::vector<unsigned int>& retained_points) {
  const size_t points = positions.size();
  const size_t n = points * dofs;
  if((dofs != 3 && dofs != 6) || masses.size() != points ||
     rotary_inertias.size() != points || stiffness.size() != n * n ||
     !(damping.empty() || damping.size() == n * n)) {
    throw shift::core::Exception(
      "The finite element model of flexible body '" + get_name() +
      "' has arrays of inconsistent sizes");
  }

  /* each degree of freedom: fixed (-1), master (its elastic coordinate) or
  slave (its index among the slaves) */
  enum class Kind { kFixed, kMaster, kSlave };
  std::vector<Kind> kind(n, Kind::kSlave);
  std::vector<int> index(n, -1);
  auto mark = [&](unsigned int p, Kind k, const char* what) {
    if(p >= points) {
      throw shift::core::Exception("Flexible body '" + get_name() + "': " +
                                   what + " point " + std::to_string(p) +
                                   " is not a point of the model");
    }
    if(kind[p * dofs] != Kind::kSlave) {
      throw shift::core::Exception("Flexible body '" + get_name() +
                                   "': point " + std::to_string(p) +
                                   " is given twice as fixed or retained");
    }
    for(unsigned int d = 0; d < dofs; d++) {
      kind[p * dofs + d] = k;
    }
  };
  for(unsigned int p : fixed_points) {
    mark(p, Kind::kFixed, "fixed");
  }
  for(unsigned int p : retained_points) {
    mark(p, Kind::kMaster, "retained");
  }
  if(fixed_points.empty()) {
    throw shift::core::Exception("Flexible body '" + get_name() +
                                 "': no fixed points carry its reference "
                                 "frame");
  }
  const size_t masters = retained_points.size() * dofs;
  for(size_t k = 0; k < retained_points.size(); k++) {
    for(unsigned int d = 0; d < dofs; d++) {
      index[retained_points[k] * dofs + d] = static_cast<int>(k * dofs + d);
    }
  }
  std::vector<size_t> slave_dofs;
  for(size_t i = 0; i < n; i++) {
    if(kind[i] == Kind::kSlave) {
      index[i] = static_cast<int>(slave_dofs.size());
      slave_dofs.push_back(i);
    }
  }
  std::vector<size_t> master_dofs(masters);
  for(size_t i = 0; i < n; i++) {
    if(kind[i] == Kind::kMaster) {
      master_dofs[index[i]] = i;
    }
  }
  const size_t slaves = slave_dofs.size();
  for(size_t p = 0; p < points && dofs == 3; p++) {
    const ksl_mat3x3_t& r = rotary_inertias[p];
    double size = 0.0;
    for(int i = 0; i < 3; i++) {
      for(int j = 0; j < 3; j++) {
        size = std::fmax(size, std::fabs(r.at[i][j]));
      }
    }
    if(size > 0.0) {
      throw shift::core::Exception(
        "Flexible body '" + get_name() + "': point " + std::to_string(p) +
        " has 3 degrees of freedom, so no rotation of its own, but a rotary "
        "inertia");
    }
  }

  /* X = K_ss^-1 K_sm, by the Cholesky factor of K_ss */
  std::vector<double> x(slaves * masters, 0.0);
  if(slaves > 0) {
    std::vector<double> k_ss(slaves * slaves);
    for(size_t a = 0; a < slaves; a++) {
      for(size_t b = 0; b < slaves; b++) {
        k_ss[a * slaves + b] = stiffness[slave_dofs[a] * n + slave_dofs[b]];
      }
    }
    if(ksl_linalg_cholesky_rmo(k_ss.data(), static_cast<int>(slaves)) != 0) {
      throw shift::core::Exception(
        "Flexible body '" + get_name() +
        "': the stiffness of the degrees of freedom that are neither fixed "
        "nor retained is not positive definite (do the fixed points hold "
        "the body still?)");
    }
    std::vector<double> column(slaves), solved(slaves);
    for(size_t m = 0; m < masters; m++) {
      for(size_t a = 0; a < slaves; a++) {
        column[a] = stiffness[slave_dofs[a] * n + master_dofs[m]];
      }
      ksl_linalg_cholesky_solve_rmo(k_ss.data(), column.data(), solved.data(),
                                    static_cast<int>(slaves));
      for(size_t a = 0; a < slaves; a++) {
        x[a * masters + m] = solved[a];
      }
    }
  }

  /* G: the row of each degree of freedom */
  auto g = [&](size_t i, size_t m) {
    switch(kind[i]) {
    case Kind::kFixed:
      return 0.0;
    case Kind::kMaster:
      return index[i] == static_cast<int>(m) ? 1.0 : 0.0;
    default:
      return -x[index[i] * masters + m];
    }
  };

  /* K^eta = G^T K G and C^eta = G^T C G, over the degrees of freedom that
  are not fixed */
  std::vector<size_t> free_dofs;
  for(size_t i = 0; i < n; i++) {
    if(kind[i] != Kind::kFixed) {
      free_dofs.push_back(i);
    }
  }
  std::vector<double> gm(free_dofs.size() * masters);
  for(size_t a = 0; a < free_dofs.size(); a++) {
    for(size_t m = 0; m < masters; m++) {
      gm[a * masters + m] = g(free_dofs[a], m);
    }
  }
  auto reduce = [&](const std::vector<double>& full) {
    std::vector<double> reduced(masters * masters, 0.0);
    if(full.empty()) {
      return reduced;
    }
    std::vector<double> fg(free_dofs.size() * masters, 0.0);
    for(size_t a = 0; a < free_dofs.size(); a++) {
      for(size_t b = 0; b < free_dofs.size(); b++) {
        const double f = full[free_dofs[a] * n + free_dofs[b]];
        if(f == 0.0) {
          continue;
        }
        for(size_t m = 0; m < masters; m++) {
          fg[a * masters + m] += f * gm[b * masters + m];
        }
      }
    }
    for(size_t a = 0; a < free_dofs.size(); a++) {
      for(size_t l = 0; l < masters; l++) {
        const double ga = gm[a * masters + l];
        if(ga == 0.0) {
          continue;
        }
        for(size_t m = 0; m < masters; m++) {
          reduced[l * masters + m] += ga * fg[a * masters + m];
        }
      }
    }
    /* symmetric up to round-off */
    for(size_t l = 0; l < masters; l++) {
      for(size_t m = 0; m < l; m++) {
        const double mean =
          0.5 * (reduced[l * masters + m] + reduced[m * masters + l]);
        reduced[l * masters + m] = mean;
        reduced[m * masters + l] = mean;
      }
    }
    return reduced;
  };

  /* the shape matrix: the rows of G of each point, translations then
  rotations */
  std::vector<ksl_screw_t> shapes(points * masters);
  for(size_t p = 0; p < points; p++) {
    for(size_t m = 0; m < masters; m++) {
      ksl_screw_t& shape = shapes[p * masters + m];
      for(unsigned int d = 0; d < 6; d++) {
        shape.at[d] = d < dofs ? g(p * dofs + d, m) : 0.0;
      }
    }
  }
  SetModel(positions, masses, rotary_inertias,
           static_cast<unsigned int>(masters), shapes, reduce(stiffness),
           reduce(damping));
}
