/* Magic Formula equations adapted from Project Chrono's ChPac02Tire, see the
license notice in pac2002.h */

#include "shift/core/force_module/tire/pac2002.h"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <fstream>
#include <map>
#include <vector>

#include "ksl/coscrew.h"

#include "shift/core/exception.h"

/* ---------------------------------------------------------------------------
.tir property file
--------------------------------------------------------------------------- */

static std::string Trim(const std::string& s) {
  const size_t a = s.find_first_not_of(" \t\r\n");
  if(a == std::string::npos) {
    return "";
  }
  const size_t b = s.find_last_not_of(" \t\r\n");
  return s.substr(a, b - a + 1);
}

static std::string Upper(std::string s) {
  for(auto& c : s) {
    c = static_cast<char>(std::toupper(static_cast<unsigned char>(c)));
  }
  return s;
}

/* string value without quotes, upper case */
static std::string Word(const std::string& value) {
  std::string s = value;
  s.erase(std::remove(s.begin(), s.end(), '\''), s.end());
  return Upper(Trim(s));
}

shift::core::force_module::tire::Pac2002Coefficients
shift::core::force_module::tire::Pac2002Coefficients::Read(
  const std::string& filename) {

  std::ifstream file(filename);
  if(!file.good()) {
    throw shift::core::Exception("Tire property file not found: " + filename);
  }

  /* section -> key -> value; keys are upper case, values keep their case */
  std::map<std::string, std::map<std::string, std::string>> sections;
  std::string section;
  std::string line;
  while(std::getline(file, line)) {
    line = Trim(line);
    if(line.empty() || line[0] == '!' || line[0] == '$') {
      continue;
    }
    if(line[0] == '[') {
      section = Upper(line.substr(0, line.find(']') + 1));
      sections[section];
      continue;
    }
    const size_t eq = line.find('=');
    if(eq == std::string::npos) {
      continue; // table rows, e.g. [SHAPE]
    }
    std::string value = line.substr(eq + 1);
    const size_t comment = value.find('$');
    if(comment != std::string::npos) {
      value = value.substr(0, comment);
    }
    sections[section][Upper(Trim(line.substr(0, eq)))] = Trim(value);
  }

  const std::string prefix = "Tire property file " + filename + ": ";
  auto find = [&](const std::string& sect,
                  const std::string& key) -> const std::string* {
    auto s = sections.find(sect);
    if(s == sections.end()) {
      return nullptr;
    }
    auto k = s->second.find(key);
    return k == s->second.end() ? nullptr : &k->second;
  };

  for(const char* table : {"[DEFLECTION_LOAD_CURVE]", "[BOTTOMING_CURVE]"}) {
    if(sections.count(table)) {
      throw shift::core::Exception(prefix + table + " is not supported");
    }
  }

  /* units: SI only */
  const std::map<std::string, std::vector<std::string>> si = {
    {"LENGTH", {"METER", "M"}},
    {"FORCE", {"NEWTON", "N"}},
    {"ANGLE", {"RADIAN", "RADIANS", "RAD"}},
    {"MASS", {"KG", "KILOGRAM"}},
    {"TIME", {"SECOND", "SEC", "S"}},
    {"PRESSURE", {"PASCAL", "PA"}}};
  for(const auto& [key, names] : si) {
    const std::string* unit = find("[UNITS]", key);
    if(unit && std::find(names.begin(), names.end(), Word(*unit)) ==
                 names.end()) {
      throw shift::core::Exception(prefix + "unit " + key + " = " + *unit +
                                   " is not supported, SI units only");
    }
  }

  Pac2002Coefficients p;

  /* [MODEL] */
  if(const std::string* format = find("[MODEL]", "PROPERTY_FILE_FORMAT")) {
    const std::string f = Word(*format);
    if(f != "PAC2002" && f != "MF_05") {
      throw shift::core::Exception(prefix + "PROPERTY_FILE_FORMAT " + f +
                                   " is not PAC2002");
    }
  }
  if(const std::string* mode = find("[MODEL]", "USE_MODE")) {
    const int use_mode = std::stoi(*mode);
    p.mirror = use_mode < 0;
    p.relaxation = std::abs(use_mode) >= 10;
    p.use_mode = std::abs(use_mode) % 10;
    if(p.use_mode > 4 || std::abs(use_mode) >= 20) {
      throw shift::core::Exception(prefix + "USE_MODE " + *mode +
                                   " is not supported");
    }
  }
  if(const std::string* side = find("[MODEL]", "TYRESIDE")) {
    p.measured_left = Word(*side) != "RIGHT";
  }
  if(const std::string* fe = find("[MODEL]", "FE_METHOD")) {
    p.friction_ellipsis = Word(*fe) != "NO";
  }

  using Member = double Pac2002Coefficients::*;
  struct Entry {
    const char* section;
    const char* key;
    Member member;
  };
  static const Entry kEntries[] = {
    {"[MODEL]", "VXLOW", &Pac2002Coefficients::vxlow},
    {"[MODEL]", "LONGVL", &Pac2002Coefficients::longvl},
    {"[DIMENSION]", "UNLOADED_RADIUS", &Pac2002Coefficients::unloaded_radius},
    {"[DIMENSION]", "WIDTH", &Pac2002Coefficients::width},
    {"[VERTICAL]", "VERTICAL_STIFFNESS",
     &Pac2002Coefficients::vertical_stiffness},
    {"[VERTICAL]", "VERTICAL_DAMPING", &Pac2002Coefficients::vertical_damping},
    {"[VERTICAL]", "FNOMIN", &Pac2002Coefficients::fnomin},
    {"[VERTICAL]", "QFZ1", &Pac2002Coefficients::qfz1},
    {"[VERTICAL]", "QFZ2", &Pac2002Coefficients::qfz2},
    {"[VERTICAL]", "QFZ3", &Pac2002Coefficients::qfz3},
    {"[VERTICAL]", "QPFZ1", &Pac2002Coefficients::qpfz1},
    {"[TIRE_CONDITIONS]", "IP", &Pac2002Coefficients::ip},
    {"[TIRE_CONDITIONS]", "IP_NOM", &Pac2002Coefficients::ip_nom},
#define S(k, m) {"[SCALING_COEFFICIENTS]", k, &Pac2002Coefficients::m}
    S("LFZO", lfzo), S("LCX", lcx), S("LMUX", lmux), S("LEX", lex),
    S("LKX", lkx), S("LHX", lhx), S("LVX", lvx), S("LGAX", lgax),
    S("LCY", lcy), S("LMUY", lmuy), S("LEY", ley), S("LKY", lky),
    S("LHY", lhy), S("LVY", lvy), S("LGAY", lgay), S("LTR", ltr),
    S("LRES", lres), S("LGAZ", lgaz), S("LXAL", lxal), S("LYKA", lyka),
    S("LVYKA", lvyka), S("LS", ls), S("LSGKP", lsgkp), S("LSGAL", lsgal),
    S("LGYR", lgyr), S("LMX", lmx), S("LVMX", lvmx), S("LMY", lmy),
    S("LIP", lip), S("LKYG", lkyg), S("LCZ", lcz),
#undef S
#define X(k, m) {"[LONGITUDINAL_COEFFICIENTS]", k, &Pac2002Coefficients::m}
    X("PCX1", pcx1), X("PDX1", pdx1), X("PDX2", pdx2), X("PDX3", pdx3),
    X("PEX1", pex1), X("PEX2", pex2), X("PEX3", pex3), X("PEX4", pex4),
    X("PKX1", pkx1), X("PKX2", pkx2), X("PKX3", pkx3), X("PHX1", phx1),
    X("PHX2", phx2), X("PVX1", pvx1), X("PVX2", pvx2), X("RBX1", rbx1),
    X("RBX2", rbx2), X("RCX1", rcx1), X("REX1", rex1), X("REX2", rex2),
    X("RHX1", rhx1), X("PPX1", ppx1), X("PPX2", ppx2), X("PPX3", ppx3),
    X("PPX4", ppx4),
#undef X
#define O(k, m) {"[OVERTURNING_COEFFICIENTS]", k, &Pac2002Coefficients::m}
    O("QSX1", qsx1), O("QSX2", qsx2), O("QSX3", qsx3), O("QSX4", qsx4),
    O("QSX5", qsx5), O("QSX6", qsx6), O("QSX7", qsx7), O("QSX8", qsx8),
    O("QSX9", qsx9), O("QSX10", qsx10), O("QSX11", qsx11), O("QPX1", qpx1),
#undef O
#define Y(k, m) {"[LATERAL_COEFFICIENTS]", k, &Pac2002Coefficients::m}
    Y("PCY1", pcy1), Y("PDY1", pdy1), Y("PDY2", pdy2), Y("PDY3", pdy3),
    Y("PEY1", pey1), Y("PEY2", pey2), Y("PEY3", pey3), Y("PEY4", pey4),
    Y("PKY1", pky1), Y("PKY2", pky2), Y("PKY3", pky3), Y("PHY1", phy1),
    Y("PHY2", phy2), Y("PHY3", phy3), Y("PVY1", pvy1), Y("PVY2", pvy2),
    Y("PVY3", pvy3), Y("PVY4", pvy4), Y("RBY1", rby1), Y("RBY2", rby2),
    Y("RBY3", rby3), Y("RCY1", rcy1), Y("REY1", rey1), Y("REY2", rey2),
    Y("RHY1", rhy1), Y("RHY2", rhy2), Y("RVY1", rvy1), Y("RVY2", rvy2),
    Y("RVY3", rvy3), Y("RVY4", rvy4), Y("RVY5", rvy5), Y("RVY6", rvy6),
    Y("PPY1", ppy1), Y("PPY2", ppy2), Y("PPY3", ppy3), Y("PPY4", ppy4),
#undef Y
#define R(k, m) {"[ROLLING_COEFFICIENTS]", k, &Pac2002Coefficients::m}
    R("QSY1", qsy1), R("QSY2", qsy2), R("QSY3", qsy3), R("QSY4", qsy4),
    R("QSY5", qsy5), R("QSY6", qsy6), R("QSY7", qsy7), R("QSY8", qsy8),
#undef R
#define Z(k, m) {"[ALIGNING_COEFFICIENTS]", k, &Pac2002Coefficients::m}
    Z("QBZ1", qbz1), Z("QBZ2", qbz2), Z("QBZ3", qbz3), Z("QBZ4", qbz4),
    Z("QBZ5", qbz5), Z("QBZ9", qbz9), Z("QBZ10", qbz10), Z("QCZ1", qcz1),
    Z("QDZ1", qdz1), Z("QDZ2", qdz2), Z("QDZ3", qdz3), Z("QDZ4", qdz4),
    Z("QDZ6", qdz6), Z("QDZ7", qdz7), Z("QDZ8", qdz8), Z("QDZ9", qdz9),
    Z("QEZ1", qez1), Z("QEZ2", qez2), Z("QEZ3", qez3), Z("QEZ4", qez4),
    Z("QEZ5", qez5), Z("QHZ1", qhz1), Z("QHZ2", qhz2), Z("QHZ3", qhz3),
    Z("QHZ4", qhz4), Z("QPZ1", qpz1), Z("QPZ2", qpz2), Z("SSZ1", ssz1),
    Z("SSZ2", ssz2), Z("SSZ3", ssz3), Z("SSZ4", ssz4),
#undef Z
  };
  for(const Entry& entry : kEntries) {
    if(const std::string* value = find(entry.section, entry.key)) {
      try {
        p.*(entry.member) = std::stod(*value);
      } catch(const std::exception&) {
        throw shift::core::Exception(prefix + entry.key + " = '" + *value +
                                     "' is not a number");
      }
    }
  }

  if(p.unloaded_radius <= 0 || p.vertical_stiffness <= 0 || p.fnomin <= 0) {
    throw shift::core::Exception(prefix + "UNLOADED_RADIUS, "
                                          "VERTICAL_STIFFNESS and FNOMIN must "
                                          "be positive");
  }

  /* linear vertical stiffness, unless given */
  if(!find("[VERTICAL]", "QFZ1")) {
    p.qfz1 = p.vertical_stiffness * p.unloaded_radius / p.fnomin;
  }

  /* without tire conditions the pressure dependencies are off */
  if(!find("[TIRE_CONDITIONS]", "IP") || !find("[TIRE_CONDITIONS]", "IP_NOM")) {
    p.ip = p.ip_nom = 200000.0;
    p.ppx1 = p.ppx2 = p.ppx3 = p.ppx4 = 0;
    p.ppy1 = p.ppy2 = p.ppy3 = p.ppy4 = 0;
    p.qsy8 = 0;
    p.qpfz1 = 0;
  }
  return p;
}

void shift::core::force_module::tire::Pac2002Coefficients::Mirror() {
  for(double* c : {&rhx1, &qsx1, &pey3, &phy1, &phy2, &pvy1, &pvy2, &rby3,
                   &rvy1, &rvy2, &qbz4, &qdz3, &qdz6, &qdz7, &qez4, &qhz1,
                   &qhz2, &ssz1}) {
    *c = -*c;
  }
}

/* ---------------------------------------------------------------------------
model
--------------------------------------------------------------------------- */

static double Signum(double x) {
  return (x > 0.0) - (x < 0.0);
}

double shift::core::force_module::tire::Pac2002::SineStep(double x, double x1,
                                                          double y1, double x2,
                                                          double y2) {
  if(x <= x1) {
    return y1;
  }
  if(x >= x2) {
    return y2;
  }
  const double xx = (x - x1) / (x2 - x1);
  return y1 + (y2 - y1) * (xx - std::sin(2.0 * M_PI * xx) / (2.0 * M_PI));
}

shift::core::force_module::tire::Pac2002::Pac2002(
  const Pac2002Coefficients& coefficients, double gravity)
  : p_(coefficients)
  , gravity_(gravity)
  , gamma_limit_(3.0 * M_PI / 180.0) {
  fz0_prime_ = p_.fnomin * p_.lfzo;
  const double pi0_prime = p_.ip_nom * p_.lip;
  dpi_ = (p_.ip - pi0_prime) / pi0_prime;
}

double shift::core::force_module::tire::Pac2002::NormalStiffnessForce(
  double depth, double gamma) const {
  const double r0 = p_.unloaded_radius;
  return (p_.qfz1 * depth / r0 + p_.qfz2 * std::pow(depth / r0, 2) +
          p_.qfz3 * std::pow(gamma, 2) * depth / r0) *
         (1.0 + p_.qpfz1 * dpi_) * p_.lcz * p_.fnomin;
}

void shift::core::force_module::tire::Pac2002::CalcFxyMz(
  double& Fx, double& Fy, double& Mz, double kappa, double alpha, double Fz,
  double gamma) const {
  const double dfz0 = dfz0_;
  const double dpi = dpi_;

  /* longitudinal, pure slip */
  const double Cx = p_.pcx1 * p_.lcx;
  const double Shx = (p_.phx1 + p_.phx2 * dfz0) * p_.lhx;
  const double Svx = Fz * (p_.pvx1 + p_.pvx2 * dfz0) * p_.lvx * p_.lmux;
  const double kappa_x = kappa + Shx;
  const double gamma_x = gamma * p_.lgax;
  double Ex = (p_.pex1 + p_.pex2 * dfz0 + p_.pex3 * std::pow(dfz0, 2)) *
              (1.0 - p_.pex4 * Signum(kappa_x)) * p_.lex;
  Ex = std::min(Ex, 1.0);
  const double mu_x =
    std::abs((p_.pdx1 + p_.pdx2 * dfz0) *
             (1.0 + p_.ppx3 * dpi + p_.ppx4 * std::pow(dpi, 2)) *
             (1.0 - p_.pdx3 * std::pow(gamma_x, 2)) * p_.lmux);
  const double Dx = mu_x * Fz;
  const double Kx = Fz * (p_.pkx1 + p_.pkx2 * dfz0) * std::exp(p_.pkx3 * dfz0) *
                    (1.0 + p_.ppx1 * dpi + p_.ppx2 * std::pow(dpi, 2)) *
                    p_.lkx;
  const double Bx = Kx / (Cx * Dx + 0.1);
  const double X1 =
    std::clamp(Bx * kappa_x, -M_PI_2 + 0.01, M_PI_2 - 0.01);
  const double Fx0 =
    Dx * std::sin(Cx * std::atan(X1 - Ex * (X1 - std::atan(X1)))) + Svx;

  /* lateral, pure slip */
  const double gamma_y = gamma * p_.lgay;
  const double Cy = p_.pcy1 * p_.lcy;
  const double Ky0 =
    p_.pky1 * p_.fnomin * (1 + p_.ppy1 * dpi) *
    std::sin(2.0 * std::atan(Fz / (p_.pky2 * fz0_prime_ *
                                   (1.0 + p_.ppy2 * dpi)))) *
    p_.lfzo * p_.lmuy;
  const double Ky = Ky0 * (1.0 - p_.pky3 * std::fabs(gamma_y));
  const double Shy = (p_.phy1 + p_.phy2 * dfz0) * p_.lhy +
                     p_.phy3 * gamma_y * p_.lkyg;
  const double alpha_y = alpha + Shy;
  const double Svy =
    Fz *
    ((p_.pvy1 + p_.pvy2 * dfz0) * p_.lvy +
     (p_.pvy3 + p_.pvy4 * dfz0) * gamma_y * p_.lkyg) *
    p_.lmuy;
  double Ey = (p_.pey1 + p_.pey2 * dfz0) *
              (1.0 - (p_.pey3 + p_.pey4 * gamma_y) * Signum(alpha_y)) * p_.ley;
  Ey = std::min(Ey, 1.0);
  const double mu_y =
    std::abs((p_.pdy1 + p_.pdy2 * dfz0) *
             (1.0 + p_.ppy3 * dpi + p_.ppy4 * std::pow(dpi, 2)) *
             (1.0 + p_.pdy3 * std::pow(gamma_y, 2)) * p_.lmuy);
  const double Dy = mu_y * Fz;
  const double By = Ky / (Cy * Dy + 0.1);
  const double Y1 =
    std::clamp(By * alpha_y, -M_PI_2 + 0.01, M_PI_2 - 0.01);
  const double Fy0 =
    Dy * std::sin(Cy * std::atan(Y1 - Ey * (Y1 - std::atan(Y1)))) + Svy;

  /* aligning moment: pneumatic trail and residual moment */
  const double R0 = p_.unloaded_radius;
  const double gamma_z = gamma * p_.lgaz;
  const double Sht = p_.qhz1 + p_.qhz2 * dfz0 +
                     (p_.qhz3 + p_.qhz4 * dfz0) * gamma_z;
  const double Shf = Shy + Svy / Ky;
  const double alpha_r = alpha + Shf;
  const double alpha_t = alpha + Sht;
  const double Ct = p_.qcz1;
  const double Bt =
    std::abs((p_.qbz1 + p_.qbz2 * dfz0 + p_.qbz3 * std::pow(dfz0, 2)) *
             (1.0 + p_.qbz4 * gamma_z + p_.qbz5 * std::abs(gamma_z)) * p_.lky /
             p_.lmuy);
  double Et = (p_.qez1 + p_.qez2 * dfz0 + p_.qez3 * std::pow(dfz0, 2)) *
              (1.0 + (p_.qez4 + p_.qez5 * gamma_z) *
                       ((2.0 / M_PI) * std::atan(Bt * Ct * alpha_t)));
  Et = std::min(Et, 1.0);
  const double Dt = Fz * (p_.qdz1 + p_.qdz2 * dfz0) * (1.0 - p_.qpz1 * dpi) *
                    (1.0 + p_.qdz3 * gamma_z + p_.qdz4 * std::pow(gamma_z, 2)) *
                    R0 / fz0_prime_ * p_.ltr;
  const double t =
    Dt *
    std::cos(Ct * std::atan(Bt * alpha_t -
                            Et * (Bt * alpha_t - std::atan(Bt * alpha_t)))) *
    std::cos(alpha);
  const double Br = p_.qbz9 * p_.lky / p_.lmuy + p_.qbz10 * By * Cy;
  const double Dr = Fz *
                    ((p_.qdz6 + p_.qdz7 * dfz0) * p_.lres +
                     (p_.qdz8 + p_.qdz9 * dfz0) * (1.0 + p_.qpz2 * dpi) *
                       gamma_z) *
                    R0 * p_.lmuy;
  const double Cr = 1.0;
  const double Mzr = Dr * std::cos(Cr * std::atan(Br * alpha_r)) *
                     std::cos(alpha);

  switch(p_.use_mode) {
    case 1:
      Fx = Fx0;
      Fy = 0;
      Mz = 0;
      break;
    case 2:
      Fx = 0;
      Fy = Fy0;
      Mz = -t * Fy0 + Mzr;
      break;
    case 3:
      Fx = Fx0;
      Fy = Fy0;
      Mz = -t * Fy0 + Mzr;
      break;
    case 4:
      if(p_.friction_ellipsis) {
        /* combined slip by the friction ellipsis (Adams). Chrono's
        mu_x_c / mu_x_act and mu_y_c / mu_y_act, rewritten without the
        quotients that are 0 / 0 at zero slip; tan(beta) = |alpha_s| /
        |kappa_c| */
        const double kappa_c = std::abs(kappa + Shx + Svx / Kx);
        const double alpha_s = std::abs(std::sin(alpha + Shy + Svy / Ky));
        const double mu_x_act = std::abs((Fx0 - Svx) / Fz);
        const double mu_y_act = std::abs((Fy0 - Svy) / Fz);
        const double mu_x_max = Dx / Fz;
        const double mu_y_max = Dy / Fz;
        const double hx = std::hypot(kappa_c, mu_x_act * alpha_s / mu_y_max);
        const double hy = std::hypot(mu_y_act * kappa_c / mu_x_max, alpha_s);
        Fx = hx > 0.0 ? Fx0 * kappa_c / hx : Fx0;
        Fy = hy > 0.0 ? Fy0 * alpha_s / hy : Fy0;
        Mz = -t * Fy + Mzr;
      } else {
        /* combined slip by the Pacejka coefficients */
        const double Shxa = p_.rhx1;
        const double Cxa = p_.rcx1;
        const double alpha_s = alpha + Shxa;
        const double Exa = std::min(p_.rex1 + p_.rex2 * dfz0, 1.0);
        const double Bxa =
          std::abs(p_.rbx1 * std::cos(std::atan(p_.rbx2 * kappa)) * p_.lxal);
        const double Gxa =
          std::cos(Cxa * std::atan(Bxa * alpha_s -
                                   Exa * (Bxa * alpha_s -
                                          std::atan(Bxa * alpha_s)))) /
          std::cos(Cxa * std::atan(Bxa * Shxa -
                                   Exa * (Bxa * Shxa - std::atan(Bxa * Shxa))));
        Fx = Fx0 * Gxa;

        const double Shyk = p_.rhy1 + p_.rhy2 * dfz0;
        const double kappa_s = kappa + Shyk;
        const double Cyk = p_.rcy1;
        const double Eyk = std::min(p_.rey1 + p_.rey2 * dfz0, 1.0);
        const double Byk = p_.rby1 *
                           std::cos(std::atan(p_.rby2 * (alpha - p_.rby3))) *
                           p_.lyka;
        const double Dvyk = mu_y * Fz *
                            (p_.rvy1 + p_.rvy2 * dfz0 + p_.rvy3 * gamma) *
                            std::cos(std::atan(p_.rvy4 * alpha));
        const double Svyk =
          Dvyk * std::sin(p_.rvy5 * std::atan(p_.rvy6 * kappa)) * p_.lvyka;
        const double Gyk =
          std::cos(Cyk * std::atan(Byk * kappa_s -
                                   Eyk * (Byk * kappa_s -
                                          std::atan(Byk * kappa_s)))) /
          std::cos(Cyk * std::atan(Byk * Shyk -
                                   Eyk * (Byk * Shyk - std::atan(Byk * Shyk))));
        Fy = Fy0 * Gyk + Svyk;

        /* MF 5.2: the sign of alpha_t (Chrono uses the sign of kappa) */
        const double alpha_teq =
          std::atan(std::sqrt(std::pow(std::tan(alpha_t), 2) +
                              std::pow(Kx / Ky, 2) * std::pow(kappa, 2))) *
          Signum(alpha_t);
        const double alpha_req =
          std::atan(std::sqrt(std::pow(std::tan(alpha_r), 2) +
                              std::pow(Kx / Ky, 2) * std::pow(kappa, 2))) *
          Signum(alpha_r);
        const double tc =
          Dt *
          std::cos(Ct * std::atan(Bt * alpha_teq -
                                  Et * (Bt * alpha_teq -
                                        std::atan(Bt * alpha_teq)))) *
          std::cos(alpha);
        const double s = (p_.ssz1 + p_.ssz2 * Fy / fz0_prime_ +
                          (p_.ssz3 + p_.ssz4 * dfz0) * gamma) *
                         R0 * p_.ls;
        const double Mzrc =
          Dr * std::cos(Cr * std::atan(Br * alpha_req)) * std::cos(alpha);
        Mz = -tc * (Fy - Svyk) + Mzrc + s * Fx;
      }
      break;
    default:
      Fx = 0;
      Fy = 0;
      Mz = 0;
  }
}

double shift::core::force_module::tire::Pac2002::CalcMx(double Fy, double Fz,
                                                        double gamma) const {
  return p_.unloaded_radius * Fz *
         (p_.qsx3 * Fy / fz0_prime_ +
          p_.qsx4 * std::cos(p_.qsx5 * std::atan(std::pow(Fz / fz0_prime_, 2))) *
            std::sin(p_.qsx7 * gamma +
                     p_.qsx8 * std::atan(p_.qsx9 * Fy / fz0_prime_)) +
          (p_.qsx10 * std::atan(p_.qsx11 * Fz / fz0_prime_) -
           p_.qsx2 * (1.0 + p_.qpx1 * dpi_)) *
            gamma +
          p_.qsx1 * p_.lvmx) *
         p_.lmx;
}

double shift::core::force_module::tire::Pac2002::CalcMy(double Fx, double Fz,
                                                        double gamma,
                                                        double vx) const {
  const double V0 = std::sqrt(gravity_ * p_.unloaded_radius);
  return Fz * p_.unloaded_radius *
         (p_.qsy1 + p_.qsy2 * Fx / p_.fnomin + p_.qsy3 * std::fabs(vx / V0) +
          p_.qsy4 * std::pow(vx / V0, 4) + p_.qsy5 * std::pow(gamma, 2) +
          p_.qsy6 * std::pow(gamma, 2) * Fz / p_.fnomin) *
         (std::pow(Fz / p_.fnomin, p_.qsy7) *
          std::pow(p_.ip / p_.ip_nom, p_.qsy8)) *
         p_.lmy;
}

void shift::core::force_module::tire::Pac2002::Evaluate(
  const Contact& contact, const double* states, double* state_rates,
  ksl_coscrew_t* force_w) {

  *force_w = ksl_coscrew(0, 0, 0, 0, 0, 0);
  const double brx = states[0]; // bristle deflections, ISO W axes
  const double bry = states[1];
  state_rates[0] = -100.0 * brx;
  state_rates[1] = -100.0 * bry;
  if(!contact.in_contact) {
    return;
  }

  /* kinematics in the ISO W axes: (x, y, z)_ISO = (x, -y, -z)_SAE; the
  spin axis of the ISO wheel points left, the opposite of the hub y axis */
  const ksl_vec3_t& v = contact.center_velocity_w;
  const double vx = v.x;
  const double vy = -v.y;
  const double vz = -v.z;
  const double omega = -contact.omega;
  const double gamma =
    std::clamp(std::atan2(contact.spin_axis_w.z, contact.spin_axis_w.y),
               -gamma_limit_, gamma_limit_);

  /* normal force; the depth decreases with a positive (upward) vz */
  const double depth = contact.depth;
  const double Fz =
    NormalStiffnessForce(depth, gamma) + p_.vertical_damping * (-vz);
  if(Fz <= 0.0) {
    return;
  }
  dfz0_ = (Fz - fz0_prime_) / fz0_prime_;

  /* slips; R_eff is Rill's estimate, as in Chrono */
  const double r0 = p_.unloaded_radius;
  const double r_eff = (2.0 * r0 + (r0 - depth)) / 3.0;
  const double abs_vx = std::abs(vx);
  const double vsx = vx - omega * r_eff;
  const double vsy = vy;
  const double epsilon = 0.1;
  const double kappa = std::clamp(-vsx / (abs_vx + epsilon), -1.0, 1.0);
  const double alpha = std::clamp(std::atan2(vsy, abs_vx + epsilon),
                                  -M_PI_2 + 0.01, M_PI_2 - 0.01);

  double Fx = 0, Fy = 0, Mx = 0, My = 0, Mz = 0;
  switch(p_.use_mode) {
    case 1:
      CalcFxyMz(Fx, Fy, Mz, kappa, 0.0, Fz, gamma);
      My = CalcMy(Fx, Fz, gamma, abs_vx);
      break;
    case 2:
      CalcFxyMz(Fx, Fy, Mz, 0.0, alpha, Fz, gamma);
      Mx = CalcMx(Fy, Fz, gamma);
      break;
    case 3:
    case 4: {
      /* Dahl friction at low speed: bristle deflections b,
      db/dt = vs - sigma0 b |vs| / fc, F = -(sigma0 b + sigma1 db/dt),
      limited to the friction circle. The rate sigma0 |vs| / fc grows
      without bound as the load vanishes; Chrono integrates it implicitly,
      here the integrators are explicit, so the saturation load of the
      bristles is at least kMinBristleLoad FNOMIN (the force stays limited
      by the actual load) */
      const double fc = Fz;
      const double fb = std::max(Fz, kMinBristleLoad * p_.fnomin);
      const double brx_dot = vsx - kSigma0 * brx * std::fabs(vsx) / fb;
      const double bry_dot = vsy - kSigma0 * bry * std::fabs(vsy) / fb;
      state_rates[0] = brx_dot;
      state_rates[1] = bry_dot;
      double Fx0 = -(kSigma0 * brx + kSigma1 * brx_dot);
      double Fy0 = -(kSigma0 * bry + kSigma1 * bry_dot);
      const double f0 = std::hypot(Fx0, Fy0);
      if(f0 > fc) {
        Fx0 *= fc / f0;
        Fy0 *= fc / f0;
      }

      double Fx_ss = 0, Fy_ss = 0;
      CalcFxyMz(Fx_ss, Fy_ss, Mz, kappa, alpha, Fz, gamma);
      const double blend =
        SineStep(abs_vx, kFrictionBlendBegin, 0.0, kFrictionBlendEnd, 1.0);
      Fx = (1.0 - blend) * Fx0 + blend * Fx_ss;
      Fy = (1.0 - blend) * Fy0 + blend * Fy_ss;
      My = CalcMy(Fx, Fz, gamma, abs_vx);
      Mx = CalcMx(Fy, Fz, gamma);
    } break;
    default:
      break;
  }

  /* rolling resistance opposes the rolling, fading in below VXLOW */
  const double rolling = omega * r_eff;
  My = -My * std::copysign(SineStep(std::fabs(rolling), 0.0, 0.0, p_.vxlow,
                                    1.0),
                           rolling);

  /* ISO W to the SAE w frame of tire::Contact */
  force_w->lin.x = Fx;
  force_w->lin.y = -Fy;
  force_w->lin.z = -Fz;
  force_w->ang.x = Mx;
  force_w->ang.y = -My;
  force_w->ang.z = -Mz;
}
