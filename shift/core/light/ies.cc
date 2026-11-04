#include "shift/core/light/ies.h"

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <fstream>
#include <sstream>

#include "shift/core/context.h"
#include "shift/core/exception.h"
#include "shift/core/utility.h"

bool shift::core::light::IES::registered_ =
  shift::core::Factory<shift::core::light::Base>::get()->RegisterModule(
    "ies", shift::core::light::IES::Create);

std::unique_ptr<shift::core::light::IES> shift::core::light::IES::Create() {
  return std::make_unique<shift::core::light::IES>();
}

void shift::core::light::IES::DeclareParameters() {
  shift::core::light::Base::DeclareParameters();
  DeclareParameter("filename", &filename_,
               "IES LM-63 photometric file of the luminaire, next to the "
               "model file or in an include directory.",
               true);
}

std::string shift::core::light::IES::FindFile(const std::string& name) const {
  if(!name.empty() && name[0] == '/') {
    return name;
  }
  const std::string file =
    shift::core::utility::GetParentPath(context_->get_filename()) + "/" + name;
  if(shift::core::utility::FileExists(file)) {
    return file;
  }
  for(const auto& directory :
      (*context_)["model_parameters"]["include_directories"]
        .get<std::vector<std::string>>()) {
    const std::string candidate = directory + "/" + name;
    if(shift::core::utility::FileExists(candidate)) {
      return candidate;
    }
  }
  throw shift::core::Exception("Light " + get_name() + ": file " + name +
                               " not found");
}

void shift::core::light::IES::Load() {
  Read(FindFile(filename_));
}

/* IES LM-63 (1986, 1991, 1995 and 2002): keyword lines up to the TILT line,
then numbers separated by blanks, commas or line breaks:
  [lamp to luminaire geometry, tilt angle count n, n angles, n factors]
                                                     (only if TILT=INCLUDE)
  lamp count, lumens per lamp, candela multiplier, vertical angle count,
  horizontal angle count, photometric type, units type, width, length,
  height, ballast factor, ballast lamp photometric factor, input watts,
  the vertical angles, the horizontal angles, and the candela values for
  each horizontal angle over the vertical angles */
void shift::core::light::IES::Read(const std::string& path) {
  const std::string prefix = "Light " + get_name() + ": " + path + ": ";
  std::ifstream file(path);
  if(!file) {
    throw shift::core::Exception(prefix + "could not open the file");
  }
  std::string line;
  std::string tilt;
  while(std::getline(file, line)) {
    const size_t start = line.find_first_not_of(" \t");
    if(start != std::string::npos && line.compare(start, 5, "TILT=") == 0) {
      tilt = line.substr(start + 5);
      tilt.erase(tilt.find_last_not_of(" \t\r") + 1);
      break;
    }
  }
  if(tilt.empty()) {
    throw shift::core::Exception(prefix + "no TILT= line; not an IES LM-63 "
                                 "file");
  }
  std::stringstream rest;
  rest << file.rdbuf();
  std::string text = rest.str();
  std::replace(text.begin(), text.end(), ',', ' ');
  std::istringstream numbers(text);
  auto next = [&]() {
    double value;
    if(!(numbers >> value)) {
      throw shift::core::Exception(prefix + "the file ends early or holds a "
                                   "value that is not a number");
    }
    return value;
  };
  if(tilt == "INCLUDE") {
    next(); // lamp to luminaire geometry
    const int count = int(next());
    for(int i = 0; i < 2 * count; i++) {
      next(); // tilt angles and factors: they change the output of lamps
              // that are tilted, ignored here
    }
  } else if(tilt != "NONE") {
    // the tilt data is in another file; it only scales the output of a
    // tilted lamp, so the distribution is read without it
  }
  next(); // lamp count
  next(); // lumens per lamp
  const double multiplier = next();
  const int vertical_count = int(next());
  const int horizontal_count = int(next());
  const int photometric_type = int(next());
  for(int i = 0; i < 7; i++) {
    next(); // units, width, length, height, ballast factor, ballast lamp
            // photometric factor, input watts
  }
  if(photometric_type != 1) {
    throw shift::core::Exception(prefix + "only type C photometry (1) is "
                                 "read, not type " +
                                 std::to_string(photometric_type));
  }
  if(vertical_count < 1 || horizontal_count < 1) {
    throw shift::core::Exception(prefix + "no angles");
  }
  vertical_.resize(vertical_count);
  horizontal_.resize(horizontal_count);
  for(auto& v : vertical_) {
    v = next();
  }
  for(auto& h : horizontal_) {
    h = next();
  }
  candela_.resize(size_t(vertical_count) * horizontal_count);
  peak_ = 0.0;
  for(auto& c : candela_) {
    c = multiplier * next();
    peak_ = std::max(peak_, c);
  }
  if(!std::is_sorted(vertical_.begin(), vertical_.end()) ||
     !std::is_sorted(horizontal_.begin(), horizontal_.end())) {
    throw shift::core::Exception(prefix + "the angles must increase");
  }
  if(!(peak_ > 0.0)) {
    throw shift::core::Exception(prefix + "the luminaire emits no light");
  }
}

/* index i and fraction t of x between a[i] and a[i + 1], clamped to a */
static void Bracket(const std::vector<double>& a, double x, size_t& i,
                    double& t) {
  if(a.size() < 2 || x <= a.front()) {
    i = 0;
    t = 0.0;
    return;
  }
  if(x >= a.back()) {
    i = a.size() - 2;
    t = 1.0;
    return;
  }
  i = size_t(std::upper_bound(a.begin(), a.end(), x) - a.begin()) - 1;
  const double span = a[i + 1] - a[i];
  t = span > 0.0 ? (x - a[i]) / span : 0.0;
}

double shift::core::light::IES::Distribution(double vertical,
                                             double horizontal) const {
  if(candela_.empty()) {
    return 0.0;
  }
  const double v = vertical * 180.0 / M_PI;
  if(v < vertical_.front() - 1e-9 || v > vertical_.back() + 1e-9) {
    return 0.0; // no light outside the vertical angles of the file
  }
  /* the horizontal angles a file covers tell its symmetry */
  double h = std::fmod(horizontal * 180.0 / M_PI, 360.0);
  if(h < 0.0) {
    h += 360.0;
  }
  const double first = horizontal_.front();
  const double last = horizontal_.back();
  if(horizontal_.size() == 1) {
    h = first; // the same in all horizontal directions
  } else if(first == 0.0 && last == 90.0) { // symmetric in each quadrant
    h = h > 180.0 ? 360.0 - h : h;
    h = h > 90.0 ? 180.0 - h : h;
  } else if(first == 0.0 && last == 180.0) { // about the 0-180 plane
    h = h > 180.0 ? 360.0 - h : h;
  } else if(first == 90.0 && last == 270.0) { // about the 90-270 plane
    h = h < 90.0 ? 180.0 - h : (h > 270.0 ? 540.0 - h : h);
  }
  size_t i, j;
  double s, t;
  Bracket(horizontal_, h, i, s);
  Bracket(vertical_, v, j, t);
  const size_t n = vertical_.size();
  auto at = [&](size_t hi, size_t vi) {
    hi = std::min(hi, horizontal_.size() - 1);
    vi = std::min(vi, n - 1);
    return candela_[hi * n + vi];
  };
  const double c0 = (1.0 - t) * at(i, j) + t * at(i, j + 1);
  const double c1 = (1.0 - t) * at(i + 1, j) + t * at(i + 1, j + 1);
  return ((1.0 - s) * c0 + s * c1) / peak_;
}
