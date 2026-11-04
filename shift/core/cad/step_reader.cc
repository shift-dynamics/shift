#include "shift/core/cad/step_reader.h"

#include "shift/config.h"
#include "shift/core/exception.h"

#if SHIFT_HAVE_STEP

#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <iostream>
#include <sstream>
#include <unordered_map>
#include <unordered_set>

#include <cleditor/STEPfile.h>
#include <clstepcore/STEPattribute.h>
#include <clstepcore/STEPcomplex.h>
#include <clstepcore/Registry.h>
#include <clstepcore/instmgr.h>
#include <clstepcore/sdai.h>
#include <schema.h>

#include "shift/core/log.h"

#endif

namespace shift {
namespace core {
namespace cad {

#if !SHIFT_HAVE_STEP

bool StepSupported() {
  return false;
}

Model ReadStep(const std::string& path) {
  throw shift::core::Exception(
    path + ": shift was built without STEP support (meson option step, "
           "which needs STEPcode)");
}

#else

bool StepSupported() {
  return true;
}

/*!
@brief value of an attribute, parsed from its text in the exchange file
format (ISO 10303-21): unset ($ or *), a reference (#12), a number, a string,
an enumeration (.T. or T), a typed value (LENGTH_MEASURE(25.4)) or a list
*/
struct StepValue {
  enum class Kind { kUnset, kRef, kNumber, kString, kEnum, kTyped, kList };
  Kind kind = Kind::kUnset;
  int ref = 0;
  double number = 0.0;
  std::string text; //!< string, enumeration or type name
  std::vector<StepValue> items; //!< list, or the value of a typed value

  bool IsUnset() const {
    return kind == Kind::kUnset;
  }
  /*! the number of a number or of a typed number */
  double Number() const {
    if(kind == Kind::kTyped && items.size() == 1) {
      return items[0].Number();
    }
    return kind == Kind::kNumber ? number : 0.0;
  }
  bool True() const {
    return kind == Kind::kEnum && (text == "T" || text == "TRUE");
  }

  static StepValue Parse(const std::string& s) {
    size_t i = 0;
    StepValue v = ParseAt(s, i);
    return v;
  }

private:
  static void Skip(const std::string& s, size_t& i) {
    while(i < s.size() && std::isspace(static_cast<unsigned char>(s[i]))) {
      ++i;
    }
  }

  static StepValue ParseAt(const std::string& s, size_t& i) {
    StepValue v;
    Skip(s, i);
    if(i >= s.size()) {
      return v;
    }
    const char c = s[i];
    if(c == '$' || c == '*') {
      ++i;
    } else if(c == '#') {
      ++i;
      size_t start = i;
      while(i < s.size() && std::isdigit(static_cast<unsigned char>(s[i]))) {
        ++i;
      }
      v.kind = Kind::kRef;
      v.ref = std::atoi(s.substr(start, i - start).c_str());
    } else if(c == '(') {
      ++i;
      v.kind = Kind::kList;
      Skip(s, i);
      if(i < s.size() && s[i] == ')') {
        ++i;
        return v;
      }
      while(i < s.size()) {
        v.items.push_back(ParseAt(s, i));
        Skip(s, i);
        if(i < s.size() && s[i] == ',') {
          ++i;
          continue;
        }
        if(i < s.size() && s[i] == ')') {
          ++i;
        }
        break;
      }
    } else if(c == '\'') {
      ++i;
      v.kind = Kind::kString;
      while(i < s.size()) {
        if(s[i] == '\'') {
          if(i + 1 < s.size() && s[i + 1] == '\'') {
            v.text += '\'';
            i += 2;
            continue;
          }
          ++i;
          break;
        }
        v.text += s[i++];
      }
    } else if(c == '-' || c == '+' || c == '.' ||
              std::isdigit(static_cast<unsigned char>(c))) {
      if(c == '.' && i + 1 < s.size() &&
         std::isalpha(static_cast<unsigned char>(s[i + 1]))) {
        // .ENUM.
        ++i;
        size_t start = i;
        while(i < s.size() && s[i] != '.') {
          ++i;
        }
        v.kind = Kind::kEnum;
        v.text = s.substr(start, i - start);
        ++i;
      } else {
        const char* begin = s.c_str() + i;
        char* end = nullptr;
        v.number = std::strtod(begin, &end);
        v.kind = Kind::kNumber;
        i += end - begin;
      }
    } else if(std::isalpha(static_cast<unsigned char>(c)) || c == '_') {
      size_t start = i;
      while(i < s.size() && (std::isalnum(static_cast<unsigned char>(s[i])) ||
                             s[i] == '_')) {
        ++i;
      }
      std::string word = s.substr(start, i - start);
      Skip(s, i);
      if(i < s.size() && s[i] == '(') {
        // typed value: NAME(value)
        ++i;
        v.kind = Kind::kTyped;
        v.text = word;
        v.items.push_back(ParseAt(s, i));
        Skip(s, i);
        if(i < s.size() && s[i] == ')') {
          ++i;
        }
      } else {
        v.kind = Kind::kEnum;
        v.text = word;
      }
    } else {
      ++i; // unexpected character
    }
    return v;
  }
};

using Instance = SDAI_Application_instance;

/*!
@brief transforms the instances read by STEPcode into a Model; see ReadStep
*/
class StepReader {
public:
  explicit StepReader(const std::string& path)
    : registry_(SchemaInit), path_(path) {
  }

  Model Read();

private:
  /* lengths to meters and angles to radians, of a representation */
  struct Units {
    double length = 1e-3;
    double angle = 1.0;
  };

  Registry registry_;
  InstMgr instances_;
  std::string path_;
  Model model_;
  Units units_;

  std::unordered_map<int, int> curves_;
  std::unordered_map<int, int> surfaces_;
  std::unordered_map<int, int> vertices_;
  std::unordered_map<int, int> edges_;
  std::unordered_map<int, int> faces_;
  std::unordered_map<int, int> shells_;
  std::unordered_map<int, int> bodies_;
  std::unordered_map<int, int> shapes_;
  /* scale of the parameters (u, v) of each surface from the file's to the
   * model's, by surface index */
  std::vector<Vec2> parameter_scale_;
  /* scale of the parameter of each 3D curve, by curve index */
  std::vector<double> curve_parameter_scale_;
  std::unordered_set<std::string> warned_;

  /* access to instances and attributes */
  Instance* Get(int id);
  Instance* Get(const StepValue& v) {
    return v.kind == StepValue::Kind::kRef ? Get(v.ref) : nullptr;
  }
  bool IsA(Instance* e, const char* entity);
  static STEPattribute* FindAttribute(Instance* e, const char* name);
  StepValue Attribute(Instance* e, const char* name);
  Instance* Ref(Instance* e, const char* name) {
    return Get(Attribute(e, name));
  }
  double Real(Instance* e, const char* name) {
    return Attribute(e, name).Number();
  }
  std::string String(Instance* e, const char* name) {
    StepValue v = Attribute(e, name);
    return v.kind == StepValue::Kind::kString ? v.text : std::string();
  }
  bool Bool(Instance* e, const char* name) {
    return Attribute(e, name).True();
  }
  std::vector<Instance*> Refs(Instance* e, const char* name);
  static std::string Label(Instance* e);
  void Warn(Instance* e, const std::string& message);

  /* units */
  Units UnitsOf(Instance* representation);
  double UnitFactor(Instance* unit, int depth = 0);

  /* geometry */
  template <int D>
  bool Point(Instance* e, const std::array<double, D>& scale,
             std::array<double, D>& p);
  template <int D>
  bool Direction(Instance* e, std::array<double, D>& d);
  Frame Placement(Instance* e);
  Frame Placement2(Instance* e, const Vec2& scale);
  template <int D>
  bool ConvertCurve(Instance* e, const std::array<double, D>& scale,
                    Curve<D>& curve, double& parameter_scale);
  template <int D>
  bool BSplineCurve(Instance* e, const std::array<double, D>& scale,
                    NurbsCurve<D>& nurbs);
  bool Knots(Instance* e, const char* multiplicities, const char* knots,
             int degree, int count, std::vector<double>& out);
  int Curve3d(Instance* e);
  int SurfaceOf(Instance* e);
  bool BSplineSurface(Instance* e, NurbsSurface& nurbs);

  /* topology */
  int VertexOf(Instance* e);
  int EdgeOf(Instance* e);
  void AddPcurve(Edge& edge, Instance* pcurve);
  int FaceOf(Instance* e);
  int ShellOf(Instance* e);
  int BodyOf(Instance* e);
  void AssignPcurves();

  /* shapes */
  int ShapeOf(Instance* representation);
  void ReadShapes();
  void ReadNames();
  void ReadRelationships();
  void ReadColors();
  bool FindColor(Instance* e, Color& color, int depth);
};

Instance* StepReader::Get(int id) {
  MgrNode* node = instances_.FindFileId(id);
  return node ? node->GetApplication_instance() : nullptr;
}

bool StepReader::IsA(Instance* e, const char* entity) {
  if(!e) {
    return false;
  }
  const EntityDescriptor* d = registry_.FindEntity(entity);
  return d && e->IsA(d);
}

/* an attribute is named as declared ("definition"), or, where a subtype
redeclares it, also qualified by the supertype that declares it
("property_definition_representation.definition"); the redeclaration holds no
value (*), so a match with a value is preferred */
static bool AttributeNameMatches(const char* attribute, const char* name) {
  if(!strcasecmp(attribute, name)) {
    return true;
  }
  const char* dot = std::strrchr(attribute, '.');
  return dot && !strcasecmp(dot + 1, name);
}

STEPattribute* StepReader::FindAttribute(Instance* e, const char* name) {
  STEPattribute* found = nullptr;
  auto search = [&](Instance* part) {
    part->ResetAttributes();
    STEPattribute* a;
    while((a = part->NextAttribute())) {
      if(AttributeNameMatches(a->Name(), name)) {
        if(a->asStr() != "*") {
          return a;
        }
        if(!found) {
          found = a;
        }
      }
    }
    return static_cast<STEPattribute*>(nullptr);
  };
  if(e->IsComplex()) {
    for(STEPcomplex* p = static_cast<STEPcomplex*>(e)->head; p; p = p->sc) {
      if(STEPattribute* a = search(p)) {
        return a;
      }
    }
    return found;
  }
  if(STEPattribute* a = search(e)) {
    return a;
  }
  return found;
}

StepValue StepReader::Attribute(Instance* e, const char* name) {
  if(!e) {
    return StepValue();
  }
  STEPattribute* a = FindAttribute(e, name);
  if(!a) {
    return StepValue();
  }
  return StepValue::Parse(a->asStr());
}

std::vector<Instance*> StepReader::Refs(Instance* e, const char* name) {
  std::vector<Instance*> out;
  StepValue v = Attribute(e, name);
  for(const auto& item : v.items) {
    if(Instance* r = Get(item)) {
      out.push_back(r);
    }
  }
  return out;
}

std::string StepReader::Label(Instance* e) {
  if(!e) {
    return "(missing entity)";
  }
  std::string name = e->EntityName();
  std::transform(name.begin(), name.end(), name.begin(), ::toupper);
  return "#" + std::to_string(e->StepFileId()) + " " + name;
}

void StepReader::Warn(Instance* e, const std::string& message) {
  std::string w = Label(e) + ": " + message;
  if(warned_.insert(w).second) {
    model_.warnings.push_back(w);
  }
}

/* units */

double StepReader::UnitFactor(Instance* unit, int depth) {
  if(!unit || depth > 8) {
    return 1.0;
  }
  if(IsA(unit, "si_unit")) {
    StepValue prefix = Attribute(unit, "prefix");
    double f = 1.0;
    if(prefix.kind == StepValue::Kind::kEnum) {
      static const std::unordered_map<std::string, double> prefixes = {
        {"EXA", 1e18},  {"PETA", 1e15}, {"TERA", 1e12}, {"GIGA", 1e9},
        {"MEGA", 1e6},  {"KILO", 1e3},  {"HECTO", 1e2}, {"DECA", 1e1},
        {"DECI", 1e-1}, {"CENTI", 1e-2}, {"MILLI", 1e-3}, {"MICRO", 1e-6},
        {"NANO", 1e-9}, {"PICO", 1e-12}, {"FEMTO", 1e-15}, {"ATTO", 1e-18}};
      std::string p = prefix.text;
      std::transform(p.begin(), p.end(), p.begin(), ::toupper);
      auto it = prefixes.find(p);
      if(it != prefixes.end()) {
        f = it->second;
      }
    }
    return f;
  }
  if(IsA(unit, "conversion_based_unit")) {
    Instance* m = Ref(unit, "conversion_factor");
    double value = Attribute(m, "value_component").Number();
    return value * UnitFactor(Ref(m, "unit_component"), depth + 1);
  }
  Warn(unit, "unit not supported, taken as the SI unit");
  return 1.0;
}

StepReader::Units StepReader::UnitsOf(Instance* representation) {
  Units u;
  Instance* context = Ref(representation, "context_of_items");
  bool length = false;
  for(Instance* unit : Refs(context, "units")) {
    if(IsA(unit, "length_unit")) {
      u.length = UnitFactor(unit);
      length = true;
    } else if(IsA(unit, "plane_angle_unit")) {
      u.angle = UnitFactor(unit);
    }
  }
  if(!length) {
    Warn(representation, "no length unit, taken as millimeters");
  }
  return u;
}

/* geometry */

template <int D>
bool StepReader::Point(Instance* e, const std::array<double, D>& scale,
                       std::array<double, D>& p) {
  StepValue c = Attribute(e, "coordinates");
  if(c.items.size() < D) {
    Warn(e, "point with fewer than " + std::to_string(D) + " coordinates");
    return false;
  }
  for(int i = 0; i < D; ++i) {
    p[i] = c.items[i].Number() * scale[i];
  }
  return true;
}

template <int D>
bool StepReader::Direction(Instance* e, std::array<double, D>& d) {
  StepValue c = Attribute(e, "direction_ratios");
  if(c.items.size() < D) {
    return false;
  }
  double n = 0.0;
  for(int i = 0; i < D; ++i) {
    d[i] = c.items[i].Number();
    n += d[i] * d[i];
  }
  n = std::sqrt(n);
  if(n == 0.0) {
    return false;
  }
  for(int i = 0; i < D; ++i) {
    d[i] /= n;
  }
  return true;
}

static Vec3 Cross(const Vec3& a, const Vec3& b) {
  return {a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2],
          a[0] * b[1] - a[1] * b[0]};
}

static double Dot(const Vec3& a, const Vec3& b) {
  return a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
}

static Vec3 Normalized(const Vec3& a) {
  double n = std::sqrt(Dot(a, a));
  return n > 0.0 ? Vec3{a[0] / n, a[1] / n, a[2] / n} : a;
}

/* a unit vector perpendicular to the unit vector z */
static Vec3 Perpendicular(const Vec3& z) {
  Vec3 t = std::fabs(z[0]) < 0.9 ? Vec3{1.0, 0.0, 0.0} : Vec3{0.0, 1.0, 0.0};
  return Normalized(Cross(Cross(z, t), z));
}

Frame StepReader::Placement(Instance* e) {
  Frame f;
  if(!e) {
    return f;
  }
  const Vec3 scale{units_.length, units_.length, units_.length};
  Point<3>(Ref(e, "location"), scale, f.origin);
  if(IsA(e, "axis1_placement")) {
    Vec3 z{0.0, 0.0, 1.0};
    Direction<3>(Ref(e, "axis"), z);
    f.z = z;
    f.x = Perpendicular(z);
    f.y = Cross(f.z, f.x);
    return f;
  }
  Vec3 z{0.0, 0.0, 1.0};
  Vec3 x{1.0, 0.0, 0.0};
  bool has_x = Direction<3>(Ref(e, "ref_direction"), x);
  Direction<3>(Ref(e, "axis"), z);
  Vec3 xp{x[0] - Dot(x, z) * z[0], x[1] - Dot(x, z) * z[1],
          x[2] - Dot(x, z) * z[2]};
  if(!has_x || Dot(xp, xp) < 1e-20) {
    xp = Perpendicular(z);
  }
  f.z = z;
  f.x = Normalized(xp);
  f.y = Cross(f.z, f.x);
  return f;
}

/* a 2D placement, as a frame in its xy plane */
Frame StepReader::Placement2(Instance* e, const Vec2& scale) {
  Frame f;
  if(!e) {
    return f;
  }
  Vec2 o{0.0, 0.0};
  Point<2>(Ref(e, "location"), scale, o);
  f.origin = {o[0], o[1], 0.0};
  Vec2 x{1.0, 0.0};
  Direction<2>(Ref(e, "ref_direction"), x);
  f.x = {x[0], x[1], 0.0};
  f.y = {-x[1], x[0], 0.0};
  return f;
}

bool StepReader::Knots(Instance* e, const char* multiplicities,
                       const char* knots, int degree, int count,
                       std::vector<double>& out) {
  out.clear();
  StepValue m = Attribute(e, multiplicities);
  StepValue k = Attribute(e, knots);
  if(m.kind == StepValue::Kind::kList && !m.items.empty()) {
    if(m.items.size() != k.items.size()) {
      Warn(e, "knot multiplicities and knots differ in number");
      return false;
    }
    for(size_t i = 0; i < k.items.size(); ++i) {
      for(int j = 0; j < static_cast<int>(m.items[i].Number()); ++j) {
        out.push_back(k.items[i].Number());
      }
    }
  } else if(IsA(e, "bezier_curve") || IsA(e, "bezier_surface")) {
    // segments of degree + 1 points sharing their end points
    int segments = degree > 0 ? (count - 1) / degree : 0;
    for(int s = 0; s <= segments; ++s) {
      int mult = (s == 0 || s == segments) ? degree + 1 : degree;
      for(int j = 0; j < mult; ++j) {
        out.push_back(s);
      }
    }
  } else if(IsA(e, "uniform_curve") || IsA(e, "uniform_surface")) {
    for(int i = 0; i < count + degree + 1; ++i) {
      out.push_back(i - degree);
    }
  } else if(IsA(e, "quasi_uniform_curve") || IsA(e, "quasi_uniform_surface")) {
    int interior = count - degree - 1;
    for(int j = 0; j <= degree; ++j) {
      out.push_back(0.0);
    }
    for(int i = 1; i <= interior; ++i) {
      out.push_back(i);
    }
    for(int j = 0; j <= degree; ++j) {
      out.push_back(interior + 1);
    }
  } else {
    Warn(e, "B-spline without knots");
    return false;
  }
  if(static_cast<int>(out.size()) != count + degree + 1) {
    Warn(e, "B-spline with " + std::to_string(out.size()) + " knots for " +
              std::to_string(count) + " control points of degree " +
              std::to_string(degree));
    return false;
  }
  return true;
}

template <int D>
bool StepReader::BSplineCurve(Instance* e, const std::array<double, D>& scale,
                              NurbsCurve<D>& nurbs) {
  nurbs.degree = static_cast<int>(Real(e, "degree"));
  for(Instance* p : Refs(e, "control_points_list")) {
    std::array<double, D> q{};
    if(!Point<D>(p, scale, q)) {
      return false;
    }
    nurbs.points.push_back(q);
  }
  const int n = static_cast<int>(nurbs.points.size());
  if(n < 2 || nurbs.degree < 1) {
    Warn(e, "B-spline curve with too few control points");
    return false;
  }
  if(!Knots(e, "knot_multiplicities", "knots", nurbs.degree, n, nurbs.knots)) {
    return false;
  }
  if(IsA(e, "rational_b_spline_curve")) {
    StepValue w = Attribute(e, "weights_data");
    if(static_cast<int>(w.items.size()) != n) {
      Warn(e, "weights and control points differ in number");
      return false;
    }
    for(const auto& x : w.items) {
      nurbs.weights.push_back(x.Number());
    }
  }
  return true;
}

template <int D>
bool StepReader::ConvertCurve(Instance* e, const std::array<double, D>& scale,
                              Curve<D>& curve, double& parameter_scale) {
  curve.step_id = e->StepFileId();
  parameter_scale = 1.0;
  if(IsA(e, "line")) {
    curve.type = Curve<D>::Type::kLine;
    if(!Point<D>(Ref(e, "pnt"), scale, curve.origin)) {
      return false;
    }
    Instance* v = Ref(e, "dir");
    std::array<double, D> d{};
    if(!Direction<D>(Ref(v, "orientation"), d)) {
      Warn(e, "line without a direction");
      return false;
    }
    double magnitude = Real(v, "magnitude");
    for(int i = 0; i < D; ++i) {
      curve.direction[i] = d[i] * magnitude * scale[i];
    }
    return true;
  }
  if(IsA(e, "circle") || IsA(e, "ellipse")) {
    curve.type = Curve<D>::Type::kConic;
    double r1, r2;
    if(IsA(e, "circle")) {
      r1 = r2 = Real(e, "radius");
    } else {
      r1 = Real(e, "semi_axis_1");
      r2 = Real(e, "semi_axis_2");
    }
    Instance* position = Ref(e, "position");
    Frame f;
    if constexpr(D == 2) {
      f = Placement2(position, {1.0, 1.0});
      // conic in the unscaled parameters, then scaled along each axis
      Vec2 o;
      Point<2>(Ref(position, "location"), scale, o);
      curve.origin = o;
      for(int i = 0; i < 2; ++i) {
        curve.a[i] = r1 * f.x[i] * scale[i];
        curve.b[i] = r2 * f.y[i] * scale[i];
      }
    } else {
      f = Placement(position);
      curve.origin = f.origin;
      for(int i = 0; i < 3; ++i) {
        curve.a[i] = r1 * f.x[i] * scale[i];
        curve.b[i] = r2 * f.y[i] * scale[i];
      }
    }
    parameter_scale = units_.angle;
    return true;
  }
  if(IsA(e, "polyline")) {
    curve.type = Curve<D>::Type::kNurbs;
    auto& nurbs = curve.nurbs;
    nurbs.degree = 1;
    for(Instance* p : Refs(e, "points")) {
      std::array<double, D> q{};
      if(!Point<D>(p, scale, q)) {
        return false;
      }
      nurbs.points.push_back(q);
    }
    const int n = static_cast<int>(nurbs.points.size());
    if(n < 2) {
      Warn(e, "polyline with fewer than two points");
      return false;
    }
    // the parameter is 1 per segment, starting from 1
    nurbs.knots.push_back(1.0);
    for(int i = 0; i < n; ++i) {
      nurbs.knots.push_back(1.0 + i);
    }
    nurbs.knots.push_back(n);
    return true;
  }
  if(IsA(e, "b_spline_curve")) {
    curve.type = Curve<D>::Type::kNurbs;
    return BSplineCurve<D>(e, scale, curve.nurbs);
  }
  if(IsA(e, "trimmed_curve")) {
    Warn(e, "trimmed curve read as its basis curve");
    return ConvertCurve<D>(Ref(e, "basis_curve"), scale, curve,
                           parameter_scale);
  }
  Warn(e, "curve not supported");
  return false;
}

int StepReader::Curve3d(Instance* e) {
  if(!e) {
    return -1;
  }
  auto it = curves_.find(e->StepFileId());
  if(it != curves_.end()) {
    return it->second;
  }
  Curve3 curve;
  double parameter_scale = 1.0;
  const Vec3 scale{units_.length, units_.length, units_.length};
  int index = -1;
  if(ConvertCurve<3>(e, scale, curve, parameter_scale)) {
    index = static_cast<int>(model_.curves.size());
    model_.curves.push_back(curve);
    curve_parameter_scale_.push_back(parameter_scale);
  }
  curves_[e->StepFileId()] = index;
  return index;
}

bool StepReader::BSplineSurface(Instance* e, NurbsSurface& nurbs) {
  nurbs.u_degree = static_cast<int>(Real(e, "u_degree"));
  nurbs.v_degree = static_cast<int>(Real(e, "v_degree"));
  StepValue rows = Attribute(e, "control_points_list");
  const Vec3 scale{units_.length, units_.length, units_.length};
  nurbs.u_count = static_cast<int>(rows.items.size());
  nurbs.v_count = nurbs.u_count ? static_cast<int>(rows.items[0].items.size())
                                : 0;
  for(const auto& row : rows.items) {
    if(static_cast<int>(row.items.size()) != nurbs.v_count) {
      Warn(e, "rows of control points differ in length");
      return false;
    }
    for(const auto& item : row.items) {
      Vec3 p;
      if(!Point<3>(Get(item), scale, p)) {
        return false;
      }
      nurbs.points.push_back(p);
    }
  }
  if(nurbs.u_count < 2 || nurbs.v_count < 2 || nurbs.u_degree < 1 ||
     nurbs.v_degree < 1) {
    Warn(e, "B-spline surface with too few control points");
    return false;
  }
  if(!Knots(e, "u_multiplicities", "u_knots", nurbs.u_degree, nurbs.u_count,
            nurbs.u_knots) ||
     !Knots(e, "v_multiplicities", "v_knots", nurbs.v_degree, nurbs.v_count,
            nurbs.v_knots)) {
    return false;
  }
  if(IsA(e, "rational_b_spline_surface")) {
    StepValue w = Attribute(e, "weights_data");
    if(static_cast<int>(w.items.size()) != nurbs.u_count) {
      Warn(e, "weights and control points differ in number");
      return false;
    }
    for(const auto& row : w.items) {
      if(static_cast<int>(row.items.size()) != nurbs.v_count) {
        Warn(e, "weights and control points differ in number");
        return false;
      }
      for(const auto& x : row.items) {
        nurbs.weights.push_back(x.Number());
      }
    }
  }
  return true;
}

int StepReader::SurfaceOf(Instance* e) {
  if(!e) {
    return -1;
  }
  auto it = surfaces_.find(e->StepFileId());
  if(it != surfaces_.end()) {
    return it->second;
  }
  Surface s;
  s.step_id = e->StepFileId();
  const double L = units_.length;
  const double A = units_.angle;
  Vec2 parameter_scale{1.0, 1.0};
  bool ok = true;
  if(IsA(e, "plane")) {
    s.type = Surface::Type::kPlane;
    s.position = Placement(Ref(e, "position"));
    parameter_scale = {L, L};
  } else if(IsA(e, "cylindrical_surface")) {
    s.type = Surface::Type::kCylinder;
    s.position = Placement(Ref(e, "position"));
    s.radius = Real(e, "radius") * L;
    parameter_scale = {A, L};
  } else if(IsA(e, "conical_surface")) {
    s.type = Surface::Type::kCone;
    s.position = Placement(Ref(e, "position"));
    s.radius = Real(e, "radius") * L;
    s.semi_angle = Real(e, "semi_angle") * A;
    parameter_scale = {A, L};
  } else if(IsA(e, "spherical_surface")) {
    s.type = Surface::Type::kSphere;
    s.position = Placement(Ref(e, "position"));
    s.radius = Real(e, "radius") * L;
    parameter_scale = {A, A};
  } else if(IsA(e, "toroidal_surface")) {
    s.type = Surface::Type::kTorus;
    s.position = Placement(Ref(e, "position"));
    s.radius = Real(e, "major_radius") * L;
    s.minor_radius = Real(e, "minor_radius") * L;
    parameter_scale = {A, A};
  } else if(IsA(e, "b_spline_surface")) {
    s.type = Surface::Type::kNurbs;
    ok = BSplineSurface(e, s.nurbs);
  } else if(IsA(e, "surface_of_revolution")) {
    s.type = Surface::Type::kRevolution;
    s.curve = Curve3d(Ref(e, "swept_curve"));
    s.position = Placement(Ref(e, "axis_position"));
    ok = s.curve >= 0;
    if(ok) {
      parameter_scale = {A, curve_parameter_scale_[s.curve]};
    }
  } else if(IsA(e, "surface_of_linear_extrusion")) {
    s.type = Surface::Type::kExtrusion;
    s.curve = Curve3d(Ref(e, "swept_curve"));
    Instance* v = Ref(e, "extrusion_axis");
    Vec3 d{0.0, 0.0, 1.0};
    ok = s.curve >= 0 && Direction<3>(Ref(v, "orientation"), d);
    double magnitude = Real(v, "magnitude") * L;
    s.extrusion = {d[0] * magnitude, d[1] * magnitude, d[2] * magnitude};
    if(ok) {
      parameter_scale = {curve_parameter_scale_[s.curve], 1.0};
    }
  } else {
    Warn(e, "surface not supported");
    ok = false;
  }
  int index = -1;
  if(ok) {
    index = static_cast<int>(model_.surfaces.size());
    model_.surfaces.push_back(s);
    parameter_scale_.push_back(parameter_scale);
  }
  surfaces_[e->StepFileId()] = index;
  return index;
}

/* topology */

int StepReader::VertexOf(Instance* e) {
  if(!e) {
    return -1;
  }
  auto it = vertices_.find(e->StepFileId());
  if(it != vertices_.end()) {
    return it->second;
  }
  int index = -1;
  Vertex v;
  v.step_id = e->StepFileId();
  const Vec3 scale{units_.length, units_.length, units_.length};
  if(IsA(e, "vertex_point") &&
     Point<3>(Ref(e, "vertex_geometry"), scale, v.point)) {
    index = static_cast<int>(model_.vertices.size());
    model_.vertices.push_back(v);
  } else {
    Warn(e, "vertex not supported");
  }
  vertices_[e->StepFileId()] = index;
  return index;
}

void StepReader::AddPcurve(Edge& edge, Instance* pcurve) {
  Pcurve p;
  p.surface = SurfaceOf(Ref(pcurve, "basis_surface"));
  if(p.surface < 0) {
    return;
  }
  Instance* definition = Ref(pcurve, "reference_to_curve");
  std::vector<Instance*> items = Refs(definition, "items");
  if(items.empty()) {
    Warn(pcurve, "pcurve without a curve");
    return;
  }
  double parameter_scale = 1.0;
  if(!ConvertCurve<2>(items[0], parameter_scale_[p.surface], p.curve,
                      parameter_scale)) {
    return;
  }
  // a pcurve shares the parameter of its edge's curve in the file (e.g. a
  // line in (u, v) along a circle, both in degrees): give it the parameter
  // of the converted curve (radians for the circle), unless it is a conic,
  // whose parameter is an angle anyway
  if(edge.curve >= 0 && p.curve.type != Curve2::Type::kConic) {
    const double r = curve_parameter_scale_[edge.curve] / parameter_scale;
    if(r != 1.0) {
      if(p.curve.type == Curve2::Type::kLine) {
        for(double& d : p.curve.direction) {
          d /= r;
        }
      } else {
        for(double& k : p.curve.nurbs.knots) {
          k *= r;
        }
      }
    }
  }
  edge.pcurves.push_back(p);
}

int StepReader::EdgeOf(Instance* e) {
  if(!e) {
    return -1;
  }
  auto it = edges_.find(e->StepFileId());
  if(it != edges_.end()) {
    return it->second;
  }
  int index = -1;
  if(IsA(e, "edge_curve")) {
    Edge edge;
    edge.step_id = e->StepFileId();
    edge.start = VertexOf(Ref(e, "edge_start"));
    edge.end = VertexOf(Ref(e, "edge_end"));
    edge.same_sense = Bool(e, "same_sense");
    Instance* g = Ref(e, "edge_geometry");
    if(IsA(g, "surface_curve")) {
      edge.curve = Curve3d(Ref(g, "curve_3d"));
      for(Instance* a : Refs(g, "associated_geometry")) {
        if(IsA(a, "pcurve")) {
          AddPcurve(edge, a);
        }
      }
    } else if(IsA(g, "pcurve")) {
      AddPcurve(edge, g);
    } else {
      edge.curve = Curve3d(g);
    }
    if(edge.start < 0 || edge.end < 0) {
      Warn(e, "edge without its vertices");
    } else {
      if(edge.curve < 0 && edge.pcurves.empty()) {
        Warn(e, "edge without a curve");
      }
      index = static_cast<int>(model_.edges.size());
      model_.edges.push_back(edge);
    }
  } else {
    Warn(e, "edge not supported");
  }
  edges_[e->StepFileId()] = index;
  return index;
}

int StepReader::FaceOf(Instance* e) {
  if(!e) {
    return -1;
  }
  auto it = faces_.find(e->StepFileId());
  if(it != faces_.end()) {
    return it->second;
  }
  int index = -1;
  if(IsA(e, "face_surface")) {
    Face face;
    face.step_id = e->StepFileId();
    face.surface = SurfaceOf(Ref(e, "face_geometry"));
    face.same_sense = Bool(e, "same_sense");
    for(Instance* bound : Refs(e, "bounds")) {
      Loop loop;
      loop.orientation = Bool(bound, "orientation");
      loop.outer = IsA(bound, "face_outer_bound");
      Instance* l = Ref(bound, "bound");
      if(IsA(l, "edge_loop")) {
        for(Instance* oe : Refs(l, "edge_list")) {
          Coedge c;
          c.orientation = Bool(oe, "orientation");
          Instance* element = Ref(oe, "edge_element");
          while(IsA(element, "oriented_edge")) {
            // an oriented edge of an oriented edge
            c.orientation = c.orientation == Bool(element, "orientation");
            element = Ref(element, "edge_element");
          }
          c.edge = EdgeOf(element);
          if(c.edge < 0) {
            Warn(e, "loop with an edge that could not be read");
            continue;
          }
          loop.coedges.push_back(c);
        }
      } else if(IsA(l, "vertex_loop")) {
        loop.vertex = VertexOf(Ref(l, "loop_vertex"));
      } else {
        Warn(l, "loop not supported");
        continue;
      }
      face.loops.push_back(loop);
    }
    if(face.surface < 0) {
      Warn(e, "face skipped: its surface could not be read");
    } else {
      index = static_cast<int>(model_.faces.size());
      model_.faces.push_back(face);
    }
  } else {
    Warn(e, "face not supported");
  }
  faces_[e->StepFileId()] = index;
  return index;
}

int StepReader::ShellOf(Instance* e) {
  if(!e) {
    return -1;
  }
  auto it = shells_.find(e->StepFileId());
  if(it != shells_.end()) {
    return it->second;
  }
  int index = -1;
  if(IsA(e, "oriented_closed_shell") || IsA(e, "oriented_open_shell")) {
    Warn(e, "oriented shell not supported");
  } else if(IsA(e, "closed_shell") || IsA(e, "open_shell")) {
    Shell shell;
    shell.step_id = e->StepFileId();
    shell.closed = IsA(e, "closed_shell");
    for(Instance* f : Refs(e, "cfs_faces")) {
      int face = FaceOf(f);
      if(face >= 0) {
        shell.faces.push_back(face);
      }
    }
    index = static_cast<int>(model_.shells.size());
    model_.shells.push_back(shell);
  } else {
    Warn(e, "shell not supported");
  }
  shells_[e->StepFileId()] = index;
  return index;
}

int StepReader::BodyOf(Instance* e) {
  auto it = bodies_.find(e->StepFileId());
  if(it != bodies_.end()) {
    return it->second;
  }
  Body body;
  body.step_id = e->StepFileId();
  body.name = String(e, "name");
  int index = -1;
  if(IsA(e, "manifold_solid_brep")) {
    body.solid = true;
    int outer = ShellOf(Ref(e, "outer"));
    if(outer >= 0) {
      body.shells.push_back(outer);
    }
    for(Instance* v : Refs(e, "voids")) {
      int shell = ShellOf(v);
      if(shell >= 0) {
        body.shells.push_back(shell);
      }
    }
  } else if(IsA(e, "shell_based_surface_model")) {
    for(Instance* s : Refs(e, "sbsm_boundary")) {
      int shell = ShellOf(s);
      if(shell >= 0) {
        body.shells.push_back(shell);
      }
    }
  } else {
    bodies_[e->StepFileId()] = -1;
    return -1;
  }
  if(!body.shells.empty()) {
    index = static_cast<int>(model_.bodies.size());
    model_.bodies.push_back(body);
  }
  bodies_[e->StepFileId()] = index;
  return index;
}

/* pcurves of the coedges: the edge's pcurve on the face's surface; of the
two pcurves of a seam, the first for the coedge along the edge and the second
for the coedge against it */
void StepReader::AssignPcurves() {
  for(auto& face : model_.faces) {
    for(auto& loop : face.loops) {
      for(auto& c : loop.coedges) {
        const Edge& edge = model_.edges[c.edge];
        std::vector<int> on;
        for(size_t i = 0; i < edge.pcurves.size(); ++i) {
          if(edge.pcurves[i].surface == face.surface) {
            on.push_back(static_cast<int>(i));
          }
        }
        if(on.size() == 1) {
          c.pcurve = on[0];
        } else if(on.size() >= 2) {
          c.pcurve = c.orientation ? on[0] : on[1];
        }
      }
    }
  }
}

/* shapes */

int StepReader::ShapeOf(Instance* representation) {
  if(!representation) {
    return -1;
  }
  auto it = shapes_.find(representation->StepFileId());
  return it == shapes_.end() ? -1 : it->second;
}

void StepReader::ReadShapes() {
  const int n = instances_.InstanceCount();
  // shapes first, so that mapped items can refer to any of them
  std::vector<Instance*> representations;
  for(int i = 0; i < n; ++i) {
    Instance* e = instances_.GetApplication_instance(i);
    if(IsA(e, "shape_representation")) {
      Shape shape;
      shape.step_id = e->StepFileId();
      shape.name = String(e, "name");
      shapes_[e->StepFileId()] = static_cast<int>(model_.shapes.size());
      model_.shapes.push_back(shape);
      representations.push_back(e);
    }
  }
  for(Instance* r : representations) {
    units_ = UnitsOf(r);
    const int shape = ShapeOf(r);
    for(Instance* item : Refs(r, "items")) {
      if(IsA(item, "mapped_item")) {
        Instance* map = Ref(item, "mapping_source");
        Instance* child = Ref(map, "mapped_representation");
        int c = ShapeOf(child);
        if(c < 0) {
          Warn(item, "mapped item of a representation that is not a shape");
          continue;
        }
        // origin in the child's coordinates, target in this shape's
        Units parent_units = units_;
        units_ = UnitsOf(child);
        Frame origin = Placement(Ref(map, "mapping_origin"));
        units_ = parent_units;
        Frame target = Placement(Ref(item, "mapping_target"));
        model_.shapes[shape].children.push_back(
          {c, Transform::FromFrame(target) *
                Transform::FromFrame(origin).Inverse()});
        continue;
      }
      int body = BodyOf(item);
      if(body >= 0) {
        model_.shapes[shape].bodies.push_back(body);
      } else if(!IsA(item, "placement") && !IsA(item, "styled_item")) {
        Warn(item, "item of a shape not supported");
      }
    }
  }
}

/* names of shapes from their products: shape_definition_representation ->
product_definition_shape -> product_definition -> formation -> product */
void StepReader::ReadNames() {
  const int n = instances_.InstanceCount();
  for(int i = 0; i < n; ++i) {
    Instance* e = instances_.GetApplication_instance(i);
    if(!IsA(e, "shape_definition_representation")) {
      continue;
    }
    int shape = ShapeOf(Ref(e, "used_representation"));
    if(shape < 0) {
      continue;
    }
    Instance* pds = Ref(e, "definition");
    Instance* pd = Ref(pds, "definition");
    Instance* formation = Ref(pd, "formation");
    Instance* product = Ref(formation, "of_product");
    std::string name = String(product, "name");
    if(name.empty()) {
      name = String(product, "id");
    }
    if(!name.empty()) {
      model_.shapes[shape].name = name;
    }
  }
}

/* children placed by representation relationships. With a transformation
(item_defined_transformation), the child is the representation of the
product related by the assembly usage that the relationship represents
(context_dependent_shape_representation), or rep_1 if there is none; the
transformation takes transform_item_1, in rep_1, to transform_item_2, in
rep_2. Without one, the representations share their coordinates, and the
child is the one that no product uses. */
void StepReader::ReadRelationships() {
  const int n = instances_.InstanceCount();
  std::unordered_map<int, int> shape_of_definition; // product_definition id
  std::unordered_map<int, Instance*> usage;          // relationship id
  for(int i = 0; i < n; ++i) {
    Instance* e = instances_.GetApplication_instance(i);
    if(IsA(e, "shape_definition_representation")) {
      int shape = ShapeOf(Ref(e, "used_representation"));
      Instance* pd = Ref(Ref(e, "definition"), "definition");
      if(shape >= 0 && pd) {
        shape_of_definition[pd->StepFileId()] = shape;
      }
    } else if(IsA(e, "context_dependent_shape_representation")) {
      Instance* rr = Ref(e, "representation_relation");
      Instance* pds = Ref(e, "represented_product_relation");
      Instance* nauo = Ref(pds, "definition");
      if(rr && nauo) {
        usage[rr->StepFileId()] = nauo;
      }
    }
  }
  std::unordered_set<int> used_by_product;
  for(const auto& [id, shape] : shape_of_definition) {
    used_by_product.insert(shape);
  }
  for(int i = 0; i < n; ++i) {
    Instance* e = instances_.GetApplication_instance(i);
    if(!IsA(e, "representation_relationship")) {
      continue;
    }
    Instance* r1 = Ref(e, "rep_1");
    Instance* r2 = Ref(e, "rep_2");
    int s1 = ShapeOf(r1);
    int s2 = ShapeOf(r2);
    if(s1 < 0 || s2 < 0) {
      continue;
    }
    Instance* t = Ref(e, "transformation_operator");
    if(!t) {
      // shared coordinates
      bool p1 = used_by_product.count(s1) > 0;
      bool p2 = used_by_product.count(s2) > 0;
      int parent = (p2 && !p1) ? s2 : s1;
      int child = parent == s1 ? s2 : s1;
      model_.shapes[parent].children.push_back(
        {child, Transform(), !used_by_product.count(child)});
      continue;
    }
    if(!IsA(t, "item_defined_transformation")) {
      Warn(t, "transformation not supported");
      continue;
    }
    units_ = UnitsOf(r1);
    Transform p1 = Transform::FromFrame(Placement(Ref(t, "transform_item_1")));
    units_ = UnitsOf(r2);
    Transform p2 = Transform::FromFrame(Placement(Ref(t, "transform_item_2")));
    bool child_is_1 = true;
    auto u = usage.find(e->StepFileId());
    if(u != usage.end()) {
      Instance* related = Ref(u->second, "related_product_definition");
      auto c = related ? shape_of_definition.find(related->StepFileId())
                       : shape_of_definition.end();
      if(c != shape_of_definition.end()) {
        if(c->second == s2) {
          child_is_1 = false;
        } else if(c->second != s1) {
          Warn(e, "assembly usage of another product");
        }
      }
    }
    if(child_is_1) {
      model_.shapes[s2].children.push_back({s1, p2 * p1.Inverse()});
    } else {
      model_.shapes[s1].children.push_back({s2, p1 * p2.Inverse()});
    }
  }
}

/* the color of a style: the first colour_rgb or pre-defined colour reached
from it, not through the styles of curves */
bool StepReader::FindColor(Instance* e, Color& color, int depth) {
  if(!e || depth > 12) {
    return false;
  }
  if(IsA(e, "colour_rgb")) {
    color.r = static_cast<float>(Real(e, "red"));
    color.g = static_cast<float>(Real(e, "green"));
    color.b = static_cast<float>(Real(e, "blue"));
    color.set = true;
    return true;
  }
  if(IsA(e, "draughting_pre_defined_colour")) {
    static const std::unordered_map<std::string, Color> named = {
      {"red", {1, 0, 0, true}},     {"green", {0, 1, 0, true}},
      {"blue", {0, 0, 1, true}},    {"yellow", {1, 1, 0, true}},
      {"magenta", {1, 0, 1, true}}, {"cyan", {0, 1, 1, true}},
      {"black", {0, 0, 0, true}},   {"white", {1, 1, 1, true}}};
    auto it = named.find(String(e, "name"));
    if(it != named.end()) {
      color = it->second;
      return true;
    }
    return false;
  }
  if(IsA(e, "curve_style")) {
    return false;
  }
  // follow every reference of the entity's attributes
  std::vector<Instance*> next;
  auto collect = [&](Instance* part) {
    part->ResetAttributes();
    STEPattribute* a;
    while((a = part->NextAttribute())) {
      std::vector<const StepValue*> stack;
      StepValue v = StepValue::Parse(a->asStr());
      stack.push_back(&v);
      while(!stack.empty()) {
        const StepValue* x = stack.back();
        stack.pop_back();
        if(x->kind == StepValue::Kind::kRef) {
          if(Instance* r = Get(x->ref)) {
            next.push_back(r);
          }
        }
        for(const auto& item : x->items) {
          stack.push_back(&item);
        }
      }
    }
  };
  if(e->IsComplex()) {
    for(STEPcomplex* p = static_cast<STEPcomplex*>(e)->head; p; p = p->sc) {
      collect(p);
    }
  } else {
    collect(e);
  }
  for(Instance* r : next) {
    if(FindColor(r, color, depth + 1)) {
      return true;
    }
  }
  return false;
}

void StepReader::ReadColors() {
  const int n = instances_.InstanceCount();
  for(int i = 0; i < n; ++i) {
    Instance* e = instances_.GetApplication_instance(i);
    if(!IsA(e, "styled_item")) {
      continue;
    }
    Instance* item = Ref(e, "item");
    if(!item) {
      continue;
    }
    Color color;
    bool found = false;
    for(Instance* style : Refs(e, "styles")) {
      if(FindColor(style, color, 0)) {
        found = true;
        break;
      }
    }
    if(!found) {
      continue;
    }
    const int id = item->StepFileId();
    if(auto b = bodies_.find(id); b != bodies_.end() && b->second >= 0) {
      model_.bodies[b->second].color = color;
    } else if(auto f = faces_.find(id); f != faces_.end() && f->second >= 0) {
      model_.faces[f->second].color = color;
    } else if(auto s = shells_.find(id); s != shells_.end() && s->second >= 0) {
      for(int face : model_.shells[s->second].faces) {
        if(!model_.faces[face].color.set) {
          model_.faces[face].color = color;
        }
      }
    }
  }
}

Model StepReader::Read() {
  {
    std::ifstream test(path_);
    if(!test) {
      throw shift::core::Exception(path_ + ": file not found");
    }
  }
  // STEPcode reports to the standard streams; keep its messages for the log
  std::ostringstream messages;
  std::streambuf* out = std::cout.rdbuf(messages.rdbuf());
  std::streambuf* err = std::cerr.rdbuf(messages.rdbuf());
  int errors = 0;
  try {
    STEPfile file(registry_, instances_, "", false);
    file.ReadExchangeFile(path_);
    errors = file.ErrorCount();
  } catch(...) {
    std::cout.rdbuf(out);
    std::cerr.rdbuf(err);
    throw;
  }
  std::cout.rdbuf(out);
  std::cerr.rdbuf(err);
  SHIFT_TRACE("STEPcode messages for " + path_ + ":\n" + messages.str());
  if(instances_.InstanceCount() == 0) {
    throw shift::core::Exception(path_ + ": no entities could be read");
  }
  if(errors > 0) {
    model_.warnings.push_back(std::to_string(errors) +
                              " entities of the file could not be read "
                              "completely (see the trace log)");
  }

  ReadShapes();
  ReadNames();
  ReadRelationships();
  ReadColors();
  AssignPcurves();

  std::vector<bool> child(model_.shapes.size(), false);
  for(const auto& s : model_.shapes) {
    for(const auto& c : s.children) {
      child[c.shape] = true;
    }
  }
  for(size_t i = 0; i < model_.shapes.size(); ++i) {
    const auto& s = model_.shapes[i];
    if(!child[i] && (!s.bodies.empty() || !s.children.empty())) {
      model_.roots.push_back(static_cast<int>(i));
    }
  }
  for(const auto& w : model_.warnings) {
    SHIFT_WARNING(path_ + ": " + w);
  }
  return model_;
}

Model ReadStep(const std::string& path) {
  StepReader reader(path);
  return reader.Read();
}

#endif

} // namespace cad
} // namespace core
} // namespace shift
