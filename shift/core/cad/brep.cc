#include "shift/core/cad/brep.h"

#include <functional>

namespace shift {
namespace core {
namespace cad {

Vec3 Transform::Apply(const Vec3& p) const {
  const auto& r = rotation;
  return {r[0] * p[0] + r[1] * p[1] + r[2] * p[2] + translation[0],
          r[3] * p[0] + r[4] * p[1] + r[5] * p[2] + translation[1],
          r[6] * p[0] + r[7] * p[1] + r[8] * p[2] + translation[2]};
}

Transform Transform::operator*(const Transform& b) const {
  Transform c;
  for(int i = 0; i < 3; ++i) {
    for(int j = 0; j < 3; ++j) {
      double s = 0.0;
      for(int k = 0; k < 3; ++k) {
        s += rotation[3 * i + k] * b.rotation[3 * k + j];
      }
      c.rotation[3 * i + j] = s;
    }
  }
  c.translation = Apply(b.translation);
  return c;
}

Transform Transform::Inverse() const {
  Transform c;
  for(int i = 0; i < 3; ++i) {
    for(int j = 0; j < 3; ++j) {
      c.rotation[3 * i + j] = rotation[3 * j + i];
    }
  }
  for(int i = 0; i < 3; ++i) {
    c.translation[i] = -(c.rotation[3 * i] * translation[0] +
                         c.rotation[3 * i + 1] * translation[1] +
                         c.rotation[3 * i + 2] * translation[2]);
  }
  return c;
}

Transform Transform::FromFrame(const Frame& f) {
  Transform t;
  for(int i = 0; i < 3; ++i) {
    t.rotation[3 * i] = f.x[i];
    t.rotation[3 * i + 1] = f.y[i];
    t.rotation[3 * i + 2] = f.z[i];
  }
  t.translation = f.origin;
  return t;
}

std::vector<PlacedBody> Model::Flatten() const {
  std::vector<PlacedBody> placed;
  std::vector<bool> visiting(shapes.size(), false);
  std::function<void(int, int, const Transform&, const std::string&, bool)>
    visit = [&](int shape, int root, const Transform& t,
                const std::string& path, bool same_part) {
      if(visiting[shape]) {
        return; // a shape that contains itself
      }
      visiting[shape] = true;
      const Shape& s = shapes[shape];
      const std::string here =
        same_part ? path : (path.empty() ? s.name : path + "/" + s.name);
      for(int b : s.bodies) {
        placed.push_back({b, shape, root, here, t});
      }
      for(const auto& c : s.children) {
        visit(c.shape, root, t * c.transform, here, c.same_part);
      }
      visiting[shape] = false;
    };
  for(int r : roots) {
    visit(r, r, Transform(), "", false);
  }
  return placed;
}

} // namespace cad
} // namespace core
} // namespace shift
