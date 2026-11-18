// the deformation of a flexible body's mesh surface (FlexUpdate)
layout(std140, binding = 0) uniform FlexParams {
  uvec4 counts;      // x = vertices, y = shapes per vertex (columns)
  vec4 center;       // the point the rotation turns about (centroid)
  vec4 translation;  // the rigid correction (FlexUpdate)
  vec4 rotation;
  // the stress drawn (FlexUpdate::stress): x = 0 none, 1 over the range
  // y to z, 2 from 0 (a signed quantity's: -the largest) to the largest
  // magnitude on the mesh
  vec4 stress;
  // x = the quantity (FlexUpdate::Quantity), y = the yield strength, z = 1:
  // the peak drawn
  vec4 quantity;
  // the states folded into the peak (flex_peak.comp): x = their count, y =
  // the stress coefficients of each, z = 1: the peak starts again
  uvec4 peak;
} params;

// the undeformed vertices, as Vertex: position, normal, uv (8 floats)
layout(std430, binding = 1) readonly buffer Reference {
  float reference[];
};

const uint kVonMises = 0u, kTresca = 1u, kMaxPrincipal = 2u,
           kMinPrincipal = 4u, kXx = 5u, kZx = 10u, kSafetyFactor = 11u;

// the quantity of the stress s (sxx, syy, szz, txy, tyz, tzx); the safety
// factor's is the von Mises stress (shift::gui::FlexStressValue): the
// principal stresses in closed form, largest first (Smith 1961)
float StressValue(float s[6], uint quantity) {
  if(quantity >= kTresca && quantity <= kMinPrincipal) {
    float q = (s[0] + s[1] + s[2]) / 3.0;
    float a = s[0] - q, b = s[1] - q, c = s[2] - q;
    float p2 = a * a + b * b + c * c +
               2.0 * (s[3] * s[3] + s[4] * s[4] + s[5] * s[5]);
    float principal[3] = float[3](q, q, q);
    if(p2 > 0.0) {
      float p = sqrt(p2 / 6.0);
      float det = a * b * c + 2.0 * s[3] * s[4] * s[5] - a * s[4] * s[4] -
                  b * s[5] * s[5] - c * s[3] * s[3];
      float r = clamp(0.5 * det / (p * p * p), -1.0, 1.0);
      float phi = acos(r) / 3.0;
      principal[0] = q + 2.0 * p * cos(phi);
      principal[2] = q + 2.0 * p * cos(phi + 2.0 * 3.14159265358979 / 3.0);
      principal[1] = 3.0 * q - principal[0] - principal[2];
    }
    return quantity == kTresca ? principal[0] - principal[2]
                               : principal[quantity - kMaxPrincipal];
  }
  if(quantity >= kXx && quantity <= kZx) {
    return s[quantity - kXx];
  }
  float d0 = s[0] - s[1], d1 = s[1] - s[2], d2 = s[2] - s[0];
  return sqrt(0.5 * (d0 * d0 + d1 * d1 + d2 * d2) +
              3.0 * (s[3] * s[3] + s[4] * s[4] + s[5] * s[5]));
}

// whether a quantity has a sign (shift::gui::FlexStressSigned)
bool StressSigned(uint quantity) {
  return quantity != kVonMises && quantity != kTresca &&
         quantity != kSafetyFactor;
}
