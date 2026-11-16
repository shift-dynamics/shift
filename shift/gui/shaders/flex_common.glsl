// the deformation of a flexible body's mesh surface (FlexUpdate)
layout(std140, binding = 0) uniform FlexParams {
  uvec4 counts;      // x = vertices, y = shapes per vertex (columns)
  vec4 center;       // the point the rotation turns about (centroid)
  vec4 translation;  // the rigid correction (FlexUpdate)
  vec4 rotation;
} params;

// the undeformed vertices, as Vertex: position, normal, uv (8 floats)
layout(std430, binding = 1) readonly buffer Reference {
  float reference[];
};
