// the deformation of a flexible body's mesh surface (FlexUpdate)
layout(std140, binding = 0) uniform FlexParams {
  uvec4 counts;      // x = vertices, y = shapes per vertex (columns)
  vec4 center;       // the point the rotation turns about (centroid)
  vec4 translation;  // the rigid correction (FlexUpdate)
  vec4 rotation;
  // the stress drawn (FlexUpdate::stress): x = 0 none, 1 over the range
  // y to z, 2 from 0 to the largest on the mesh
  vec4 stress;
} params;

// the undeformed vertices, as Vertex: position, normal, uv (8 floats)
layout(std430, binding = 1) readonly buffer Reference {
  float reference[];
};
