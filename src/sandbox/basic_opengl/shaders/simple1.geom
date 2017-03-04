#version 330

layout (triangles) in;
layout (triangle_strip, max_vertices = 3) out;

uniform mat3 nm;

in vData {
    vec3 vertex;
    vec4 color;
} verts[];

out fData {
    vec3 vertex;
    vec4 color;
    vec3 normal;
} frag;

void main() {
    vec3 a = nm * verts[0].vertex;
    vec3 b = nm * verts[1].vertex;
    vec3 c = nm * verts[2].vertex;
    vec3 normal = normalize(cross(b - a, c - b));
    int i;
    for (i = 0; i < gl_in.length(); ++i) {
        frag.vertex = gl_in[i].gl_Position.xyz;
        frag.color = verts[i].color;
        frag.normal = normal;
        gl_Position = gl_in[i].gl_Position;
        EmitVertex();
    }
    EndPrimitive();
}
