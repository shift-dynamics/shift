core_src += files(
  'cpu_solver.cc',
  'dem.cc',
  'gpu_solver.cc',
)

# the compute shaders of the DEM, compiled to SPIR-V as C initializers
# (shift/core/dem/<name>.comp.inc)
if gpu_found
  foreach name : ['dem_spheres', 'dem_count', 'dem_scatter', 'dem_cell_sort',
                  'dem_neighbors', 'dem_flags', 'dem_forces', 'dem_grains',
                  'dem_triangles', 'dem_triangle_count', 'nsc_contacts',
                  'nsc_reverse', 'nsc_prepare', 'nsc_velocities',
                  'nsc_body_reduce', 'nsc_body_sum', 'nsc_gradient',
                  'nsc_update', 'nsc_reduce', 'nsc_control', 'nsc_finish',
                  'nsc_grains']
    core_src += custom_target(
      name + '_spirv',
      input: 'shaders' / name + '.comp',
      output: name + '.comp.inc',
      depend_files: files('shaders/dem_common.glsl',
                          'shaders/dem_hull.glsl',
                          'shaders/nsc_common.glsl'),
      command: [glslc, '--target-env=vulkan1.2', '-O', '-mfmt=c',
                '-I', meson.current_source_dir() / 'shaders',
                '-o', '@OUTPUT@', '@INPUT@'],
    )
  endforeach
endif
