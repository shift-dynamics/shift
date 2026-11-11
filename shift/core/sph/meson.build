core_src += files(
  'cpu_solver.cc',
  'gpu_solver.cc',
  'sph.cc',
)

# the compute shaders of the SPH soil, compiled to SPIR-V as C initializers
# (shift/core/sph/<name>.comp.inc)
if gpu_found
  foreach name : ['sph_bodies', 'sph_cell_sort', 'sph_count', 'sph_gather', 'sph_markers',
                  'sph_momentum', 'sph_rates', 'sph_scatter']
    core_src += custom_target(
      name + '_spirv',
      input: 'shaders' / name + '.comp',
      output: name + '.comp.inc',
      depend_files: files('shaders/sph_common.glsl'),
      command: [glslc, '--target-env=vulkan1.2', '-O', '-mfmt=c',
                '-I', meson.current_source_dir() / 'shaders',
                '-o', '@OUTPUT@', '@INPUT@'],
    )
  endforeach
endif
