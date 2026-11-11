core_src += files(
  'device.cc',
  'scan.cc',
  'segment_sum.cc',
)

# the compute shaders, compiled to SPIR-V as C initializers
# (shift/core/gpu/<name>.comp.inc) that the sources include
if gpu_found
  foreach name : ['scan_add', 'scan_blocks', 'segment_sum']
    core_src += custom_target(
      name + '_spirv',
      input: 'shaders' / name + '.comp',
      output: name + '.comp.inc',
      command: [glslc, '--target-env=vulkan1.2', '-O', '-mfmt=c',
                '-o', '@OUTPUT@', '@INPUT@'],
    )
  endforeach
endif
