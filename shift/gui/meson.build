# Qt 6 RHI (Vulkan) viewer; built as a static library linked into `shift`
qt6_dep = dependency('qt6', modules: ['Core', 'Gui', 'Widgets'], version: '>=6.7',
                     required: get_option('gui'))

shift_gui_dep = disabler()
if not qt6_dep.found()
  subdir_done()
endif

qt6 = import('qt6')

# The RHI headers live in a versioned private include directory
qt_include_dir = run_command('pkg-config', '--variable=includedir', 'Qt6Core',
                             check: true).stdout().strip()
rhi_inc = include_directories(
  qt_include_dir / 'QtGui' / qt6_dep.version() / 'QtGui', is_system: true)

# Qt host tools (qsb, rcc) are not always in PATH
qt_tool_candidates = {'qsb': [], 'rcc': []}
foreach variable : ['host_bins', 'bindir', 'libexecdir']
  dir = run_command('pkg-config', '--variable=' + variable, 'Qt6Core',
                    check: false).stdout().strip()
  if dir != ''
    qt_tool_candidates = {
      'qsb': qt_tool_candidates['qsb'] + [dir / 'qsb'],
      'rcc': qt_tool_candidates['rcc'] + [dir / 'rcc'],
    }
  endif
endforeach
qsb = find_program(qt_tool_candidates['qsb'] + ['qsb6', 'qsb'])
rcc = find_program(qt_tool_candidates['rcc'] + ['rcc6', 'rcc'])

# GLSL -> Qt shader bundles, packed into a resource
shader_bundles = []
foreach shader : ['mesh.vert', 'mesh.frag', 'fullscreen.vert', 'sky.frag',
                  'floor.vert', 'floor.frag', 'blit.frag', 'raytrace.comp',
                  'svgf_temporal.comp', 'svgf_variance.comp', 'svgf_atrous.comp',
                  'svgf_composite.comp', 'svgf_taa.comp']
  shader_bundles += custom_target(
    shader.replace('.', '_') + '_qsb',
    input: 'shaders' / shader,
    output: shader + '.qsb',
    command: [qsb, '--glsl', '310 es,430', '-o', '@OUTPUT@', '@INPUT@'],
  )
endforeach

# rcc resolves the bundle paths relative to the qrc, so it lives next to them
shader_qrc = configure_file(input: 'shaders/shaders.qrc', output: 'shaders.qrc',
                            copy: true)
shader_resources = custom_target(
  'shader_resources',
  input: shader_qrc,
  output: 'qrc_shaders.cc',
  command: [rcc, '--name', 'shaders', '-o', '@OUTPUT@', '@INPUT@'],
  depends: shader_bundles,
)

icon_resources = qt6.compile_resources(
  name: 'icons',
  sources: files('../../assets/icons/icons.qrc'),
)

gui_moc = qt6.compile_moc(
  headers: ['playback_slider.h', 'viewport.h', 'workspace.h'],
  include_directories: shift_inc,
  dependencies: qt6_dep,
)

shift_gui_lib = static_library(
  'shift_gui',
  'bvh.cc',
  'camera.cc',
  'environment.cc',
  'playback_slider.cc',
  'scene.cc',
  'simulation.cc',
  'viewport.cc',
  'workspace.cc',
  gui_moc,
  include_directories: [shift_inc, rhi_inc],
  implicit_include_directories: false,
  dependencies: [shift_core_dep, qt6_dep],
)

# resources are compiled into the executable so their static initializers
# are not dropped by the linker
shift_gui_dep = declare_dependency(
  sources: [shader_resources, icon_resources],
  link_with: shift_gui_lib,
  include_directories: [shift_inc, rhi_inc],
  dependencies: [shift_core_dep, qt6_dep],
)
