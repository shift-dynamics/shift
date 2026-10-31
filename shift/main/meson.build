argagg_dep = dependency('argagg')

executable(
  'shift-cli',
  'shift_cli.cc',
  dependencies: [shift_core_dep, argagg_dep],
  implicit_include_directories: false,
  install: true,
)

if shift_gui_dep.found()
  executable(
    'shift',
    'shift.cc',
    dependencies: shift_gui_dep,
    implicit_include_directories: false,
    install: true,
  )
endif
