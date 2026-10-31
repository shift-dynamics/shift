gtest_dep = dependency('gtest', main: false, required: get_option('tests'))
if not gtest_dep.found()
  subdir_done()
endif

shift_test = executable(
  'shift-test',
  'main.cc',
  'test_bitfield.cc',
  'test_examples.cc',
  'test_excavator.cc',
  'test_specified_coordinates.cc',
  # 'test_kdtree.cc',
  dependencies: [shift_core_dep, gtest_dep],
  implicit_include_directories: false,
)

# everything except the per-model cases, which are registered one by one below
test('shift-test', shift_test, args: ['--gtest_filter=-Examples/*'],
     timeout: 600)

# one meson test per model in examples/ (test_examples.cc discovers the same
# files at run time); re-run `meson setup --reconfigure` after adding a model
example_models = run_command(
  find_program('find'), shift_examples_dir, '-name', '*.json',
  check: true,
).stdout().strip().split('\n')

# models known not to run yet; they are expected to fail so the suite stays
# green, and meson reports an unexpected pass once they are fixed
known_failures = {
  # the 'tire' force module is not implemented (no factory registration)
  'vehicle/tire_fiala.json': 'tire',
  'vehicle/tire_fiala_control.json': 'tire',
  'vehicle/tire_fiala_rig.json': 'tire',
  'vehicle/tire_pac2002.json': 'tire',
  'vehicle/vehicle.json': 'tire',
  'vehicle/vehicle_control.json': 'tire',
  'vehicle/vehicle_control_15mps.json': 'tire',
  'vehicle/vehicle_control_5mps.json': 'tire',
  'vehicle/vehicle_control_5mps_float.json': 'tire',
  'vehicle/vehicle_control_magic.json': 'tire',
  'vehicle/vehicle_no_front_steering.json': 'tire',
  'vehicle/vehicle_no_write.json': 'tire',
  # stale backups of vehicle.json in an old format
  'vehicle/vehicle_backup.json': 'old format',
  'vehicle/vehicle_backup2.json': 'old format',
}
foreach yaw : ['3000', '4500']
  foreach control : ['0', '2', '4', '6', '8', '10']
    known_failures += {
      'vehicle/fiala/tire_fiala_control_@0@_yaw_@1@.json'.format(control, yaw): 'tire',
    }
  endforeach
endforeach

# large stress models, too slow for the regular suite (debug build with
# SHIFT_VERIFY_EQUATIONS_); run them with `meson test -C build --benchmark`
stress_models = [
  'collision/collision_test_500.json',
  'collision/collision_test_5000.json',
]

foreach model_path : example_models
  model = model_path.replace(shift_examples_dir + '/', '')
  case = model
  foreach c : ['/', '.', '-']
    case = case.replace(c, '_')
  endforeach
  args = ['--gtest_filter=Examples/Model.Simulates/' + case]
  if model in stress_models
    benchmark(model, shift_test, args: args, suite: 'examples', timeout: 0)
  else
    test(
      model,
      shift_test,
      args: args,
      suite: 'examples',
      should_fail: known_failures.has_key(model),
      timeout: 600,
    )
  endif
endforeach
