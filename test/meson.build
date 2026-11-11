gtest_dep = dependency('gtest', main: false, required: get_option('tests'))
if not gtest_dep.found()
  subdir_done()
endif

test_sources = [
  'main.cc',
  'test_bitfield.cc',
  'test_examples.cc',
  'test_excavator.cc',
  'test_specified_coordinates.cc',
  'contact/contact_record.cc',
  'contact/test_collision_geometry_input.cc',
  'contact/test_contact_force.cc',
  'test_free_body.cc',
  'test_mass_properties.cc',
  'contact/test_contact_geometry.cc',
  'contact/test_contact_primitives.cc',
  'test_mesh_sharing.cc',
  'test_tire_iowa.cc',
  'test_tire_pac2002.cc',
  'terramechanics/test_scm.cc',
  'terramechanics/test_soil_contact.cc',
  'test_terrain.cc',
  'test_unique_names.cc',
  'test_find_file.cc',
  'test_joint_axis.cc',
  'test_screw_gear_joints.cc',
  'test_rigid_joint.cc',
  'test_light.cc',
  'test_beam.cc',
  'test_belt.cc',
  'test_mesh_scale.cc',
  'test_applied_load.cc',
  'test_sensors.cc',
  'test_bushing.cc',
  'test_hydraulic_cylinder.cc',
  'test_rubber_mount.cc',
  'test_step_reader.cc',
  'test_cad_repair.cc',
  'contact/test_cad_contact.cc',
  'contact/test_cad_line_contact.cc',
  'contact/test_cad_round_contact.cc',
  'contact/test_cad_hard_contact.cc',
  'contact/test_cad_conformal_contact.cc',
  'contact/test_ray_cast.cc',
  'contact/test_terrain_contact.cc',
  'terramechanics/test_terramechanics.cc',
  'test_schedule.cc',
  'test_lead_screw.cc',
  'test_yaml.cc',
  'test_gear_mesh.cc',
  'test_joint_force.cc',
  'test_journal_bearing.cc',
  'terramechanics/test_soil_tool.cc',
  'test_spline.cc',
  'test_spring_damper.cc',
  'test_thrust_collar.cc',
  'contact/test_stick_slip.cc',
  'test_powertrain.cc',
  'gpu/test_gpu.cc',
  'terramechanics/test_sph.cc',
  'terramechanics/test_sph_terrain.cc',
]
# the compute shader of the GPU tests, as a C initializer of SPIR-V words
if gpu_found
  test_sources += custom_target(
    'test_axpy_spirv',
    input: 'gpu/axpy.comp',
    output: 'axpy.comp.inc',
    command: [glslc, '--target-env=vulkan1.2', '-O', '-mfmt=c',
              '-o', '@OUTPUT@', '@INPUT@'],
  )
endif
test_deps = [shift_core_dep, gtest_dep]

# tests of the viewer library, when it is built (option gui)
if not is_disabler(shift_gui_dep)
  test_sources += ['test_instance_bvh.cc', 'test_video_encoder.cc',
                   'test_embedded_texture.cc', 'test_camera.cc',
                   'test_cad_scene.cc', 'test_video_trimmer.cc',
                   'terramechanics/test_scm_playback.cc', 'test_shader_layouts.cc']
  test_deps += [shift_gui_dep]
endif

shift_test = executable(
  'shift-test',
  test_sources,
  dependencies: test_deps,
  implicit_include_directories: false,
)

# everything except the per-model cases, which are registered one by one below
test('shift-test', shift_test, args: ['--gtest_filter=-Examples/*:CadSpeed.*'],
     timeout: 600)

# the time contact of CAD surfaces takes, in the hard cases of
# test/contact/test_cad_hard_contact.cc
benchmark('cad_contact_speed', shift_test,
          args: ['--gtest_filter=CadSpeed.*'], timeout: 0)

# the GPU tests on Mesa's software Vulkan driver, lavapipe, so that they run
# without a GPU (on Arch Linux: the vulkan-swrast package)
if gpu_found
  lavapipe_icd = ''
  foreach d : ['/usr/share/vulkan/icd.d', '/usr/local/share/vulkan/icd.d',
               '/etc/vulkan/icd.d']
    foreach f : ['lvp_icd.x86_64.json', 'lvp_icd.aarch64.json', 'lvp_icd.json']
      if lavapipe_icd == '' and fs.exists(d / f)
        lavapipe_icd = d / f
      endif
    endforeach
  endforeach
  if lavapipe_icd != ''
    test('gpu-lavapipe', shift_test, args: ['--gtest_filter=Gpu.*'],
         env: ['VK_DRIVER_FILES=' + lavapipe_icd,
               'VK_ICD_FILENAMES=' + lavapipe_icd,
               'SHIFT_GPU_DEVICE=llvmpipe', 'SHIFT_GPU_REQUIRED=1'],
         timeout: 600)
  else
    message('lavapipe (Mesa\'s software Vulkan driver) not found: no '
            + 'gpu-lavapipe test')
  endif
endif

# one meson test per model in examples/ (test_examples.cc discovers the same
# files at run time); re-run `meson setup --reconfigure` after adding a model
example_models = run_command(
  find_program('find'), shift_examples_dir, '(', '-name', '*.json', '-o',
  '-name', '*.yaml', '-o', '-name', '*.yml', ')',
  check: true,
).stdout().strip().split('\n')

# models known not to run yet; they are expected to fail so the suite stays
# green, and meson reports an unexpected pass once they are fixed
known_failures = {}

# large stress models, too slow for the regular suite in a debug build; run
# them with `meson test -C build --benchmark`
stress_models = [
  'collision/collision_test_50.json',
  'collision/collision_test_50_free.json',
  'collision/collision_test_500.json',
  'collision/collision_test_5000.json',
  'collision/collision_test_500_free.json',
  'collision/collision_test_skull_50.json',
  'collision/collision_test_skull_50_free.json',
  'collision/collision_test_skull_500.json',
  'collision/collision_test_skull_5000.json',
  'collision/collision_test_skull_500_free.json',
  'cad/track_frame.json',
  'cad/track_loop.json',
  # the simulations of CadRound.TorusRollsAlongTheChannel and
  # CadRound.ThreadedTorusSlidesAlongTheTube, for the viewer
  'cad/torus_channel.json',
  'cad/torus_tube.json',
  # a frictionless nut screwing itself down a bolt (CadHard): each step
  # evaluates the whole helical thread of contact
  'cad/nut_on_bolt.json',
  'terramechanics/track_frame_scm.json',
  'terramechanics/track_frame_scm_fine.json',
  # the M1A2 tank: two tracks of 78 links each
  'm1a2/m1a2.json',
  'm1a2/m1a2_step.json',
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
