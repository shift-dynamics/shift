#include "shift/core/controller/rally_driver.h"

#include <algorithm>
#include <cmath>
#include <fstream>
#include <sstream>

#include "ksl/vec3.h"
#include "shift/core/assembly.h"
#include "shift/core/controller/minimum_time.h"
#include "shift/core/exception.h"
#include "shift/core/factory.h"
#include "shift/core/force_module/tire/base.h"
#include "shift/core/context.h"
#include "shift/core/terrain/base.h"
#include "shift/core/log.h"

bool shift::core::controller::RallyDriver::registered_ =
  shift::core::Factory<shift::core::controller::Base>::get()->RegisterModule(
    "rally_driver", shift::core::controller::RallyDriver::Create);

std::unique_ptr<shift::core::controller::RallyDriver>
shift::core::controller::RallyDriver::Create() {
  return std::make_unique<shift::core::controller::RallyDriver>();
}

void shift::core::controller::RallyDriver::DeclareParameters() {
  shift::core::controller::Driver::DeclareParameters();
  DeclareRallyParameters();
}

void shift::core::controller::RallyDriver::DeclareRallyParameters() {
  DeclareParameter("mass", &mass_, "mass of the vehicle (kg)");
  DeclareParameter("yaw_inertia", &yaw_inertia_,
                   "moment of inertia of the vehicle about the vertical "
                   "through its center of mass (kg m^2)");
  DeclareParameter("center_of_mass", &center_of_mass_,
                   "distance of the center of mass ahead of the frame, on "
                   "the rear axle (m), less than the wheelbase");
  DeclareParameter("center_of_mass_height", &center_of_mass_height_,
                   "height of the center of mass above the ground (m)");
  DeclareParameter("track", &track_, "distance between the wheels of an "
                                     "axle (m)");
  DeclareParameter("front_load_transfer", &front_load_transfer_,
                   "share of the lateral load transfer on the front axle, "
                   "from the roll stiffnesses; defaults to 0.5",
                   false);
  DeclareParameter("front_roll_steer", &front_roll_steer_,
                   "steer of the front wheels out of the turn by the "
                   "suspension, rad per m/s^2 of lateral acceleration; "
                   "defaults to 0",
                   false);
  DeclareParameter("rear_roll_steer", &rear_roll_steer_,
                   "steer of the rear wheels out of the turn by the "
                   "suspension, rad per m/s^2 of lateral acceleration; "
                   "defaults to 0",
                   false);
  DeclareParameter("steering_gain", &steering_gain_,
                   "angle of the road wheels per unit of the steering "
                   "output; defaults to 1",
                   false);
  DeclareParameter("front_differential", &front_differential_,
                   "the front axle's limited slip differential: [bias "
                   "torque (N m), bias ratio, slip speed (rad/s)], a slip "
                   "speed of 0 for an open one; defaults to open",
                   false)
    .set_row_dim(3);
  DeclareParameter("rear_differential", &rear_differential_,
                   "the rear axle's limited slip differential: [bias torque "
                   "(N m), bias ratio, slip speed (rad/s)], a slip speed of 0 "
                   "for an open one; defaults to open",
                   false)
    .set_row_dim(3);
  DeclareParameter("tire", &tire_name_,
                   "a tire of the wheels, a tire force module of the same "
                   "assembly with an Iowa tire (tire_type \"iowa\"); the "
                   "steady slides take that tire as it is, with its "
                   "combined_slip and its soil");
  DeclareParameter("powertrain", &powertrain_name_,
                   "the powertrain of the vehicle, a force module of the "
                   "same assembly, for the throttle of each slide's drive "
                   "torque; none if not given",
                   false);
  DeclareParameter("slide_angle", &slide_angle_,
                   "sideslip in the tightest bends, the heading turned into "
                   "the bend from the course (rad), 0 for none; defaults to "
                   "0.25",
                   false);
  DeclareParameter("slide_curvature", &slide_curvature_,
                   "curvature of the path (1/m) from which the sideslip is "
                   "slide_angle, less in proportion below it; bends of half "
                   "of it or more are slid; defaults to 0.025",
                   false);
  DeclareParameter("slide_speed", &slide_speed_,
                   "speed (m/s) below which the car is steered as by the "
                   "driver controller; defaults to 8",
                   false);
  DeclareParameter("heading_gain", &heading_gain_,
                   "yaw rate (rad/s) per rad of error of the heading; "
                   "defaults to 2",
                   false)
    .set_runtime();
  DeclareParameter("lateral_gain", &lateral_gain_,
                   "rate (1/s) at which the heading is turned back to the "
                   "path, per m of lateral error over the speed (m/s); "
                   "defaults to 1",
                   false)
    .set_runtime();
  DeclareParameter("max_heading_correction", &max_heading_correction_,
                   "greatest turn of the target heading back to the path "
                   "(rad); defaults to 0.35",
                   false)
    .set_runtime();
  DeclareParameter("yaw_rate_gain", &yaw_rate_gain_,
                   "rate (1/s) at which the yaw rate is brought to its "
                   "target on grip; defaults to 5",
                   false)
    .set_runtime();
  DeclareParameter("sideslip_steering_gain", &sideslip_steering_gain_,
                   "steering of the road wheels (rad) per rad of sideslip "
                   "from its target, in a slide; defaults to 1",
                   false)
    .set_runtime();
  DeclareParameter("yaw_damping_gain", &yaw_damping_gain_,
                   "steering of the road wheels (rad) per rad/s of yaw rate "
                   "from its target, in a slide; defaults to 0.1",
                   false)
    .set_runtime();
  DeclareParameter("speed_throttle_gain", &speed_throttle_gain_,
                   "throttle per m/s below the speed of the slide; defaults "
                   "to 0.1",
                   false)
    .set_runtime();
  DeclareParameter("speed_integral_gain", &speed_integral_gain_,
                   "rate of the throttle's integral (1/s) per m/s below the "
                   "speed of the slide; defaults to 0.2",
                   false)
    .set_runtime();
  DeclareParameter("sideslip_throttle_gain", &sideslip_throttle_gain_,
                   "throttle per rad of shortfall of the oversteer from its "
                   "target, in a slide; defaults to 1",
                   false)
    .set_runtime();
  DeclareParameter("spin_throttle_gain", &spin_throttle_gain_,
                   "throttle per unit of spin of the wheels short of its "
                   "target, in a slide with the powertrain named; 0, the "
                   "default, for the speed and sideslip throttle instead",
                   false)
    .set_runtime();
  DeclareParameter("spin_sideslip_gain", &spin_sideslip_gain_,
                   "spin of the wheels per rad of shortfall of the "
                   "oversteer from its target; defaults to 1",
                   false)
    .set_runtime();
  DeclareParameter("gear_hysteresis", &gear_hysteresis_,
                   "with a powertrain, the share more torque at the wheels "
                   "at full throttle that the next gear up or down must "
                   "have than the held one to be chosen; defaults to 0.2",
                   false)
    .set_runtime();
  DeclareParameter("shift_interval", &shift_interval_,
                   "least time between two shifts (s): the gearbox is "
                   "sequential, a gear up or down at each shift; defaults "
                   "to 0.3",
                   false)
    .set_runtime();
  DeclareParameter("steering_rate", &steering_rate_,
                   "greatest rate of the steering output (rad/s); defaults "
                   "to 1",
                   false)
    .set_runtime();
  DeclareParameter("throttle_rate", &throttle_rate_,
                   "greatest rate of the throttle (1/s); defaults to 4",
                   false)
    .set_runtime();
  DeclareParameter("steering_lag", &steering_lag_,
                   "time constant of a first-order lag of the steering "
                   "after its rate limit (s), the driver's neuromuscular "
                   "lag, about 0.1 (MacAdam 1981; Pick and Cole 2008); "
                   "defaults to 0, none",
                   false)
    .set_runtime();
  DeclareParameter("throttle_lag", &throttle_lag_,
                   "time constant of a first-order lag of the throttle "
                   "after its rate limit (s); defaults to 0, none",
                   false)
    .set_runtime();
  DeclareParameter("curvature_time", &curvature_time_,
                   "the bend ahead is the mean curvature of the path over "
                   "this time of travel at the speed (s), at least "
                   "lookahead_distance long; defaults to 0.5",
                   false)
    .set_runtime();
  DeclareParameter("regulator", &regulator_,
                   "whether the car is held in a slide by the linear "
                   "quadratic regulator of its motion about the steady "
                   "slide, with the powertrain named; defaults to false",
                   false);
  DeclareParameter("drive_inertia", &drive_inertia_,
                   "inertia of the drive at the wheels, the engine, "
                   "transmission, shafts and wheels together (kg m^2), for "
                   "the regulator; defaults to 60",
                   false);
  DeclareParameter("regulator_scales", &regulator_scales_,
                   "the errors the regulator weighs as one: of v_x (m/s), "
                   "v_y (m/s), the yaw rate (rad/s), the carrier speed "
                   "(rad/s), the lateral error (m) and the course's error "
                   "(rad), and of the steering (rad) and the drive torque "
                   "(N m); defaults to [2, 0.7, 0.1, 5, 1, 0.1, 0.1, 500]",
                   false)
    .set_row_dim(8);
  DeclareParameter("regulator_rate_scales", &regulator_rate_scales_,
                   "the rates of the steering (rad/s) and of the drive "
                   "torque (N m/s) the regulator weighs as one, if given: "
                   "the regulator then sets their changes, the last "
                   "inputs in its state (SlideEquilibrium::RateRegulator), "
                   "so that it corrects the slide smoothly; defaults to "
                   "[0, 0], none",
                   false)
    .set_row_dim(2);
  DeclareParameter("slide_fastest", &slide_fastest_,
                   "each bend's slide at the least sideslip, up to "
                   "slide_angle, within slide_speed_share of the speed of the fastest "
                   "steady slide on its radius, in place of slide_angle "
                   "scaled by the curvature over slide_curvature; defaults "
                   "to false",
                   false);
  DeclareParameter("slide_speed_share", &slide_speed_share_,
                   "with slide_fastest, the least share of the fastest "
                   "steady slide's speed that a bend's slide has, at the "
                   "least sideslip that gives it; defaults to 0.995",
                   false);
  DeclareParameter("slide_entry_share", &slide_entry_share_,
                   "a slide begun only at this share of the speed of the "
                   "bend's slide along the car, and left below slide_exit_margin less; "
                   "defaults to 0, at any speed",
                   false)
    .set_runtime();
  DeclareParameter("slide_speed_table", &slide_speed_table_,
                   "with slide_fastest: each bend's steady slides of every "
                   "sideslip up to its fastest, the regulator's slide that "
                   "of the car's speed, so that a slide begun slow is one "
                   "of less sideslip; defaults to false",
                   false);
  DeclareParameter("traction_spin", &traction_spin_,
                   "with the regulator, the greatest wheelspin on grip: "
                   "above it the throttle falls by spin_throttle_gain for "
                   "each unit of spin, as a driver feathers it on loose "
                   "ground; defaults to 0, no limit",
                   false)
    .set_runtime();
  DeclareParameter("slide_entry_rate", &slide_entry_rate_,
                   "with slide_fastest and the regulator, the greatest rate "
                   "(rad/s) at which the reference's sideslip goes from the "
                   "car's as a slide begins to the bend's slide, through "
                   "the steady slides between; defaults to 0, at once",
                   false);
  DeclareParameter("grip_sideslip_gain", &grip_sideslip_gain_,
                   "with the regulator, on grip, the rate (1/s) at which the "
                   "yaw rate's target returns the sideslip to zero: the "
                   "heading turned toward the course, so that a slide just "
                   "left does not grow; defaults to 0, none",
                   false)
    .set_runtime();
  DeclareParameter("slide_exit_margin", &slide_exit_margin_,
                   "with slide_entry_share, the share of the bend's slide's "
                   "speed below slide_entry_share at which a slide is left; "
                   "defaults to 0.1",
                   false)
    .set_runtime();
  DeclareParameter("brake_torque", &brake_torque_,
                   "the brakes' torque at the wheels at full pedal, all "
                   "four (N m): the regulator then brakes for a drive "
                   "torque below zero, left-foot braking through the "
                   "slide; defaults to 0, it does not brake",
                   false)
    .set_runtime();
  DeclareParameter("minimum_time_file", &minimum_time_file_,
                   "a CSV file for the run of least time of the planar car "
                   "(MinimumTime) from the car's station at the first "
                   "outputs to minimum_time_end, along the path as the "
                   "road's middle, made then; defaults to none",
                   false);
  DeclareParameter("minimum_time_plan", &minimum_time_plan_,
                   "a CSV file that minimum_time_file wrote, its run of "
                   "least time loaded in place of solving it, for the same "
                   "path and minimum_time_step; defaults to none",
                   false);
  DeclareParameter("minimum_time_slope", &minimum_time_slope_,
                   "the run of least time on the terrain's grade and camber "
                   "along the path, from its heights 2 m along and 1.5 m "
                   "across; false: on a level road; defaults to true",
                   false);
  DeclareParameter("minimum_time_end", &minimum_time_end_,
                   "the station (m) at which the run of least time ends; "
                   "defaults to 0, the path's end",
                   false);
  DeclareParameter("minimum_time_step", &minimum_time_step_,
                   "between the stations of the run of least time (m); "
                   "defaults to 0.5",
                   false);
  DeclareParameter("minimum_time_half_width", &minimum_time_half_width_,
                   "half the width of the road (m) that the car's outline "
                   "keeps within in the run of least time; defaults to 3.4",
                   false);
  DeclareParameter("minimum_time_outline", &minimum_time_outline_,
                   "the car's outline about its frame for the run of least "
                   "time (m): its rear (negative) and its front along x, and "
                   "its half width; defaults to [0, 0, 0], the frame's point",
                   false)
    .set_row_dim(3);
  DeclareParameter("minimum_time_steering_rate", &minimum_time_steering_rate_,
                   "the greatest rate of the road wheels' steering in the "
                   "run of least time (rad/s); defaults to 2",
                   false);
  DeclareParameter("minimum_time_start_speed", &minimum_time_start_speed_,
                   "the least speed (m/s) at which the run of least time "
                   "starts, the planar car's motion being along the road; "
                   "defaults to 3",
                   false);
  DeclareParameter("minimum_time_iterations", &minimum_time_iterations_,
                   "the most iterations of the run of least time; defaults "
                   "to 400",
                   false);
  DeclareParameter("minimum_time_track", &minimum_time_track_,
                   "the car follows the run of least time (MinimumTime), "
                   "made at the first outputs, by its time-varying linear "
                   "quadratic regulator (MinimumTime::TrackingGains) in its "
                   "stations, the drive torque the plan's with "
                   "minimum_time_wheel_gain for the wheels' speed; beyond "
                   "it, as without; needs the powertrain and brake_torque; "
                   "defaults to false",
                   false);
  DeclareParameter("minimum_time_track_scales", &minimum_time_track_scales_,
                   "the scales of the errors the tracking regulator weighs "
                   "as one: v_x, v_y (m/s), the yaw rate (rad/s), the "
                   "lateral error (m), the course and the steering (rad), "
                   "and of its inputs, the steering's change along the "
                   "road (rad/m) and the wheels' speed (rad/s); defaults "
                   "to [1, 0.5, 0.3, 0.3, 0.1, 0.2, 0.05, 1]",
                   false)
    .set_row_dim(8);
  DeclareParameter("minimum_time_wheel_gain", &minimum_time_wheel_gain_,
                   "the drive torque (N m) beyond the plan's for each rad/s "
                   "of the wheels' speed below the tracking regulator's; "
                   "defaults to 600",
                   false);
  DeclareParameter("max_lateral_error", &max_lateral_error_,
                   "greatest lateral error the regulator answers (m); "
                   "defaults to 2",
                   false)
    .set_runtime();
  DeclareParameter("speed_scheduled", &speed_scheduled_,
                   "the regulator's slide at the car's speed, the speed "
                   "then a parameter of the slide rather than one of its "
                   "errors; defaults to true",
                   false)
    .set_runtime();
  DeclareParameter("reference_time", &reference_time_,
                   "time constant (s) at which the regulator's reference goes "
                   "from the motion as a slide begins to the steady slide's; "
                   "defaults to 0.5",
                   false)
    .set_runtime();
  DeclareParameter("saturation_torque", &saturation_torque_,
                   "whether, with the regulator, the drive torque is that of "
                   "least cost with the steering where its limits hold it; "
                   "defaults to true",
                   false)
    .set_runtime();
  DeclareParameter("saturation_governor", &saturation_governor_,
                   "whether, with the regulator, its reference stays where "
                   "it is while the steering is held at its limits; "
                   "defaults to true",
                   false)
    .set_runtime();
  DeclareParameter("front_force_margin", &front_force_margin_,
                   "share of their friction that the front tires keep in "
                   "hand in a slide of the table, for the steering to hold "
                   "it; defaults to 0.02",
                   false);
  DeclareParameter("sample_time", &sample_time_,
                   "time between the outputs (s), for the regulator; "
                   "defaults to 0.05",
                   false);
  DeclareParameter("slide_speed_margin", &slide_speed_margin_,
                   "a slide begins only within this of the speed of the "
                   "profile (m/s), and ends beyond twice this, so that the "
                   "car is braked for a bend before it is slid; defaults to "
                   "3",
                   false)
    .set_runtime();
  DeclareParameter("roll_limit", &roll_limit_,
                   "roll of the body (rad) beyond which the car is not slid "
                   "and the handbrake not pulled; defaults to 0.12",
                   false)
    .set_runtime();
  DeclareParameter("slide_throttle", &slide_throttle_,
                   "throttle's integral as a slide begins, 0 to 1; defaults "
                   "to 0.4",
                   false)
    .set_runtime();
  DeclareParameter("max_slide_throttle", &max_slide_throttle_,
                   "greatest throttle in a slide, 0 to 1; defaults to 1",
                   false)
    .set_runtime();
  DeclareParameter("handbrake_threshold", &handbrake_threshold_,
                   "shortfall of the oversteer from its target (rad) beyond "
                   "which the handbrake is pulled to start a slide; defaults "
                   "to 0.15",
                   false)
    .set_runtime();
  DeclareParameter("handbrake_pull", &handbrake_pull_,
                   "handbrake of a pull, 0 to 1, 0 for none; defaults to 1",
                   false)
    .set_runtime();
  DeclareParameter("handbrake_time", &handbrake_time_,
                   "duration of a pull of the handbrake (s); defaults to 0.4",
                   false)
    .set_runtime();
  DeclareParameter("handbrake_interval", &handbrake_interval_,
                   "least time from one pull of the handbrake to the next "
                   "(s); defaults to 2",
                   false)
    .set_runtime();
  DeclareOutput("handbrake", &handbrake_, "handbrake, 0 to 1");
  DeclareOutput("sideslip", &sideslip_,
                "angle of the course of the frame from its heading, positive "
                "to the right (rad)");
  DeclareOutput("sideslip_target", &sideslip_target_,
                "target of the sideslip (rad)");
  DeclareOutput("yaw_rate", &yaw_rate_,
                "rate of yaw of the frame, about its z axis (rad/s)");
  DeclareOutput("yaw_rate_target", &yaw_rate_target_,
                "target of the yaw rate (rad/s)");
}

void shift::core::controller::RallyDriver::Connect() {
  const std::string prefix = "Rally driver '" + get_name() + "': ";
  if(!(mass_ > 0.0) || !(yaw_inertia_ > 0.0) || !(wheelbase_ > 0.0) ||
     !(center_of_mass_ > 0.0) || !(center_of_mass_ < wheelbase_) ||
     !(center_of_mass_height_ >= 0.0) || !(track_ > 0.0) ||
     !(front_load_transfer_ >= 0.0) || !(front_load_transfer_ <= 1.0) ||
     !(steering_gain_ > 0.0)) {
    throw shift::core::Exception(
      prefix + "mass, yaw_inertia, wheelbase, track and steering_gain must "
               "be positive, center_of_mass between 0 and the wheelbase, "
               "center_of_mass_height not negative and front_load_transfer "
               "from 0 to 1");
  }
  for(const auto* d : {&front_differential_, &rear_differential_}) {
    if(d->size() != 3 || !((*d)[0] >= 0.0) || !((*d)[1] >= 0.0) ||
       !((*d)[2] >= 0.0)) {
      throw shift::core::Exception(
        prefix + "a differential is [bias torque, bias ratio, slip speed], "
                 "none negative");
    }
  }
  if(!(slide_angle_ >= 0.0) || !(slide_angle_ < 0.5 * M_PI) ||
     !(slide_curvature_ > 0.0) || !(slide_speed_ > 0.0) ||
     !(heading_gain_ >= 0.0) || !(lateral_gain_ >= 0.0) ||
     !(max_heading_correction_ >= 0.0) || !(yaw_rate_gain_ >= 0.0) ||
     !(sideslip_steering_gain_ >= 0.0) || !(yaw_damping_gain_ >= 0.0) ||
     !(speed_throttle_gain_ >= 0.0) || !(speed_integral_gain_ >= 0.0) ||
     !(sideslip_throttle_gain_ >= 0.0) || !(spin_throttle_gain_ >= 0.0) ||
     !(spin_sideslip_gain_ >= 0.0) || !(gear_hysteresis_ >= 0.0) ||
     !(shift_interval_ >= 0.0) ||
     !(steering_rate_ > 0.0) || !(throttle_rate_ > 0.0) ||
     !(steering_lag_ >= 0.0) || !(throttle_lag_ >= 0.0) ||
     !(brake_torque_ >= 0.0) || !(traction_spin_ >= 0.0) ||
     !(minimum_time_end_ >= 0.0) || !(minimum_time_step_ > 0.0) ||
     !(minimum_time_half_width_ > 0.0) || minimum_time_outline_.size() != 3 ||
     !(minimum_time_steering_rate_ > 0.0) ||
     !(minimum_time_start_speed_ > 0.0) || minimum_time_iterations_ < 1 ||
     minimum_time_track_scales_.size() != 8 ||
     !(*std::min_element(minimum_time_track_scales_.begin(),
                         minimum_time_track_scales_.end()) > 0.0) ||
     !(minimum_time_wheel_gain_ >= 0.0) ||
     !(slide_entry_rate_ >= 0.0) || !(slide_exit_margin_ >= 0.0) || !(grip_sideslip_gain_ >= 0.0) ||
     (slide_entry_rate_ > 0.0 && !slide_fastest_) ||
     (slide_speed_table_ && !slide_fastest_) ||
     !(slide_speed_share_ > 0.0) || !(slide_speed_share_ <= 1.0) ||
     !(slide_entry_share_ >= 0.0) ||
     !(curvature_time_ >= 0.0) || !(roll_limit_ > 0.0) ||
     !(slide_speed_margin_ >= 0.0) || !(reference_time_ >= 0.0) ||
     !(drive_inertia_ > 0.0) || !(sample_time_ > 0.0) ||
     !(front_force_margin_ >= 0.0) || !(front_force_margin_ < 1.0) ||
     regulator_scales_.size() != 8 ||
     regulator_rate_scales_.size() != 2 ||
     (RateRegulated() &&
      !(std::min(regulator_rate_scales_[0], regulator_rate_scales_[1]) >
        0.0)) ||
     !(*std::min_element(regulator_scales_.begin(), regulator_scales_.end()) >
       0.0) ||
     !(slide_throttle_ >= 0.0) ||
     !(max_slide_throttle_ <= 1.0) ||
     !(slide_throttle_ <= max_slide_throttle_) ||
     !(handbrake_threshold_ >= 0.0) || !(handbrake_pull_ >= 0.0) ||
     !(handbrake_pull_ <= 1.0) || !(handbrake_time_ >= 0.0) ||
     !(handbrake_interval_ >= 0.0)) {
    throw shift::core::Exception(
      prefix + "slide_angle must be from 0 to pi/2, slide_curvature and "
               "slide_speed positive, 0 <= slide_throttle <= "
               "max_slide_throttle <= 1, handbrake_pull from 0 to 1, and the "
               "gains, handbrake_threshold, handbrake_time and "
               "handbrake_interval not negative");
  }
  powertrain_ = nullptr;
  if(!powertrain_name_.empty()) {
    for(const auto& module : parent_assembly_ptr_->get_force_modules()) {
      if(module->get_name() == powertrain_name_) {
        powertrain_ =
          dynamic_cast<const shift::core::force_module::Powertrain*>(
            module.get());
      }
    }
    if(powertrain_ == nullptr) {
      throw shift::core::Exception(prefix + "powertrain '" + powertrain_name_ +
                                   "' is not a powertrain of its assembly");
    }
  }
  if(regulator_ && powertrain_ == nullptr) {
    throw shift::core::Exception(prefix + "the regulator needs the powertrain");
  }
  if(minimum_time_track_ && (powertrain_ == nullptr || !(brake_torque_ > 0.0))) {
    throw shift::core::Exception(
      prefix + "minimum_time_track needs the powertrain and brake_torque");
  }
  const shift::core::force_module::tire::Iowa* iowa = nullptr;
  for(const auto& module : parent_assembly_ptr_->get_force_modules()) {
    if(module->get_name() == tire_name_) {
      const auto* tire =
        dynamic_cast<const shift::core::force_module::tire::Base*>(
          module.get());
      if(tire) {
        iowa = tire->get_iowa();
      }
    }
  }
  if(iowa == nullptr) {
    throw shift::core::Exception(prefix + "tire '" + tire_name_ +
                                 "' is not a tire of its assembly with an "
                                 "Iowa tire (tire_type \"iowa\")");
  }
  /* the tire as its JSON gives it: the controllers connect before the
  force modules load, which resolves the soil and prepares the tire */
  tire_ = *iowa;
  tire_.ResolveSoil(prefix + "tire '" + tire_name_ + "'");
  tire_.Prepare();
  /* a tire with no soil of its own takes the terrain's, under it on the
  road: the slides take the soil under the start of the path (once the
  terrain has loaded, see UpdateOutputs) */
  if(!tire_.is_soft()) {
    if(const auto* ground = StartSoil()) {
      tire_.SetSoil(*ground);
    }
  }
  BuildTable();
  /* the path and the speed profile, which takes the slides' speeds */
  shift::core::controller::Driver::Connect();
  handbrake_ = sideslip_ = sideslip_target_ = 0.0;
  yaw_rate_ = yaw_rate_target_ = 0.0;
  time_ = 0.0;
  wants_pull_ = sliding_ = in_slide_ = started_ = false;
  integral_ = powertrain_ ? 0.0 : slide_throttle_;
  speed_error_ = 0.0;
  held_gear_ = 0.0;
  last_shift_ = -INFINITY;
  pull_until_ = next_pull_ = -INFINITY;
}

shift::core::controller::SlideEquilibrium
shift::core::controller::RallyDriver::PlanarCar() const {
  shift::core::controller::SlideEquilibrium car;
  car.mass = mass_;
  car.wheelbase = wheelbase_;
  car.track = track_;
  car.center_of_mass = center_of_mass_;
  car.height = center_of_mass_height_;
  car.front_load_transfer = front_load_transfer_;
  car.front_roll_steer = front_roll_steer_;
  car.rear_roll_steer = rear_roll_steer_;
  car.gravity = kGravity;
  car.yaw_inertia = yaw_inertia_;
  car.drive_inertia = drive_inertia_;
  car.tire = tire_;
  car.front = {front_differential_[0], front_differential_[1],
               front_differential_[2]};
  car.rear = {rear_differential_[0], rear_differential_[1],
              rear_differential_[2]};
  return car;
}

void shift::core::controller::RallyDriver::PlanMinimumTime() {
  shift::core::controller::MinimumTime plan;
  plan.car = PlanarCar();
  plan.curvature = [this](double s) { return CurvatureAt(s); };
  const auto* drive = powertrain_;
  /* at full throttle in the best gear the engine turns in, below 0.97 of
  its high idle, the clutch slipping at half of it at low speed */
  plan.torque_limit = [drive](double w) -> double {
    if(drive == nullptr) {
      return INFINITY;
    }
    const double top = 0.97 * drive->get_high_idle_speed();
    double best = 0.0;
    for(int g = 1; g <= drive->get_forward_gears(); ++g) {
      const double ratio = drive->OverallRatio(g);
      if(!(ratio > 0.0) || ratio * w > top) {
        continue;
      }
      best = std::max(
        best, drive->FullWheelTorque(std::max(w, 0.5 * top / ratio), g));
    }
    return best;
  };
  if(minimum_time_slope_) {
    /* gravity along the path, to its right and normal to the road, from
    the slopes of the terrain's heights */
    plan.slope = [this](double station, double* g) {
      double ax, ay, bx, by, px, py;
      PointAt(station - 0.25, ax, ay);
      PointAt(station + 0.25, bx, by);
      PointAt(station, px, py);
      const double length = std::max(std::hypot(bx - ax, by - ay), 1e-9);
      const double tx = (bx - ax) / length, ty = (by - ay) / length;
      constexpr double along = 2.0, across = 1.5;
      double z[4];
      const bool found =
        GroundHeight(px - along * tx, py - along * ty, &z[0]) &&
        GroundHeight(px + along * tx, py + along * ty, &z[1]) &&
        GroundHeight(px + across * ty, py - across * tx, &z[2]) &&
        GroundHeight(px - across * ty, py + across * tx, &z[3]);
      if(!found) {
        return;
      }
      /* z down: the road falling ahead or to the right */
      const double grade = (z[1] - z[0]) / (2.0 * along);
      const double camber = (z[3] - z[2]) / (2.0 * across);
      const double norm = std::sqrt(1.0 + grade * grade + camber * camber);
      g[0] = kGravity * grade / norm;
      g[1] = kGravity * camber / norm;
      g[2] = kGravity / norm;
    };
  }
  plan.guess_speed = [this](double s) {
    return std::max(0.7 * ProfileSpeed(s), 3.0);
  };
  plan.brake_torque = brake_torque_;
  plan.steering_limit = max_steering_angle_ * steering_gain_;
  plan.steering_rate = minimum_time_steering_rate_;
  plan.half_width = minimum_time_half_width_;
  for(int i = 0; i < 3; ++i) {
    plan.outline[i] = minimum_time_outline_[i];
  }
  plan.start = station_;
  plan.end = minimum_time_end_ > 0.0 ? minimum_time_end_ : get_length();
  plan.step = minimum_time_step_;
  plan.start_speed = minimum_time_start_speed_;
  plan.start_lateral = lateral_error_;
  plan.max_iterations = minimum_time_iterations_;
  bool solved = false;
  if(minimum_time_plan_.empty()) {
    solved = plan.Solve();
  } else {
    /* the states (vx, vy, yaw_rate, lateral, course_error, steering,
    carrier) of each station of the file */
    std::ifstream in(minimum_time_plan_);
    std::string line;
    if(!std::getline(in, line)) {
      throw shift::core::Exception(get_name() + ": cannot read " +
                                   minimum_time_plan_);
    }
    std::vector<std::string> names;
    {
      std::stringstream header(line);
      std::string name;
      while(std::getline(header, name, ',')) {
        names.push_back(name);
      }
    }
    const char* wanted[] = {"vx", "vy", "yaw_rate", "lateral",
                            "course_error", "steering", "carrier"};
    int column[7], station_column = -1;
    for(int i = 0; i < 7; ++i) {
      const auto it = std::find(names.begin(), names.end(), wanted[i]);
      if(it == names.end()) {
        throw shift::core::Exception(get_name() + ": no " + wanted[i] +
                                     " in " + minimum_time_plan_);
      }
      column[i] = static_cast<int>(it - names.begin());
    }
    station_column = static_cast<int>(
      std::find(names.begin(), names.end(), "station") - names.begin());
    std::vector<double> states, stations;
    while(std::getline(in, line)) {
      std::vector<double> row;
      std::stringstream fields(line);
      std::string field;
      while(std::getline(fields, field, ',')) {
        row.push_back(std::stod(field));
      }
      if(row.size() != names.size()) {
        continue;
      }
      stations.push_back(row[station_column]);
      for(int i = 0; i < 7; ++i) {
        states.push_back(row[column[i]]);
      }
    }
    if(stations.size() < 2) {
      throw shift::core::Exception(get_name() + ": no run in " +
                                   minimum_time_plan_);
    }
    plan.start = stations.front();
    plan.end = stations.back();
    if(!plan.Load(states)) {
      throw shift::core::Exception(get_name() + ": the stations of " +
                                   minimum_time_plan_ +
                                   " are not minimum_time_step apart");
    }
    solved = plan.get_violation() < plan.tolerance;
  }
  const int n = plan.get_stations();
  plan_states_.clear();
  plan_torques_.clear();
  plan_gains_.clear();
  if(minimum_time_track_ &&
     plan.TrackingGains(minimum_time_track_scales_.data(), &plan_gains_)) {
    plan_start_ = plan.start;
    plan_step_ = plan.step;
    plan_states_ = plan.get_states();
    plan_torques_ = plan.get_torques();
  }
  const auto& x = plan.get_states();
  const auto& times = plan.get_times();
  const auto& torques = plan.get_torques();
  SHIFT_INFO(get_name() + ": run of least time " +
             std::to_string(times.empty() ? 0.0 : times.back()) + " s from " +
             std::to_string(plan.start) + " to " + std::to_string(plan.end) +
             " m, " + std::to_string(plan.get_iterations()) +
             " iterations, constraints within " +
             std::to_string(plan.get_violation()) +
             (solved ? "" : " (not solved)"));
  if(minimum_time_file_.empty()) {
    return;
  }
  std::ofstream out(minimum_time_file_);
  out << "station,time,x,y,vx,vy,yaw_rate,lateral,course_error,sideslip,"
         "heading_error,steering,carrier,spin,drive_torque,torque_limit,"
         "speed,curvature\n";
  out.precision(9);
  const double radius = tire_.unloaded_radius;
  for(int k = 0; k < n; ++k) {
    const double* s = &x[k * shift::core::controller::MinimumTime::kStates];
    const double station = plan.start + k * plan.step;
    double px, py, ax, ay, bx, by;
    PointAt(station, px, py);
    PointAt(station - 0.25, ax, ay);
    PointAt(station + 0.25, bx, by);
    const double length = std::max(std::hypot(bx - ax, by - ay), 1e-9);
    /* the right of the path, x to y with z down */
    const double rx = -(by - ay) / length, ry = (bx - ax) / length;
    const double beta = std::atan2(s[1], s[0]);
    out << station << "," << times[k] << "," << px + s[3] * rx << ","
        << py + s[3] * ry << "," << s[0] << "," << s[1] << "," << s[2] << ","
        << s[3] << "," << s[4] << "," << beta << "," << s[4] - beta << ","
        << s[5] << "," << s[6] << "," << s[6] * radius / s[0] - 1.0 << ","
        << torques[k] << "," << plan.torque_limit(s[6]) << ","
        << std::hypot(s[0], s[1]) << "," << CurvatureAt(station) << "\n";
  }

}

bool shift::core::controller::RallyDriver::TrackPlan(double vx, double vy,
                                                     double yaw_rate,
                                                     double course_error) {
  constexpr int ns = shift::core::controller::MinimumTime::kStates;
  constexpr int nt = shift::core::controller::MinimumTime::kTrackStates;
  constexpr int ni = shift::core::controller::MinimumTime::kTrackInputs;
  const int stations = static_cast<int>(plan_torques_.size());
  if(stations < 2 || !(vx > 0.5)) {
    return false;
  }
  const double f = (station_ - plan_start_) / plan_step_;
  if(!(f >= 0.0) || f >= stations - 1) {
    return false;
  }
  /* the plan at the station, between its two nearest */
  const int k = static_cast<int>(f);
  const double w = f - k;
  double ref[ns], gain[ni * nt];
  for(int i = 0; i < ns; ++i) {
    ref[i] = (1.0 - w) * plan_states_[k * ns + i] +
             w * plan_states_[(k + 1) * ns + i];
  }
  for(int i = 0; i < ni * nt; ++i) {
    gain[i] = (1.0 - w) * plan_gains_[k * ni * nt + i] +
              w * plan_gains_[(k + 1) * ni * nt + i];
  }
  const double torque_ref =
    (1.0 - w) * plan_torques_[k] + w * plan_torques_[k + 1];
  /* the steering's last, at the road wheels, and its change along the road
  in the plan */
  const double steered = last_steering_ * steering_gain_;
  const int next = std::min(k + 1, stations - 1);
  const double change =
    (plan_states_[next * ns + 5] - plan_states_[k * ns + 5]) / plan_step_;
  const double error[nt] = {vx - ref[0],
                            vy - ref[1],
                            yaw_rate - ref[2],
                            lateral_error_ - ref[3],
                            course_error - ref[4],
                            steered - ref[5]};
  double rate = change, omega = ref[6];
  for(int j = 0; j < nt; ++j) {
    rate -= gain[j] * error[j];
    omega -= gain[nt + j] * error[j];
  }
  /* the change over the output step, at the speed along the road */
  const double along = std::max(vx * std::cos(course_error) /
                                  std::max(1.0 - CurvatureAt(station_) *
                                                   lateral_error_,
                                           0.1),
                                0.0);
  const double delta = steered + rate * along * output_step_;
  target_speed_ = std::hypot(ref[0], ref[1]);
  sideslip_target_ = std::atan2(ref[1], ref[0]);
  yaw_rate_target_ = ref[2];
  /* the steering within its travel and rate */
  steering_ =
    std::clamp(delta / steering_gain_, -max_steering_angle_, max_steering_angle_);
  if(rate_limited_) {
    steering_ =
      std::clamp(steering_, last_steering_ - steering_rate_ * output_step_,
                 last_steering_ + steering_rate_ * output_step_);
  }
  /* the wheels' speed by the drive torque, in pedals, in the gear for it */
  const double measured = powertrain_->get_wheel_speed();
  const double torque =
    torque_ref + minimum_time_wheel_gain_ * (omega - measured);
  const int gear = SlideGear(std::max(omega, 0.0));
  gear_ = gear;
  throttle_ = handbrake_ > 0.0 || !(torque > 0.0)
                ? 0.0
                : std::min(powertrain_->SteadyThrottle(std::max(measured, 0.0),
                                                       gear, torque),
                           1.0);
  brake_ = !(handbrake_ > 0.0) && torque < 0.0
             ? std::min(-torque / brake_torque_, 1.0)
             : 0.0;
  return true;
}

void shift::core::controller::RallyDriver::BuildTable() {
  table_curvature_.clear();
  table_sideslip_.clear();
  table_speed_.clear();
  table_steering_.clear();
  table_spin_.clear();
  table_torque_.clear();
  table_gain_.clear();
  table_coupling_.clear();
  table_column_.clear();
  for(std::vector<double>* c :
      {&column_speed_, &column_sideslip_, &column_steering_, &column_spin_,
       &column_torque_, &column_coupling_, &column_gain_, &column_angle_}) {
    c->clear();
  }
  if(!(slide_angle_ > 0.0)) {
    return;
  }
  const shift::core::controller::SlideEquilibrium car = PlanarCar();
  /* the weights of the regulator, the inverse squares of the scales */
  constexpr int ns = shift::core::controller::SlideEquilibrium::kPathStates;
  constexpr int ni = shift::core::controller::SlideEquilibrium::kInputs;
  double q[ns * ns] = {}, w[ni * ni] = {};
  for(int i = 0; i < ns; ++i) {
    q[i * ns + i] = 1.0 / (regulator_scales_[i] * regulator_scales_[i]);
  }
  for(int i = 0; i < ni; ++i) {
    w[i * ni + i] =
      1.0 / (regulator_scales_[ns + i] * regulator_scales_[ns + i]);
  }
  /* from the gentlest slide bend to the tightest, each from the last, at
  least to a radius of 10 m */
  const int n = 36;
  const double lowest = 0.5 * slide_curvature_;
  const double highest = std::max(4.0 * slide_curvature_, 0.1);
  /* a slide within the steering's travel, its front tires with
  front_force_margin of their friction in hand to hold it */
  const double steering_limit = max_steering_angle_ * steering_gain_;
  /* the gains of the steering and the torque of a slide, each of the path's
  states and the last steering and torque */
  auto regulate = [&](double curvature, double sideslip,
                      const shift::core::controller::SlideEquilibrium::State&
                        s,
                      double* gain, double* coupling) {
    std::fill(gain, gain + kGains, 0.0);
    *coupling = 0.0;
    if(!regulator_) {
      return;
    }
    double a[ns * ns], b[ns * ni];
    car.Linearize(sideslip, s, a, b);
    bool solved;
    if(!RateRegulated()) {
      double k[ni * ns];
      solved = shift::core::controller::SlideEquilibrium::Regulator(
        a, b, q, w, sample_time_, k, coupling);
      for(int i = 0; i < ni; ++i) {
        std::copy(k + i * ns, k + (i + 1) * ns, gain + i * (ns + ni));
      }
    } else {
      double rates[ni * ni] = {};
      for(int i = 0; i < ni; ++i) {
        rates[i * ni + i] =
          1.0 / (regulator_rate_scales_[i] * regulator_rate_scales_[i]);
      }
      solved = shift::core::controller::SlideEquilibrium::RateRegulator(
        a, b, q, w, rates, sample_time_, gain, coupling);
    }
    if(!solved) {
      throw shift::core::Exception(
        "Rally driver '" + get_name() + "': no regulator of the slide on "
        "the radius " + std::to_string(1.0 / curvature) + " m at " +
        std::to_string(sideslip * 180.0 / M_PI) + " deg");
    }
  };
  shift::core::controller::SlideEquilibrium::State state;
  bool have = false;
  for(int k = 0; k < n; ++k) {
    const double curvature = lowest + (highest - lowest) * k / (n - 1);
    double sideslip = 0.0;
    bool found = false;
    auto admissible = [&](double beta,
                          shift::core::controller::SlideEquilibrium::State* s) {
      const bool solved = (have && car.Solve(1.0 / curvature, beta, s)) ||
                          car.Find(1.0 / curvature, beta, s);
      return solved && std::fabs(s->steering) <= steering_limit &&
             car.FrontUse(*s) <= 1.0 - front_force_margin_;
    };
    if(slide_fastest_) {
      /* the least sideslip, up to slide_angle, of a slide within slide_speed_share of
      the fastest steady one */
      constexpr int m = 24;
      double speeds[m], fastest = 0.0;
      shift::core::controller::SlideEquilibrium::State states[m];
      for(int j = 0; j < m; ++j) {
        states[j] = state;
        const double beta = -slide_angle_ * (j + 1) / m;
        speeds[j] = admissible(beta, &states[j]) ? states[j].speed : 0.0;
        fastest = std::max(fastest, speeds[j]);
      }
      for(int j = 0; j < m && !found; ++j) {
        if(fastest > 0.0 && speeds[j] >= slide_speed_share_ * fastest) {
          sideslip = -slide_angle_ * (j + 1) / m;
          state = states[j];
          found = true;
        }
      }
      if(found && (slide_speed_table_ || slide_entry_rate_ > 0.0)) {
        /* the slides to it, each faster than the last */
        table_column_.push_back(column_speed_.size());
        double last = 0.0;
        for(int j = 0; j < m && -slide_angle_ * (j + 1) / m >= sideslip;
            ++j) {
          if(!(speeds[j] > last)) {
            continue;
          }
          last = speeds[j];
          const double beta = -slide_angle_ * (j + 1) / m;
          double gain[kGains], coupling;
          regulate(curvature, beta, states[j], gain, &coupling);
          column_speed_.push_back(states[j].speed);
          column_sideslip_.push_back(beta);
          column_angle_.push_back(-beta);
          column_steering_.push_back(states[j].steering);
          column_spin_.push_back(states[j].spin);
          column_torque_.push_back(states[j].drive_torque);
          column_coupling_.push_back(coupling);
          column_gain_.insert(column_gain_.end(), gain, gain + kGains);
          SHIFT_DEBUG("Rally driver '" + get_name() + "': slide on " +
                      std::to_string(1.0 / curvature) + " m at " +
                      std::to_string(beta * 180.0 / M_PI) + " deg: " +
                      std::to_string(states[j].speed) + " m/s, spin " +
                      std::to_string(states[j].spin) + ", drive " +
                      std::to_string(states[j].drive_torque) + " N m");
        }
      }
    }
    const double target =
      -slide_angle_ * std::min(1.0, curvature / slide_curvature_);
    for(int tries = 0; tries < 16 && !found; ++tries) {
      sideslip = target * std::pow(0.85, tries);
      shift::core::controller::SlideEquilibrium::State s = state;
      found = admissible(sideslip, &s);
      if(found) {
        state = s;
      }
    }
    if(!found) {
      throw shift::core::Exception("Rally driver '" + get_name() +
                                   "': no steady slide on the radius " +
                                   std::to_string(1.0 / curvature) + " m");
    }
    have = true;
    table_curvature_.push_back(curvature);
    table_sideslip_.push_back(sideslip);
    table_speed_.push_back(state.speed);
    table_steering_.push_back(state.steering);
    table_spin_.push_back(state.spin);
    table_torque_.push_back(state.drive_torque);
    double gain[kGains], coupling;
    regulate(curvature, sideslip, state, gain, &coupling);
    table_gain_.insert(table_gain_.end(), gain, gain + kGains);
    table_coupling_.push_back(coupling);
    SHIFT_INFO("Rally driver '" + get_name() + "': slide on " +
                   std::to_string(1.0 / curvature) + " m at " +
                   std::to_string(sideslip * 180.0 / M_PI) +
                   " deg: " + std::to_string(state.speed) + " m/s, steering " +
                   std::to_string(state.steering) + " rad, spin " +
                   std::to_string(state.spin) + ", drive " +
                   std::to_string(state.drive_torque) + " N m, front use " +
                   std::to_string(car.FrontUse(state)));
  }
  if(!table_column_.empty()) {
    table_column_.push_back(column_speed_.size());
  }
}

void shift::core::controller::RallyDriver::SlideAtSpeed(
  double curvature, double speed, double* sideslip, double* steering,
  double* spin, double* torque, double* gain, double* coupling) const {
  double at;
  SlideInColumns(curvature, column_speed_, speed, &at, sideslip, steering,
                 spin, torque, gain, coupling);
}

void shift::core::controller::RallyDriver::SlideAtSideslip(
  double curvature, double sideslip, double* speed, double* steering,
  double* spin, double* torque, double* gain, double* coupling) const {
  double at;
  SlideInColumns(curvature, column_angle_, -sideslip, speed, &at, steering,
                 spin, torque, gain, coupling);
}

void shift::core::controller::RallyDriver::SlideInColumns(
  double curvature, const std::vector<double>& key, double value,
  double* speed, double* sideslip, double* steering, double* spin,
  double* torque, double* gain, double* coupling) const {
  const size_t n = table_curvature_.size();
  if(table_column_.size() != n + 1 || n < 2) {
    Slide(curvature, sideslip, speed, steering, spin, torque, gain,
          coupling);
    return;
  }
  const double k = std::fabs(curvature);
  size_t i = 0;
  double u = 0.0;
  if(k >= table_curvature_.back()) {
    i = n - 2;
    u = 1.0;
  } else if(k > table_curvature_.front()) {
    i = std::upper_bound(table_curvature_.begin(), table_curvature_.end(), k) -
        table_curvature_.begin() - 1;
    u = (k - table_curvature_[i]) /
        (table_curvature_[i + 1] - table_curvature_[i]);
  }
  /* each of the two bends' slides at the value, within its least and its
  fastest, then between the bends */
  double out[6 + kGains] = {};
  for(int bend = 0; bend < 2; ++bend) {
    const size_t begin = table_column_[i + bend];
    const size_t end = table_column_[i + bend + 1];
    size_t j = begin;
    double v = 0.0;
    if(end - begin >= 2) {
      if(value >= key[end - 1]) {
        j = end - 2;
        v = 1.0;
      } else if(value > key[begin]) {
        j = std::upper_bound(key.begin() + begin, key.begin() + end, value) -
            key.begin() - 1;
        v = (value - key[j]) / (key[j + 1] - key[j]);
      }
    }
    const size_t l = end - begin >= 2 ? j + 1 : j;
    auto at = [&](const std::vector<double>& t, size_t stride, size_t c) {
      return t[j * stride + c] + v * (t[l * stride + c] - t[j * stride + c]);
    };
    const double values[6] = {at(column_sideslip_, 1, 0),
                              at(column_steering_, 1, 0),
                              at(column_spin_, 1, 0),
                              at(column_torque_, 1, 0),
                              at(column_coupling_, 1, 0),
                              at(column_speed_, 1, 0)};
    const double weight = bend == 0 ? 1.0 - u : u;
    for(int c = 0; c < 6; ++c) {
      out[c] += weight * values[c];
    }
    for(int c = 0; c < kGains; ++c) {
      out[6 + c] += weight * at(column_gain_, kGains, c);
    }
  }
  *sideslip = out[0];
  *steering = out[1];
  *speed = out[5];
  if(spin) {
    *spin = out[2];
  }
  if(torque) {
    *torque = out[3];
  }
  if(coupling) {
    *coupling = out[4];
  }
  if(gain) {
    std::copy(out + 6, out + 6 + kGains, gain);
  }
}

void shift::core::controller::RallyDriver::Slide(double curvature,
                                                 double* sideslip,
                                                 double* speed,
                                                 double* steering,
                                                 double* spin,
                                                 double* torque,
                                                 double* gain,
                                                 double* coupling) const {
  const double k = std::fabs(curvature);
  const size_t n = table_curvature_.size();
  if(n == 0) {
    *sideslip = *speed = *steering = 0.0;
    if(spin) {
      *spin = 0.0;
    }
    if(torque) {
      *torque = 0.0;
    }
    if(gain) {
      std::fill(gain, gain + kGains, 0.0);
    }
    if(coupling) {
      *coupling = 0.0;
    }
    return;
  }
  size_t i = 0;
  double u = 0.0;
  if(k <= table_curvature_.front()) {
    i = 0;
  } else if(k >= table_curvature_.back()) {
    i = n - 2;
    u = 1.0;
  } else {
    i = std::upper_bound(table_curvature_.begin(), table_curvature_.end(), k) -
        table_curvature_.begin() - 1;
    u = (k - table_curvature_[i]) /
        (table_curvature_[i + 1] - table_curvature_[i]);
  }
  if(n == 1) {
    i = 0;
    u = 0.0;
  }
  auto at = [&](const std::vector<double>& t) {
    return n == 1 ? t[0] : t[i] + u * (t[i + 1] - t[i]);
  };
  *sideslip = at(table_sideslip_);
  *speed = at(table_speed_);
  *steering = at(table_steering_);
  if(spin) {
    *spin = at(table_spin_);
  }
  if(torque) {
    *torque = at(table_torque_);
  }
  if(coupling) {
    *coupling = at(table_coupling_);
  }
  if(gain) {
    constexpr int m = kGains;
    for(int j = 0; j < m; ++j) {
      gain[j] = n == 1 ? table_gain_[j]
                       : table_gain_[i * m + j] +
                           u * (table_gain_[(i + 1) * m + j] -
                                table_gain_[i * m + j]);
    }
  }
}

double shift::core::controller::RallyDriver::CornerSpeed(
  double curvature) const {
  if(!IsSlideBend(curvature) || table_curvature_.empty()) {
    return shift::core::controller::Driver::CornerSpeed(curvature);
  }
  double sideslip, speed, steering;
  Slide(curvature, &sideslip, &speed, &steering);
  return speed * std::cos(sideslip);
}

int shift::core::controller::RallyDriver::SlideGear(double wheel_speed) const {
  /* the wheels' speed the drive needs, the car's speed with the spin it
  plans, but no more than the transmission's output shaft measures: spin
  past the plan is lost traction, which a driver does not shift up on */
  const int n = powertrain_->get_forward_gears();
  const double measured = powertrain_->get_wheel_speed();
  const double planned = std::max(wheel_speed, 0.0);
  const double w = measured > 0.0 ? std::min(measured, planned) : planned;
  const int held = static_cast<int>(std::lround(held_gear_));
  if(held < 1 || held > n) {
    return 1;
  }
  /* a gear up or down, as a sequential box shifts, if it has a share
  gear_hysteresis more torque at the wheels at full throttle than the held
  one: the engine kept in its power band, torque in hand for the slide */
  int gear = held;
  double most = powertrain_->FullWheelTorque(w, held);
  for(int g : {held - 1, held + 1}) {
    if(g < 1 || g > n) {
      continue;
    }
    const double reserve = powertrain_->FullWheelTorque(w, g);
    if(reserve > (1.0 + gear_hysteresis_) * std::max(most, 0.0) &&
       powertrain_->OverallRatio(g) * w <
         0.97 * powertrain_->get_high_idle_speed()) {
      most = reserve;
      gear = g;
    }
  }
  return gear;
}

bool shift::core::controller::RallyDriver::GroundHeight(double x, double y,
                                                        double* z) const {
  const ksl_vec3_t point = {{x, y, 0.0}};
  for(auto terrain : context_->get_active_terrain()) {
    ksl_vec3_t surface, normal;
    if(terrain->Query(point, surface, normal)) {
      *z = surface.z;
      return true;
    }
  }
  return false;
}

const shift::core::force_module::SoilProperties*
shift::core::controller::RallyDriver::StartSoil() const {
  if(path_.size() < 2) {
    return nullptr;
  }
  const ksl_vec3_t start = {{path_[0], path_[1], 0.0}};
  for(auto terrain : context_->get_active_terrain()) {
    if(const auto* ground = terrain->SoilAt(start)) {
      return ground;
    }
  }
  return nullptr;
}

void shift::core::controller::RallyDriver::UpdateOutputs(double t,
                                                         const double* y) {
  /* the terrain loads after the controllers connect: at the first outputs,
  a tire with no soil of its own that now finds the terrain's under the
  start takes it, and the slides and the speed profile are made again */
  if(!started_ && !tire_.is_soft() && StartSoil()) {
    Connect();
  }
  /* the throttle's integral over the last output step, from slide_throttle
  as a slide begins */
  const double dt = std::max(t - time_, 0.0);
  if(in_slide_) {
    integral_ =
      sliding_ ? std::clamp(integral_ + speed_integral_gain_ * speed_error_ * dt,
                            -max_slide_throttle_, max_slide_throttle_)
               : (powertrain_ ? 0.0 : slide_throttle_);
  }
  sliding_ = in_slide_;
  const bool first = !started_;
  started_ = true;
  time_ = t;
  const double last_steering = steering_, last_throttle = throttle_;
  last_steering_ = steering_;
  output_step_ = dt;
  rate_limited_ = !first;
  shift::core::controller::Driver::UpdateOutputs(t, y);
  if(first && (!minimum_time_file_.empty() || minimum_time_track_)) {
    PlanMinimumTime();
  }
  /* a pull of the handbrake, if one is wanted and none was lately: from
  this output on, so that the outputs of this step show it */
  if(wants_pull_ && handbrake_pull_ > 0.0 && t >= next_pull_ &&
     t >= pull_until_) {
    pull_until_ = t + handbrake_time_;
    next_pull_ = t + handbrake_interval_;
    shift::core::controller::Driver::UpdateOutputs(t, y);
  }
  /* the steering and the throttle move from their last outputs at their
  greatest rates */
  if(!first) {
    steering_ = std::clamp(steering_, last_steering - steering_rate_ * dt,
                           last_steering + steering_rate_ * dt);
    throttle_ = std::clamp(throttle_, last_throttle - throttle_rate_ * dt,
                           last_throttle + throttle_rate_ * dt);
    /* then through the driver's lags, exact for a command held over dt */
    if(steering_lag_ > 0.0) {
      steering_ = last_steering + (1.0 - std::exp(-dt / steering_lag_)) *
                                    (steering_ - last_steering);
    }
    if(throttle_lag_ > 0.0) {
      throttle_ = last_throttle + (1.0 - std::exp(-dt / throttle_lag_)) *
                                    (throttle_ - last_throttle);
    }
  }
  /* the gearbox is sequential: a gear up or down from the held one, at
  most once in shift_interval */
  const int held = static_cast<int>(std::lround(held_gear_));
  const int gear = static_cast<int>(std::lround(gear_));
  if(!first && held >= 1 && gear >= 1 && gear != held) {
    gear_ = t - last_shift_ < shift_interval_
              ? held
              : std::clamp(gear, held - 1, held + 1);
  }
  if(static_cast<int>(std::lround(gear_)) != held) {
    last_shift_ = t;
  }
  held_gear_ = gear_;
}

void shift::core::controller::RallyDriver::Service(const double*, double*) {
  /* the outputs are held from one output to the next */
}

double shift::core::controller::RallyDriver::FrontFriction() const {
  return tire_.friction_coefficient * mass_ * kGravity * center_of_mass_ /
         wheelbase_;
}

double shift::core::controller::RallyDriver::FrontSlipAngle(double f) const {
  /* the lateral force of the two tires, each of half the axle's load */
  const double limit = 0.5 * FrontFriction();
  const double q = std::min(0.5 * std::fabs(f), limit);
  /* up to the slip angle of the peak, or on the loose ground's shape, to
  where the force has nearly all of the friction */
  const bool loose = tire_.is_loose(tire_.is_soft());
  double lo = 0.0, hi = loose && tire_.has_loose_shape()
                          ? 1.4
                          : 2.5 * limit / tire_.cornering_stiffness;
  for(int k = 0; k < 50; ++k) {
    const double alpha = 0.5 * (lo + hi);
    if(tire_.LateralForce(alpha, limit, loose) < q) {
      lo = alpha;
    } else {
      hi = alpha;
    }
  }
  const double alpha = 0.5 * (lo + hi);
  return f < 0.0 ? -alpha : alpha;
}

void shift::core::controller::RallyDriver::DriveMotion(
  const ksl_SE3_t& pose, const ksl_vec3_t& velocity, double yaw_rate) {
  /* the driver controller's outputs: the station, the lateral error, the
  speed profile, the throttle, the brake and the gear, and pure pursuit,
  kept when slow */
  shift::core::controller::Driver::DriveMotion(pose, velocity, yaw_rate);
  const double vx = ksl_dot_vv(&pose.R.v0, &velocity);
  const double vy = ksl_dot_vv(&pose.R.v1, &velocity);
  const double v = std::hypot(vx, vy);
  yaw_rate_ = yaw_rate;
  wants_pull_ = false;
  in_slide_ = false;
  handbrake_ = time_ < pull_until_ ? handbrake_pull_ : 0.0;
  if(handbrake_ > 0.0) {
    throttle_ = 0.0;
  }
  sideslip_ = vx > 0.0 ? std::atan2(vy, vx) : 0.0;
  sideslip_target_ = yaw_rate_target_ = 0.0;
  if(!plan_torques_.empty()) {
    /* along the plan: the course's error from the path's direction */
    double ax, ay, bx, by;
    PointAt(station_ - 0.5, ax, ay);
    PointAt(station_ + 0.5, bx, by);
    const double direction = std::atan2(by - ay, bx - ax);
    const double heading = std::atan2(pose.R.v0.y, pose.R.v0.x);
    const double course =
      std::remainder(heading + sideslip_ - direction, 2.0 * M_PI);
    if(TrackPlan(vx, vy, yaw_rate, course)) {
      return;
    }
  }
  if(v <= slide_speed_ || vx <= 0.0) {
    return;
  }

  /* the bend at the frame, a little ahead, its steady slide and the target
  sideslip */
  const double preview = 0.5 * (lookahead_distance_ + lookahead_time_ * vx);
  const double window = std::max(lookahead_distance_, curvature_time_ * vx);
  double curvature = 0.0;
  for(int k = 0; k < 5; ++k) {
    curvature += 0.2 * CurvatureAt(station_ + preview + window * (k - 2) / 4.0);
  }
  const double side = curvature > 0.0 ? 1.0 : curvature < 0.0 ? -1.0 : 0.0;
  /* into a slide at half of slide_curvature, out of it below a third, and
  not with the body rolled past roll_limit, on a slope across the road */
  const double roll = std::asin(std::clamp(pose.R.v1.z, -1.0, 1.0));
  const bool level = std::fabs(roll) <= roll_limit_;
  const double margin = slide_speed_margin_;
  bool slide =
    !table_curvature_.empty() && slide_angle_ > 0.0 && level &&
    vx <= target_speed_ + (sliding_ ? 2.0 : 1.0) * margin &&
    std::fabs(curvature) >= (sliding_ ? 1.0 / 3.0 : 0.5) * slide_curvature_;
  if(slide && slide_entry_share_ > 0.0) {
    /* and, with slide_entry_share, fast enough for the bend's slide: in
    at that share of its speed along the car, out below slide_exit_margin
    less */
    double beta, speed, steering;
    Slide(curvature, &beta, &speed, &steering);
    slide = vx >= (slide_entry_share_ - (sliding_ ? slide_exit_margin_ : 0.0)) * speed *
                    std::cos(beta);
  }
  double slide_sideslip = 0.0, slide_speed = 0.0, slide_steering = 0.0;
  double slide_spin = 0.0, slide_torque = 0.0, gain[kGains] = {};
  double coupling = 0.0;
  if(slide) {
    Slide(curvature, &slide_sideslip, &slide_speed, &slide_steering,
          &slide_spin, &slide_torque, gain, &coupling);
    if(regulator_ && slide_speed_table_ && !table_column_.empty()) {
      /* the slide of the car's speed, of less sideslip if slower than the
      fastest; the fastest's speed along the car kept for the speed's term */
      const double fastest_sideslip = slide_sideslip;
      SlideAtSpeed(curvature, v, &slide_sideslip, &slide_steering,
                   &slide_spin, &slide_torque, gain, &coupling);
      slide_speed *= std::cos(fastest_sideslip) / std::cos(slide_sideslip);
    }
    if(regulator_ && slide_entry_rate_ > 0.0 && !table_column_.empty()) {
      /* the reference's sideslip into the bend from the car's as the slide
      begins, to the bend's slide at slide_entry_rate, through the steady
      slides between */
      const double angle = -slide_sideslip;
      entry_angle_ =
        sliding_ ? std::min(entry_angle_ + slide_entry_rate_ * output_step_,
                            angle)
                 : std::clamp(-side * sideslip_, 0.0, angle);
      if(entry_angle_ < angle) {
        /* the bend's slide's speed kept, so that the car is not slowed to
        the slower slides between and out of the slide */
        double between;
        SlideAtSideslip(curvature, -entry_angle_, &between,
                        &slide_steering, &slide_spin, &slide_torque, gain,
                        &coupling);
        slide_sideslip = -entry_angle_;
      }
    }
    sideslip_target_ = side * slide_sideslip;
  } else if(!level || regulator_) {
    sideslip_target_ = 0.0;
  } else {
    sideslip_target_ = -side * slide_angle_ *
                       std::min(1.0, std::fabs(curvature) / slide_curvature_);
  }

  /* the target heading, along the path less the target sideslip, and the
  target yaw rate */
  double ax, ay, bx, by;
  PointAt(station_ - 0.5, ax, ay);
  PointAt(station_ + 0.5, bx, by);
  const double direction = std::atan2(by - ay, bx - ax);
  const double heading = std::atan2(pose.R.v0.y, pose.R.v0.x);
  const double correction =
    std::clamp(std::atan(lateral_gain_ * lateral_error_ / v),
               -max_heading_correction_, max_heading_correction_);
  /* with the regulator, the course, heading and sideslip, is steered to the
  path, so that the yaw of a slide just left asks no turn */
  const double heading_error = std::remainder(
    heading + (regulator_ ? sideslip_ : 0.0) -
      (direction - sideslip_target_ - correction),
    2.0 * M_PI);
  yaw_rate_target_ = v * curvature - heading_gain_ * heading_error;
  if(regulator_ && !slide) {
    /* on grip, the sideslip's error decays at grip_sideslip_gain, as
    d beta / dt = a_y / v - r */
    yaw_rate_target_ += grip_sideslip_gain_ * (sideslip_ - sideslip_target_);
  }
  if(regulator_) {
    /* within what the tires give */
    const double most = tire_.friction_coefficient * kGravity / v;
    yaw_rate_target_ = std::clamp(yaw_rate_target_, -most, most);
  }

  if(!slide) {
    /* on grip: the front lateral force that brings the yaw rate to it, and
    the steering that gives the front axle that force */
    const double a = wheelbase_ - center_of_mass_;
    const double b = center_of_mass_;
    const double rear_force = mass_ * v * yaw_rate_target_ * a / wheelbase_;
    const double limit = 0.98 * FrontFriction();
    const double front_force = std::clamp(
      (b * rear_force -
       yaw_inertia_ * yaw_rate_gain_ * (yaw_rate - yaw_rate_target_)) /
        a,
      -limit, limit);
    steering_ = (std::atan2(vy + wheelbase_ * yaw_rate, vx) +
                 FrontSlipAngle(front_force)) /
                steering_gain_;
    steering_ =
      std::clamp(steering_, -max_steering_angle_, max_steering_angle_);
    if(regulator_ && powertrain_) {
      /* the throttle of the profile as a share of the drive torque the
      tires can take, in the gear that needs least throttle for it, as in a
      slide, so that the gear does not jump between the two */
      const double radius = tire_.unloaded_radius;
      const double traction =
        tire_.friction_coefficient * mass_ * kGravity * radius;
      const double torque = throttle_ * traction;
      const double w = 1.05 * vx / radius;
      const int gear = SlideGear(w);
      gear_ = gear;
      throttle_ = torque > 0.0 ? std::min(powertrain_->SteadyThrottle(
                                            w, gear, torque),
                                          1.0)
                               : 0.0;
      /* less, past traction_spin, by spin_throttle_gain for each unit of
      the spin from the engine's speed in the gear of the last outputs */
      const int held = static_cast<int>(std::lround(held_gear_));
      const double ratio = powertrain_->OverallRatio(held);
      if(traction_spin_ > 0.0 && ratio > 0.0 && held == gear) {
        const double spin = powertrain_->get_engine_speed() * radius /
                              (ratio * std::max(vx, 2.0)) -
                            1.0;
        throttle_ = std::max(
          throttle_ - spin_throttle_gain_ * std::max(spin - traction_spin_, 0.0),
          0.0);
      }
    }
    return;
  }

  in_slide_ = true;
  if(regulator_) {
    /* in a slide, by the regulator: the error of the motion from the steady
    slide's, in a right bend's terms, the yaw rate's target turning the
    course to the path */
    speed_error_ = target_speed_ - vx;
    const double radius = tire_.unloaded_radius;
    /* the slide at the car's speed, as Goh et al. (2020): the speed a
    parameter of the slide, not an error of it */
    const double vx_e = speed_scheduled_ ? vx : slide_speed * std::cos(slide_sideslip);
    const double vy_e = vx_e * std::tan(slide_sideslip);
    const double course_error =
      std::remainder(heading + sideslip_ - direction, 2.0 * M_PI);
    const double r_ref = (speed_scheduled_ ? vx_e / std::cos(slide_sideslip)
                                           : slide_speed) *
                         std::fabs(curvature);
    yaw_rate_target_ = side * r_ref;
    const double w_e = (1.0 + slide_spin) * vx_e / radius;
    const int gear = SlideGear((1.0 + slide_spin) * vx / radius);
    const int held = static_cast<int>(std::lround(held_gear_));
    const double ratio = powertrain_->OverallRatio(held);
    const double w = ratio > 0.0 && held == gear
                       ? powertrain_->get_engine_speed() / ratio
                       : (1.0 + slide_spin) * vx / radius;
    /* the reference: from the motion and the inputs as the slide begins, to
    the steady slide's with the time constant reference_time */
    const double steady[6] = {vx_e,
                              side * vy_e,
                              side * r_ref,
                              w_e,
                              side * slide_steering,
                              slide_torque};
    if(!sliding_) {
      const double start[6] = {vx,
                               vy,
                               yaw_rate,
                               w,
                               last_steering_ * steering_gain_,
                               slide_torque};
      std::copy(start, start + 6, reference_);
      regulator_torque_ = reference_[5];
      steering_saturated_ = false;
    } else if(!(saturation_governor_ && steering_saturated_)) {
      const double blend =
        reference_time_ > 0.0
          ? 1.0 - std::exp(-std::max(time_ - reference_at_, 0.0) /
                           reference_time_)
          : 1.0;
      for(int j = 0; j < 6; ++j) {
        reference_[j] += blend * (steady[j] - reference_[j]);
      }
    }
    reference_at_ = time_;
    yaw_rate_target_ = reference_[2];
    /* the errors, in a right bend's terms, each within two of its scales,
    the path's within their limits */
    auto within = [](double x, double limit) {
      return std::clamp(x, -limit, limit);
    };
    const double z[6] = {
      within(vx - reference_[0], 2.0 * regulator_scales_[0]),
      within(side * (vy - reference_[1]), 2.0 * regulator_scales_[1]),
      within(side * (yaw_rate - reference_[2]), 2.0 * regulator_scales_[2]),
      within(w - reference_[3], 2.0 * regulator_scales_[3]),
      side * within(lateral_error_, max_lateral_error_),
      side * within(course_error, max_heading_correction_)};
    wants_pull_ = false;
    /* with regulator_rate_scales, the changes of the last steering and
    torque, in their state with the path's */
    const bool rates = RateRegulated();
    const double x[8] = {
      z[0], z[1], z[2], z[3], z[4], z[5],
      rates ? side * (last_steering_ * steering_gain_ - reference_[4]) : 0.0,
      rates ? regulator_torque_ - reference_[5] : 0.0};
    double delta = x[6], torque = reference_[5] + x[7];
    for(int j = 0; j < 8; ++j) {
      delta -= gain[j] * x[j];
      torque -= gain[8 + j] * x[j];
    }
    if(speed_scheduled_) {
      /* the speed to the steady slide's by the drive torque alone */
      torque -= gain[8] *
                within(vx - slide_speed * std::cos(slide_sideslip),
                       2.0 * regulator_scales_[0]);
    }
    /* the steering within its limits, and the torque of least cost with
    the steering where they hold it */
    const double wanted = (reference_[4] + side * delta) / steering_gain_;
    steering_ = std::clamp(wanted, -max_steering_angle_, max_steering_angle_);
    if(rate_limited_) {
      steering_ =
        std::clamp(steering_, last_steering_ - steering_rate_ * output_step_,
                   last_steering_ + steering_rate_ * output_step_);
    }
    steering_saturated_ = std::fabs(steering_ - wanted) > 1e-9;
    if(saturation_torque_ && steering_saturated_) {
      const double held = side * (steering_ * steering_gain_ - reference_[4]);
      torque += coupling * (delta - held);
    }
    regulator_torque_ = torque;
    gear_ = gear;
    throttle_ = handbrake_ > 0.0 || !(torque > 0.0)
                  ? 0.0
                  : std::min(powertrain_->SteadyThrottle(w, gear, torque),
                             max_slide_throttle_);
    /* a torque below zero by the brakes, with brake_torque: the axles turn
    together, so that the brakes take it as the engine gives it */
    brake_ = brake_torque_ > 0.0 && !(handbrake_ > 0.0) && torque < 0.0
               ? std::min(-torque / brake_torque_, 1.0)
               : 0.0;
    return;
  }
  /* in a slide: the steering about the slide's, for the sideslip and the
  yaw rate */
  steering_ = (side * slide_steering +
               sideslip_steering_gain_ * (sideslip_ - sideslip_target_) -
               yaw_damping_gain_ * (yaw_rate - yaw_rate_target_)) /
              steering_gain_;
  steering_ = std::clamp(steering_, -max_steering_angle_, max_steering_angle_);

  /* the throttle that holds the slide's speed and turns the car for its
  sideslip, and a pull of the handbrake to start a slide */
  speed_error_ = target_speed_ - vx;
  const double shortfall = std::fabs(sideslip_target_) + side * sideslip_;
  wants_pull_ = shortfall > handbrake_threshold_;
  if(handbrake_ > 0.0) {
    return;
  }
  double base = sliding_ ? integral_ : (powertrain_ ? 0.0 : slide_throttle_);
  if(powertrain_) {
    /* the gear of the most torque in hand, kept within 0.1 of throttle */
    const double rolling = tire_.unloaded_radius -
                           0.25 * mass_ * kGravity / tire_.vertical_stiffness;
    const double wheel_speed = (1.0 + slide_spin) * vx / rolling;
    const int held = static_cast<int>(std::lround(held_gear_));
    const int gear = SlideGear(wheel_speed);
    if(gear >= 1) {
      gear_ = gear;
      const double feedforward =
        powertrain_->SteadyThrottle(wheel_speed, gear, slide_torque);
      base += feedforward;
      /* the spin from the engine's speed, in the gear of the last outputs */
      const double ratio = powertrain_->OverallRatio(held);
      if(spin_throttle_gain_ > 0.0 && ratio > 0.0 && held == gear) {
        const double spin =
          powertrain_->get_engine_speed() * rolling / (ratio * vx) - 1.0;
        const double target =
          std::clamp(slide_spin + spin_sideslip_gain_ * shortfall, 0.0, 1.0);
        throttle_ = std::clamp(feedforward +
                                 spin_throttle_gain_ * (target - spin) +
                                 speed_throttle_gain_ * speed_error_,
                               0.0, max_slide_throttle_);
        if(vx < target_speed_ + 2.0) {
          brake_ = 0.0;
        }
        return;
      }
    }
  }
  throttle_ = std::clamp(base + speed_throttle_gain_ * speed_error_ +
                           sideslip_throttle_gain_ * shortfall,
                         0.0, max_slide_throttle_);
  if(vx < target_speed_ + 2.0) {
    brake_ = 0.0;
  }
}
