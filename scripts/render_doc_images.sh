#!/bin/sh
# Render the example images of the documentation with the viewer, ray
# traced:
#   scripts/render_doc_images.sh [image name ...]
# renders all of them, or those named. SHIFT_BUILD names the build
# directory (default: build).
# Each line of the list below is: image name, model, simulated time (s),
# then optional viewer arguments (e.g. --view AZIMUTH,ELEVATION).
set -e
root=$(cd "$(dirname "$0")/.." && pwd)
build=$(cd "${SHIFT_BUILD:-$root/build}" && pwd)
viewer="$build/shift/main/shift"
out="$root/doc/sphinx/images/examples"
mkdir -p "$out"

while read -r name model time args; do
  case "$name" in ''|'#'*) continue ;; esac
  if [ $# -gt 0 ] && ! echo " $* " | grep -q " $name "; then
    continue
  fi
  dir=$(dirname "$root/examples/$model")
  # the loggers of a model write next to it; remove what this run writes
  before=$(ls "$dir")
  log=$(mktemp)
  if (cd "$dir" && "$viewer" "$(basename "$model")" --image "$out/$name.jpg" \
    --time "$time" --size 960x640 --samples 256 $args) >"$log" 2>&1; then
    grep '^shift:' "$log"
  else
    echo "failed: $name; the last lines of its output:" >&2
    tail -n 5 "$log" >&2
  fi
  rm -f "$log"
  for file in $(ls "$dir"); do
    case "$file" in *.h5|*.csv)
      echo "$before" | grep -qx "$file" || rm -f "$dir/$file" ;;
    esac
  done
done <<'LIST'
double_pendulum     pendulum/double_pendulum.json                1.0
slider_crank        slider_crank/slider_crank.json               0.5
bricard             bricard/bricard.json                         1.0
klann_linkage       klann_linkage/klann_linkage.json             1.0  --view 20,70
klann_walker        klann_linkage/klann_walker.json              1.0  --view 50,22
sarrus              sarrus/sarrus.json                           1.0
screw_and_gear      joints/screw_and_gear.json                   2.3  --view 270,10
brake               brake/brake.json                             0    --view 60,40
abiflexx_d4_1300    abiflexx_d4_1300/abiflexx_d4_1300.json       0.5
kuka_iiwa           kuka_lbr_iiwa_14_r820/kuka_iiwa.json         0    --view 40,20
excavator_simple    excavator/excavator_simple.json              0
excavator           excavator/excavator.json                     0    --view 120,20
bushing             force_modules/bushing.json                   2    --view 35,15
rubber_mount        force_modules/rubber_mount.json              0.2  --view 35,20
beam                force_modules/beam.json                      0.5  --view 75,12
belt                force_modules/belt.json                      1.0  --view 20,65
vehicle_control     vehicle/vehicle_control.json                 2.0
vehicle_terrain     vehicle/vehicle_terrain.json                 3.0
tire_fiala          vehicle/tire_fiala.json                      0.5
raptor              raptor/raptor.json                           0    --view 40,18
polo_r_wrc          polo_r_wrc/polo_r_wrc.json                   0    --view 40,18
polo_r_wrc_suspension polo_r_wrc/polo_r_wrc_suspension.json     8.0  --view 35,15 --follow chassis
m1a2                m1a2/m1a2.json                               1.0  --view 40,18
collision_50        collision/collision_test_50.json             1.0
collision_e_ch      collision/collision_test_e_ch.json           0.5
terrain             terrain/bricard_asphalt.json                 1.0
cad_parts           cad/cad_parts.json                           0
track_loop          cad/track_loop.json                          0    --view 30,25
track_frame         cad/track_frame.json                         0    --view 30,25
cube                scripting/cube.json                          0
LIST
