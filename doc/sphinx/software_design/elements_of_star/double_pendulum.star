{
  "model_parameters": {
    "star_version": "2.0.1",
    "units": {
      "angle": "radians",
      "length": "meters",
      "mass": "kilograms",
      "time": "seconds"
    },
    "gravity": [
      0.0,
      -9.81,
      0
    ]
  },
  "fixed": {
    "frames": {
      "overhead":{
        "T": [6.0, -6.0, -6.0]
      }
    },
    "lights": {
      "overhead_1":{
        "type": "point",
        "parent": ["fixed", "overhead"],
        "Ka": [0.4, 0.4, 0.4],
        "Kd" : [0.6, 0.6, 0.6],
        "Ks" : [0.2, 0.2, 0.2]
      }
    }
  },
  "assembly": {
    "double_pendulum": {
      "active": true,
      "bodies": {
        "link1": {
          "body_def": [
            "link.obj"
          ],
          "frames": {
            "1h_plus": {
              "T": [0.0, 1.0, 0.0]
            }
          },
          "center_of_mass": [0.0, 0.5, 0.0],
          "mass": 1,
          "moment_of_inertia": {
            "Izx": 0.0,
            "Ixx": 1.0,
            "Ixy": 0.0,
            "Izz": 1.0,
            "Iyy": 1.0,
            "Iyz": 0.0
          }
        },
        "link2": {
          "body_def": [
            "link.obj"
          ],
          "center_of_mass": [0.0, 0.5, 0.0],
          "mass": 1,
          "moment_of_inertia": {
            "Izx": 0.0,
            "Ixx": 1.0,
            "Ixy": 0.0,
            "Izz": 1.0,
            "Iyy": 1.0,
            "Iyz": 0.0
          }
        }
      },
      "joints": {
        "fixed_to_link1": {
          "body_frame_pairs": [
            [
              "fixed",
              "origin"
            ],
            [
              "link1",
              "origin"
            ]
          ],
          "type": "Rz",
          "initial_displacement": 0.2,
          "initial_velocity": 0.2
        },
        "link1_to_link2": {
          "body_frame_pairs": [
            [
              "link1",
              "1h_plus"
            ],
            [
              "link2",
              "origin"
            ]
          ],
          "type": "Rz",
          "initial_displacement": 0.0,
          "initial_velocity": 0.0
        }
      },
      "restraints": {
        "fixed_to_link1": {
          "type": "spring",
          "joint": "fixed_to_link1",
          "coefficients": [0.5, 0]
        }
      }
    }
  }
}
