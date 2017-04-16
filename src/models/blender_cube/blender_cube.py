#!/usr/bin/env python

'''
The "Model" Python class is used script the creation
of JSON files

'''

import json


class Body(object):
    def __init__(
            self,
            body_name,
            definition=None,
            mass=0,
            center_of_mass=[0., 0., 0.],
            frames=None,
            moment_of_inertia={
                "Ixx": 0.0,
                "Ixy": 0.0,
                "Izx": 0.0,
                "Iyy": 0.0,
                "Iyz": 0.0,
                "Izz": 0.0
            }):
        self.name = body_name
        self.definition = definition
        self.mass = mass
        self.center_of_mass = center_of_mass
        self.frames = frames
        self.moment_of_inertia = moment_of_inertia


class Frame(object):
    def __init__(self, frame_name, T=None, R=None):
        self.name = frame_name
        self.T = T
        self.R = R


class Joint(object):
    def __init__(self,
                 joint_name,
                 body_frame_pairs=None,
                 joint_type="Rx",
                 initial_displacement=0.,
                 initial_velocity=0):

        self.name = joint_name
        self.body_frame_pairs = body_frame_pairs
        self.type = joint_type
        self.initial_displacement = initial_displacement
        self.initial_velocity = initial_velocity


class Model(object):
    def __init__(self):
        self.data = {}
        self.data["model_parameters"] = {
            "format": "0.2",
            "units": {
                "angle": "radians",
                "length": "meters",
                "mass": "kilograms",
                "time": "seconds"
            },
            "gravity": [0.0, -9.81, 0.0],
            "include_dir": [
                "."
            ]
        }
        self.data["assemblies"] = []

    def add_include_dir(self, include_dir_name):
        self.data["model_parameters"]["include_dir"].append(include_dir_name)

    def find_assembly(self, assembly_path):
        assemblies = self.data["assemblies"]
        for path in assembly_path:
            assembly = assemblies[
                [x[0] for x in enumerate(assemblies) if x[1]["name"] ==
                 path][0]]
            if "assemblies" in assembly:
                assemblies = assembly["assemblies"]
            else:
                break
        return assembly

    def add_assembly(self, assembly_name, path=None):
        assembly = {
            "name": assembly_name,
            "active": True,
            "bodies": [],
            "joints": []
        }
        if not path:

            self.data[
                "assemblies"].append(assembly)
        else:
            assembly_path = self.find_assembly(path)
            if "assemblies" not in assembly_path:
                assembly_path["assemblies"] = []
                assembly_path["assemblies"].append(assembly)

    def add_body(self,
                 body_obj,
                 assembly_path):
        body = {
            "name": body_obj.name,
            "definition": body_obj.definition,
            "mass": body_obj.mass,
            "center_of_mass": body_obj.center_of_mass,
            "frames": body_obj.frames,
            "moment_of_inertia": body_obj.moment_of_inertia
        }
        self.find_assembly(assembly_path)["bodies"].append(body)

    def add_joint(self,
                  joint_obj,
                  assembly_path):
        joint = {
            "joint_name": joint_obj.name,
            "body_frame_pairs": joint_obj.body_frame_pairs,
            "type": joint_obj.type,
            "initial_displacement": joint_obj.initial_displacement,
            "initial_velocity": joint_obj.initial_velocity}
        self.find_assembly(assembly_path)["joints"].append(joint)

    def write_to_file(self, filename):
        with open(filename, "w") as outfile:
            json.dump(self.data, outfile, indent=4)


if __name__ == "__main__":
	
    blender_cube = Model()
    blender_cube.add_include_dir("models/blender_cube")
    blender_cube.add_assembly("cube_assembly")
    blender_cube.add_body(Body("cube"), ["cube_assembly"])
    blender_cube.add_joint(Joint("joint1"), ["cube_assembly"])
    #print(my_model.find_assembly(["assem1"]))
    #print(my_model.data)
    blender_cube.write_to_file('blender_cube.json')
