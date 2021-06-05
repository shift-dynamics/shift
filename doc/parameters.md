# Named parameters

## Overview
*Shift* provides a mechanism to load named parameters from a JSON input file and to query those parameters within the source code at run-time. Because the class hierarcy within *Shift* source code follows the hierarchy defined in JSON input file, this mechanism allows any **Shift** object to access other **Shift** objects or their parameters programmatically using the path to the object defined in the input file. This mechanism for introspection offers the following advantages:

* **Simplifies adding new parameters**: Loading values from your JSON input file into your *Shift* data structures is simple, requiring a single call to a simple helper function. There is no need for developers of *Shift* objects to write JSON parsing code. JSON parsing is handled automatically within the `shift::core::parameter::Value` class.
* **Simplifies access to parameters**: *Shift* objects contain a `std::map` of their keys and values which can be used to retrieve a type safe constant reference or constant pointer to the underlying data. There is no need to write get functions to make data accessible to other classes. The method for accessing data should be familiar to developers who have used a Python dictionary.
* **Simplifies debugging**: The JSON parsing code is moved to one file in one location, which simplifies debugging the model parsing pipeline. Additionally, `shift::core::parameter::*` classes provide a default `Print()` method, which can be used to print named parameters to verify correct loading of the model.
* **Flexibility**: The `shift::core::parameter::Value` class allows for flexibility in loading objects from JSON files. For example, multiple keys can map to the same object (for example, the keys `"R"` and `"rotation"` could both map data to a 3x3 rotation matrix internally -- to allow an alternative short-hand version of the key in the input file) and data can be transformed from one type to another during loading (for example, a quaternion can be transformed to a 3x3 rotation matrix)

## Adding named parameters
To map keys in a JSON input file to members of a class, populate the `DeclareParameters()` function with the key, member variable, description and whether the data is required. During class construction, the `DeclareParameters()` function is called to create the mapping. The following code demonstrates how keys are added to the `shift::core::body::Rigid` class.

```c
void Shift::Frame::DeclareParameters() {
  DeclareParameter("name", &name_, "descriptive frame name", true);

  DeclareParameter(
    "active", &active_,
    "indicates whether frame is active, defaults to true if not specified",
    false);

  DeclareParameter(
    "R", &d_wrt_origin_.R,
    "specify orientation as a 3x3 rotation matrix in row-major "
    "order, defaults to identity if not specified; the keys 'rotation', 'R', "
    "'euler_angles', 'quaternion', 'q' or 'Q' can be used to designate an "
    "orientation. If multiple key-value pairs are specified for "
    "orientation, only one of the entries will be loaded.",
    false);

  ...
}
```

## Supported data types
Presently, *Shift* provides support to load strings, doubles, vectors of strings, doubles or integers and all data structures used in the kinematic subroutine library (KSL) - the math library used internally by *Shift*.

The following data types are supported:

* `std::string`
* `std::vector<std::string>`
* `std::vector<std::vector<std::string>>`
* `double`
* `int`
* `std::vector<int>`
* `std::vector<double>`
* `ksl_vec3_t`
* `ksl_vec4_t`
* `ksl_quaternion_t`
* `ksl_screw_t`
* `ksl_coscrew_t`
* `ksl_inertia_t`
* `ksl_mat3x3_t`
* `ksl_mat4x4_t`
* `ksl_eulerangles_t`
* `bool`


## Loading factory objects from JSON files

When loading objects from a JSON file, *Shift* uses the reserved key `"type"` to determine what specialization of the base class to use when creating the corresponding object in C++. The type is then used to create an instance of the object using a factory method.

Some *Shift* objects have a default type. For example, the default type for bodies is `rigid`. Therefore, if no type is specified, a rigid body will be created. Other objects, such as a joints, do not have a default type, and the `type` key must exist in the JSON file. Consider the following definition of a revolute joint in a model input file:

```python
"joints": [
   {
    "name": "A",
    "parent": [
      ["2"],
      "origin"
    ],
    "child": [
      ["fixed"],
      "distal"
    ],
    "type": "revolute",
    "axis": [1, 0, 0],
    "initial_displacement": 0.776,
    "initial_velocity": 0.02623322119
   },
   ...
```

The key `"type"` is processed to determine the joint specialization the user has specified. The key is then used to generate a specialization of the class (`shift::core::joint::Revolute`). Then the JSON file is parsed by calling the `ReadJSON()` member function for `shift::core::joint::Revolute`, stepping through each `shift::core::parameter::Value` datastructure within its `shift::core::parameter::List` base class. The parameters were added in the `DeclareParameters()` function during class construction. During processing of each `shift::core::parameter::Value`, the data is mapped from the JSON file to the class member variables using classes and class member functions defined in the following files. In this way, there is no need for developers of class specializations to write logic for parsing model input files.

## Accessing named parameters

### Lists
The `[]` operator for the `shift::core::parameter::List` class is overloaded to allow lookup by key. This method of accessing a hierarchical data structure should be familiar to developers who have used a Python dictionary.

```c
context["model_parameters"]["units"]["length"].Print();
```

Although the syntax for traversing the hierarchy is identical to a Python dictionary, because C++ is a statically typed language, the `Shift::Value` object must be cast to its appropriate type before using its underlying data in code. *Shift* provides a type-safe mechanism to request the underlying value. To access the variable `"length"` as a string, use the templated `get()` function. The `get()` functions performs type-safe conversion of its underlying `std::any` data structure to the requested format as:

```c
const std::string& length = context["model_parameters"]["units"]["length"].get<std::string>();
```

If the requested format does not match the underlying format, an exception will be thrown.

Similarly, it is possible to get a raw pointer to the underlying data as:

```c
const std::string* length_ptr = context["model_parameters"]["units"]["length"].get_data<std::string>();
```

For debugging purposes, it is possible to query the type of object as:

```c
std::cout << context["model_parameters"]["units"]["length"].get_type_name();

>>> std::string
```

The `get_type_name()` function returns the *demangled* human-readable name of the underlying type.

Similarly, it is also possible to obtain the value's type id or type index. The below code would typically not be used at run-time, but may be helpful for debugging purposes when developing new *Shift* objects.

```c
std::string value;
std::cout << (std::type_index(typeid(&value)) ==
               context["model_parameters"]["units"]["length"].get_type_index()
              ? "true"
              : "false")
  << std::endl;

>>> true
```

### Arrays
The `shift::core::parameter::Array` and `shift::core::parameter::Factory` array classes support lookup by name or by index using the `[]` operator. For example, consider the *Shift* hierarchy below:

```c
{
  "assemblies": [
    {
      "name": "my_model",
      "bodies": [
        {
          "name": "first_body"
          ...
        },
        {
          "name": "second_body"
          ...
        }
      ]
      ...
    }
  ]
}
```

In the above example `...` indicates that there would be other key-value pairs which are omitted here. After the above file is loaded into a `shift::core::Context` data structure, the second body in the array can be accessed by its index as:

```c
shift::core::body::Base* body =
  context["assemblies"][0]["bodies"][1].get_data<shift::core::body::Base>();
```

The above method is similar to traversing a Python dictionary. It is also possible to look up a value by its descriptive `"name"` key as:

```c
shift::core::body::Base* body =
  context["assemblies"]["my_model"]["bodies"]["second_body"].get_data<shift::core::body::Base>();
```

## Classes to support introspection
Named parameters are supported with the addition of new base classes:

* `shift::core::parameter::Value`: provides a mechanism for type safe conversion and JSON parsing functionality.
* `shift::core::parameter::List`: provides a map of keys to a number of `shift::core::parameter::Value` objects.
* `shift::core::parameter::Array` an array of non-factory *Shift* objects, such as `Shift::Assembly` and `Shift::Frame` classes.
* `shift::core::parameter::FactoryArray`: an array of factory *Shift* objects, such as `shift::core::body::Base` or `shift::core::joint::Base`.

When parsing the JSON input file is parsed, the data under the `name` key word in the code block above will be parsed and loaded into the `name_` member object. The type passed to overloaded `DeclareParameter()` function is used to determine the appropriate way to parse the JSON file. In this example,  `name_` is a pointer to a string, so a string is expected in the JSON input file. If the data is incorrectly formatted, an exception will be thrown.

The following code demonstrates how an object specialization is created in *Shift* internal code using the factory method. The example is provided to describe how the classes are generated internally, but it would never be necessary for a developer of a *Shift* extension (such as a class specialization of a `shift::core::body::Base` or `shift::core::joint::Base`) to write this code themselves.

```c
#include "shift/core/factory.h"
#include "shift/core/joint/base.h"

std::string joint_type(value["type"].asString());
auto joint = shift::core::Factory<shift::core::joint::Base>::get()->Create(joint_type);
joint->ReadJSON(value);
```
