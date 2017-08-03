.. _contributing:

.. title:: Contributing



Contributing
====================

Roadmap
-----------

Presently, the plan for *star2* is to make the core solver, graphical user interface and C API free and open source. Contributors to star2 are sought, particularly if you are interested in incorporating star2 into your research. Possible development tasks may include:

  * Interfaces and APIs
     - C++ API
     - Python API
     - ROS interface
     - OpenAI interface
     - Web interface via Emscriptem
     - Web interface via web sockets
     - Conversion from URDF to *star2* JSON input file format
     - Solidworks or Inventor plugins to generate *star2* JSON input files from CAD models
  * Graphics
     - Dynamic shadow mapping
     - Texture mapping
     - Skybox
     - Extinction
     - Tools to position light source as a function of time of day / geographic location
  * GUI
     - Interactively build/position mechanisms and save changes to disk using JSON file format
     - Graphic design of icons
  * Contact detection
     - Bounding volume hierarchy
     - Convex hull decomposition and GJK Algorithm
  * Force modules
     - Pacejka's "magic" tire model
     - Kelvin-Voigt, Nikravesh and/or Zhiying/Qishao dissipative contact force models
     - Lugre bristle model or other friction models
     - ANCF or modal representation of flexible bodies
  * Core solver
     - Support transition to new kinematics library currently in development
     - Investigate use of Intel IPP
     - Implement additional core solvers
     - Debug spherical joints
     - Debug models with rigid joint attached to fixed inertial frame
  * Documentation
     - Set up and document models to demonstrate core functionality
     - Contribute to description of theory in "Developer Reference"
  * Other
     - Move JSON parsing/serializing functionality from loader.c to .c and .h files associated with bodies, joints, etc.
     - Test build on Mac or Windows machines

If you are interested to contribute to any of these development tasks, or have other development/research ideas, please contact the author.

Coding Style
------------

This guide describes the coding style used in *star2*.

.. figure:: code_quality.png
    :width: 600px
    :align: center

    credit: xkcd


Comments
********

* Use Doxygen style comment blocks for all functions and member functions in
  .c and .cpp files. Members of classes and structs should be documented in the
  corresponding .h or .hpp file. Doxygen style comment blocks should accompany
  every function in the declaration itself, not in the header file. In this way
  the description of how to use the code and the code itself are found in the
  same location and can more easily be maintained. Add [in] and [out] to every
  variable in the call list - note that this is particularly useful for
  subroutines which may change/modify data found at an address specified in one
  of the inputs to the subroutine.

  These comment blocks are intended to convey to other users of your code
  how to use/call the functions and what values are returned.

.. code::

    /*!
      @brief adds two to an integer

      Detailed description here.

      @param bar [in] an integer to add two to.
      @return output integer.
    */
    int foo(int bar) {
      return bar + 2;
    }


* Use comment blocks (\\\* \*\\) to describe how code works inline with your
  code. Use proper English in your comment blocks. Be succinct,
  but provide sufficient detail so that others can understand what you
  are doing. Do not write comments in all caps. Do not use comment blocks to
  temporarily comment out blocks of code for debugging purposes. The use of
  comment blocks within code conveys that these are intentional messages
  written for the benefit of other developers of the code, not necessarily for
  end-users of the code. Comment blocks may be interspersed anywhere within the
  code -- where clarification is deemed necessary.


.. code::

    \* This is a descriptive comment. *\

    \* This is a descriptive comment that spans multiple lines. Do not use
    asterisks at the beginning of each line within a comment block. Modern
    interpreters highlight comment blocks in a different color so it is not
    necessary to add the additional asterisks to make the comment block stand
    out from the rest of the code. The use of asterisks at the beginning of
    each line within a comment block makes it more difficult to maintain. Use
    an opening and closing mark only. *\


* Use the double slash (\\\\) comment to temporarily comment out a block of
  code for debugging purposes. The use of the (\\\\) comment conveys to
  other developers that the commented out block of code is a candidate for
  removal once the program has been verified to work correctly.

.. code::

    \\ someTestFunction();
    someOtherFunction();


Indentation / Spaces
********************

* Use two spaces for indentation in C and C++. (Use 4 spaces in Python.)

* Provide spaces after commas and around operators to improve readability.
  For example:

.. code::

    someFunction(a, b, c);

    int a = 1;

    if(a < 2) {
      printf("%d\n", a);
    }


* Always use brackets {} in for loops, even if it is just a single line. This
  makes it easier to debug or add additional functions to a for loop later. For
  loops should follow the convention:

.. code::

    for(int i = 0; i < 5; i++) {
      printf("%d\n", i);
    }


* if/else statements should follow the convention:

.. code::

    if(a < b) {
      printf("a < b\n");
    } else if(a == b) {
      printf("a == b\n");
    } else {
      printf("a > b\n");
    }


* switch statements follow the convention:


.. code::

    switch(value) {
      case CASE1: {
        someFunction();
      } break;
      case CASE2: {
        someOtherFunction();
      } break;
      default: {
        defaultFunction();
      } break;
    }


* Pointers follow the convention:

.. code::

    double* a;


Scope of variables
******************

* Try to reduce the scope of variables as practical. For example, it is
  preferred to declare an integer within a for loop, rather than declaring an
  integer at the beginning of a function and reusing it in multiple for loops.
  Reducing the scope of variables makes no different in execution speed or stack
  allocation and is easier to read and debug. An example of the preferred
  syntax is shown below:

.. code::

    void testFunction() {
      for(int i = 0; i < 3; i++) {
        printf("d ", i);
      }
      printf("\n");
      for(int i = 0; i < 10; i++) {
        printf("%d ", i);
      }
      printf("\n");
    }


Classes
*******

* Use CamelCase or CamelCaps for class names
* use mixedCase for member functions.
* preface all member variables with *m_*
* An example .h file containing a class:


.. code::

    class MyTreeItem : public QTreeWidgetItem {
      public:
        MyTreeItem(int type) {
          m_object = NULL;
          m_userType = type;
        }
        ~MyTreeItem() {}
        QString m_name; /*!< the name of the tree item*/
        void* m_object; /*!< pointer to an object associated with this tree item*/
        int m_userType; /*!< an integer specifying the user type associated
                             with this tree item: (1002: assembly,
                             1003: body group, 1004: body, 1005: frame,
                             1006: joint group, 1007: joint, 1008: light group,
                             1009: light, 1010: restraint group,
                             1011: restraint, 1012: io group, 1013: io)*/
    };


Other
*****

* Avoid global variables and macros.
* Avoid declaring multiple variables on a single line. Each declaration should
  be on its own line.
* If functions already exist in standard C libraries, it is preferred to use
  those functions over custom functions for the same purpose.
* As possible, full--dense numerical algorithms should be written to take
  advantage of the cblas library for moderate to large sized vectors/matrices.
