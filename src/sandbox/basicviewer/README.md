# Basic Viewer OpenGL Template

This standalone viewer application is a basic template for testing new functionality before integrating into main Star2 GUI application. This example model loads one or more .obj files specified in a .json file within the scene directory. In this simple example, the two obj files are tetrahedron. It is possible to add other obj files to the scene directory and modify the json file accordingly to point to the new files.

This simple test program uses only simple or wireframe shading.

## Building and running the test program

    mkdir build
    cd build
    cmake ..
    make
    ./bin/viewer

To open a model, go to the file menu and select *Open*. Navigate up one directory from the bin folder. Select scene.json in the scene folder.

## Starting a new viewer project
To test new functionality, copy the "basicviewer" folder to a new directory within sandbox.
