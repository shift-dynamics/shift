# Kinematics Subroutine Library

The files in this folder are a work in progress and are intended to eventually
replace the kinematics.c library in the core folder.

The functions in this library have replaced most cblas calls with for loops using the *__restrict__* keyword to avoid additional checks for pointer aliasing.
