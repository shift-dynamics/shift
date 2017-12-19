Git Submodule Directory
=======================

This directory contains git submodules. It can be populated when initially cloning the repository by::

  git clone --recursive git@github.com:kriswehage/shift.git


To update the repositories, issue the command from within the shift folder::

  git submodule update --recursive


If you have cloned the *Shift* repository without the recursive flag as noted above, you can populate this folder by running this command from within the shift folder as::

  git submodule update --init --recursive
