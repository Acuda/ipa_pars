# IDE README:

This file provides commands to build the workspace and .project files for the eclipse IDE. Import the project and work with cpp eclipse.

# Workflow

In catkin_ws:

 - rm -r  build/ devel/
 - catkin_make --force-cmake -G"Eclipse CDT4 - Unix Makefiles"
 - awk -f $(rospack find mk)/eclipse.awk build/.project > build/.project_with_env && mv build/.project_with_env build/.project

Start Eclipse:

 - source .bashrc enviroment, by launching it using <bash -i -c "eclipse">
 - File->Import->Existing projects into workspace
 - Fix unresolved includes: Project properties: C/C++ General->Preprocessor Include Paths->Providers->"CDT GCC Built-in Compiler Settings [Shared]
 - Index Rebuild

# Changes in .bashrc:

 - export EDITOR='vim'
 - export JAVA_HOME=/opt/Oracle_Java/jdk1.8.0_102
 - export LC_NUMERIC=C
 - source /opt/ros/indigo/setup.bash
 - source ~/git/catkin_ws/devel/setup.bash

# Richtiges Chaining:

 - zuerst source /opt/ros/indigo/setup.bash
 - dann caktkin_ws bauen
 - catkin_ws sourcen

 - falls es bin_ws gibt muss dieser VOR dem bauen des catkin_ws gesourcet sein.
