#/bin/bash

# Locate workspace
substring="upc_mrn/manifest.xml"
path_upc_mrn="$(locate $substring)"
path_ws=${path_upc_mrn/%$substring/}  
echo "Workspace path: "$path_ws

# Source Hydro 
source /opt/ros/hydro/setup.bash
echo "-> Done Hydro source"

# Workspace source
setup="setup.bash"
source $path_ws$setup
echo "-> Done Workspace source"

# Configure GAZEBO in Bashrc
gazebo_models="upc_mrn/worlds/gazebo/models/"
str=$path_ws$gazebo_models
echo "source /usr/share/gazebo/setup.sh" >> ~/.bashrc
echo "export GAZEBO_MODEL_PATH=$str:\$GAZEBO_MODEL_PATH" >> ~/.bashrc 
echo "export GAZEBO_RESOURCE_PATH=$str:\$RESOURCE_PATH" >> ~/.bashrc 
echo "-> Done Gazebo bashrc exports"
