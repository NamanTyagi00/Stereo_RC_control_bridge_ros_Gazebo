In This project we will move our robot using keyboard and visualise stereo camera sensor output

# dependencies
Proper ros installation - Jazzy

gazebo bridge - ros_gz

rest dependencies are mention in package.xml files of 
`robot` and `controller` packages

# Setup
create a workspace

`mkdir -p {$worskpace_folder}/src`

clone the package `robot` and `controller`

`cd {$workspace_folder}`

now build the project 

After building source you local_setup.bash

`source ./install/local_setup.bash`

now launch the launch file in robot/launch

`ros2 launch robot diff_drive.launch.py`

# Demo

[DEMO_VIDEO](https://youtu.be/GjSaAKIPAD8)


