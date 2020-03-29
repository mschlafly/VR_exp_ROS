## VR_exp_ROS
This folder contains the final code used to conduct the human subject tests for the darpa project. Please reference the pdf document for more details for how to install and run this code.

# ROS packages:
- dist_autonomy - This package creates the visual occlusion and object detection distributions and publishes them to topics /visual_dist and /obj_dist both of type Target_dist.msg, a custom message in the dist_autonomy package. To create the visual occlusion distribution, visual_occlusion.cpp subscribes to the player's current position (topic /person_position of type person_position.msg in the user_input package). Within visual_occlusion.cpp, the assumed sight capabilities of the player are defined (can see 45 degrees in each direction and 10 unity units in front) and the locations of the buildings are hard-coded. The code references a ROS parameter to find out whether the 'low' or 'high' complexity environment is being used. obj_detect_dist.py subscribes to the /obj_dist topic of type object_position.msg in the dist_autonomy package and creates gaussian peaks around the detected objects. If the same object is re-detected, the location of the gaussian distribution is updated. If it has been 15 seconds since the object was last detected, it is removed from the distribution. pub_grid.py subscribes to the /target_distribution and publishes the distribution in a GridMap format for visualization in rviz.
- user_input - This package connects to a TCP socket to receive user inputs from the computer running the Tanvas and send back the player's position. All user commands are published to the /input topic from input_array.msg in the user_input package. It subscribes to /person_position topic from the person_position.msg in the user_input package. This package also contains several custom messages for the purposes of data collection: player_info.msg, treasure_info.msg, and object_position.msg. Topics with these messages are published to from unity. For it to work, a rosnode (in this case save_data.cpp) must subscribe to these topics, even if the rosnode is never run.
- file_server - This package creates the websocket for communication with unity. It depends on the rosbridge_server package and can be launched using ros_shape_communication.launch. More documentation on how to use and set up the C# websocket can be found at
- rt_ergodic_control - This repo contains a python library for the RT ergodic controller for nonlinear systems. The nost up-to-date code can be found here, ``https://github.com/i-abr/rt_ergodic_control". We uses the code and parameters in this repo.

# System requirements
- ROS melodic
- Ubuntu desktop 18.04.4 LTS from ubuntu.com/download/desktop
- ros-melodic-catkin
- python package gym
