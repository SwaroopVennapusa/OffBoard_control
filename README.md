# OffBoard_control

This repo is the implementation of offboard control for https://github.com/Open-UAV/cps_challenge_2020

# Objective

- There is a probe in the world, data mule the probe with the iris drone.
- Goto the pb rock and the drone should orbit the fragile geologic feature such that a good map can be generated with its imagery. Use ORB-SLAM2 to evaluate mapping.
- When carrying out the mapping task, ensure coverage for the rock, i.e., we want good map of all of the rock, including basal contact, and also the top, so that  centroid and footprint angles can be estimated effectively from the pointcloud.
- Smooth motions will potentially generate better rock maps. Velocity control may be smoother than position control, so choose your mission segment controllers wisely. Also, orbiting the rock a few times using a helical pattern, while trying to maintain a fixed distance to the rock is ideal. 
- When the mission is complete, the drone needs to land on the rover, awaiting the next assignment.

# Setup

- Create a workspace and clone the cps challenge
- Either create a separate workspace for this repository or copy the script in the previously created repository
- Create a workspace for ORB-SLAM2 and clone https://github.com/raulmur/ORB_SLAM2
- Update any of the yaml in the examples and add the iris drone front camera parameters

# Implementation

- In one terminal, launch the world.
` roslaunch cps_challenge_2020 phase-1.launch `
- In another terminal run the node to control the drone.
` rosrun offboard_py offb_vel.py `
- In another terminal run ORB-SLAM2
` rosrun ORB_SLAM2 Mono /root/ORB_SLAM2/Vocabulary/ORBvoc.txt /root/ORB_SLAM2/Examples/Monocular/myDrone.yaml `
- Running rqt is optional but we can check the downward facing camera to check the probe while data muling and also access forward facing camera while orbiting the rock

# Final Output

![Screenshot from 2023-04-19 19-51-31](https://user-images.githubusercontent.com/126584953/234472910-746af700-4b21-4dfb-afb5-eeb783171bdb.png)



![Screenshot from 2023-04-19 19-51-57](https://user-images.githubusercontent.com/126584953/234472951-72196513-db37-49e2-a883-79232e899aac.png)



![Screenshot from 2023-04-19 19-53-24](https://user-images.githubusercontent.com/126584953/234472976-b2efec35-9df6-4a56-bc61-fa3d9b51f5f0.png)



![Screenshot from 2023-04-19 20-32-34](https://user-images.githubusercontent.com/126584953/234472992-bf8462f1-fe01-4d35-84df-5fc258b221d8.png)



https://user-images.githubusercontent.com/126584953/234473014-5af411b1-577d-4df8-bdc0-fba222549972.mp4



