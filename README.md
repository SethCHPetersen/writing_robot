# writing_robot
writing_robot drives a ur3e robot to write on a surface.

This project was made to be used with a UR3e and an Intel Realsense Camera. 

The goal of this project is to have a UR3e write on a surface using a marker. Originally the goal was to use force/torque sensing along with positional control to accuratley write on the surface. However, due to time constraits the porject currently only uses positional control and thus needs a writing utensil with a signiffigant amout of compliance. This compliance is needed due to the inaccuracies of the point cloud data provided by the intel realsense camera. The camera seems to provide point cloud data with the advertised one persenent error (one percent of depth distance for a given point), however, if a noncompliant writing utenstil is used, this is easily too much, bringing up the need for force/torque sensing without a compliant utensil. 

To use the Intel Realsense depth camera, the camera must first be calibrated to the robot workspace. This is done using the repository built off of https://github.com/IFL-CAMP/easy_handeye, and more specifically https://github.com/portgasray/ur5_realsense_calibration. However there are many parameters that need to be edited in order to correctly calibrate to the depth camera, so my own version of the calibration can be found here =============.


Once the camera calibration is done, to use this repository, you will need a workspace with a src folder.
```
mkdir -p catkin_ws/src && cd catkin_ws/src
```

Then, clone the repository into the source folder.
```
git clone https://github.com/SethCHPetersen/writing_robot.git
```
Now build it and it should be good to go.
```
cd ../
catkin_make
```




