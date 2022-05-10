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

Within the downloaded repository, there are two packages, "control_with_force", and "find_writing_surface". Control with force is the dirrectory for moving the UR with force control however this is incomplete and as files have been left out in order for the package to build in its current state. "find_writing_surface" is the dirrectory used to connect to the camera, find the piece of paper on the table infront of the robot, and publish the position of the papers corners on the topic "PaperVector". In order to use the calibration performed previously, the translation vector and quaternion must be obtained from the output of the calibration procedure and filled into the arrays "translation" and "quaternion". Then the points found will all be published with respect to the robots base cordinate system. While this works, the method I use to find the paper is unreliable and still needs imporvement to be truely reliable. 

In order to use the quaternion provided by the calibration, quaternion math must be used. To do this I wrote my own library found in "find_writing_surface/include/quanternionMath". This library was quite difficult to produce, however provided an excellent way to learn the math behind robot quaternions, and Denavit–Hartenberg parameters. An example of how this library is used is in "find_writing_surface/src/find_writing_surface_node.cpp" however I do plan to full commit the library and make it more accesably as I imagine it will become very useful to me in the future as well. 

While a small portion of "find_writing_surface_node.cpp" is only to compute the point found relative to the robot base. I wrote the rest to connect to the camera, pull its data, connect the data with image data from the camera, and find the paper to write on within the image. While some of the code was taken from examples provided by openCV and intel realsense, there was still a signiffigant amount of work put into connecting and degugging everything.

In order to control the robot, I made another library, moveUR at "control_ur_with_force/include/moveUR". To use this library, one makes a "moveUR" object comprised of six "Link" objects. These links need to be constructed using the Denavit–Hartenberg parameters and can be found at https://www.universal-robots.com/articles/ur/application-installation/dh-parameters-for-calculations-of-kinematics-and-dynamics/. On this webpage the parameters needed to contstruct the links can be found. An example of this in action can be foud commented out in the main section of "control_ur_with_force/src/control_ur_with_force_node.cpp". This library contains methods to move the robot with positional control only, and will be updated with force control once the original goal of the project is completed. 

All this comes to being able to launch the node to find and publish the paper location
```
rosrun find_writing_surface find_writing_surface_node 
```
Then the position can be driving to using.
```
rosrun control_ur_with_force control_ur_with_force_node
```

As the goal is still to use positoinal and force control to move the robot, running control_ur_with_force_node will only bring the robot to the first corner of the papper and the other corners will have to be manually entered into control_ur_with_force_node. Evantuall all the paper positional data will be pulled automatically from the previously mentioned ros topic, but becuase I am still working on the force control this did not seem more nececcary as still shows the connection of the camera, calibration and robot all working together correctly with postional control. 

Despite this project not yet completed, the result so far shows sucsessful connectoin between the calibration, intel realsense, and the UR3e. Currently it is limited becuase of the large error in the positional data, but it could be used for many other applications requiring control of the UR via postional data provided by the Intel Realsense. Being that I am still working on the project this final result should be greatly changed with the addition of force/torque sensing. 

While the result may be underwhelming so far, what I have learned from the project is not. Starting off this project I had never used ROS, linux or Cmake. Learning those took a signiffigant amount of time this semester but the pay off is showing. I see why these tools are used as they make robotics much easier once the large learning curve is over come. The calibration portion of this project was the next hurdle. It was very difficult as the repo I was using had little doccumentation. This forced me to dig deep into the calibration process in order to troubleshoot. Troubleshooting turned out to be one of the my biggest take aways from calibration. I am now much better at finding problems with ROS packages and fixing them. Becuase I learned how to complete robot camera calibration, this helped in writing my library for working with quaternions and transforming the data from the realsense to be used by the robot. 

In writing the library to work with the quaternion produced by the camera calibration, I learned quaternion algebra and lots more about linear algebra in general. I used many different online reasources in order to gain a great enough understanding of quaternions such as 3BlueOneBrown on youtube, which I highly recomened. Knowing little to nothing about them before starting this project, they seemed completely foreign at first. Now though I can understand and implement them wherever neccecary. 

Lastly, making my own library to control the UR3e proved to be a great learning experiance as well. In this I wrote my own forward kinematics, and control scheme for the robot. I leared the Denavit–Hartenberg parameters and much more about the math and control sequence nececcary to control a robot. I also learned much more about ROS topics and how to use them. While I have learned a lot about positoin and force control I still have more to learn in order to complete this project. 

I am very happy with the outcome of this project, mainly in what I have leerned from it. In all the aspects mentioned above, I basically went from near zero knowledge to being able to make a working system involving camera robot calibration,  







