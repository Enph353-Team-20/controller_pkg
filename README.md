# controller_pkg
Controller package
Tyler Wilson and Peter Goubarev

ENPH 353 is a project-based course that teaches students how to use modern computing tools, with a focus on computer vision and machine learning. Some of the tools discussed include the Robot Operating System (ROS) and Gazebo simulator, OpenCV library, the SIFT algorithm, neural networks, and reinforcement learning. The final project for the course is a competition, in which teams of students train a robot to navigate a simulated world, resembling the task of creating a self-driving car. The goals for the robot are to navigate the world reliably, locate parked cars and read their license plates, and avoid obstacles such as pedestrians and other vehicles. Points would be awarded for returning the correct license plates and completing a loop of the course while points would be deducted for veering off the road or colliding with obstacles.

Our team’s goal was to complete all the challenges laid out in the competition, giving us the ability to get a perfect score. This included us being able to:
- Reliably complete an outer loop of the course
- Complete a loop of the inside without colliding with the car
- Identify and avoid pedestrians
- Reliably identify and return the license plates on all the vehicles

A video of our robot in action can be viewed here:
https://youtu.be/k3zzYlgzKM8

This repository is a robot controller for ENPH353 final competition. Includes robot driving algorithm, license plate location and interface with neural network, and interface with simulation and robot operating system. 

![alt text](https://github.com/Enph353-Team-20/controller_pkg/blob/master/2022-12-15%20E353%20Architecture.drawio%20(1).png)

Scoring node
- Starting and stopping of timer
- Sending plate predictions to scorekeeper  

Robot PID algorithm:
- Navigation using road lines, road surface, pedestrian crossing, and time

License Plate location algorithm: 
- Examines each camera frame to determine if there is a license plate on screen worth looking at. If there is, it performs a perspective transform to produce a flattened image of plate + parking stall number. 
- Use of HSV filter to isolate colour of license plate box
- Examination of camera frame with the largest total area of license plate box colour 
- Perspective transform to flatten plate image and glue to image of parking stall ID.

Neural network helper classes:
- Network manager class which splits up the flattened plate image into individual characters, passes them to the correct networks, then combines the results into a single plate and parking stall ID prediction.
- Network 1: license plate letter identification
- Network 2: car locator identification

Interface with robot operating system
