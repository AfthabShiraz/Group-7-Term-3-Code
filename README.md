# Group-7-Term-3-Code

Welcome to our code!

This project contains code for a four wheeled robot. The robot is designed to traverse a variety of obstacles, including but not limited to wall following, tunnels, stairs, and crossing gaps. Our code is designed such that a button on the robot can control what code is being run, allowing the robot to use different code when at different stages of the obstcacly course. The actuators of our system are 4 dc motors for the wheels and 2 servo motors. The sensors on the robot include line following sensors and distance sensors. The project is run on an arduino giga and the chassis was constructed through a mix of custom 3d-printed components and laser cut pieces.

Our robotics code has three paths, controlled by a button. Every loop the kill switch is first checked, so in order to start the robot the user must press this button. Afterwards a counter on a separate button is used to decide whether line following, wall following, or the lava pit code should be activated. Within each of these paths, the necessary sensors collect data and apply the appropriate input to the motors. Pressing the button changes the state in an A-B-C-A-B-C manner, with each mode the letters representing the different modes.
