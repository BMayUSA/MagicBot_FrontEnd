# magicbot_frontend
Web browser front end to control the MagicBot's base and arm.  Commands are passed through a Rosbridge web socket.
Pages are intended to be hosted by a private access point on a RaspberryPi running ROS aboard the MagicBot.
None of these pages will work unless connected to the access point with a Roscore running.
Pages are available in both desktop and mobile modes.


-Index.html: simply a landing page where users can select whether to use the base or arm controller.


-Base_Control files:

After connecting to Rosbridge, the current Lidar data is visualized.
Additionally, a recent history of the robot's odometry can be seen as a line trailing the robot as it moves around its environment.
By touching or clicking on the page a virtual joystick is presented which can be used to control the motion of the base.

-Arm_Control files:

Sliders represent the current angle of each joint of the robot's arm once connected to Rosbridge.
In the default mode, each time a slider is updated a message is sent to update the corresponding joint.
Selecting the "send full configuration" mode allows the user to adjust all joints then click the send button to update all joints at once.


Laboratory4PROGRESS | University of Michigan | Summer 2017

Brian May
